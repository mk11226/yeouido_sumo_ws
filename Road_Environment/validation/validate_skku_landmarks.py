"""Integration audit: real SUMO queues, ego completion, and optional GUI captures."""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET
from Road_Environment.runtime import prepare_runtime


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gui',action='store_true')
    parser.add_argument('--seed',type=int,default=43)
    parser.add_argument('--output-dir',type=Path,required=True)
    args=parser.parse_args()
    prepare_runtime()
    import traci
    import sumolib
    from Road_Environment.build.build_skku_commute import OUT,sha256,write_json
    output=args.output_dir.resolve();output.mkdir(parents=True,exist_ok=True)
    net=sumolib.net.readNet(str(OUT/'skku_commute.net.xml'))
    route=json.loads((OUT/'route.json').read_text())
    path=[net.getEdge(e) for e in route['edge_ids']]
    toll=json.loads((OUT/'congestion.manifest.json').read_text())['toll_edge']
    ti=route['edge_ids'].index(toll)
    mainline=[e for e in path if e.getName()=='봉담과천로' and e.getType()=='highway.trunk']
    assert len(mainline)>=20
    for edge in mainline:
        assert edge.getLaneNumber()==(6 if edge.getID()==toll else 4)
    for a,b in zip(mainline,mainline[1:]):
        pairs={(c.getFromLane().getIndex(),c.getToLane().getIndex()) for c in a.getConnections(b)}
        assert {i for i,j in pairs}==set(range(a.getLaneNumber())), (a.getID(),pairs)
        assert {j for i,j in pairs}==set(range(b.getLaneNumber())), (b.getID(),pairs)
    upstream=[];length=0
    for e in reversed(path[:ti]):
        upstream.append(e.getID());length+=e.getLength()
        if length>2000:break
    seoul=[e.getID() for e in path if net.convertXY2LonLat(*e.getShape()[0])[1]>=37.46]
    report=dict(seed=args.seed,gui=args.gui,arrived=False,collisions=0,teleports=0,
                four_lane_mainline_edges=len(mainline)-1,toll_lanes=6,lane_connections_passed=True,
                max_toll_upstream_halted=0,max_toll_upstream_slow=0,max_seoul_halted=0,toll_ego_samples=0,
                toll_ego_slow_samples=0,seoul_ego_samples=0,seoul_ego_stopped_samples=0,
                input_sha256={p.name:sha256(p) for p in OUT.glob('*.xml')},samples=[])
    cmd=[sumolib.checkBinary('sumo-gui' if args.gui else 'sumo'),'-c',str(OUT/'skku_commute.sumocfg'),
         '--seed',str(args.seed),'--no-step-log','true','--duration-log.disable','true',
         '--log',str(output/'sumo.log')]
    if args.gui:cmd+=['--start','--quit-on-end','true','--delay','0','--window-size','1400,1000']
    traci.start(cmd)
    from Road_Environment.validation.audit_skku_toll import TollAudit
    toll_audit=TollAudit(traci,net,path,toll,output)
    def screenshot(name, coord, extent):
        x,y=net.convertLonLat2XY(*coord)
        traci.gui.setBoundary('View #0',x-extent/2,y-extent/2,x+extent/2,y+extent/2)
        traci.gui.screenshot('View #0',str(output/(name+'.png')),1400,1000)
    try:
        for step in range(14400):
            traci.simulationStep();now=traci.simulation.getTime()
            toll_audit.step()
            if args.gui and step==20:screenshot('han_river',(126.995,37.518),2000)
            if args.gui and step==24:screenshot('station',(127.001,37.582),850)
            if args.gui and step==28:screenshot('mainline_four_lanes',(126.9938,37.356),300)
            if args.gui and step==32:screenshot('uiwang_six_lanes',(126.9846,37.3477),250)
            if args.gui and step==3600:screenshot('uiwang_active_booths',(126.9846,37.3477),320)
            if args.gui and step==3604:screenshot('uiwang_active_merge',(126.983,37.34612),250)
            report['collisions']+=traci.simulation.getCollidingVehiclesNumber()
            report['teleports']+=traci.simulation.getStartingTeleportNumber()
            if 'ego' in traci.simulation.getArrivedIDList():
                report['arrived']=True;report['arrival_time_s']=now;break
            if step%20==0:
                halted=sum(traci.edge.getLastStepHaltingNumber(e) for e in upstream)
                slow=sum(traci.edge.getLastStepVehicleNumber(e) for e in upstream
                         if 0 <= traci.edge.getLastStepMeanSpeed(e) < 5)
                report['max_toll_upstream_slow']=max(report['max_toll_upstream_slow'],slow)
                seoul_halted=sum(traci.edge.getLastStepHaltingNumber(e) for e in seoul)
                report['max_toll_upstream_halted']=max(report['max_toll_upstream_halted'],halted)
                report['max_seoul_halted']=max(report['max_seoul_halted'],seoul_halted)
                report['samples'].append(dict(time_s=now,toll_upstream_halted=halted,seoul_halted=seoul_halted,toll_upstream_slow=slow))
                if 'ego' in traci.vehicle.getIDList():
                    road=traci.vehicle.getRoadID('ego');speed=traci.vehicle.getSpeed('ego')
                    if road in upstream or road==toll:
                        report['toll_ego_samples']+=1
                        report['toll_ego_slow_samples']+=int(speed<3)
                        if args.gui and not report.get('toll_capture'):
                            screenshot('uiwang_queue',(126.9868,37.3502),1100);report['toll_capture']=now
                    if road in seoul:
                        report['seoul_ego_samples']+=1
                        report['seoul_ego_stopped_samples']+=int(speed<.1)
            if step%1200==0:print(f't={now:.0f} toll queue max={report["max_toll_upstream_halted"]}',flush=True)
        report['last_time_s']=traci.simulation.getTime()
    finally:
        report['toll_lane_usage']=toll_audit.close()
        traci.close()
        write_json(output/'audit.json',report)
    assert report['arrived'], 'Ego did not finish'
    assert report['collisions']==report['teleports']==0, 'Collision or teleport detected'
    assert report['max_toll_upstream_slow']>=10, 'No meaningful upstream toll queue'
    assert report['toll_ego_slow_samples']>0, 'Ego did not encounter toll congestion'
    assert min(report['toll_lane_usage']['serviced_per_booth'])>=10, 'A toll booth is unused'
    assert min(report['toll_lane_usage']['completed_merge_per_booth'])>=10, 'A booth has no completed merge'
    print(json.dumps({k:v for k,v in report.items() if k not in ('samples','input_sha256')},indent=2))


if __name__=='__main__':main()
