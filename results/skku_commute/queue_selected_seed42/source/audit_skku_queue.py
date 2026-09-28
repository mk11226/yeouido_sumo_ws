"""Summarize station waits, real nearby traffic and the observed queue lane maneuver."""
import argparse
import csv
import json
import math
from pathlib import Path

STATIONS = {'구반포역': (2966.621209716075, 24394.8116307226),
            '성균관대역': (1107.0210818392225, 2121.134873584844)}


def audit(folder):
    report=json.loads((folder/'summary.json').read_text())
    rows=list(csv.DictReader((folder/'trajectory.csv').open()))
    step=report['step_length_s']
    waits={name:sum(step for r in rows if float(r['speed_mps'])<.1 and
                    math.hypot(float(r['x_m'])-x,float(r['y_m'])-y)<600)
           for name,(x,y) in STATIONS.items()}
    motion=report.get('queue_lane_change',{})
    events=motion.get('events',[])
    queue=next((e for e in events if e['event']=='queue_detected'),None)
    observed=[e for e in events if e['event']=='lane_observed' and queue and e['time_s']>=queue['time_s']]
    sequence=([queue['lane']] if queue else [])+[e['lane'] for e in observed]
    safe_samples=[int(r['nearby_vehicles_250m']) for r in rows
                  if r.get('nearby_vehicles_250m') and float(r['speed_mps'])>=15
                  and float(r['posted_speed_mps'])>=19.4 and not r['road_id'].startswith(':')]
    discontinuities=[]
    for a,b in zip(rows,rows[1:]):
        dt=float(b['time_s'])-float(a['time_s'])
        moved=math.hypot(float(b['x_m'])-float(a['x_m']),float(b['y_m'])-float(a['y_m']))
        if moved>max(float(a['speed_mps']),float(b['speed_mps']))*dt+2:
            discontinuities.append(dict(time_s=float(b['time_s']),distance_m=moved))
    result=dict(station_radius_m=600,stopped_speed_threshold_mps=.1,station_stopped_seconds=waits,
                target_seconds=200,target_band_seconds=[150,250],
                within_target_band={name:150<=value<=250 for name,value in waits.items()},
                actual_lane_sequence_sumo=sequence,
                first_lane_then_queue_change_observed=bool(motion.get('leftmost_cruising_observed') and queue
                    and queue['lane']==3 and any(e['lane']<3 for e in observed) and motion.get('completed')),
                maneuver=motion,moving_open_road_neighbors_250m=dict(samples=len(safe_samples),
                    mean=sum(safe_samples)/len(safe_samples) if safe_samples else None,max=max(safe_samples,default=0)),
                ego_arrived=report['arrived'],ego_collision=report['ego_collision'],
                ego_duration_s=report['ego_duration_s'],background_collision_reports=len(report['collision_reports']),
                teleport_events=len(report['teleport_events']),motion_discontinuities=discontinuities)
    (folder/'queue_audit.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_dir',type=Path)
    args=parser.parse_args()
    print(json.dumps(audit(args.run_dir),ensure_ascii=False,indent=2))
