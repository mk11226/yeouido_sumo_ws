"""Observe actual lane use and complete 4->6->4 passages without controlling cars."""
import csv
from collections import Counter
import re


class TollAudit:
    def __init__(self, conn, net, path, toll_id, output):
        import traci.constants as tc
        self.tc=tc;self.conn=conn;self.toll=toll_id
        ti=next(i for i,e in enumerate(path) if e.getID()==toll_id)
        self.before=path[ti-1].getID();self.after=path[ti+1].getID()
        self.lanes={e.getID():[l.getID() for l in e.getLanes()] for e in path[ti-1:ti+2]}
        for lanes in self.lanes.values():
            for lane in lanes:conn.lane.subscribe(lane,[tc.LAST_STEP_VEHICLE_ID_LIST])
        self.seen={};self.entered=[set() for _ in range(6)];self.served=[set() for _ in range(6)]
        self.merged=[set() for _ in range(6)];self.expansion=Counter();self.merge=Counter()
        self.toll_changes=0;self.cohorts={};self.trace_ids=set();self.traces={}
        self.max_simultaneous_lanes=0
        self.events_file=(output/'toll_lane_events.csv').open('w',newline='')
        self.events=csv.writer(self.events_file);self.events.writerow(['time_s','vehicle','from_lane','to_lane','served_booth'])
        self.trace_file=(output/'toll_motion.csv').open('w',newline='')
        self.trace=csv.writer(self.trace_file);self.trace.writerow(['time_s','vehicle','lane','lane_position_m','x_m','y_m','speed_mps'])

    def step(self):
        conn=self.conn;tc=self.tc;now=conn.simulation.getTime();current={};occupied=set()
        for edge,lanes in self.lanes.items():
            for index,lane in enumerate(lanes):
                for vid in (conn.lane.getSubscriptionResults(lane) or {}).get(tc.LAST_STEP_VEHICLE_ID_LIST,[]):
                    current[vid]=(edge,index,lane)
                    state=self.seen.setdefault(vid,{'last':None,'booth':None,'entered_from_four':False})
                    old=state['last']
                    if edge==self.toll:
                        self.entered[index].add(vid);occupied.add(index)
                        if old and old[0]==self.before:
                            self.expansion[f'{old[1]}->{index}']+=1
                            state['entered_from_four']=True
                        if old and old[0]==self.toll and old[1]!=index:self.toll_changes+=1
                    if edge==self.after and old and old[0]==self.toll:
                        self.merge[f'{old[1]}->{index}']+=1
                        if state['booth'] is not None and state['entered_from_four']:
                            self.merged[state['booth']].add(vid)
                    if old and old[2]!=lane:
                        self.events.writerow([now,vid,old[2],lane,state['booth']])
                    state['last']=(edge,index,lane)
                    match=re.search(r'_booth([0-5])\.',vid)
                    target=match.group(1) if match else 'ego' if vid=='ego' else None
                    if target is not None and target not in self.cohorts:
                        self.cohorts[target]=vid;self.trace_ids.add(vid)
                        conn.vehicle.subscribe(vid,[tc.VAR_POSITION,tc.VAR_SPEED,tc.VAR_LANE_ID,tc.VAR_LANEPOSITION])
        self.max_simultaneous_lanes=max(self.max_simultaneous_lanes,len(occupied))
        for vid in conn.simulation.getStopStartingVehiclesIDList():
            if vid in current and current[vid][0]==self.toll:
                index=current[vid][1];self.served[index].add(vid);self.seen[vid]['booth']=index
        for vid in list(self.trace_ids):
            data=conn.vehicle.getSubscriptionResults(vid)
            if data:
                x,y=data[tc.VAR_POSITION]
                self.trace.writerow([now,vid,data[tc.VAR_LANE_ID],data[tc.VAR_LANEPOSITION],x,y,data[tc.VAR_SPEED]])
            if vid in current and current[vid][0]==self.after:
                self.trace_ids.remove(vid);conn.vehicle.unsubscribe(vid)

    def close(self):
        self.events_file.close();self.trace_file.close()
        return dict(vehicles_seen_per_toll_lane=list(map(len,self.entered)),
                    serviced_per_booth=list(map(len,self.served)),
                    completed_merge_per_booth=list(map(len,self.merged)),
                    expansion_transitions=dict(self.expansion),merge_transitions=dict(self.merge),
                    within_toll_lane_changes=self.toll_changes,
                    max_simultaneously_occupied_booth_lanes=self.max_simultaneous_lanes,
                    trajectory_vehicle_ids=self.cohorts)
