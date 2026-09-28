"""Request leftmost-lane cruising, then safe lane changes when a queue is detected."""
from pathlib import Path


def build_plan(path, output):
    from Road_Environment.build.build_skku_commute import write_json
    toll = next(i for i,e in enumerate(path) if e.getID()=='507449308')
    remaining={};distance=0
    for edge in reversed(path[:toll]):
        remaining[edge.getID()]=distance+edge.getLength();distance+=edge.getLength()
    plan=dict(enabled=True,toll_edge='507449308',completion_edge=path[toll+1].getID(),
              distance_to_toll_at_edge_start_m=remaining,
              lane_counts={e.getID():e.getLaneNumber() for e in path[:toll]},
              hold_from_m=4500,queue_response_distance_m=1400,
              queue_edges=['1224608699','489526373','489526373-AddedOffRampEdge','507449308'],
              first_lane_definition='leftmost lane; SUMO index lane_count - 1',
              safety='Default SUMO safety checks retained; requests may be refused',
              behavior='Keep original route through toll; request adjacent right lane after queue detection')
    write_json(Path(output)/'queue_lane_change.json',plan)
    return plan


class QueueLaneChange:
    def __init__(self,conn,plan):
        self.conn=conn;self.plan=plan;self.state='approaching';self.next_request=0
        self.report={'state':'approaching','leftmost_cruising_observed':False,'toll_observed':False,
                     'completed':False,'queue_response_triggered':False,'events':[]}
        self.last_lane=None

    def step(self,now):
        c=self.conn;p=self.plan;edge=c.vehicle.getRoadID('ego')
        if edge==p['completion_edge']:
            self.report['completed']=True;self.state='completed'
        if edge==p['toll_edge']:
            if not self.report['toll_observed']:
                self.report['events'].append(dict(time_s=now,event='entered_toll'))
            self.report['toll_observed']=True;self.state='in_toll'
        self.report['state']=self.state
        if self.state in ('completed','in_toll') or edge.startswith(':'):return
        if edge not in p['distance_to_toll_at_edge_start_m']:return
        distance=p['distance_to_toll_at_edge_start_m'][edge]-c.vehicle.getLanePosition('ego')
        lanes=p['lane_counts'][edge];lane=c.vehicle.getLaneIndex('ego')
        if lanes<4 or distance>p['hold_from_m']:return
        if self.state=='approaching':self.state='holding_first_lane'
        if self.last_lane!=lane:
            self.report['events'].append(dict(time_s=now,event='lane_observed',lane=lane,edge=edge,
                                             distance_to_toll_m=distance,state=self.state))
            self.last_lane=lane
        if self.state=='holding_first_lane':
            if lane==lanes-1:
                self.report['leftmost_cruising_observed']=True
            queue=sum(c.edge.getLastStepHaltingNumber(e) for e in p['queue_edges'])
            if distance<=p['queue_response_distance_m'] and queue>=3:
                self.state='responding_to_queue';self.report['queue_response_triggered']=True
                self.report['events'].append(dict(time_s=now,event='queue_detected',distance_to_toll_m=distance,
                                                 halted_vehicles=queue,lane=lane))
        if now>=self.next_request:
            # Only request a neighboring lane after congestion; never moveTo or disable safety.
            target=lanes-1 if self.state=='holding_first_lane' else max(0,lane-1)
            c.vehicle.changeLane('ego',target,3)
            self.next_request=now+3
        self.report['state']=self.state
