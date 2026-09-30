"""Request leftmost-lane cruising, then safe lane changes when a queue is detected."""
from pathlib import Path
import xml.etree.ElementTree as ET


def protect_toll_connections(conn, network_path, toll_edge):
    """Keep vehicles on their connector through the toll fan-out/merge.

    These internal lanes converge/diverge and are not parallel road lanes.
    Booth selection remains possible on the approach and on the plaza itself.
    Apply to all vehicles, retaining SUMO collision and longitudinal checks.
    """
    root = ET.parse(network_path).getroot()
    connectors = {c.get('via') for c in root.findall('connection')
                  if c.get('from') == toll_edge or c.get('to') == toll_edge}
    connectors.discard(None)
    for lane in sorted(connectors):
        conn.lane.setChangePermissions(lane, [], 1)
        conn.lane.setChangePermissions(lane, [], -1)
    return sorted(connectors)


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
              local_queue_distance_m=120,local_queue_speed_mps=5,
              alignment_distance_m=500,
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
        self.original_lane_mode = None
        self.gap_vehicle = None
        self.gap_until = 0
        self.gap_origin = None

    def _cooperate_for_gap(self, now):
        """Ask one adjacent follower to yield briefly; never freeze ego.

        This explicitly models a cooperative driver, rather than assuming
        that unmodified surrounding traffic will always make room.
        """
        if now < self.gap_until:
            return
        v = self.conn.vehicle
        rear = [(vid, gap) for vid, gap in v.getNeighbors('ego', 1)
                if 0 <= gap <= 60 and v.getRoadID(vid) == v.getRoadID('ego')]
        if not rear:
            return
        vid, gap = min(rear, key=lambda item: item[1])
        v.openGap(vid, 2.0, 15.0, 8.0, 1.0, 2.0, 'ego')
        self.gap_vehicle = vid
        self.gap_until = now + 8
        self.gap_origin = (v.getRoadID('ego'), v.getLaneIndex('ego'))
        self.report['events'].append(dict(time_s=now,event='cooperative_gap',vehicle=vid,gap_m=gap))

    def _release_gap(self):
        if self.gap_vehicle is not None:
            if self.gap_vehicle in self.conn.vehicle.getIDList():
                self.conn.vehicle.deactivateGapControl(self.gap_vehicle)
            self.gap_vehicle = None
            self.gap_until = 0
            self.gap_origin = None

    def _safe_adjacent(self, direction):
        v = self.conn.vehicle
        speed = v.getSpeed('ego')
        # Query all neighbors, not just those SUMO already marks as blockers.
        for vehicle, gap in v.getNeighbors('ego', 2 if direction > 0 else 3):
            other_speed = v.getSpeed(vehicle)
            if gap < max(5 + speed, 5 + max(0, speed-other_speed)*3):
                return False
        for vehicle, gap in v.getNeighbors('ego', 0 if direction > 0 else 1):
            other_speed = v.getSpeed(vehicle)
            if gap < max(5 + other_speed, 5 + max(0, other_speed-speed)*3):
                return False
        return True

    def step(self,now):
        c=self.conn;p=self.plan;edge=c.vehicle.getRoadID('ego')
        if self.gap_vehicle is not None and (
                now >= self.gap_until or edge != self.gap_origin[0]
                or (c.vehicle.getLaneIndex('ego') != self.gap_origin[1]
                    and abs(c.vehicle.getLateralLanePosition('ego')) < .1)):
            self._release_gap()
        if edge==p['completion_edge']:
            self.report['completed']=True;self.state='completed'
        if edge==p['toll_edge']:
            if not self.report['toll_observed']:
                self.report['events'].append(dict(time_s=now,event='entered_toll'))
            self.report['toll_observed']=True;self.state='in_toll'
        if self.state in ('completed', 'in_toll') and self.original_lane_mode is not None:
            c.vehicle.setLaneChangeMode('ego', self.original_lane_mode)
            self.original_lane_mode = None
            self._release_gap()
        self.report['state']=self.state
        if self.state in ('completed','in_toll') or edge.startswith(':'):return
        if edge not in p['distance_to_toll_at_edge_start_m']:return
        distance=p['distance_to_toll_at_edge_start_m'][edge]-c.vehicle.getLanePosition('ego')
        lanes=p['lane_counts'][edge];lane=c.vehicle.getLaneIndex('ego')
        if lane == 0 and abs(c.vehicle.getLateralLanePosition('ego')) < .1:
            self._release_gap()
        if lanes<4 or distance>p['hold_from_m']:return
        if self.state=='approaching':
            self.state='holding_first_lane'
            self.original_lane_mode = c.vehicle.getLaneChangeMode('ego')
            # TraCI owns lane selection here; SUMO still enforces safe gaps.
            c.vehicle.setLaneChangeMode('ego', 512)
        if self.last_lane!=lane:
            self.report['events'].append(dict(time_s=now,event='lane_observed',lane=lane,edge=edge,
                                             distance_to_toll_m=distance,state=self.state))
            self.last_lane=lane
        if self.state=='holding_first_lane':
            if lane==lanes-1:
                self.report['leftmost_cruising_observed']=True
            leader = c.vehicle.getLeader('ego', p.get('local_queue_distance_m',120))
            queue = bool(leader and leader[1] <= p.get('local_queue_distance_m',120)
                         and c.vehicle.getSpeed(leader[0]) < p.get('local_queue_speed_mps',5))
            if lane==lanes-1 and distance<=p['queue_response_distance_m'] and queue:
                self.state='responding_to_queue';self.report['queue_response_triggered']=True
                self.report['events'].append(dict(time_s=now,event='queue_detected',distance_to_toll_m=distance,
                                                 leader=leader[0],leader_gap_m=leader[1],lane=lane))
            elif distance <= p.get('alignment_distance_m',500):
                self.state='aligning_for_toll'
                self.report['events'].append(dict(time_s=now,event='toll_alignment',distance_to_toll_m=distance,lane=lane))
        if now>=self.next_request:
            target=min(lanes-1,lane+1) if self.state=='holding_first_lane' else max(0,lane-1)
            # Complete a maneuver before asking for another, and do not start
            # one that would carry lateral movement into a junction connector.
            remaining = c.lane.getLength(c.vehicle.getLaneID('ego')) - c.vehicle.getLanePosition('ego')
            if (target != lane and abs(c.vehicle.getLateralLanePosition('ego')) < .1
                    and remaining > max(15, c.vehicle.getSpeed('ego')*4 + 5)):
                if target < lane or self._safe_adjacent(1):
                    # Send the intention even before a gap exists: mode 512
                    # retains SUMO's secure-gap checks and speed adaptation.
                    # Withholding this request prevents native cooperation.
                    if target < lane and not self._safe_adjacent(-1):
                        self._cooperate_for_gap(now)
                    c.vehicle.changeLane('ego',target,3)
                    self.report['events'].append(dict(time_s=now,event='lane_requested',from_lane=lane,to_lane=target))
                    self.next_request=now+3.5
        self.report['state']=self.state
