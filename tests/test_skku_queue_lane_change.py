from types import SimpleNamespace
from pathlib import Path
import unittest
from Road_Environment.control.skku_queue_lane_change import QueueLaneChange, protect_toll_connections


class QueueLaneChangeTests(unittest.TestCase):
    def test_toll_connectors_protected_but_booth_lanes_still_available(self):
        calls = []
        conn = SimpleNamespace(lane=SimpleNamespace(
            setChangePermissions=lambda *args: calls.append(args)))
        network = Path(__file__).resolve().parents[1] / 'env/skku_commute/skku_commute.net.xml'
        protected = protect_toll_connections(conn, network, '507449308')
        # The formerly colliding 3->2 internal-lane change must be forbidden.
        for lane in (':729262282_1_2', ':729262282_1_3'):
            self.assertIn((lane, [], 1), calls)
            self.assertIn((lane, [], -1), calls)
        self.assertTrue(protected)
        self.assertTrue(all(lane.startswith(':') for lane in protected))
        self.assertEqual(len(calls), 2 * len(protected))
        self.assertFalse(any(lane.startswith('507449308_') for lane, _, _ in calls))

    def test_left_lane_then_adjacent_lane_only_after_queue(self):
        pos={'distance':3000,'lane':3,'halted':0};requests=[];slowing=[];gaps=[];released=[]
        vehicle=SimpleNamespace(getRoadID=lambda _: 'main',getLanePosition=lambda _:4000-pos['distance'],
                                getLaneIndex=lambda _:pos['lane'],changeLane=lambda *a:requests.append(a),
                                getLaneChangeMode=lambda _:1621,setLaneChangeMode=lambda *a:None,
                                getLeader=lambda *a:('leader',50) if pos['halted'] else None,
                                getSpeed=lambda vid:0 if vid=='leader' else 8,
                                getLaneID=lambda _:'main_3',getLateralLanePosition=lambda _:0,
                                getNeighbors=lambda *a:[],slowDown=lambda *a:slowing.append(a),
                                openGap=lambda *a:gaps.append(a),getIDList=lambda:['ego','rear'],
                                deactivateGapControl=lambda vid:released.append(vid))
        conn=SimpleNamespace(vehicle=vehicle,lane=SimpleNamespace(getLength=lambda _:4000))
        plan=dict(toll_edge='toll',completion_edge='after',distance_to_toll_at_edge_start_m={'main':4000},
                  lane_counts={'main':4},hold_from_m=4500,queue_response_distance_m=1400,queue_edges=['main'])
        controller=QueueLaneChange(conn,plan)
        controller.step(0)
        self.assertTrue(controller.report['leftmost_cruising_observed'])
        self.assertEqual(requests,[])
        pos.update(distance=1300)
        controller.step(3)
        self.assertFalse(controller.report['queue_response_triggered'])
        self.assertEqual(requests,[])
        pos['halted']=5
        controller.step(6)
        self.assertTrue(controller.report['queue_response_triggered'])
        self.assertEqual(requests[-1],('ego',2,3))
        pos['lane']=2
        controller.step(10)
        self.assertEqual(requests[-1],('ego',1,3))

        # A blocked merge still sends an intention under secure-gap mode 512,
        # asks the follower to cooperate, and never pins ego's speed at zero.
        vehicle.getNeighbors=lambda _,mode:[('rear',2)] if mode==1 else []
        vehicle.getSpeed=lambda vid:0 if vid=='ego' else 8
        pos['lane']=1
        controller.step(14)
        self.assertEqual(requests[-1],('ego',0,3))
        self.assertEqual(len(requests),3)
        self.assertEqual(slowing,[])
        self.assertEqual(gaps,[('rear',2.0,15.0,8.0,1.0,2.0,'ego')])
        # Even with a free gap, never start a lane change at a connector entry.
        vehicle.getNeighbors=lambda *a:[]
        pos['distance']=10
        controller.step(18)
        self.assertEqual(len(requests),3)
        controller.step(22)
        self.assertEqual(released,['rear'])

    def test_cooperation_does_not_brake_a_vehicle_alongside_or_far_behind(self):
        calls=[]
        vehicle=SimpleNamespace(getNeighbors=lambda *a:[('alongside',-2),('far',100)],
                                getRoadID=lambda _:'main',openGap=lambda *a:calls.append(a))
        controller=QueueLaneChange(SimpleNamespace(vehicle=vehicle),{})
        controller._cooperate_for_gap(10)
        self.assertEqual(calls,[])
