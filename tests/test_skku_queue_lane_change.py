from types import SimpleNamespace
import unittest
from skku_queue_lane_change import QueueLaneChange


class QueueLaneChangeTests(unittest.TestCase):
    def test_left_lane_then_adjacent_lane_only_after_queue(self):
        pos={'distance':3000,'lane':3,'halted':0};requests=[]
        vehicle=SimpleNamespace(getRoadID=lambda _: 'main',getLanePosition=lambda _:4000-pos['distance'],
                                getLaneIndex=lambda _:pos['lane'],changeLane=lambda *a:requests.append(a))
        conn=SimpleNamespace(vehicle=vehicle,edge=SimpleNamespace(getLastStepHaltingNumber=lambda _:pos['halted']))
        plan=dict(toll_edge='toll',completion_edge='after',distance_to_toll_at_edge_start_m={'main':4000},
                  lane_counts={'main':4},hold_from_m=4500,queue_response_distance_m=1400,queue_edges=['main'])
        controller=QueueLaneChange(conn,plan)
        controller.step(0)
        self.assertTrue(controller.report['leftmost_cruising_observed'])
        self.assertEqual(requests[-1],('ego',3,3))
        pos.update(distance=1300)
        controller.step(3)
        self.assertFalse(controller.report['queue_response_triggered'])
        self.assertEqual(requests[-1][1],3)
        pos['halted']=5
        controller.step(6)
        self.assertTrue(controller.report['queue_response_triggered'])
        self.assertEqual(requests[-1],('ego',2,3))
        pos['lane']=2
        controller.step(9)
        self.assertEqual(requests[-1],('ego',1,3))
