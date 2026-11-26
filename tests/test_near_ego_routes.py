import random
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

import traci.constants as tc
from env.fixed_population import FixedPopulation


class NearEgoRoutesTests(unittest.TestCase):
    def setup_case(self, distance=20, ego=True, exiting=False, internal=False):
        manager = object.__new__(FixedPopulation)
        manager.rng = random.Random(7)
        manager.pending = {}
        manager.subscribed = {'fp_1'}
        manager.cancelled_pending = 0
        lane = SimpleNamespace(allows=lambda kind: True)
        connection = SimpleNamespace(getFromLane=lambda: lane, getToLane=lambda: lane)
        tail, onward = Mock(), Mock()
        tail.getOutgoing.return_value = {} if exiting else {onward: [connection]}
        onward.allows.return_value = True
        onward.getID.return_value = 'onward'
        onward.getLength.return_value = 600
        onward.getOutgoing.return_value = {}
        manager.net = SimpleNamespace(getEdge=lambda edge: tail)
        vehicle = Mock()
        vehicle.getIDList.return_value = ['fp_1']+(['ego'] if ego else [])
        vehicle.getPosition.return_value = (0, 0)
        vehicle.getAllSubscriptionResults.return_value = {
            'fp_1': {tc.VAR_POSITION: (distance, 0)}}
        vehicle.getRoadID.return_value = ':junction' if internal else 'last'
        vehicle.getRoute.return_value = ('first', 'last')
        vehicle.getRouteIndex.return_value = 1
        return manager, SimpleNamespace(vehicle=vehicle)

    def test_nearby_arrival_gets_continuation_without_deleting_or_moving(self):
        m, api = self.setup_case()
        self.assertEqual(m.protect_near_ego(api), 1)
        api.vehicle.setRoute.assert_called_once_with('fp_1', ['last', 'onward'])
        api.vehicle.remove.assert_not_called()
        api.vehicle.moveTo.assert_not_called()

    def test_far_traffic_is_unchanged(self):
        m, api = self.setup_case(distance=200)
        self.assertEqual(m.protect_near_ego(api), 0)
        api.vehicle.setRoute.assert_not_called()

    def test_no_ego_does_not_change_traffic(self):
        m, api = self.setup_case(ego=False)
        self.assertEqual(m.protect_near_ego(api), 0)
        api.vehicle.getAllSubscriptionResults.assert_not_called()

    def test_true_exit_is_not_given_an_invalid_route(self):
        m, api = self.setup_case(exiting=True)
        self.assertEqual(m.protect_near_ego(api), 0)
        api.vehicle.setRoute.assert_not_called()

    def test_junction_traffic_is_protected_too(self):
        m, api = self.setup_case(internal=True)
        self.assertEqual(m.protect_near_ego(api), 1)
        api.vehicle.setRoute.assert_called_once()

    def test_route_with_plenty_remaining_is_unchanged(self):
        m, api = self.setup_case()
        api.vehicle.getRoute.return_value = ('first', 'second', 'third', 'last')
        api.vehicle.getRouteIndex.return_value = 0
        m.net.getEdge('first').getLength.return_value = 500
        self.assertEqual(m.protect_near_ego(api), 0)

    def test_new_traffic_cannot_spawn_inside_ego_area(self):
        m, api = self.setup_case()
        m.lanes = {'lane': SimpleNamespace(getShape=lambda: [(0, 0), (1000, 0)])}
        self.assertFalse(m.spawn_allowed(api, 'lane', 80))
        self.assertFalse(m.spawn_allowed(api, 'lane', 120))
        self.assertTrue(m.spawn_allowed(api, 'lane', 200))

    def test_direct_add_also_rejects_nearby_insertion(self):
        m, api = self.setup_case()
        m.lanes = {'lane': SimpleNamespace(getShape=lambda: [(0, 0), (1000, 0)])}
        self.assertFalse(m.add(api, 'lane', 80))
        api.vehicle.add.assert_not_called()

    def test_pending_insertion_is_cancelled_before_entering_near_ego(self):
        m, api = self.setup_case()
        m.lanes = {'lane': SimpleNamespace(getShape=lambda: [(0, 0), (1000, 0)])}
        m.pending = {'fp_wait': ('lane', 50, 0)}
        m.protect_near_ego(api)
        self.assertEqual(m.pending, {})
        self.assertEqual(m.cancelled_pending, 1)
        api.vehicle.remove.assert_called_once_with('fp_wait', reason=tc.REMOVE_VAPORIZED)


if __name__ == '__main__':
    unittest.main()
