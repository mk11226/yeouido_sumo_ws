import unittest
import random
from types import SimpleNamespace

from env.fixed_population import FixedPopulation, make_population, free_position, quotas
import traci.constants as tc


class FixedPopulationTests(unittest.TestCase):
    def test_integer_allocation_preserves_total_and_proportions(self):
        weights = {"short": 50, "long": 500, "wide": 1200}
        for total in (0, 1, 462, 3338, 4417):
            result = quotas(total, weights)
            self.assertEqual(sum(result.values()), total)
            for key in weights:
                self.assertLess(abs(result[key] - total * weights[key] / sum(weights.values())), 1)

    def test_no_space_is_rejected_and_clear_gap_is_found(self):
        self.assertIsNone(free_position(50, [15, 25, 35]))
        position = free_position(200, [30, 60, 180])
        self.assertTrue(15 <= position <= 185)
        self.assertTrue(all(abs(position - p) >= 15 for p in (30, 60, 180)))

    def manager(self, target, pending):
        manager = object.__new__(FixedPopulation)
        manager.target = target
        manager.pending = pending
        manager.subscribed = set()
        manager.subscribe = lambda api, ids: None
        manager.rng = random.Random(7)
        manager.lanes, manager.quota = {}, {}
        return manager

    def api(self, ids):
        return SimpleNamespace(
            simulation=SimpleNamespace(getTime=lambda: 5, getArrivedIDList=lambda: (),
                getStartingTeleportIDList=lambda: (), getEndingTeleportIDList=lambda: (),
                getMinExpectedNumber=lambda: len(ids)),
            vehicle=SimpleNamespace(getIDList=lambda: ids, getAllSubscriptionResults=lambda: {},
                                    getPosition=lambda vid: (100000, 100000)))

    def test_pending_insertion_consumes_budget_and_ego_does_not(self):
        manager = self.manager(3, {"fp_3": ("lane", 50, 0)})
        self.assertEqual(manager.tick(self.api(("fp_1", "fp_2", "ego"))), 0)
        self.assertEqual(len(manager.pending), 1)

    def test_departed_pending_request_is_not_counted_twice(self):
        manager = self.manager(3, {"fp_3": ("lane", 50, 0)})
        self.assertEqual(manager.tick(self.api(("fp_1", "fp_2", "fp_3"))), 0)
        self.assertEqual(manager.pending, {})

    def test_lowering_target_does_not_delete_active_vehicles(self):
        manager = self.manager(1, {})
        self.assertEqual(manager.tick(self.api(("fp_1", "fp_2", "fp_3", "ego"))), 0)

    def test_off_road_teleporting_vehicle_keeps_its_slot(self):
        manager = self.manager(1, {})
        api = self.api(())
        api.simulation.getMinExpectedNumber = lambda: 1
        # The off-road vehicle is absent from getIDList but remains expected.
        self.assertEqual(manager.tick(api), 0)

    def replenishing_manager(self):
        manager = self.manager(3, {})
        manager.lanes = {"lane": SimpleNamespace(getLength=lambda: 200,
                                                getShape=lambda: [(0, 0), (200, 0)])}
        manager.quota = {"lane": 3}
        calls = []
        def add(api, lane, pos, depart_speed):
            calls.append((lane, pos, depart_speed))
            manager.pending[f"fp_new_{len(calls)}"] = (lane, pos, 5)
        manager.add = add
        return manager, calls

    def test_arrivals_are_replaced_without_exceeding_total_inventory(self):
        manager, calls = self.replenishing_manager()
        api = self.api(("fp_2", "ego"))
        api.simulation.getArrivedIDList = lambda: ("fp_1",)
        api.vehicle.getAllSubscriptionResults = lambda: {
            "fp_2": {tc.VAR_LANE_ID: "lane", tc.VAR_LANEPOSITION: 100}}
        self.assertEqual(manager.tick(api), 2)
        self.assertEqual(manager.expected_population(api), 3)
        self.assertEqual(manager.tick(api), 0)
        self.assertEqual(len(calls), 2)
        self.assertTrue(all(c[2] == "max" for c in calls))

    def test_replenishment_limit_and_snapshot_zero_limit_are_respected(self):
        manager, calls = self.replenishing_manager()
        api = self.api(())
        self.assertEqual(manager.tick(api, limit=0), 0)
        self.assertEqual(calls, [])
        self.assertEqual(manager.tick(api, limit=1), 1)
        self.assertEqual(manager.expected_population(api), 1)

    def test_teleport_sentinel_is_excluded_from_displayed_speed(self):
        manager = self.manager(2, {})
        manager.lanes = {"lane": SimpleNamespace(getEdge=lambda: SimpleNamespace(getID=lambda: "road"))}
        manager.created = manager.cancelled_pending = 0
        api = self.api(("fp_1",))
        api.simulation.getMinExpectedNumber = lambda: 2
        api.vehicle.getAllSubscriptionResults = lambda: {
            "fp_1": {tc.VAR_LANE_ID: "lane", tc.VAR_SPEED: 10},
            "fp_2": {tc.VAR_LANE_ID: "", tc.VAR_SPEED: tc.INVALID_DOUBLE_VALUE}}
        result = manager.stats(api)
        self.assertEqual(result["registered"], 1)
        self.assertEqual(result["on_road"], 1)
        self.assertEqual(result["in_transit"], 1)
        self.assertEqual(result["mean_speed_kmh"], 36)

    def test_all_balanced_departure_lanes_have_a_valid_turn(self):
        manager = make_population(4417)
        counts = {}
        for key, lane in manager.lanes.items():
            route = manager.route_for(key)
            self.assertGreaterEqual(len(route), 2)
            if manager.route_edges:
                self.assertLessEqual(len(route), manager.route_edges)
            self.assertEqual(route[0], lane.getEdge().getID())
            self.assertIn(route[1], manager.next_edges[key])
            counts[route[0]] = counts.get(route[0], 0) + manager.quota[key]
        self.assertEqual(sum(manager.quota.values()), 4417)
        self.assertTrue(all(n >= 1 for n in counts.values()))


if __name__ == "__main__":
    unittest.main()
