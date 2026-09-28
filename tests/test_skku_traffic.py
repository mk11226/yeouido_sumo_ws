"""Check generated traffic against the actual SUMO lane connections."""
import json
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

from Road_Environment.runtime import prepare_runtime


class AreaTrafficTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        prepare_runtime()
        import sumolib
        cls.base = Path(__file__).resolve().parents[1] / 'env/skku_commute'
        cls.net = sumolib.net.readNet(str(cls.base / 'skku_commute.net.xml'))
        cls.routes = json.loads((cls.base / 'manifest.json').read_text())['background']['routes']

    def test_added_routes_are_connected_and_include_off_route_roads(self):
        ego = set(json.loads((self.base / 'route.json').read_text())['edge_ids'])
        for kind in ('opposing', 'cross_street', 'freeway_extra', 'freeway_opposing'):
            selected = [r for r in self.routes if r['kind'] == kind]
            self.assertGreater(len(selected), 0)
            for route in selected:
                if kind != 'freeway_extra':
                    self.assertTrue(set(route['edge_ids']) - ego, route['id'])
                edges = [self.net.getEdge(e) for e in route['edge_ids']]
                self.assertEqual(len(edges), len(set(edges)), route['id'])
                for a, b in zip(edges, edges[1:]):
                    self.assertTrue(any(c.getFromLane().allows('passenger') and c.getToLane().allows('passenger')
                                        for c in a.getConnections(b)), (route['id'], a.getID(), b.getID()))

    def test_local_demand_respects_shared_road_budget(self):
        from collections import Counter
        loads = Counter()
        for route in self.routes:
            if route['kind'] == 'cross_street':
                for eid in route['edge_ids']:
                    loads[eid] += route['expected_vehicles_per_hour']
        segment_vph = json.loads((self.base / 'manifest.json').read_text())['background']['segment_vph']
        for eid, rate in loads.items():
            self.assertLessEqual(rate, segment_vph * .6 * self.net.getEdge(eid).getLaneNumber() + 1e-6)

    def test_toll_routes_keep_six_booth_assignments(self):
        root = ET.parse(self.base / 'skku_commute.rou.xml').getroot()
        for route in self.routes:
            if route['assigned_booth_lanes']:
                flows = [f for f in root.findall('flow') if f.get('route') == route['id']]
                self.assertEqual({f.find('stop').get('lane') for f in flows},
                                 {f'507449308_{i}' for i in range(6)})
                self.assertNotEqual(route['edge_ids'][0], '507449308')
