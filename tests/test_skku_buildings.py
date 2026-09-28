"""Geometry checks that do not require SUMO or a downloaded OSM snapshot."""
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

from Road_Environment.build.build_skku_buildings import assemble_rings, finalize_polygons, make_footprints, polygon_area


class BuildingGeometryTests(unittest.TestCase):
    def setUp(self):
        self.nodes = {1: (0., 0.), 2: (10., 0.), 3: (10., 10.), 4: (0., 10.),
                      5: (2., 2.), 6: (4., 2.), 7: (4., 4.), 8: (2., 4.),
                      9: (20., 0.), 10: (30., 0.), 11: (30., 10.), 12: (20., 10.)}
        self.ways = {1: {"nodes": [1, 2, 3], "tags": {}},
                     2: {"nodes": [1, 4, 3], "tags": {}},
                     3: {"nodes": [5, 6, 7], "tags": {}},
                     4: {"nodes": [5, 8, 7], "tags": {}},
                     5: {"nodes": [9, 10, 11, 12, 9], "tags": {"building:levels": "2"}}}

    def test_disjoint_outer_and_split_courtyard(self):
        relation = {99: {"tags": {"type": "multipolygon", "building": "yes", "building:levels": "5"},
                         "members": [{"type": "way", "ref": str(i), "role": "inner" if i in (3, 4) else "outer"}
                                     for i in self.ways]}}
        shapes, skipped = make_footprints(self.nodes, self.ways, relation)
        self.assertEqual(skipped, [])
        self.assertEqual(len(shapes), 3)
        outers = [shape for shape in shapes if shape["role"] == "outer"]
        self.assertEqual(sum(polygon_area([self.nodes[n] for n in shape["nodes"]]) for shape in outers), 200)
        self.assertEqual(next(shape for shape in shapes if shape["role"] == "inner")["parent"],
                         "building_relation_99_part_0")
        part = next(shape for shape in outers if shape["member_ways"] == [5])
        self.assertEqual(part["tags"]["building:levels"], "5")
        self.assertEqual(part["tags"]["source:osm:member_way:5:building:levels"], "2")

    def test_open_ring_is_not_invented(self):
        rings, failures = assemble_rings([1], self.ways, self.nodes)
        self.assertEqual(rings, [])
        self.assertEqual(failures[0]["reason"], "open_or_branching_member_component")

    def test_source_closure_and_whitespace_metadata_survive_projection(self):
        with tempfile.TemporaryDirectory() as folder:
            raw, output = Path(folder) / "raw.xml", Path(folder) / "out.xml"
            raw.write_text('<additional><poly id="1" shape="0,0 2,0 2,2 0,.05">'
                           '<param key="building" value="yes"/><param key="name" value="Original"/>'
                           '</poly></additional>')
            mapping = {"1": {"id": "building_way_1", "feature": "way/1", "role": "outer",
                             "nodes": [1, 2, 3, 4, 1], "member_ways": [1],
                             "tags": {"building": "yes", "name": "Original\r"}}}
            retained, skipped, closures, attrs = finalize_polygons(raw, output, mapping)
            self.assertEqual(skipped, [])
            self.assertEqual(len(closures), 1)
            self.assertEqual(retained[0]["points"][0], retained[0]["points"][-1])
            self.assertEqual(len(attrs), 1)
            poly = ET.parse(output).getroot().find("poly")
            self.assertEqual(poly.find("param[@key='name']").get("value"), "Original\r")


if __name__ == "__main__":
    unittest.main()
