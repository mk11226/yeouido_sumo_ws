"""The Daebang exit must remain an unsignaled four-lane continuation."""
import shutil
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
import math

from yeouido.config import NET, sumo_bin
from yeouido.corrections import lane_counts
from yeouido.road_corrections import (
    DAEBANG_EXIT, DAEBANG_OUT, DAEBANG_THROUGH, correct_daebang_exit,
)


def canonical(element):
    return element.tag, sorted(element.attrib.items()), [canonical(e) for e in element]


class LaneExpansionTests(unittest.TestCase):
    @unittest.skipUnless(NET.exists(), 'SUMO network required')
    def test_mullae_left_two_lanes_turn_towards_mullae_park(self):
        root = ET.parse(NET).getroot()
        rows = [c for c in root.findall('connection') if c.get('from') == '1180000404']
        actual = {i: {c.get('to') for c in rows if int(c.get('fromLane')) == i} for i in range(4)}
        self.assertEqual(actual, {0: {'1180063700', '1180058000'}, 1: {'1180063700'},
                                  2: {'1180015302'}, 3: {'1180015302'}})
        left_lanes = {(int(c.get('fromLane')), int(c.get('toLane')))
                      for c in rows if c.get('to') == '1180015302'}
        self.assertEqual(left_lanes, {(2, 0), (3, 1)})

    def test_left_expansion_preserves_right_boundary(self):
        link = dict(id='test', lanes=3, shape=[[126.9, 37.5], [126.9, 37.501]])
        for side, expected_shift in [('left', -1.6), ('right', 1.6)]:
            widened, skipped = lane_counts([link], [dict(edge='test', lanes=4, side=side)])
            self.assertFalse(skipped)
            self.assertEqual(widened[0]['lanes'], 4)
            dx = (widened[0]['shape'][0][0] - link['shape'][0][0]) * 111320 * math.cos(math.radians(37.5))
            self.assertAlmostEqual(dx, expected_shift, places=5)

    def test_existing_configuration_keeps_default_expansion_side(self):
        link = dict(id='test', lanes=3, shape=[[126.9, 37.5], [126.9, 37.501]])
        default, _ = lane_counts([link], [dict(edge='test', lanes=4)])
        right, _ = lane_counts([link], [dict(edge='test', lanes=4, side='right')])
        self.assertEqual(default, right)

    @unittest.skipUnless(NET.exists(), 'SUMO network required')
    def test_yeouigyo_four_lanes_have_requested_destinations(self):
        root = ET.parse(NET).getroot()
        edge = root.find("edge[@id='1180019202']")
        self.assertEqual(len(edge.findall('lane')), 4)
        actual = {i: set() for i in range(4)}
        for c in root.findall('connection'):
            if c.get('from') == '1180019202':
                actual[int(c.get('fromLane'))].add(c.get('to'))
        self.assertEqual(actual, {0: {'1180017901', '1180019102'},
                                  1: {'1180017901'},
                                  2: {'1180017901'},
                                  3: {'1190006903'}})

    @unittest.skipUnless(NET.exists(), 'SUMO network required')
    def test_three_seoul_lanes_continue_to_the_bridge(self):
        root = ET.parse(NET).getroot()
        straight_lanes = {int(c.get('fromLane')) for c in root.findall('connection')
                          if c.get('from') == '1180017901' and c.get('to') == '1180017902'}
        self.assertEqual(straight_lanes, {0, 1, 2})
        self.assertTrue(any(c.get('from') == '1180017901' and c.get('fromLane') == '0'
                            and c.get('to') == '1180071100' for c in root.findall('connection')))
        corridor = ['1180017901', '1180017902', '1180020100', '1180022101', '1180022103']
        for eid in corridor:
            self.assertEqual(len(root.find(f"edge[@id='{eid}']").findall('lane')), 3)
        for a, b in zip(corridor, corridor[1:] + ['1180022103_pocket']):
            connections = {(int(c.get('fromLane')), int(c.get('toLane')))
                           for c in root.findall('connection')
                           if c.get('from') == a and c.get('to') == b}
            self.assertTrue({(0, 0), (1, 1), (2, 2)} <= connections, (a, b))


@unittest.skipUnless(NET.exists() and shutil.which(sumo_bin('netconvert')), 'SUMO network required')
class DaebangExitTests(unittest.TestCase):
    def test_four_separate_through_lanes_and_other_signals_preserved_on_rebuild(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / 'test.net.xml'
            shutil.copyfile(NET, path)
            previous = ET.parse(path).getroot()
            original_signals = {t.get('id'): canonical(t) for t in previous.findall('tlLogic')
                                if t.get('id') != DAEBANG_EXIT}
            for _ in range(2):
                self.assertTrue(correct_daebang_exit(path))
                root = ET.parse(path).getroot()
                self.assertEqual(root.find(f"junction[@id='{DAEBANG_EXIT}']").get('type'), 'priority')
                self.assertEqual({t.get('id'): canonical(t) for t in root.findall('tlLogic')},
                                 original_signals)
                actual = [c for c in root.findall('connection') if c.get('from') in DAEBANG_THROUGH]
                self.assertEqual(len(actual), 4)
                for c in actual:
                    self.assertEqual(c.get('to'), DAEBANG_OUT)
                    self.assertEqual(c.get('dir'), 's')
                    self.assertIsNone(c.get('tl'))
                    target_lane = DAEBANG_THROUGH[c.get('from')][int(c.get('fromLane'))]
                    self.assertEqual(int(c.get('toLane')), target_lane)


if __name__ == '__main__':
    unittest.main()
