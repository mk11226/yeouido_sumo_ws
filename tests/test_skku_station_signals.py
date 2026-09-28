"""Safety intervals and cycle budgets of the station signal overrides."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


class StationSignalTests(unittest.TestCase):
    def test_states_clearance_and_cycle_preserved(self):
        base = Path(__file__).resolve().parents[1] / 'env/skku_commute'
        original = ET.parse(base / 'skku_commute.net.xml').getroot()
        overrides = ET.parse(base / 'station_signals.add.xml').getroot()
        self.assertEqual(len(overrides), 5)
        for program in overrides:
            before = original.find(f"tlLogic[@id='{program.get('id')}'][@programID='0']").findall('phase')
            after = program.findall('phase')
            self.assertEqual(len(before), len(after))
            self.assertEqual(sum(float(p.get('duration')) for p in before),
                             sum(float(p.get('duration')) for p in after))
            for old, new in zip(before, after):
                self.assertEqual(old.get('state'), new.get('state'))
                half_steps = float(new.get('duration')) / .5
                self.assertAlmostEqual(half_steps, round(half_steps))
                if 'y' in old.get('state') or set(old.get('state')) == {'r'}:
                    self.assertEqual(old.attrib, new.attrib)
                else:
                    self.assertGreaterEqual(float(new.get('duration')), 5)

    def test_configuration_loads_overrides(self):
        base = Path(__file__).resolve().parents[1] / 'env/skku_commute'
        cfg = ET.parse(base / 'skku_commute.sumocfg')
        self.assertIn('station_signals.add.xml', cfg.find('input/additional-files').get('value').split(','))
