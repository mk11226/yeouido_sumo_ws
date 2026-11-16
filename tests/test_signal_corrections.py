import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from yeouido.signal_corrections import protect_yeoui2_left


class ProtectedLeftTests(unittest.TestCase):
    def fixture(self, folder):
        path = Path(folder) / 'net.xml'
        root = ET.Element('net')
        for edge, direction, index in [('1180031402', 's', 0),
                                        ('1180030100', 's', 1),
                                        ('1180030100', 'l', 2),
                                        ('1180030100', 'l', 3),
                                        ('1180030100', 'r', 4)]:
            ET.SubElement(root, 'connection', {'from': edge, 'dir': direction,
                                              'tl': 'south', 'linkIndex': str(index)})
        tl = ET.SubElement(root, 'tlLogic', id='south', type='actuated', programID='0')
        for duration, state in [(20, 'GGggG'), (3, 'yGggG'), (8, 'rGGGG'),
                                (3, 'ryyyy'), (3, 'rrrrr')]:
            ET.SubElement(tl, 'phase', duration=str(duration), state=state)
        ET.ElementTree(root).write(path)
        return path

    def test_left_waits_through_opposing_green_yellow_and_clearance(self):
        with tempfile.TemporaryDirectory() as folder:
            path = self.fixture(folder)
            protect_yeoui2_left(path)
            phases = ET.parse(path).getroot().findall('tlLogic/phase')
            self.assertEqual([p.get('state') for p in phases],
                             ['GGrrG', 'yGrrG', 'rGrrG', 'rGGGG', 'ryyyy', 'rrrrr'])
            self.assertEqual(float(phases[2].get('duration')), 3)
            self.assertTrue(all(p.get('state')[1] == 'G' for p in phases[:4]))

    def test_reapplying_does_not_add_more_clearance_phases(self):
        with tempfile.TemporaryDirectory() as folder:
            path = self.fixture(folder)
            protect_yeoui2_left(path)
            before = path.read_bytes()
            protect_yeoui2_left(path)
            self.assertEqual(before, path.read_bytes())

    def test_different_network_is_unchanged(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'net.xml'
            path.write_text('<net/>')
            self.assertFalse(protect_yeoui2_left(path))
            self.assertEqual(path.read_text(), '<net/>')


if __name__ == '__main__':
    unittest.main()
