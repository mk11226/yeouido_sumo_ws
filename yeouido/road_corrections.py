"""User-confirmed road topology corrections after netconvert generation."""
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

from yeouido.config import sumo_bin

DAEBANG_EXIT = 'cluster_1190018001_1190018002_1190018003_1190018004'
DAEBANG_THROUGH = {
    '1180012812': (0, 1),  # right-hand surface lanes
    '1180062700': (2, 3),  # left-hand underpass lanes
}
DAEBANG_OUT = '1180012808'


def correct_daebang_exit(path):
    """Two tunnel lanes + two surface lanes continue as four unsignaled lanes.

    No left/U-turns, lane drop or crossing lane connections at this exit.
    Recompute internal geometry and priority rules with netconvert, then copy
    only this junction back so every other junction and signal stays intact.
    """
    path = Path(path)
    tree = ET.parse(path)
    root = tree.getroot()
    node = root.find(f"junction[@id='{DAEBANG_EXIT}']")
    if node is None:
        return False
    incoming = {c.get('from') for c in root.findall('connection')
                if c.get('tl') == DAEBANG_EXIT}
    # Idempotent rebuilds also work after the signal has already been removed.
    incoming.update(lane.rsplit('_', 1)[0] for lane in node.get('incLanes').split())
    for eid in DAEBANG_THROUGH:
        edge = root.find(f"edge[@id='{eid}']")
        if edge is None or len(edge.findall('lane')) != 2:
            raise ValueError(f'대방지하차도 출구 2차로 진입로 누락: {eid}')
    out = root.find(f"edge[@id='{DAEBANG_OUT}']")
    if out is None or len(out.findall('lane')) != 4:
        raise ValueError('대방지하차도 출구의 4차로 진출로가 없습니다.')
    with tempfile.TemporaryDirectory(prefix='daebang-exit-') as temp:
        temp = Path(temp)
        nodes = ET.Element('nodes')
        ET.SubElement(nodes, 'node', id=DAEBANG_EXIT, type='priority')
        ET.ElementTree(nodes).write(temp / 'exit.nod.xml')
        connections = ET.Element('connections')
        # Remove all generated turns and fan-out connections on these approaches.
        for eid, target_lanes in DAEBANG_THROUGH.items():
            for target in sorted({c.get('to') for c in root.findall('connection')
                                  if c.get('from') == eid}):
                ET.SubElement(connections, 'delete', {'from': eid, 'to': target})
            for lane, target_lane in enumerate(target_lanes):
                ET.SubElement(connections, 'connection', {
                    'from': eid, 'to': DAEBANG_OUT,
                    'fromLane': str(lane), 'toLane': str(target_lane)})
        ET.ElementTree(connections).write(temp / 'exit.con.xml')
        result = subprocess.run([
            sumo_bin('netconvert'), '-s', str(path),
            '-n', str(temp / 'exit.nod.xml'), '-x', str(temp / 'exit.con.xml'),
            '-o', str(temp / 'exit.net.xml'), '--no-warnings', 'true'],
            capture_output=True, text=True)
        if result.returncode:
            raise RuntimeError(f'대방지하차도 출구 재생성 실패: {result.stderr}')
        generated = ET.parse(temp / 'exit.net.xml').getroot()

    def local(element):
        eid = element.get('id', '')
        if element.tag == 'tlLogic':
            return eid == DAEBANG_EXIT
        if element.tag in ('edge', 'junction'):
            return eid == DAEBANG_EXIT or eid.startswith(':' + DAEBANG_EXIT + '_')
        if element.tag == 'connection':
            return (element.get('from') in incoming
                    or element.get('from', '').startswith(':' + DAEBANG_EXIT + '_'))
        return False

    for tag in ('edge', 'tlLogic', 'junction', 'connection'):
        removed = [e for e in root if e.tag == tag and local(e)]
        additions = [e for e in generated if e.tag == tag and local(e)]
        if not removed:
            continue
        index = list(root).index(removed[0])
        for element in removed:
            root.remove(element)
        for element in additions:
            root.insert(index, element)
            index += 1
    if root.find(f"tlLogic[@id='{DAEBANG_EXIT}']") is not None:
        raise ValueError('대방지하차도 출구에 신호가 남았습니다.')
    actual = {(c.get('from'), int(c.get('fromLane')), c.get('to'), int(c.get('toLane')))
              for c in root.findall('connection') if c.get('from') in DAEBANG_THROUGH}
    expected = {(eid, lane, DAEBANG_OUT, target) for eid, targets in DAEBANG_THROUGH.items()
                for lane, target in enumerate(targets)}
    if actual != expected:
        raise ValueError('대방지하차도 출구 4차로 직진 연결이 일치하지 않습니다.')
    tree.write(path, encoding='utf-8', xml_declaration=True)
    return True
