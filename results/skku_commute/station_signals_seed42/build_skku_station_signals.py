"""Extend existing straight-through greens near Gubanpo and SKKU stations."""
import copy
import json
from pathlib import Path
import xml.etree.ElementTree as ET

OUT = Path(__file__).resolve().parent / 'env/skku_commute'
# Approach edges on the Seoul -> Suwon route; retain every existing signal state.
APPROACHES = {
    '1252436244#4': '구반포역 서쪽 신반포로',
    '-1117538960': '구반포역 서쪽 동작대로 접속부',
    '775456751#6': '성균관대역 북쪽 서부로',
    '555231771#2': '성균관대역 앞 서부로 첫 직진',
    '488392556': '성균관대역 앞 서부로 다음 직진',
}


def build():
    from build_skku_commute import write_xml, write_json, sha256
    root = ET.parse(OUT / 'skku_commute.net.xml').getroot()
    route = json.loads((OUT / 'route.json').read_text())['edge_ids']
    targets = {}
    for edge, label in APPROACHES.items():
        following = route[route.index(edge) + 1]
        connections = [c for c in root.findall('connection')
                       if c.get('from') == edge and c.get('to') == following
                       and c.get('dir') == 's' and c.get('tl')]
        if not connections:
            raise ValueError(f'No controlled straight movement: {edge}')
        for c in connections:
            targets.setdefault(c.get('tl'), {})[int(c.get('linkIndex'))] = label
    additional = ET.Element('additional')
    changes = []
    for tid, links in targets.items():
        original = root.find(f"tlLogic[@id='{tid}'][@programID='0']")
        if original is None or original.get('type') != 'static':
            raise ValueError(f'Expected baseline static program: {tid}')
        program = copy.deepcopy(original)
        program.set('programID', 'station_straight_priority')
        edits = []
        for i, phase in enumerate(program.findall('phase')):
            labels = sorted({label for link, label in links.items() if phase.get('state')[link] == 'G'})
            if not labels:
                continue
            old = float(phase.get('duration'))
            phase.set('duration', f'{old + 20:g}')
            edits.append(dict(phase_index=i, approaches=labels, before_s=old, after_s=old+20))
        if not edits:
            raise ValueError(f'No protected straight green: {tid}')
        additional.append(program)
        changes.append(dict(tls_id=tid, phases=edits,
                            cycle_before_s=sum(float(p.get('duration')) for p in original.findall('phase')),
                            cycle_after_s=sum(float(p.get('duration')) for p in program.findall('phase'))))
    write_xml(OUT / 'station_signals.add.xml', additional)
    report = dict(synthetic=True, program_id='station_straight_priority',
                  network_sha256=sha256(OUT / 'skku_commute.net.xml'),
                  unchanged='signal states, other greens, yellow and all-red durations', changes=changes)
    write_json(OUT / 'station_signals.manifest.json', report)
    return report


if __name__ == '__main__':
    print(json.dumps(build(), ensure_ascii=False, indent=2))
