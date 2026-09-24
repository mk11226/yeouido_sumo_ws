import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import sumolib

root = Path('/home/mk/bmil_ws')
scenario = root / 'env/skku_commute'
network = scenario / 'skku_commute.net.xml'
net = sumolib.net.readNet(str(network), withInternal=True, withPrograms=True)
route = json.loads((scenario / 'route.json').read_text())['edge_ids']
rows, missing, tls_ids = [], [], set()
for a, b in zip(route, route[1:]):
    connections = [c for c in net.getEdge(a).getConnections(net.getEdge(b)) if c.allows('passenger')]
    if not connections:
        missing.append([a, b])
    for connection in connections:
        tls_id = connection.getTLSID()
        if not tls_id:
            continue
        tls_ids.add(tls_id)
        link = connection.getTLLinkIndex()
        programs = net.getTLS(tls_id).getPrograms()
        for program_id, program in programs.items():
            phases = program.getPhases()
            states = [p.state[link] for p in phases]
            rows.append(dict(from_edge=a, to_edge=b, from_lane=connection.getFromLane().getID(),
                to_lane=connection.getToLane().getID(), tls_id=tls_id, link_index=link,
                program_id=program_id, program_type=program.getType(),
                cycle_s=sum(p.duration for p in phases), states=states,
                has_green=any(s in 'gG' for s in states), has_red=any(s in 'rR' for s in states)))
bad = [r for r in rows if not (r['has_green'] and r['has_red'])]
manifest = json.loads((scenario / 'manifest.json').read_text())
promotion = json.loads((scenario / 'signals_source/roads_with_signal_crossings.osm.promotion.json').read_text())
insertions = []
for item in promotion['reviewed_midblock_insertions']:
    node_id = item['derived_node_id']
    connections = [r for r in rows if r['tls_id'] == node_id]
    insertions.append(dict(id=item['id'], node_id=node_id,
                           controlled_route_connection_count=len(connections)))
report = dict(network_sha256=hashlib.sha256(network.read_bytes()).hexdigest(),
    route_controller_count=len(tls_ids), controlled_connection_program_count=len(rows),
    missing_route_connections=missing, signals_without_green_or_red=bad,
    inserted_midblock_signals=insertions,
    building_polygon_count=len(ET.parse(scenario / 'buildings.poly.xml').getroot().findall('poly')),
    cycles_s=sorted(set(r['cycle_s'] for r in rows)),
    passed=not missing and not bad and len(tls_ids)==manifest['route']['signal_controller_count']
        and all(i['controlled_route_connection_count'] > 0 for i in insertions),
    limitation='Checks generated SUMO connectivity and programs, not current real-world position or phase accuracy.',
    connections=rows)
output = root / 'results/skku_commute/signal_program_audit.json'
output.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
print(json.dumps({k: v for k,v in report.items() if k != 'connections'}, ensure_ascii=False, indent=2))
assert report['passed']
