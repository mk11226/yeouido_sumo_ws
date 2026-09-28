"""Apply the requested southbound Bongdam-Gwacheon 4/6-lane correction.

Lane counts are a user-specified scenario correction, not a new field survey.
Use netconvert's plain-XML patch interface so lane geometry, junctions and
connections are rebuilt together. The archived OSM input remains untouched.
"""
from datetime import datetime, timezone
import gzip
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

from Road_Environment.build.build_skku_commute import OUT, binary, sha256, write_json, write_xml


def apply(net, path, scenario=OUT):
    import sumolib
    from sumolib.geomhelper import move2side
    scenario=Path(scenario)
    network=scenario/'skku_commute.net.xml'
    # Named mainline edges on the southbound commute, including merge sections.
    mainline=[e for e in path if e.getName()=='봉담과천로' and e.getType()=='highway.trunk']
    toll_id='507449308'
    if toll_id not in {e.getID() for e in mainline} or len(mainline)<20:
        raise ValueError('The expected southbound mainline / toll edge is missing')
    desired={e.getID():6 if e.getID()==toll_id else 4 for e in mainline}
    report_path=scenario/'lanes.manifest.json'
    previous=json.loads(report_path.read_text()) if report_path.exists() else None
    if all(e.getLaneNumber()==desired[e.getID()] for e in mainline):
        if not report_path.exists():
            raise ValueError('Lane counts match but correction provenance is missing')
        if previous.get('profile_version')==3:
            return False
    before=sha256(network)
    source=scenario/'lanes_source';source.mkdir(exist_ok=True)
    archive=source/f'{before}.net.xml.gz'
    if not archive.exists():
        archive.write_bytes(gzip.compress(network.read_bytes(),mtime=0))
    edges=ET.Element('edges')
    for e in mainline:
        # Keep the median-side boundary fixed; widen towards the outer shoulder.
        shift=(desired[e.getID()]-e.getLaneNumber())*e.getLane(0).getWidth()/2
        shape=move2side(e.getRawShape(),shift)
        ET.SubElement(edges,'edge',id=e.getID(),numLanes=str(desired[e.getID()]),
                      spreadType='center',shape=' '.join(f'{x:.2f},{y:.2f}' for x,y in shape))
    write_xml(scenario/'lanes.edg.xml',edges)
    affected={e.getID() for e in mainline}
    for e in mainline:
        affected.update(i.getID() for i in e.getIncoming() if not i.getFunction())
    connections=ET.Element('connections')
    for edge_id in sorted(affected):
        ET.SubElement(connections,'connection',attrib={'from':edge_id,'reset':'true'})
    ti=next(i for i,e in enumerate(path) if e.getID()==toll_id)
    # Expand on the shoulder side, and give all six booth lanes an exit.
    # Default guessing leaves the two extra lanes without an onward connection.
    for a,b,pairs in [(path[ti-1],path[ti],[(0,0),(0,1),(0,2),(1,3),(2,4),(3,5)]),
                      (path[ti],path[ti+1],[(0,0),(1,0),(2,0),(3,1),(4,2),(5,3)])]:
        for target in a.getOutgoing():
            if target.getFunction():
                continue
            use=pairs if target.getID()==b.getID() else [
                (c.getFromLane().getIndex(),c.getToLane().getIndex()) for c in a.getConnections(target)]
            for source_lane,target_lane in use:
                ET.SubElement(connections,'connection',attrib={'from':a.getID(),'to':target.getID(),
                              'fromLane':str(source_lane),'toLane':str(target_lane)})
    write_xml(scenario/'lanes.con.xml',connections)
    nodes=ET.Element('nodes')
    ET.SubElement(nodes,'node',id=path[ti].getToNode().getID(),type='zipper')
    write_xml(scenario/'lanes.nod.xml',nodes)
    temporary=scenario/'lanes.updated.net.xml'
    command=[binary('netconvert'),'--sumo-net-file',str(network),
             '--edge-files',str(scenario/'lanes.edg.xml'),
             '--connection-files',str(scenario/'lanes.con.xml'),
             '--node-files',str(scenario/'lanes.nod.xml'),
             '--output-file',str(temporary),'--offset.disable-normalization','true',
             '--no-turnarounds','true','--tls.uncontrolled-within','true']
    result=subprocess.run(command,capture_output=True,text=True)
    (scenario/'lanes.build.log').write_text(result.stdout+result.stderr)
    if result.returncode:
        raise RuntimeError('Lane conversion failed; see lanes.build.log')
    rebuilt=sumolib.net.readNet(str(temporary),withInternal=True)
    if rebuilt.getLocationOffset()!=net.getLocationOffset():
        raise ValueError('Network offset changed; background projection would be invalid')
    rows=[]
    for e in mainline:
        updated=rebuilt.getEdge(e.getID())
        assert updated.getLaneNumber()==desired[e.getID()]
        rows.append(dict(id=e.getID(),before=e.getLaneNumber(),after=updated.getLaneNumber(),
                         length_m=updated.getLength(),outgoing_lanes=[len(l.getOutgoing()) for l in updated.getLanes()]))
    for a,b in zip(path,path[1:]):
        if not rebuilt.getEdge(a.getID()).getConnections(rebuilt.getEdge(b.getID())):
            raise ValueError(f'Route disconnected at {a.getID()} -> {b.getID()}')
        if a.getID() in desired and b.getID() in desired:
            pairs=[(c.getFromLane().getIndex(),c.getToLane().getIndex())
                   for c in rebuilt.getEdge(a.getID()).getConnections(rebuilt.getEdge(b.getID()))]
            if ({i for i,j in pairs}!=set(range(desired[a.getID()]))
                    or {j for i,j in pairs}!=set(range(desired[b.getID()]))):
                raise ValueError(f'A mainline lane has no onward/inbound connection: {a.getID()}')
    # Lane edits are remote from city signals. Reject unintended timing changes.
    old_tls={e.get('id'):ET.tostring(e) for e in ET.parse(network).getroot().findall('tlLogic')}
    new_tls={e.get('id'):ET.tostring(e) for e in ET.parse(temporary).getroot().findall('tlLogic')}
    removed_tls=set(old_tls)-set(new_tls)
    active_tls={c.get('tl') for c in ET.parse(network).getroot().findall('connection') if c.get('tl')}
    if (set(new_tls)-set(old_tls) or removed_tls & active_tls
            or any(old_tls[k]!=new_tls[k] for k in new_tls)):
        raise ValueError('Lane patch unexpectedly changed traffic-light programs')
    temporary.replace(network)
    write_json(report_path,dict(created_utc=datetime.now(timezone.utc).isoformat(),
        profile_version=3,previous_correction=previous,merge_type='zipper',
        basis='User-requested scenario correction: southbound mainline 4 lanes, Uiwang toll approach 6 lanes',
        field_survey_verified=False,toll_edge=toll_id,edges=rows,command=command,
        network_before_sha256=before,network_after_sha256=sha256(network),
        archived_network=str(archive.relative_to(scenario)),
        offset_unchanged=True,active_signal_programs_unchanged=True,
        removed_unused_signal_programs=sorted(removed_tls),
        geometry='Existing median boundary retained; added lanes extend towards outer shoulder'))
    return True
