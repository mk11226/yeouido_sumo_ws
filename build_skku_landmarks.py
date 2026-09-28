"""Build river polygons and Korean landmark badges from archived OSM data.

No live download on rebuild. Source queries and retrieval dates are archived in
landmarks_source. Labels use PNG textures because SUMO's built-in Roboto font
cannot render Korean glyphs. Pillow and a Korean font are required for rebuilds.
"""
from pathlib import Path
import json
import math
import subprocess
import xml.etree.ElementTree as ET
from PIL import Image, ImageDraw, ImageFont
from build_skku_commute import OUT, write_json, write_xml, sha256
from build_skku_buildings import assemble_rings

# Only major Han River crossings; omit approach viaducts and small bridges.
MAJOR_BRIDGES = {'반포대교', '잠수교', '동작대교', '한남대교', '한강대교', '동호대교'}


def clipped(points, bounds):
    """Sutherland-Hodgman clipping to the map's geographic rectangle."""
    points = list(points[:-1])
    for axis, limit, sign in [(0,bounds[0],1),(0,bounds[2],-1),(1,bounds[1],1),(1,bounds[3],-1)]:
        result = []
        if not points:
            return []
        previous = points[-1]
        for current in points:
            pi = sign*(previous[axis]-limit) >= 0
            ci = sign*(current[axis]-limit) >= 0
            if pi != ci:
                t = (limit-previous[axis])/(current[axis]-previous[axis])
                result.append(tuple(previous[k]+t*(current[k]-previous[k]) for k in (0,1)))
            if ci:
                result.append(current)
            previous = current
        points = result
    return points + points[:1] if len(set(points)) >= 3 else []


def build(net, path):
    from sumolib.geomhelper import polygonOffsetAndDistanceToPoint
    source = OUT / 'landmarks_source'
    features = json.loads((source/'features.json').read_text())['elements']
    water = json.loads((source/'water.json').read_text())['elements']
    root = ET.Element('additional')
    bounds = (126.94,37.28,127.04,37.60)
    water_count = 0
    for relation in water:
        if relation.get('tags', {}).get('water') != 'river':
            continue
        nodes, ways = {}, {}
        for member in relation.get('members', []):
            if member['type'] != 'way' or not member.get('geometry'):
                continue
            refs = [(p['lon'],p['lat']) for p in member['geometry']]
            nodes.update({p:p for p in refs})
            ways[member['ref']] = {'nodes':refs}
        for role, color, layer in [('outer','104,174,222','-10'),('inner','245,247,249','-9')]:
            ids = [m['ref'] for m in relation['members'] if m['type']=='way' and m.get('role')==role]
            rings, failures = assemble_rings(ids, ways, nodes)
            if failures:
                raise ValueError(f'Incomplete water geometry: {failures}')
            for ring, _ in rings:
                points = clipped(ring, bounds)
                if not points:
                    continue
                xy = [net.convertLonLat2XY(*p) for p in points]
                ET.SubElement(root,'poly',id=f'water_{relation["id"]}_{role}_{water_count}',
                              type='water' if role=='outer' else 'island', color=color,fill='true',layer=layer,
                              shape=' '.join(f'{x:.2f},{y:.2f}' for x,y in xy))
                water_count += 1
    if not water_count:
        raise ValueError('No river polygon found')
    roads = ET.parse(OUT/'source/merged.osm.xml').getroot()
    road_nodes = {e.get('id'):(float(e.get('lon')),float(e.get('lat'))) for e in roads.findall('node')}
    candidates = []
    for e in json.loads((source/'bridges.json').read_text())['elements']:
        t=e.get('tags',{});name=t.get('name:ko') or t.get('name')
        center=e.get('center',e)
        if name in MAJOR_BRIDGES and 'lon' in center:
            candidates.append((name,'bridge',(center['lon'],center['lat']),f'{e["type"]}/{e["id"]}'))
    for e in features:
        t=e.get('tags',{}); name=t.get('name:ko') or t.get('name')
        if e['type']=='node' and t.get('railway')=='station' and name:
            if t.get('station') not in (None, 'subway'):
                continue
            candidates.append((name if name.endswith('역') else name+'역','station',(e['lon'],e['lat']),f'node/{e["id"]}'))
    bridge_groups = {}
    for e in roads:
        t={a.get('k'):a.get('v') for a in e.findall('tag')}
        name=t.get('name:ko') or t.get('name')
        if e.tag=='node' and t.get('barrier')=='toll_booth' and name:
            label = name if name.endswith(('TG','톨게이트')) else name+' 톨게이트'
            candidates.append((label,'toll',road_nodes[e.get('id')],f'node/{e.get("id")}'))
        if e.tag=='way' and t.get('bridge') not in (None,'no') and name:
            coords=[road_nodes[n.get('ref')] for n in e.findall('nd')]
            if any(37.505 < lat < 37.535 for lon,lat in coords):
                bridge_groups.setdefault(name,[]).extend(coords)
    for name, coords in bridge_groups.items():
        if name not in MAJOR_BRIDGES:
            continue
        candidates.append((name,'bridge',tuple(sum(p[k] for p in coords)/len(coords) for k in (0,1)),'archived OSM bridge ways'))
    candidates.append(('한강','river',(126.994,37.518),'OSM river polygon'))
    font_path=subprocess.check_output(['fc-match',':lang=ko','-f','%{file}'],text=True)
    font=ImageFont.truetype(font_path,40)
    images=OUT/'landmark_labels';images.mkdir(exist_ok=True)
    records=[];seen=[]
    colors={'station':'#196b49','toll':'#a54a16','bridge':'#245a9d','river':'#245a9d'}
    for name,kind,coord,origin in candidates:
        x,y=net.convertLonLat2XY(*coord)
        distance=min(polygonOffsetAndDistanceToPoint((x,y),e.getShape())[1] for e in path)
        if distance>1200:
            continue
        if any(n==name and (kind=='bridge' or math.hypot(x-a,y-b)<750) for n,a,b in seen):
            continue
        seen.append((name,x,y))
        index=len(records);pid=f'landmark_{index:03d}'
        box=font.getbbox(name);w=box[2]-box[0]+32;h=68
        image=Image.new('RGBA',(w,h));draw=ImageDraw.Draw(image)
        draw.rounded_rectangle((1,1,w-2,h-2),radius=12,fill=(255,255,255,242),outline=colors[kind],width=3)
        draw.text((16,8-box[1]),name,font=font,fill=colors[kind])
        image.save(images/f'{pid}.png')
        ET.SubElement(root,'poi',id=pid,name=name,type=kind,x=f'{x:.2f}',y=f'{y:.2f}',color=colors[kind],layer='15',width='5',height='5')
        label_x=x;label_y=y+22
        if name=='의왕 톨게이트':
            label_x=x-90;label_y=y+35
            ET.SubElement(root,'poly',id=pid+'_leader',type='label_leader',fill='false',
                          color=colors[kind],layer='14',lineWidth='0.5',
                          shape=f'{x:.2f},{y:.2f} {label_x+w*.17:.2f},{label_y:.2f}')
        # Banpo and Jamsu are stacked bridges; do not cover one name with the other.
        if name=='잠수교':
            label_y=y-30
        ET.SubElement(root,'poi',id=pid+'_label',name=name,type='label',x=f'{label_x:.2f}',y=f'{label_y:.2f}',color='255,255,255',layer='20',
                      imgFile=f'landmark_labels/{pid}.png',width=f'{w*.34:.2f}',height=f'{h*.34:.2f}')
        records.append(dict(id=pid,name=name,kind=kind,lon_lat=coord,x=x,y=y,distance_to_route_m=distance,source=origin))
    used_images={r['id']+'.png' for r in records}
    for image_path in images.glob('landmark_*.png'):
        if image_path.name not in used_images:
            image_path.unlink()
    write_xml(OUT/'landmarks.add.xml',root)
    report=dict(water_polygons=water_count,landmarks=records,source_sha256={p.name:sha256(p) for p in source.glob('*.json')},
                font=font_path,attribution='© OpenStreetMap contributors, ODbL 1.0',
                scope='OSM named stations/toll booths within 1200m of route; major Han River crossings only',
                label_images_sha256={p.name:sha256(p) for p in sorted(images.glob('*.png'))})
    write_json(OUT/'landmarks.manifest.json',report)
    return report


if __name__=='__main__':
    from run_skku_commute import prepare_runtime
    prepare_runtime()
    import sumolib
    net=sumolib.net.readNet(str(OUT/'skku_commute.net.xml'))
    route=json.loads((OUT/'route.json').read_text())
    print(json.dumps(build(net,[net.getEdge(e) for e in route['edge_ids']]),ensure_ascii=False,indent=2))
