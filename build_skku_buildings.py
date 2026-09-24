"""Import real OSM building footprints beside the SKKU commute network.

This only writes building outputs; it does not modify the scenario configuration,
road network, traffic, or the main scenario manifest. OSM multipolygon members
are assembled into exact closed rings before polyconvert projection. Courtyards
are white masks below the road layer, not native polygon holes or 3D geometry.

    .venv-hyehwa/bin/python build_skku_buildings.py --overview
"""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parent
BUILDING_LAYER, COURTYARD_LAYER = -4.0, -3.9


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def is_building(tags):
    return tags.get("building", "").lower() not in ("", "no", "false", "0")


def read_osm(path):
    nodes, ways, relations = {}, {}, {}
    for _, element in ET.iterparse(path, events=("end",)):
        if element.tag not in ("node", "way", "relation"):
            continue
        osm_id = int(element.attrib["id"])
        if element.tag == "node":
            nodes[osm_id] = (float(element.attrib["lon"]), float(element.attrib["lat"]))
        else:
            tags = {tag.attrib["k"]: tag.attrib["v"] for tag in element.findall("tag")}
            if element.tag == "way":
                ways[osm_id] = {"nodes": [int(nd.attrib["ref"]) for nd in element.findall("nd")], "tags": tags}
            else:
                relations[osm_id] = {"members": [dict(member.attrib) for member in element.findall("member")], "tags": tags}
        element.clear()
    return nodes, ways, relations


def polygon_area(points):
    """Absolute shoelace area, translated first to avoid coordinate cancellation."""
    if len(points) < 4:
        return 0.0
    x0, y0 = points[0]
    return abs(sum((a[0] - x0) * (b[1] - y0) - (b[0] - x0) * (a[1] - y0)
                   for a, b in zip(points, points[1:]))) / 2


def canonical_ring(refs):
    core = list(refs[:-1])
    smallest = min(range(len(core)), key=lambda i: core[i])
    forward = core[smallest:] + core[:smallest]
    reverse = [forward[0], *reversed(forward[1:])]
    result = min(forward, reverse)
    return result + [result[0]]


def checked_ring(refs, nodes):
    refs = [ref for i, ref in enumerate(refs) if not i or ref != refs[i - 1]]
    if any(ref not in nodes for ref in refs):
        return None, "missing_node"
    if not refs or len(refs) < 4:
        return None, "too_few_vertices"
    if refs[0] != refs[-1] and nodes[refs[0]] == nodes[refs[-1]]:
        refs[-1] = refs[0]  # Same actual coordinate, without inventing a segment.
    if refs[0] != refs[-1]:
        return None, "open_ring"
    points = [nodes[ref] for ref in refs]
    if not all(math.isfinite(v) for point in points for v in point):
        return None, "nonfinite_coordinate"
    if len(set(points[:-1])) < 3 or polygon_area(points) <= 1e-15:
        return None, "zero_area"
    return canonical_ring(refs), None


def assemble_rings(member_ids, ways, nodes):
    """Assemble only unambiguous endpoint-connected cycles; never bridge gaps."""
    rings, failures, open_ways = [], [], {}
    for way_id in sorted(set(member_ids)):
        way = ways.get(way_id)
        if way is None:
            failures.append({"member_ways": [way_id], "reason": "missing_way"})
            continue
        refs = way["nodes"]
        if refs and refs[0] in nodes and refs[-1] in nodes and nodes[refs[0]] == nodes[refs[-1]]:
            ring, reason = checked_ring(refs, nodes)
            if ring:
                rings.append((ring, [way_id]))
            else:
                failures.append({"member_ways": [way_id], "reason": reason})
        elif len(refs) >= 2:
            open_ways[way_id] = list(refs)
        else:
            failures.append({"member_ways": [way_id], "reason": "too_few_vertices"})
    endpoints = defaultdict(set)
    for way_id, refs in open_ways.items():
        endpoints[refs[0]].add(way_id)
        endpoints[refs[-1]].add(way_id)
    unused = set(open_ways)
    while unused:
        first = min(unused)
        component, stack = set(), [first]
        while stack:
            way_id = stack.pop()
            if way_id in component:
                continue
            component.add(way_id)
            for endpoint in (open_ways[way_id][0], open_ways[way_id][-1]):
                stack.extend(endpoints[endpoint] - component)
        unused.difference_update(component)
        degrees = Counter(endpoint for way_id in component
                          for endpoint in (open_ways[way_id][0], open_ways[way_id][-1]))
        if any(degree != 2 for degree in degrees.values()):
            failures.append({"member_ways": sorted(component), "reason": "open_or_branching_member_component"})
            continue
        refs = list(open_ways[first])
        remaining = component - {first}
        while remaining:
            candidates = endpoints[refs[-1]] & remaining
            if len(candidates) != 1:
                break
            way_id = next(iter(candidates))
            part = open_ways[way_id]
            refs.extend(part[1:] if part[0] == refs[-1] else list(reversed(part))[1:])
            remaining.remove(way_id)
        ring, reason = checked_ring(refs, nodes) if not remaining else (None, "ambiguous_ring")
        if ring:
            rings.append((ring, sorted(component)))
        else:
            failures.append({"member_ways": sorted(component), "reason": reason})
    rings.sort(key=lambda item: tuple(item[0]))
    return rings, failures


def inside_ring(point, polygon):
    x, y = point
    inside = False
    for (ax, ay), (bx, by) in zip(polygon, polygon[1:]):
        if (ay > y) != (by > y) and x < (bx - ax) * (y - ay) / (by - ay) + ax:
            inside = not inside
    return inside


def relation_shape_tags(relation_tags, member_ids, ways):
    """Preserve relation and member metadata without confusing conflicting tags."""
    tags = dict(relation_tags)
    for member_id in member_ids:
        member_tags = ways[member_id]["tags"]
        for key, value in member_tags.items():
            tags[f"source:osm:member_way:{member_id}:{key}"] = value
            # One member is an independently mapped closed footprint. Its
            # missing relation attributes (e.g. campus building name) are known,
            # whereas attributes of separate open boundary pieces may conflict.
            if len(member_ids) == 1:
                tags.setdefault(key, value)
    return tags


def make_footprints(nodes, ways, relations):
    shapes, skipped, represented_members = [], [], set()
    for relation_id, relation in sorted(relations.items()):
        tags = relation["tags"]
        if not is_building(tags):
            continue
        feature = f"relation/{relation_id}"
        members = relation["members"]
        unsupported = [member for member in members if member.get("type") != "way"]
        if unsupported:
            skipped.append({"feature": feature, "reason": "non_way_relation_members", "members": unsupported})
        outer_ids = [int(m["ref"]) for m in members if m.get("type") == "way" and m.get("role", "") in ("", "outer")]
        inner_ids = [int(m["ref"]) for m in members if m.get("type") == "way" and m.get("role") == "inner"]
        outers, failures = assemble_rings(outer_ids, ways, nodes)
        skipped.extend({"feature": feature, "role": "outer", **failure} for failure in failures)
        inner_rings, failures = assemble_rings(inner_ids, ways, nodes)
        skipped.extend({"feature": feature, "role": "inner", **failure} for failure in failures)
        for index, (refs, members) in enumerate(outers):
            shapes.append({"id": f"building_relation_{relation_id}_part_{index}", "feature": feature,
                           "role": "outer", "nodes": refs, "member_ways": members,
                           "tags": relation_shape_tags(tags, members, ways)})
            represented_members.update(members)
        polygons = [[nodes[ref] for ref in refs] for refs, _ in outers]
        for index, (refs, members) in enumerate(inner_rings):
            # Reject masks outside every parent rather than whitening arbitrary
            # map regions because of an invalid OSM relation.
            containing = [i for i, polygon in enumerate(polygons)
                          if all(inside_ring(nodes[ref], polygon) for ref in refs[:-1])]
            if not containing:
                skipped.append({"feature": feature, "role": "inner", "member_ways": members,
                                "reason": "inner_not_contained_in_outer"})
                continue
            parent = min(containing, key=lambda i: polygon_area(polygons[i]))
            shapes.append({"id": f"courtyard_relation_{relation_id}_part_{index}", "feature": feature,
                           "role": "inner", "nodes": refs, "member_ways": members,
                           "tags": relation_shape_tags(tags, members, ways),
                           "parent": f"building_relation_{relation_id}_part_{parent}"})
            represented_members.update(members)
    for way_id, way in sorted(ways.items()):
        if not is_building(way["tags"]) or way_id in represented_members:
            continue
        refs, reason = checked_ring(way["nodes"], nodes)
        if refs:
            shapes.append({"id": f"building_way_{way_id}", "feature": f"way/{way_id}",
                           "role": "outer", "nodes": refs, "member_ways": [way_id], "tags": way["tags"]})
        else:
            skipped.append({"feature": f"way/{way_id}", "role": "outer", "reason": reason})
    return shapes, skipped


def write_inputs(shapes, nodes, normalized_path, typemap_path):
    mapping = {}
    used_nodes = {ref for shape in shapes for ref in shape["nodes"]}
    with normalized_path.open("wb") as handle:
        handle.write(b'<?xml version="1.0" encoding="utf-8"?>\n<osm version="0.6" generator="SKKU footprint ring assembler">\n')
        for ref in sorted(used_nodes):
            lon, lat = nodes[ref]
            element = ET.Element("node", id=str(ref), lon=f"{lon:.8f}", lat=f"{lat:.8f}")
            handle.write(ET.tostring(element, encoding="utf-8") + b"\n")
        for shape in shapes:
            # Synthetic import IDs are deterministic and stay within int64.
            numeric_id = 8_000_000_000_000_000 + int(hashlib.sha256(shape["id"].encode()).hexdigest()[:12], 16)
            if str(numeric_id) in mapping:
                raise RuntimeError("Synthetic polygon identifier collision")
            mapping[str(numeric_id)] = shape
            way = ET.Element("way", id=str(numeric_id))
            for ref in shape["nodes"]:
                ET.SubElement(way, "nd", ref=str(ref))
            tags = dict(shape["tags"])
            if shape["role"] == "inner":
                tags["source:original_building"] = tags.get("building", "")
                tags["building"] = "courtyard_mask"
            tags.update({"source:osm:feature": shape["feature"],
                         "source:osm:geometry_role": shape["role"],
                         "source:osm:member_ways": ",".join(map(str, shape["member_ways"]))})
            if "parent" in shape:
                tags["source:osm:parent_polygon"] = shape["parent"]
            for key, value in sorted(tags.items()):
                ET.SubElement(way, "tag", k=key, v=value)
            handle.write(ET.tostring(way, encoding="utf-8") + b"\n")
        handle.write(b"</osm>\n")
    types = ET.Element("polygonTypes")
    ET.SubElement(types, "polygonType", id="building", name="building",
                  color="0.73,0.76,0.78", layer=str(BUILDING_LAYER), fill="force", discard="false")
    for kind in ("university", "school", "college"):
        ET.SubElement(types, "polygonType", id="building." + kind, name="building.education",
                      color="0.78,0.73,0.64", layer=str(BUILDING_LAYER), fill="force", discard="false")
    ET.SubElement(types, "polygonType", id="building.courtyard_mask", name="courtyard_mask",
                  color="1,1,1", layer=str(COURTYARD_LAYER), fill="force", discard="false")
    ET.indent(types)
    ET.ElementTree(types).write(typemap_path, encoding="utf-8", xml_declaration=True)
    return mapping


def finalize_polygons(raw_path, output, mapping):
    root = ET.parse(raw_path).getroot()
    retained, skipped, restored_closures, restored_attributes, seen = [], [], [], [], set()
    final = ET.Element("additional")
    final.append(ET.Comment("OpenStreetMap contributors, ODbL: https://www.openstreetmap.org/copyright"))
    for element in root.findall("poly"):
        shape = mapping.get(element.attrib["id"])
        if shape is None:
            raise RuntimeError(f"Unexpected imported polygon: {element.attrib['id']}")
        imported_tags = {param.get("key"): param for param in element.findall("param")}
        for key, value in shape["tags"].items():
            retained_key = "source:original_building" if key == "building" and shape["role"] == "inner" else key
            param = imported_tags.get(retained_key)
            if param is None or param.get("value") != value:
                # SUMO trims whitespace around imported tag values. Preserve
                # the exact original value, with XML escaping handled by ET.
                restored_attributes.append({"polygon": shape["id"], "key": retained_key})
                if param is None:
                    param = ET.SubElement(element, "param", key=retained_key)
                param.set("value", value)
        points = [tuple(map(float, point.split(",")[:2])) for point in element.attrib["shape"].split()]
        # polyconvert removes successive vertices less than about 0.1 m apart,
        # including the repeated first point of an explicitly closed OSM way.
        # Restore that original closing edge, not a gap in an open source ring.
        if (len(points) >= 3 and points[0] != points[-1]
                and shape["nodes"][0] == shape["nodes"][-1]
                and math.dist(points[0], points[-1]) <= .11):
            restored_closures.append({"polygon": shape["id"], "closing_segment_m": math.dist(points[0], points[-1])})
            points.append(points[0])
            element.set("shape", " ".join(f"{x:.6f},{y:.6f}" for x, y in points))
        reason = None
        if len(points) < 4 or points[0] != points[-1]:
            reason = "projected_shape_not_closed"
        elif not all(math.isfinite(value) for point in points for value in point):
            reason = "projected_shape_not_finite"
        elif polygon_area(points) <= 0.01:
            reason = "projected_area_too_small"
        if reason:
            skipped.append({"feature": shape["feature"], "polygon": shape["id"], "reason": reason})
            continue
        if shape["id"] in seen:
            raise RuntimeError("Duplicate final building polygon ID")
        seen.add(shape["id"])
        element.set("id", shape["id"])
        element.set("layer", str(COURTYARD_LAYER if shape["role"] == "inner" else BUILDING_LAYER))
        element.set("fill", "true")
        if shape["tags"].get("name"):
            element.set("name", shape["tags"]["name"])
        final.append(element)
        retained.append({**shape, "points": points, "area_m2": polygon_area(points)})
    parent_ids = {shape["id"] for shape in retained if shape["role"] == "outer"}
    for element in list(final):
        if not isinstance(element.tag, str) or element.tag != "poly":
            continue
        shape = next((shape for shape in retained if shape["id"] == element.attrib["id"] and shape["role"] == "inner"), None) if element.attrib["id"].startswith("courtyard_") else None
        if shape and shape["parent"] not in parent_ids:
            final.remove(element)
            retained.remove(shape)
            skipped.append({"feature": shape["feature"], "polygon": shape["id"], "reason": "parent_polygon_not_imported"})
    ET.indent(final)
    ET.ElementTree(final).write(output, encoding="utf-8", xml_declaration=True)
    return retained, skipped, restored_closures, restored_attributes


def overview(scenario, shapes, output):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.collections import PolyCollection, LineCollection
    import sumolib
    net = sumolib.net.readNet(str(scenario / "skku_commute.net.xml"), withInternal=True)
    # The main builder saves route.json before regenerating traffic demand, so
    # the old .rou.xml may still contain edge IDs from the previous network.
    route_path = scenario / "route.json"
    if route_path.exists():
        edge_ids = json.loads(route_path.read_text())["edge_ids"]
    else:
        route_xml = ET.parse(scenario / "skku_commute.rou.xml")
        vehicle = route_xml.find("./vehicle[@id='ego']")
        route = vehicle.find("route")
        if route is None:
            route = route_xml.find(f"./route[@id='{vehicle.attrib['route']}']")
        edge_ids = route.attrib["edges"].split()
    edges = [net.getEdge(edge) for edge in edge_ids]
    halfway = sum(edge.getLength() for edge in edges) / 2
    walked, middle = 0.0, edges[len(edges) // 2]
    for edge in edges:
        walked += edge.getLength()
        if walked >= halfway:
            middle = edge
            break
    centers = [net.convertLonLat2XY(126.997131, 37.584865), middle.getShape()[len(middle.getShape()) // 2],
               net.convertLonLat2XY(126.974074, 37.290649)]
    lanes = [lane.getShape() for edge in net.getEdges(withInternal=True) for lane in edge.getLanes()]
    fig, axes = plt.subplots(1, 3, figsize=(15, 5.8))
    for ax, center, title in zip(axes, centers, ["Seoul SKKU main gate", "Middle of commute route", "Suwon SKKU main gate"]):
        cx, cy = center
        for role, color, zorder in [("outer", "#bac2c7", 1), ("inner", "white", 2)]:
            polygons = [shape["points"] for shape in shapes if shape["role"] == role
                        and any(abs(x - cx) < 650 and abs(y - cy) < 650 for x, y in shape["points"])]
            ax.add_collection(PolyCollection(polygons, facecolors=color, edgecolors="#a9b2b8" if role == "outer" else "white", linewidths=.2, zorder=zorder))
        nearby_lanes = [line for line in lanes if any(abs(x - cx) < 650 and abs(y - cy) < 650 for x, y in line)]
        ax.add_collection(LineCollection(nearby_lanes, colors="#58636b", linewidths=.7, zorder=3))
        ax.set(xlim=(cx - 600, cx + 600), ylim=(cy - 600, cy + 600), aspect="equal", title=title)
        ax.set_xticks([]); ax.set_yticks([])
        ax.plot([cx - 520, cx - 320], [cy - 520, cy - 520], color="#25323b", lw=2)
        ax.text(cx - 420, cy - 495, "200 m", ha="center", fontsize=8)
    fig.suptitle("Actual OSM building footprints along the SKKU commute", fontsize=16)
    fig.text(.03, .08, "2D footprints; courtyard masks; no invented building heights or 3D facades.", fontsize=10)
    fig.text(.03, .035, "© OpenStreetMap contributors · ODbL · https://www.openstreetmap.org/copyright", fontsize=9)
    fig.tight_layout(rect=[0, .11, 1, .96])
    fig.savefig(output, dpi=160, facecolor="white")
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--scenario-dir", type=Path, default=ROOT / "env/skku_commute")
    parser.add_argument("--source", type=Path)
    parser.add_argument("--overview", action="store_true")
    args = parser.parse_args()
    scenario = args.scenario_dir.resolve()
    source = (args.source or scenario / "buildings_source/merged.osm.xml").resolve()
    network = scenario / "skku_commute.net.xml"
    for path in (source, network):
        if not path.is_file():
            raise FileNotFoundError(path)
    os.environ["PATH"] = os.pathsep.join([str(Path(sys.executable).parent), str(ROOT / ".venv-hyehwa/bin"), os.environ.get("PATH", "")])
    binary = shutil.which("polyconvert")
    if binary is None:
        raise RuntimeError("polyconvert is unavailable")
    start = time.monotonic()
    network_hash = sha256(network)
    nodes, ways, relations = read_osm(source)
    shapes, skipped = make_footprints(nodes, ways, relations)
    if not shapes:
        raise RuntimeError("No valid building footprints found")
    typemap = scenario / "buildings.typ.xml"
    normalized = scenario / "buildings.normalized.osm.xml"
    raw = scenario / "buildings.projected.raw.xml"
    output = scenario / "buildings.poly.xml"
    log = scenario / "buildings.build.log"
    mapping = write_inputs(shapes, nodes, normalized, typemap)
    command = [binary, "--net-file", str(network), "--osm-files", str(normalized),
               "--type-file", str(typemap), "--all-attributes", "true", "--discard", "true",
               "--prune.in-net", "true", "--prune.in-net.offsets", "650,650,650,650",
               "--precision", "3", "--output-file", str(raw)]
    with log.open("w", encoding="utf-8") as handle:
        subprocess.run(command, stdout=handle, stderr=subprocess.STDOUT, check=True)
    retained, invalid, restored_closures, restored_attributes = finalize_polygons(raw, output, mapping)
    if not retained:
        raise RuntimeError("No valid polygons survived projection")
    if sha256(network) != network_hash:
        raise RuntimeError("Road network changed during building conversion")
    skipped.extend(invalid)
    buildings = [shape for shape in retained if shape["role"] == "outer"]
    masks = [shape for shape in retained if shape["role"] == "inner"]
    unique_features = defaultdict(list)
    for shape in buildings:
        unique_features[shape["feature"]].append(shape["tags"])
    imported_ids = {shape["id"] for shape in retained}
    not_imported = [shape["id"] for shape in shapes if shape["id"] not in imported_ids]
    manifest = {
        "created_utc": datetime.now(timezone.utc).isoformat(), "builder_sha256": sha256(__file__),
        "source": str(source), "source_sha256": sha256(source), "network_sha256": network_hash,
        "source_manifest": str(source.parent / "manifest.json"),
        "source_manifest_sha256": sha256(source.parent / "manifest.json") if (source.parent / "manifest.json").exists() else None,
        "polyconvert_version": subprocess.check_output([binary, "--version"], text=True).strip(),
        "polyconvert_command": command,
        "projection": dict(ET.parse(network).getroot().find("location").attrib),
        "source_counts": {"nodes": len(nodes), "ways": len(ways), "relations": len(relations),
                          "building_tagged_ways": sum(is_building(way["tags"]) for way in ways.values()),
                          "building_tagged_relations": sum(is_building(relation["tags"]) for relation in relations.values())},
        "counts": {"unique_osm_building_features": len(unique_features),
                   "outer_footprint_polygons": len(buildings), "courtyard_mask_polygons": len(masks),
                   "total_polygons": len(retained),
                   "features_with_name": sum(any(tags.get("name") for tags in group) for group in unique_features.values()),
                   "features_with_height_tag": sum(any(tags.get("height") for tags in group) for group in unique_features.values()),
                   "features_with_levels_tag": sum(any(tags.get("building:levels") for tags in group) for group in unique_features.values()),
                   "known_source_closures_restored": len(restored_closures),
                   "source_attribute_values_restored": len(restored_attributes),
                   "skipped_or_unsupported_records": len(skipped), "prepared_shapes_not_imported": len(not_imported)},
        "validation": {"unique_ids": True, "finite_coordinates": True, "closed_rings": True,
                       "original_osm_attributes_retained": True,
                       "positive_projected_area": True, "minimum_area_m2": min(shape["area_m2"] for shape in retained),
                       "building_layer": BUILDING_LAYER, "courtyard_layer": COURTYARD_LAYER,
                       "road_network_not_modified": True},
        "skipped_or_unsupported": skipped, "not_imported_polygon_ids": not_imported,
        "source_closures_restored_after_polyconvert": restored_closures,
        "source_attribute_values_restored_after_polyconvert": restored_attributes,
        "limitations": ["Unique OSM features are mapping objects, not a guarantee of unique physical buildings.",
                        "Heights/levels are retained only when present in OSM; missing values are not invented.",
                        "Two-dimensional building footprints, not 3D facades or extruded buildings.",
                        "Courtyards are white masks, not native polygon holes; masks can cover other overlapping background polygons.",
                        "Incomplete or branching member rings are skipped rather than closed with invented segments.",
                        "OSM may omit buildings or have outdated outlines. Input nodes are retained without manual geographic shifts."],
        "attribution": "© OpenStreetMap contributors, ODbL, https://www.openstreetmap.org/copyright",
    }
    if args.overview:
        picture = scenario / "buildings.overview.png"
        overview(scenario, retained, picture)
        manifest["overview"] = picture.name
    manifest["output_sha256"] = {path.name: sha256(path) for path in (typemap, normalized, output, log)}
    manifest["wall_seconds"] = time.monotonic() - start
    (scenario / "buildings.manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(manifest["counts"], ensure_ascii=False, indent=2))
    print(f"[saved] {output}")


if __name__ == "__main__":
    main()
