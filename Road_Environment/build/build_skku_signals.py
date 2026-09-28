"""Map documented signal positions onto the SKKU SUMO driving route.

Locations come from archived official datasets and explicit OSM signal tags.
Programs are synthetic. Physical mounting arms are grouped by route position;
their coordinates are not vehicle headings or measured stop-line coordinates.
Every accepted/rejected match is written for inspection. No signal is guessed
merely because a road is an intersection.
"""
from __future__ import annotations

from collections import Counter
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

import numpy as np
from Road_Environment.runtime import prepare_runtime

prepare_runtime()
import sumolib
from sumolib.geomhelper import polygonOffsetAndDistanceToPoint, polyLength

from Road_Environment.paths import ROOT
OUT = ROOT / "env/skku_commute"


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def promote_osm_crossings(source, target):
    """Retain the archived source; change only a derived import document."""
    tree = ET.parse(source)
    promoted = []
    for node in tree.getroot().findall("node"):
        tags = {t.get("k"): t.get("v") for t in node.findall("tag")}
        signaled = "traffic_signals" in tags.get("crossing", "").split(";") or tags.get("crossing:signals") == "yes"
        mixed = "traffic_signals" in tags.get("highway", "").split(";")
        if (signaled or mixed) and tags.get("highway") != "traffic_signals":
            highway = node.find("tag[@k='highway']")
            if highway is None:
                highway = ET.SubElement(node, "tag", k="highway")
            highway.set("v", "traffic_signals")
            promoted.append(dict(osm_node_id=node.get("id"), lon=float(node.get("lon")),
                                 lat=float(node.get("lat")), original_tags=tags))
    review_path = Path(source).parent.parent / "signals_source" / "placement_review.json"
    insertions = []
    if review_path.exists():
        review = json.loads(review_path.read_text())
        if review["source_osm_sha256"] != sha(source):
            raise RuntimeError("Placement review was made against a different OSM source")
        for relative, digest in review["normalized_input_sha256"].items():
            if sha(Path(source).parent.parent / relative) != digest:
                raise RuntimeError(f"Placement review is stale for {relative}")
        from pyproj import Transformer
        project = Transformer.from_crs(4326, 32652, always_xy=True)
        osm = tree.getroot()
        nodes = {n.get("id"): n for n in osm.findall("node")}
        ways = {w.get("id"): w for w in osm.findall("way")}
        for insertion in review.get("midblock_insertions", []):
            way = ways[str(insertion["way_id"])]
            refs = way.findall("nd")
            points = [project.transform(float(nodes[r.get("ref")].get("lon")),
                                        float(nodes[r.get("ref")].get("lat"))) for r in refs]
            point = project.transform(insertion["lon"], insertion["lat"])
            closest_vertex = min(range(len(points)), key=lambda i: math.dist(point, points[i]))
            if math.dist(point, points[closest_vertex]) <= 2:
                node_id = refs[closest_vertex].get("ref")
                node = nodes[node_id]
                reused = True
            else:
                segment = min(range(len(points)-1), key=lambda i: polygonOffsetAndDistanceToPoint(point, points[i:i+2])[1])
                if polygonOffsetAndDistanceToPoint(point, points[segment:segment+2])[1] > 1:
                    raise RuntimeError("Reviewed signal projection is not on its source OSM way")
                node_id = str(9_000_000_000_000 + int(hashlib.sha256(insertion["id"].encode()).hexdigest()[:10], 16))
                if node_id in nodes:
                    raise RuntimeError("Derived signal node ID collides with source OSM")
                node = ET.Element("node", id=node_id, lon=str(insertion["lon"]), lat=str(insertion["lat"]), version="1")
                osm.insert(list(osm).index(osm.find("way")), node)
                way.insert(list(way).index(refs[segment+1]), ET.Element("nd", ref=node_id))
                reused = False
            tag = node.find("tag[@k='highway']")
            if tag is None:
                tag = ET.SubElement(node, "tag", k="highway")
            tag.set("v", "traffic_signals")
            ET.SubElement(node, "tag", k="source", v="Official signal positions projected onto OSM road; local placement_review.json")
            insertions.append(dict(insertion, derived_node_id=node_id, reused_original_node=reused))
    Path(target).parent.mkdir(parents=True, exist_ok=True)
    tree.write(target, encoding="utf-8", xml_declaration=True)
    save(Path(target).with_suffix(".promotion.json"), dict(source_sha256=sha(source),
         derived_sha256=sha(target), promoted_count=len(promoted), records=promoted,
         reviewed_midblock_insertions=insertions))
    return promoted


def load_official(scenario):
    records, inputs = [], []
    for relative in ("signals_source/seoul/normalized_signals.json", "signals_source/gyeonggi_normalized.json"):
        path = scenario / relative
        if path.exists():
            data = json.loads(path.read_text())
            rows = data["records"]
            for row in rows:
                records.append(dict(row, dataset_file=relative))
            inputs.append(dict(path=relative, sha256=sha(path), record_count=len(rows)))
    return records, inputs


def route_geometry(net, path):
    shapes, cumulative, total = [], [], 0.
    for edge in path:
        shape = edge.getShape(includeJunctions=True)
        shapes.append(shape)
        cumulative.append(total)
        total += polyLength(shape)
    return shapes, cumulative, total


def grade_separated(edge):
    if edge.getType() in ("highway.trunk", "highway.motorway", "highway.trunk_link", "highway.motorway_link"):
        return True
    if edge.getParam("bridge", "no") not in ("no", "false", "0", "") or edge.getParam("tunnel", "no") not in ("no", "false", "0", ""):
        return True
    return edge.getParam("layer", "0") not in ("0", "0.0", "")


def locate(point, shapes, cumulative):
    best = (float("inf"), 0., 0)
    for i, shape in enumerate(shapes):
        offset, distance = polygonOffsetAndDistanceToPoint(point, shape)
        if distance < best[0]:
            best = distance, cumulative[i] + offset, i
    return best


def augment(net, path, network_path, scenario=OUT):
    network_path, scenario = Path(network_path), Path(scenario)
    records, inputs = load_official(scenario)
    if not inputs:
        return None
    review_path = scenario / "signals_source" / "placement_review.json"
    review = json.loads(review_path.read_text()) if review_path.exists() else {}
    excluded_ids = set(review.get("excluded_record_ids", []))
    shapes, cumulative, total = route_geometry(net, path)
    route_ids = {e.getID() for e in path}
    candidates = []
    for i, edge in enumerate(path[:-1]):
        node = edge.getToNode()
        # Do not transfer a roadside/underpass signal onto an expressway lane.
        if any(grade_separated(e) for e in (edge, path[i+1])) and not edge.getTLS():
            continue
        if (not edge.getTLS() and len(node.getIncoming()) == 1
                and any(e.getType() in ("highway.motorway_link", "highway.trunk_link") for e in node.getOutgoing())):
            continue
        location = node.getCoord()
        distance, progress, _ = locate(location, shapes, cumulative)
        if distance <= 45:
            candidates.append(dict(node=node, xy=location, progress=progress,
                                   existing=bool(edge.getTLS()),
                                   branches=len(node.getIncoming())))
    accepted_points, rejected_points = [], []
    for record in records:
        if record["id"] in excluded_ids:
            rejected_points.append(dict(id=record["id"], reason="reviewed_off_route_or_ambiguous_road_level",
                                        dataset_file=record["dataset_file"]))
            continue
        if record.get("signal_kind") not in ("vehicle", "controller") or record.get("eligible_vehicle_signal") is False:
            rejected_points.append(dict(id=record["id"], reason="not_confirmed_vehicle_signal",
                                        signal_kind=record.get("signal_kind"), dataset_file=record["dataset_file"]))
            continue
        xy = net.convertLonLat2XY(float(record["lon"]), float(record["lat"]))
        distance, progress, edge_index = locate(xy, shapes, cumulative)
        if distance > 45:
            rejected_points.append(dict(id=record["id"], reason="more_than_45m_from_route", distance_m=distance,
                                        dataset_file=record["dataset_file"]))
            continue
        if grade_separated(path[edge_index]) and not path[edge_index].getTLS():
            rejected_points.append(dict(id=record["id"], reason="expressway_or_grade_separation_review",
                                        dataset_file=record["dataset_file"]))
            continue
        nearby = [(e, d) for e, d in net.getNeighboringEdges(*xy, r=60, includeJunctions=True)
                  if not e.getFunction() and e.allows("passenger")]
        if nearby:
            closest, closest_distance = min(nearby, key=lambda x: x[1])
            if closest.getID() not in route_ids and grade_separated(closest) and closest_distance + 5 < distance:
                rejected_points.append(dict(id=record["id"], reason="closer_off_route_ramp_bridge_or_tunnel",
                    closest_edge=closest.getID(), closest_distance_m=closest_distance, route_distance_m=distance,
                    dataset_file=record["dataset_file"]))
                continue
        accepted_points.append(dict(record, xy=xy, route_distance_m=distance, route_progress_m=progress))
    # Physical heads around one junction are not independent signal controllers.
    groups = []
    for record in sorted(accepted_points, key=lambda r: r["route_progress_m"]):
        if (not groups or record["route_progress_m"] - groups[-1][-1]["route_progress_m"] > 65
                or record["route_progress_m"] - groups[-1][0]["route_progress_m"] > 100):
            groups.append([])
        groups[-1].append(record)
    matches, unmatched, selected = [], [], set()
    for group in groups:
        progress = float(np.median([r["route_progress_m"] for r in group]))
        valid = [c for c in candidates if abs(c["progress"]-progress) <= 80
                 and max(math.dist(c["xy"], r["xy"]) for r in group) <= 80]
        if not valid:
            unmatched.append(dict(reason="no_eligible_route_node_within_80m", records=group))
            continue
        choice = min(valid, key=lambda c: abs(c["progress"]-progress)
                     - (12 if c["existing"] else 0) - (8 if c["branches"] >= 3 else 0))
        node = choice["node"]
        if not choice["existing"]:
            selected.add(node.getID())
        lon, lat = net.convertXY2LonLat(*choice["xy"])
        matches.append(dict(sumo_node_id=node.getID(), sumo_lon=lon, sumo_lat=lat,
                            route_progress_m=choice["progress"], existing_signal=choice["existing"],
                            grouping_span_m=max(r["route_progress_m"] for r in group)-min(r["route_progress_m"] for r in group),
                            min_source_distance_m=min(math.dist(choice["xy"], r["xy"]) for r in group),
                            max_source_distance_m=max(math.dist(choice["xy"], r["xy"]) for r in group), records=group))
    base = scenario / "skku_commute.osm_signals.net.xml"
    shutil.copy2(network_path, base)
    binary = Path(sys.executable).parent / "netconvert"
    command = [str(binary), "--sumo-net-file", str(base), "--output-file", str(network_path),
               "--tls.default-type", "static", "--tls.cycle.time", "90", "--tls.red.time", "25",
               "--tls.allred.time", "2", "--tls.yellow.time", "3",
               "--tls.uncontrolled-within", "--no-turnarounds"]
    if selected:
        command += ["--tls.set", ",".join(sorted(selected))]
    save(scenario / "signals.mapping_plan.json", dict(command=command, matches=matches,
         unmatched_groups=unmatched, rejected_records=rejected_points))
    result = subprocess.run(command, capture_output=True, text=True)
    (scenario / "signals.build.log").write_text(result.stdout + result.stderr, encoding="utf-8")
    if result.returncode:
        shutil.copy2(base, network_path)
        raise RuntimeError(f"Signal network conversion failed ({result.returncode}); see signals.build.log")
    report = dict(created_utc=datetime.now(timezone.utc).isoformat(), source_inputs=inputs,
                  builder_sha256=sha(__file__), network_before_sha256=sha(base), network_after_sha256=sha(network_path),
                  command=command, official_records=len(records), eligible_near_route_records=len(accepted_points),
                  matched_groups=len(matches), selected_nodes=len(selected), unmatched_groups=unmatched,
                  rejected_counts=dict(Counter(r["reason"] for r in rejected_points)),
                  rejected_records=rejected_points, matches=matches,
                  placement_review_sha256=sha(review_path) if review_path.exists() else None,
                  location_method="Actual official mounting points/controller positions, grouped along route and mapped to nearby SUMO road nodes; no junction-only guessing",
                  assumptions=["Signal programs are synthetic, not measured real cycles",
                               "A mounting arm point is not an exact stop line or intersection center",
                               "Datasets have different reference dates; complete current real-world coverage is not certified",
                               "Unmatched and unclassified positions require review, not arbitrary insertion"])
    save(scenario / "signals.manifest.json", report)
    return report
