"""Build the archived Seoul -> Suwon SKKU road corridor for SUMO.

Run download_skku_corridor.py first. OSM geometry is real map data; missing
speeds/lanes, generated signals and background demand are simulation assumptions.
This creates a SUMO baseline scenario, not a trained reinforcement-learning agent.
"""
from __future__ import annotations

import argparse
from bisect import bisect_right
from collections import Counter, defaultdict
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "env" / "skku_commute"
START = (126.9972919, 37.5847826)
END = (126.9738000, 37.2906472)
WAY_IDS = ("37401405", "561229641")
# Used only when the corresponding OSM tag is absent. Values are not surveyed.
DEFAULTS = {
    "motorway": (100, 2, 15, True), "motorway_link": (60, 1, 14, True),
    "trunk": (80, 2, 13, False), "trunk_link": (50, 1, 12, False),
    "primary": (50, 2, 11, False), "primary_link": (40, 1, 10, False),
    "secondary": (50, 2, 9, False), "secondary_link": (40, 1, 8, False),
    "tertiary": (40, 1, 7, False), "tertiary_link": (30, 1, 6, False),
    "unclassified": (30, 1, 5, False), "residential": (30, 1, 4, False),
    "living_street": (20, 1, 3, False), "service": (20, 1, 2, False),
    "road": (30, 1, 1, False),
}


def binary(name):
    local = Path(sys.executable).parent / name
    path = str(local) if local.exists() else shutil.which(name)
    if not path:
        raise RuntimeError(f"Install requirements-skku.txt: cannot find {name}")
    return path


def write_json(path, data):
    Path(path).write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def write_xml(path, root):
    ET.indent(root)
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def convert():
    source = OUT / "source" / "merged.osm.xml"
    if not source.exists():
        raise RuntimeError("Run download_skku_corridor.py first; merged.osm.xml is missing")
    # Preserve explicit signalized crossings and mid-block signals. OSM often
    # represents these as highway=crossing, rather than highway=traffic_signals.
    from build_skku_signals import promote_osm_crossings
    import_source = OUT / "signals_source" / "roads_with_signal_crossings.osm.xml"
    promote_osm_crossings(source, import_source)
    types = ET.Element("types")
    for name, (speed, lanes, priority, oneway) in DEFAULTS.items():
        ET.SubElement(types, "type", id="highway." + name, numLanes=str(lanes),
                      speed=f"{speed / 3.6:.6f}", priority=str(priority),
                      oneway=str(oneway).lower(), allow="passenger bus taxi delivery truck emergency")
    write_xml(OUT / "skku_commute.typ.xml", types)
    cmd = [binary("netconvert"), "--osm-files", str(import_source),
           "--type-files", str(OUT / "skku_commute.typ.xml"),
           "--output-file", str(OUT / "skku_commute.net.xml"),
           "--keep-edges.by-vclass", "passenger", "--keep-edges.components", "1",
           "--geometry.remove", "--ramps.guess", "--ramps.guess-acceleration-lanes",
           "--junctions.join", "--tls.guess-signals", "--tls.join",
           "--tls.uncontrolled-within", "--tls.default-type", "static",
           "--tls.cycle.time", "90", "--tls.red.time", "25", "--tls.allred.time", "2",
           "--tls.yellow.time", "3", "--no-turnarounds", "--roundabouts.guess",
           "--junctions.corner-detail", "5", "--output.street-names",
           "--osm.turn-lanes", "--osm.lane-access", "--osm.annotate-defaults",
           "--osm.extra-attributes", "highway,maxspeed,lanes,oneway,layer,bridge,tunnel"]
    result = subprocess.run(cmd, capture_output=True, text=True)
    (OUT / "build.log").write_text(result.stdout + result.stderr, encoding="utf-8")
    if result.returncode:
        raise RuntimeError(f"netconvert failed; see {OUT / 'build.log'}")
    return cmd


def endpoint_candidates(net, coord, way_id):
    from sumolib.geomhelper import polygonOffsetAndDistanceToPoint, polyLength
    point = net.convertLonLat2XY(*coord)
    candidates = []
    for edge in net.getEdges():
        if edge.getFunction() or not edge.allows("passenger"):
            continue
        # Keep the documented public road, not an adjacent private campus lane.
        if edge.getID().lstrip("-").split("#")[0] != way_id:
            continue
        pos, distance = polygonOffsetAndDistanceToPoint(point, edge.getShape())
        pos *= edge.getLength() / max(polyLength(edge.getShape()), 1e-6)
        if distance <= 70:
            candidates.append((distance, edge, min(max(pos, 5.0), edge.getLength() - 0.1)))
    if not candidates:
        raise RuntimeError(f"No public-road edge within 70m of {coord}, OSM way {way_id}")
    return sorted(candidates, key=lambda x: x[0])[:6]


def make_route(net):
    choices = []
    for sd, start, spos in endpoint_candidates(net, START, WAY_IDS[0]):
        for ed, end, epos in endpoint_candidates(net, END, WAY_IDS[1]):
            path, cost = net.getFastestPath(start, end, vClass="passenger", fromPos=spos, toPos=epos)
            if path:
                # Snap distance matters, but both valid driving directions may be considered.
                choices.append((cost + sd + ed, path, cost, spos, epos, sd, ed))
    if not choices:
        raise RuntimeError("No legal connected passenger path between the two public gate roads")
    _, path, cost, spos, epos, sd, ed = min(choices, key=lambda x: x[0])
    for a, b in zip(path, path[1:]):
        if not any(c.getFromLane().allows("passenger") and c.getToLane().allows("passenger")
                   for c in a.getConnections(b)):
            raise RuntimeError(f"Missing legal lane connection: {a.getID()} -> {b.getID()}")
    length = sum(e.getLength() for e in path) - spos - path[-1].getLength() + epos
    internal_length = 0.
    for a, b in zip(path, path[1:]):
        connections = [c for c in a.getConnections(b) if c.getFromLane().allows("passenger")
                       and c.getToLane().allows("passenger")]
        internal, distance = net.getInternalPath(connections)
        internal_length += distance if internal else 0.
    express = sum(e.getLength() for e in path if e.getType() in ("highway.motorway", "highway.trunk"))
    if not 30_000 < length < 60_000 or express < 3000:
        raise RuntimeError(f"Unexpected route length {length:.0f}m or expressway length {express:.0f}m")
    names = []
    for edge in path:
        name = edge.getName()
        if name and (not names or names[-1] != name):
            names.append(name)
    details = dict(id="seoul_to_suwon", edge_ids=[e.getID() for e in path],
                   length_excluding_internal_m=round(length, 2), free_flow_cost_s=round(cost, 2),
                   length_with_minimum_internal_m=round(length + internal_length, 2),
                   depart_pos_m=spos, arrival_pos_m=epos,
                   start_lon_lat=START, end_lon_lat=END, start_snap_distance_m=sd, end_snap_distance_m=ed,
                   expressway_mainline_length_m=round(express, 2),
                   max_edge_speed_kmh=round(max(e.getSpeed() for e in path) * 3.6, 2),
                   named_roads=names,
                   signal_controller_count=len({e.getTLS().getID() for e in path if e.getTLS()}),
                   edges=[dict(id=e.getID(), name=e.getName(), road_type=e.getType(),
                               length_m=e.getLength(), lanes=e.getLaneNumber(), speed_kmh=e.getSpeed()*3.6)
                          for e in path])
    write_json(OUT / "route.json", details)
    return path, details


def area_traffic(net, path, corridor_routes, segment_vph):
    """Connected opposing and cross-street trips; all demand is synthetic."""
    def legal(edge):
        return not edge.getFunction() and edge.allows("passenger")

    def direction(edge):
        shape = edge.getShape()
        dx, dy = shape[-1][0] - shape[0][0], shape[-1][1] - shape[0][1]
        length = math.hypot(dx, dy) or 1
        return dx / length, dy / length

    def alignment(a, b):
        x, y = direction(a)
        u, v = direction(b)
        return x * u + y * v

    def connected(a, b):
        return any(c.getFromLane().allows("passenger") and c.getToLane().allows("passenger")
                   for c in a.getConnections(b))

    def opposite(edge):
        shape = edge.getShape()
        x, y = shape[len(shape)//2]
        candidates = [(distance, candidate.getID(), candidate)
                      for candidate, distance in net.getNeighboringEdges(x, y, 100)
                      if legal(candidate) and alignment(edge, candidate) < -.7
                      and candidate.getType() == edge.getType()]
        return min(candidates, default=(0, "", None))[2]

    routes, seen = [], set()
    for _, edges, rate, kind in corridor_routes:
        if kind not in ("seoul", "corridor"):
            continue
        start, end = opposite(edges[-1]), opposite(edges[0])
        if start is None or end is None:
            continue
        reverse, _ = net.getFastestPath(start, end, vClass="passenger")
        if reverse and 1000 < sum(e.getLength() for e in reverse) < 9000:
            key = tuple(e.getID() for e in reverse)
            if key not in seen:
                seen.add(key)
                routes.append((f"opposing_{len(routes):03d}", list(reverse), rate, "opposing"))

    def extend(edges, backwards):
        distance = 0
        while distance < 650:
            anchor = edges[0] if backwards else edges[-1]
            neighbors = anchor.getIncoming() if backwards else anchor.getOutgoing()
            choices = [e for e in neighbors if legal(e) and e not in edges
                       and alignment(e, anchor) > -.5
                       and (connected(e, anchor) if backwards else connected(anchor, e))]
            if not choices:
                break
            chosen = max(choices, key=lambda e: (alignment(e, anchor), e.getLaneNumber(), e.getID()))
            if backwards:
                edges.insert(0, chosen)
            else:
                edges.append(chosen)
            distance += chosen.getLength()
        return edges

    # Include both carriageways' junctions, with through and legal turning trips.
    nodes = {edge.getToNode() for edge in path}
    for _, edges, _, _ in routes:
        nodes.update(e.getToNode() for e in edges)
    main_edges = set(path)
    main_edges.update(e for _, edges, _, _ in routes for e in edges)
    populated_nodes = []
    for node in sorted(nodes, key=lambda n: ("traffic_light" not in n.getType(), n.getID())):
        if len(node.getIncoming()) < 2 or len(node.getOutgoing()) < 2:
            continue
        x, y = node.getCoord()
        if any(math.hypot(x - px, y - py) < 400 for px, py in populated_nodes):
            continue
        count_before = len(routes)
        for incoming in sorted(node.getIncoming(), key=lambda e: e.getID()):
            if not legal(incoming):
                continue
            choices = [e for e in incoming.getOutgoing() if legal(e)
                       and connected(incoming, e) and alignment(incoming, e) > -.7
                       and not (incoming in main_edges and e in main_edges)]
            choices.sort(key=lambda e: (-alignment(incoming, e), e.getID()))
            # One through movement and one legal turn per approach; split its budget.
            choices = choices[:2]
            for outgoing in choices:
                edges = extend(extend([incoming, outgoing], True), False)
                key = tuple(e.getID() for e in edges)
                if key in seen or sum(e.getLength() for e in edges) < 700:
                    continue
                seen.add(key)
                _, latitude = net.convertXY2LonLat(*node.getCoord())
                capacity = min(incoming.getLaneNumber(), outgoing.getLaneNumber(), 3)
                rate = segment_vph * (.45 if latitude >= 37.46 else .30) * capacity / len(choices)
                routes.append((f"cross_{len(routes):03d}", edges, rate, "cross_street"))
        if len(routes) > count_before:
            populated_nodes.append((x, y))
    # Overlapping local trips share an additional per-lane demand budget.
    # Scale each route by its busiest edge so every edge stays within the cap.
    loads = defaultdict(float)
    for _, edges, rate, kind in routes:
        if kind == "cross_street":
            for edge in edges:
                loads[edge] += rate
    balanced = []
    for rid, edges, rate, kind in routes:
        if kind == "cross_street":
            rate *= min([1.] + [segment_vph * .6 * edge.getLaneNumber() / loads[edge]
                               for edge in edges if loads[edge] > 0])
        balanced.append((rid, edges, rate, kind))
    return balanced


def make_demand(net, path, details, segment_vph=100):
    root = ET.Element("routes")
    ET.SubElement(root, "vType", id="ego_sedan", vClass="passenger", guiShape="passenger/sedan",
                  carFollowModel="IDM", accel="2.2", decel="4.5", emergencyDecel="9", tau="1.2",
                  minGap="2.5", length="4.8", maxSpeed="36.11", speedFactor="1.0", speedDev="0",
                  color="1,0.12,0.12")
    mix = ET.SubElement(root, "vTypeDistribution", id="commute_traffic")
    for model, prob, color in [("IDM", "0.6", "0.2,0.55,0.95"), ("Krauss", "0.4", "0.95,0.7,0.15")]:
        ET.SubElement(mix, "vType", id="commute_" + model.lower(), probability=prob,
                      vClass="passenger", guiShape="passenger/sedan", carFollowModel=model,
                      accel="2.0", decel="4.5", emergencyDecel="9", tau="1.3", minGap="2.5",
                      length="4.8", maxSpeed="36.11", speedFactor="normc(0.95,0.08,0.7,1.1)", color=color)
    toll_mix = ET.SubElement(root, "vTypeDistribution", id="toll_traffic")
    for vehicle_type in mix:
        attributes = dict(vehicle_type.attrib)
        attributes["id"] += "_toll"
        # Do not drift right across the plaza before moving back to an assigned booth.
        ET.SubElement(toll_mix, "vType", **attributes, lcKeepRight="0", lcStrategic="5")
    area_mix = ET.SubElement(root, "vTypeDistribution", id="area_traffic")
    for vehicle_type in mix:
        attributes = dict(vehicle_type.attrib)
        attributes.update(id=attributes["id"] + "_area", tau="1.8",
                          jmTimegapMinor="2", jmExtraGap="2")
        ET.SubElement(area_mix, "vType", **attributes)
    ET.SubElement(root, "route", id="seoul_to_suwon", edges=" ".join(details["edge_ids"]))
    # Overlapping ~5km routes distribute demand along the full corridor. Starting
    # every vehicle at Seoul would leave Suwon empty for the first half hour.
    cumulative = [0.]
    for edge in path:
        cumulative.append(cumulative[-1] + edge.getLength())
    toll = toll_approach(net, path)
    booth_positions = toll_booth_positions(net, toll)
    routes, used = [], set()
    cursor = 0
    while cursor < cumulative[-1] - 1000:
        start = min(bisect_right(cumulative, cursor) - 1, len(path) - 1)
        if path[start].getID() == toll.getID():
            start = max(0, start - 1)  # Enter from four lanes; never spawn into a booth lane.
        end = next((i for i in range(start + 1, len(path)) if cumulative[i] >= cursor + 5000), len(path))
        part = path[start:end]
        key = tuple(e.getID() for e in part)
        if len(part) > 1 and key not in used:
            used.add(key)
            _, latitude = net.convertXY2LonLat(*part[0].getShape()[0])
            seoul = latitude >= 37.46
            routes.append((f"corridor_{len(routes):02d}", part,
                           segment_vph * (2.0 if latitude >= 37.54 else 1.3 if seoul else 1), "seoul" if seoul else "corridor"))
        cursor += 2500
    # Legal small feeder routes introduce junction conflicts and merging traffic.
    # Use actual SUMO connections; never invent a straight shortcut through nodes.
    last_cross = -1000.
    for index in range(1, len(path) - 4):
        if cumulative[index] - last_cross < 1800:
            continue
        edge = path[index]
        feeders = sorted((e for e in edge.getIncoming() if e not in path and e.allows("passenger")
                          and not e.getFunction() and e.getLength() >= 35), key=lambda e: e.getID())
        for feeder in feeders:
            goal = path[min(index + 5, len(path)-1)]
            spur, _ = net.getFastestPath(feeder, goal, vClass="passenger")
            if spur and 200 < sum(e.getLength() for e in spur) < 2500:
                _, latitude = net.convertXY2LonLat(*edge.getShape()[0])
                routes.append((f"feeder_{len(routes):02d}", spur,
                               segment_vph * (1.0 if latitude >= 37.46 else .6), "feeder"))
                last_cross = cumulative[index]
                break
    routes.extend(area_traffic(net, path, routes, segment_vph))
    ti = path.index(toll)
    start = max(0, bisect_right(cumulative, cumulative[ti] - 2200) - 1)
    end = min(len(path), bisect_right(cumulative, cumulative[ti] + 2200))
    routes.append(("uiwang_approach", path[start:end], segment_vph * 12, "toll_approach"))
    from build_skku_freeway_traffic import routes as freeway_routes
    routes.extend(freeway_routes(net, path, segment_vph))
    for rid, edges, _, _ in routes:
        ET.SubElement(root, "route", id=rid, edges=" ".join(e.getID() for e in edges))
    from skku_queue_lane_change import build_plan
    build_plan(path, OUT)
    ego = ET.Element("vehicle", id="ego", type="ego_sedan", route="seoul_to_suwon", depart="300",
                  departLane="best", departSpeed="0", departPos=f"{details['depart_pos_m']:.3f}",
                  arrivalPos=f"{details['arrival_pos_m']:.3f}")
    ET.SubElement(ego, "stop", lane=toll.getID()+"_0", endPos=f"{booth_positions[0]:.2f}",
                  duration="6", parking="false", actType="synthetic toll service")
    # Larger accepted lane-change gaps for the denser bidirectional scenario.
    for vehicle_type in root.iter("vType"):
        vehicle_type.set("lcAssertive", "0.5")
    traffic = []
    for i, (rid, edges, vph, kind) in enumerate(routes):
        begin = 1 + i * .1
        booth_lanes = range(6) if toll in edges else [None]
        for lane in booth_lanes:
            suffix = f"_booth{lane}" if lane is not None else ""
            flow = ET.SubElement(root, "flow", id="flow_" + rid + suffix,
                          type=("toll_traffic" if lane is not None else
                                "area_traffic" if kind in ("opposing", "cross_street") else "commute_traffic"), route=rid,
                          begin=f"{begin+(lane*3600/vph if lane is not None and vph else 0):.3f}", end="10800", vehsPerHour=f"{vph/(6 if lane is not None else 1):.8f}",
                          departLane="random" if kind.startswith("freeway_") else "best", departSpeed="max")
            if lane is not None:
                ET.SubElement(flow, "stop", lane=f"{toll.getID()}_{lane}",
                              endPos=f"{booth_positions[lane]:.2f}", duration="6", parking="false",
                              actType="synthetic toll service")
        traffic.append(dict(id=rid, kind=kind, expected_vehicles_per_hour=vph,
                            flow_count=6 if toll in edges else 1,
                            assigned_booth_lanes=list(range(6)) if toll in edges else [],
                            edge_ids=[e.getID() for e in edges]))
    demand=list(root.findall("flow"))+[ego]
    for flow in root.findall("flow"):
        root.remove(flow)
    root.extend(sorted(demand,key=lambda e:float(e.get("begin",e.get("depart","0")))))
    write_xml(OUT / "skku_commute.rou.xml", root)
    return traffic


def toll_approach(net, path):
    """Snap archived southbound Uiwang toll booth to the actual ego route."""
    from sumolib.geomhelper import polygonOffsetAndDistanceToPoint
    node = ET.parse(OUT / "source/merged.osm.xml").find("./node[@id='1142439460']")
    xy = net.convertLonLat2XY(float(node.get("lon")), float(node.get("lat")))
    distance, edge = min((polygonOffsetAndDistanceToPoint(xy, e.getShape())[1], e)
                         for e in path if e.getName() == "봉담과천로")
    if distance > 40:
        raise ValueError("Uiwang booth is no longer on this route")
    return edge


def congestion(net, path):
    edge = toll_approach(net, path)
    root = ET.Element("additional")
    vss = ET.SubElement(root, "variableSpeedSign", id="uiwang_toll_capacity",
                        lanes=" ".join(l.getID() for l in edge.getLanes() if l.allows("passenger")))
    ET.SubElement(vss, "step", time="0", speed="2.0")
    write_xml(OUT / "congestion.add.xml", root)
    write_json(OUT / "congestion.manifest.json", dict(synthetic=True, toll_osm_node="1142439460",
               toll_edge=edge.getID(), controlled_length_m=edge.getLength(), speed_mps=2.0,
               ego_warmup_s=300, seoul_corridor_multiplier=2.0, seoul_south_multiplier=1.3, toll_demand_multiplier=12,
               booth_service_duration_s=6, booth_lane_share=[1/6]*6,
               ego_booth_lane=0, lanechange_duration_s=3,
               booth_positions_m=toll_booth_positions(net, edge),
               assumption="Synthetic equal-share booth choices and 6s service; safe native lane changes and zipper merge, not surveyed toll operations"))


def toll_booth_positions(net, edge):
    from sumolib.geomhelper import polygonOffsetAndDistanceToPoint, polyLength
    node = ET.parse(OUT / "source/merged.osm.xml").find("./node[@id='1142439460']")
    xy = net.convertLonLat2XY(float(node.get("lon")), float(node.get("lat")))
    positions = []
    for lane in edge.getLanes():
        offset, _ = polygonOffsetAndDistanceToPoint(xy, lane.getShape())
        pos = offset * lane.getLength() / polyLength(lane.getShape())
        if not 50 < pos < lane.getLength() - 50:
            raise ValueError("Toll stop must leave room to approach and merge")
        positions.append(pos)
    return positions


def configuration(net):
    root = ET.Element("configuration")
    inputs = ET.SubElement(root, "input")
    ET.SubElement(inputs, "net-file", value="skku_commute.net.xml")
    ET.SubElement(inputs, "route-files", value="skku_commute.rou.xml")
    additions = ["campuses.add.xml"]
    if (OUT / "buildings.poly.xml").exists():
        additions.append("buildings.poly.xml")
    additions.extend(["landmarks.add.xml", "congestion.add.xml", "station_signals.add.xml"])
    ET.SubElement(inputs, "additional-files", value=",".join(additions))
    time = ET.SubElement(root, "time")
    ET.SubElement(time, "step-length", value="0.5")
    ET.SubElement(time, "end", value="10800")
    process = ET.SubElement(root, "processing")
    ET.SubElement(process, "time-to-teleport", value="-1")
    ET.SubElement(process, "collision.action", value="remove")
    ET.SubElement(process, "collision.check-junctions", value="true")
    ET.SubElement(process, "lanechange.duration", value="3")
    report = ET.SubElement(root, "report")
    ET.SubElement(report, "no-step-log", value="true")
    gui = ET.SubElement(root, "gui_only")
    ET.SubElement(gui, "gui-settings-file", value="gui.view.xml")
    write_xml(OUT / "skku_commute.sumocfg", root)
    view = ET.Element("viewsettings")
    scheme = ET.SubElement(view, "scheme", name="SKKU campus commute")
    ET.SubElement(scheme, "vehicles", vehicleMode="0", vehicleQuality="2", vehicle_minSize="0",
                  vehicle_exaggeration="1", vehicle_constantSize="false", showBlinker="true")
    ET.SubElement(scheme, "polys", polyName_show="false", polyType_show="false", polyUseCustomLayer="false", poly_minSize="0")
    ET.SubElement(scheme, "pois", poiName_show="false", poiType_show="false", poi_minSize="0", poi_exaggeration="1")
    write_xml(OUT / "gui.view.xml", view)
    addition = ET.Element("additional")
    for pid, coord, color in [("SKKU_Seoul_main_gate", START, "0.1,0.7,0.3"),
                              ("SKKU_Suwon_main_gate", END, "0.9,0.2,0.2")]:
        x, y = net.convertLonLat2XY(*coord)
        ET.SubElement(addition, "poi", id=pid, x=str(x), y=str(y), color=color, layer="10", width="12", height="12")
    write_xml(OUT / "campuses.add.xml", addition)


def plot_network(net, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
    from matplotlib.patches import Polygon
    from matplotlib.font_manager import FontProperties
    fig = plt.figure(figsize=(12, 13), facecolor="#f7f8fa")
    grid = fig.add_gridspec(3, 2, width_ratios=[1.3, 1], wspace=.10, hspace=.22)
    main = fig.add_subplot(grid[:, 0])
    axes = [main] + [fig.add_subplot(grid[i, 1]) for i in range(3)]
    normal = [e.getShape() for e in net.getEdges() if not e.getFunction()]
    urban = [e.getShape() for e in path if e.getType() not in ("highway.motorway", "highway.trunk")]
    express = [e.getShape() for e in path if e.getType() in ("highway.motorway", "highway.trunk")]
    signal_points = list({e.getToNode().getID(): e.getToNode().getCoord() for e in path if e.getTLS()}.values())
    for ax in axes:
        for poly in ET.parse(OUT / "landmarks.add.xml").getroot().findall("poly"):
            if poly.get("type") not in ("water", "island"):
                continue
            points = [tuple(map(float, p.split(','))) for p in poly.get("shape").split()]
            color = tuple(int(v) / 255 for v in poly.get("color").split(','))
            ax.add_patch(Polygon(points, facecolor=color, edgecolor="none", zorder=0))
        ax.add_collection(LineCollection(normal, colors="#c8d0d8", linewidths=.35))
        ax.add_collection(LineCollection(urban, colors="#147ac2", linewidths=2))
        ax.add_collection(LineCollection(express, colors="#eb7934", linewidths=2.3))
        if signal_points:
            ax.scatter(*zip(*signal_points), s=13, c="#c43f56", edgecolors="white", linewidths=.3, zorder=5)
        ax.set_aspect("equal")
        ax.set_facecolor("#f7f8fa")
        ax.set_xticks([])
        ax.set_yticks([])
        for spine in ax.spines.values():
            spine.set_visible(False)
    points = [p for e in path for p in e.getShape()]
    xs, ys = zip(*points)
    main.set_xlim(min(xs)-1600, max(xs)+1600)
    main.set_ylim(min(ys)-1400, max(ys)+1400)
    start, end = (net.convertLonLat2XY(*c) for c in (START, END))
    for label, point, color in [("Seoul campus", start, "#119c57"), ("Suwon campus", end, "#db3d3d")]:
        main.scatter(*point, s=65, color=color, zorder=8)
        main.annotate(label, point, xytext=(8, 8), textcoords="offset points", fontsize=10, fontweight="bold")
    merge = next((e for e in path if e.getType() in ("highway.motorway_link", "highway.trunk_link")), path[len(path)//2])
    centers = [start, merge.getShape()[0], end]
    for ax, center, title in zip(axes[1:], centers, ["Seoul: departure and urban junctions", "Expressway access / merge", "Suwon: arrival at the main gate"]):
        x, y = center
        ax.set_xlim(x-900, x+900)
        ax.set_ylim(y-650, y+650)
        ax.set_title(title, fontsize=11, loc="left")
    main.set_title("Seoul → Suwon\nFull driving corridor", fontsize=15, loc="left")
    landmarks = json.loads((OUT / "landmarks.manifest.json").read_text())
    font = FontProperties(fname=landmarks["font"], size=8)
    for ax in axes[1:]:
        xmin, xmax = ax.get_xlim()
        ymin, ymax = ax.get_ylim()
        for place in landmarks["landmarks"]:
            if xmin < place["x"] < xmax and ymin < place["y"] < ymax:
                ax.annotate(place["name"], (place["x"], place["y"]), fontproperties=font,
                            xytext=(3, 3), textcoords="offset points", color="#196b49")
    fig.text(.12, .045, "Blue: urban / access roads   Orange: motorway / trunk   Red dots: signal control\n© OpenStreetMap contributors · Signal positions: OSM + official datasets · Synthetic signal plans", fontsize=9)
    fig.savefig(OUT / "network.png", dpi=160, bbox_inches="tight")
    plt.close(fig)


def main():
    from run_skku_commute import prepare_runtime
    prepare_runtime()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--skip-convert", action="store_true", help="Reuse existing .net.xml")
    parser.add_argument("--segment-vph", type=float, default=100, help="Synthetic flow rate per overlapping corridor segment")
    args = parser.parse_args()
    if not 0 <= args.segment_vph <= 1800:
        parser.error("--segment-vph must be between 0 and 1800")
    OUT.mkdir(parents=True, exist_ok=True)
    old = json.loads((OUT / "manifest.json").read_text()) if (OUT / "manifest.json").exists() else {}
    if args.skip_convert:
        if old.get("source_osm_sha256") != sha256(OUT / "source" / "merged.osm.xml"):
            raise RuntimeError("OSM source changed or prior manifest missing; rebuild without --skip-convert")
        if old.get("files_sha256", {}).get("skku_commute.net.xml") != sha256(OUT / "skku_commute.net.xml"):
            raise RuntimeError("Network hash differs from prior manifest; rebuild without --skip-convert")
    command = None if args.skip_convert else convert()
    import sumolib
    net = sumolib.net.readNet(str(OUT / "skku_commute.net.xml"), withInternal=True)
    path, route = make_route(net)
    if not args.skip_convert:
        from build_skku_signals import augment
        signal_report = augment(net, path, OUT / "skku_commute.net.xml")
        if signal_report:
            net = sumolib.net.readNet(str(OUT / "skku_commute.net.xml"), withInternal=True)
            path, route = make_route(net)
        if (OUT / "buildings_source" / "merged.osm.xml").exists():
            subprocess.run([sys.executable, str(ROOT / "build_skku_buildings.py"), "--overview"], check=True)
    from build_skku_lanes import apply as apply_lane_correction
    if apply_lane_correction(net, path):
        net = sumolib.net.readNet(str(OUT / "skku_commute.net.xml"), withInternal=True)
        path, route = make_route(net)
    traffic = make_demand(net, path, route, args.segment_vph)
    from build_skku_landmarks import build
    build(net, path)
    congestion(net, path)
    from build_skku_station_signals import build as build_station_signals
    build_station_signals()
    configuration(net)
    plot_network(net, path)
    all_edges = [e for e in net.getEdges() if not e.getFunction()]
    source_tags = defaultdict(Counter)
    used_ways = {e.getID().lstrip("-").split("#")[0] for e in path}
    for _, element in ET.iterparse(OUT / "source" / "merged.osm.xml", events=("end",)):
        if element.tag == "way":
            tags = {t.get("k"): t.get("v") for t in element.findall("tag")}
            if tags.get("highway"):
                for scope in (["all", "route"] if element.get("id") in used_ways else ["all"]):
                    source_tags[scope]["ways"] += 1
                    for key in ("maxspeed", "lanes", "oneway", "bridge", "tunnel"):
                        source_tags[scope][key + "_tagged"] += key in tags
            element.clear()
    manifest = dict(created_utc=datetime.now(timezone.utc).isoformat(),
                    sumo_version=subprocess.check_output([binary("sumo"), "--version"], text=True).splitlines()[0],
                    source_manifest="source/manifest.json", source_osm_sha256=sha256(OUT / "source" / "merged.osm.xml"),
                    builder_sha256=sha256(__file__),
                    netconvert_command=command or old.get("netconvert_command"),
                    network=dict(edges=len(all_edges), junctions=len(net.getNodes()),
                                 signals=len(net.getTrafficLights()),
                                 road_types=dict(Counter(e.getType() for e in all_edges))),
                    route={k: v for k, v in route.items() if k not in ("edges", "edge_ids")},
                    background=dict(synthetic=True, segment_vph=args.segment_vph,
                                    expected_total_departures_per_hour=sum(t["expected_vehicles_per_hour"] for t in traffic),
                                    routes=traffic),
                    default_road_attributes={k: dict(speed_kmh=v[0], lanes=v[1], oneway=v[3]) for k, v in DEFAULTS.items()},
                    osm_tag_coverage=dict(source_tags),
                    assumptions=["OSM road topology; no surveyed lane geometry or signal timing",
                                 "Missing speed/lanes use documented typemap defaults; OSM tags take precedence",
                                 "Traffic light positions use explicit OSM signal tags and archived official datasets; programs are synthetic",
                                 "Missing current official positions and unmatched records prevent a claim of complete present-day signal coverage",
                                 "Ramp lane guesses and demand are synthetic",
                                 "Public roads outside main gates; no campus internal driving",
                                 "SUMO IDM ego uses built-in safety and automatic lane changes; no PPO model attached",
                                 "OSRM identifies the download corridor; SUMO computes its own legal fastest route"],
                    files_sha256={name: sha256(OUT / name) for name in ["skku_commute.net.xml", "skku_commute.typ.xml", "skku_commute.rou.xml", "skku_commute.sumocfg", "route.json", "gui.view.xml", "campuses.add.xml", "build.log", "buildings.poly.xml", "buildings.manifest.json", "signals.manifest.json", "landmarks.add.xml", "landmarks.manifest.json", "congestion.add.xml", "congestion.manifest.json", "station_signals.add.xml", "station_signals.manifest.json", "queue_lane_change.json"] if (OUT / name).exists()})
    if (OUT / "signals.manifest.json").exists():
        signal_report = json.loads((OUT / "signals.manifest.json").read_text())
        manifest["signal_location_augmentation"] = {k: signal_report[k] for k in ("source_inputs", "selected_nodes", "matched_groups", "assumptions")}
    if (OUT / "buildings.manifest.json").exists():
        manifest["building_footprints"] = json.loads((OUT / "buildings.manifest.json").read_text())["counts"]
    manifest["lane_correction"] = json.loads((OUT / "lanes.manifest.json").read_text())
    for name in ("lanes.edg.xml", "lanes.con.xml", "lanes.nod.xml", "lanes.manifest.json"):
        manifest["files_sha256"][name] = sha256(OUT / name)
    write_json(OUT / "manifest.json", manifest)
    print(json.dumps({"output": str(OUT), "network": manifest["network"], "route": manifest["route"],
                      "background_flows": sum(t["flow_count"] for t in traffic)}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
