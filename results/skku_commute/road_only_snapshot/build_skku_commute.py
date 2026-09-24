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
        raise RuntimeError(f"Install requirements-hyehwa.txt: cannot find {name}")
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
    types = ET.Element("types")
    for name, (speed, lanes, priority, oneway) in DEFAULTS.items():
        ET.SubElement(types, "type", id="highway." + name, numLanes=str(lanes),
                      speed=f"{speed / 3.6:.6f}", priority=str(priority),
                      oneway=str(oneway).lower(), allow="passenger bus taxi delivery truck emergency")
    write_xml(OUT / "skku_commute.typ.xml", types)
    cmd = [binary("netconvert"), "--osm-files", str(source),
           "--type-files", str(OUT / "skku_commute.typ.xml"),
           "--output-file", str(OUT / "skku_commute.net.xml"),
           "--keep-edges.by-vclass", "passenger", "--keep-edges.components", "1",
           "--geometry.remove", "--ramps.guess", "--ramps.guess-acceleration-lanes",
           "--junctions.join", "--tls.guess-signals", "--tls.discard-simple", "--tls.join",
           "--tls.uncontrolled-within", "--tls.default-type", "actuated",
           "--tls.cycle.time", "90", "--no-turnarounds", "--roundabouts.guess",
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
    ET.SubElement(root, "route", id="seoul_to_suwon", edges=" ".join(details["edge_ids"]))
    # Overlapping ~5km routes distribute demand along the full corridor. Starting
    # every vehicle at Seoul would leave Suwon empty for the first half hour.
    cumulative = [0.]
    for edge in path:
        cumulative.append(cumulative[-1] + edge.getLength())
    routes, used = [], set()
    cursor = 0
    while cursor < cumulative[-1] - 1000:
        start = min(bisect_right(cumulative, cursor) - 1, len(path) - 1)
        end = next((i for i in range(start + 1, len(path)) if cumulative[i] >= cursor + 5000), len(path))
        part = path[start:end]
        key = tuple(e.getID() for e in part)
        if len(part) > 1 and key not in used:
            used.add(key)
            routes.append((f"corridor_{len(routes):02d}", part, segment_vph, "corridor"))
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
                routes.append((f"feeder_{len(routes):02d}", spur, segment_vph * .6, "feeder"))
                last_cross = cumulative[index]
                break
    for rid, edges, _, _ in routes:
        ET.SubElement(root, "route", id=rid, edges=" ".join(e.getID() for e in edges))
    ET.SubElement(root, "vehicle", id="ego", type="ego_sedan", route="seoul_to_suwon", depart="0",
                  departLane="best", departSpeed="0", departPos=f"{details['depart_pos_m']:.3f}",
                  arrivalPos=f"{details['arrival_pos_m']:.3f}")
    traffic = []
    for i, (rid, edges, vph, kind) in enumerate(routes):
        begin = 1 + i * .1
        ET.SubElement(root, "flow", id="flow_" + rid, type="commute_traffic", route=rid,
                      begin=f"{begin:.1f}", end="7200", probability=f"{vph/3600:.8f}",
                      departLane="best", departSpeed="max")
        traffic.append(dict(id=rid, kind=kind, expected_vehicles_per_hour=vph,
                            edge_ids=[e.getID() for e in edges]))
    write_xml(OUT / "skku_commute.rou.xml", root)
    return traffic


def configuration(net):
    root = ET.Element("configuration")
    inputs = ET.SubElement(root, "input")
    ET.SubElement(inputs, "net-file", value="skku_commute.net.xml")
    ET.SubElement(inputs, "route-files", value="skku_commute.rou.xml")
    ET.SubElement(inputs, "additional-files", value="campuses.add.xml")
    time = ET.SubElement(root, "time")
    ET.SubElement(time, "step-length", value="0.5")
    ET.SubElement(time, "end", value="7200")
    process = ET.SubElement(root, "processing")
    ET.SubElement(process, "time-to-teleport", value="-1")
    ET.SubElement(process, "collision.action", value="remove")
    ET.SubElement(process, "collision.check-junctions", value="true")
    report = ET.SubElement(root, "report")
    ET.SubElement(report, "no-step-log", value="true")
    gui = ET.SubElement(root, "gui_only")
    ET.SubElement(gui, "gui-settings-file", value="gui.view.xml")
    write_xml(OUT / "skku_commute.sumocfg", root)
    view = ET.Element("viewsettings")
    scheme = ET.SubElement(view, "scheme", name="SKKU campus commute")
    ET.SubElement(scheme, "vehicles", vehicleMode="0", vehicleQuality="2", vehicle_minSize="0",
                  vehicle_exaggeration="1", vehicle_constantSize="false", showBlinker="true")
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
    fig = plt.figure(figsize=(12, 13), facecolor="#f7f8fa")
    grid = fig.add_gridspec(3, 2, width_ratios=[1.3, 1], wspace=.10, hspace=.22)
    main = fig.add_subplot(grid[:, 0])
    axes = [main] + [fig.add_subplot(grid[i, 1]) for i in range(3)]
    normal = [e.getShape() for e in net.getEdges() if not e.getFunction()]
    urban = [e.getShape() for e in path if e.getType() not in ("highway.motorway", "highway.trunk")]
    express = [e.getShape() for e in path if e.getType() in ("highway.motorway", "highway.trunk")]
    for ax in axes:
        ax.add_collection(LineCollection(normal, colors="#c8d0d8", linewidths=.35))
        ax.add_collection(LineCollection(urban, colors="#147ac2", linewidths=2))
        ax.add_collection(LineCollection(express, colors="#eb7934", linewidths=2.3))
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
    fig.text(.12, .045, "Blue: urban / access roads   Orange: motorway / trunk\n© OpenStreetMap contributors · Road geometry, not live traffic · Synthetic signal plans", fontsize=9)
    fig.savefig(OUT / "network.png", dpi=160, bbox_inches="tight")
    plt.close(fig)


def main():
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
    traffic = make_demand(net, path, route, args.segment_vph)
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
                                 "Traffic light programs, ramp lane guesses and demand are synthetic",
                                 "Public roads outside main gates; no campus internal driving",
                                 "SUMO IDM ego uses built-in safety and automatic lane changes; no PPO model attached",
                                 "OSRM identifies the download corridor; SUMO computes its own legal fastest route"],
                    files_sha256={name: sha256(OUT / name) for name in ["skku_commute.net.xml", "skku_commute.typ.xml", "skku_commute.rou.xml", "skku_commute.sumocfg", "route.json", "gui.view.xml", "campuses.add.xml", "build.log"]})
    write_json(OUT / "manifest.json", manifest)
    print(json.dumps({"output": str(OUT), "network": manifest["network"], "route": manifest["route"],
                      "background_flows": len(traffic)}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
