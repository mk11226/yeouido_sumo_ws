"""Build a reproducible, OSM-derived Hyehwa SUMO experiment.

Geometry/one-way/lane tags come from the archived OSM snapshot. Missing speed
limits, signal plans and traffic demand are explicit simulation assumptions,
not measurements of Seoul traffic. See env/hyehwa/manifest.json.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import urllib.request
from datetime import datetime, timezone
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "env" / "hyehwa"
SOURCE_BBOX = (126.994, 37.580, 127.010, 37.594)
CORE_BBOX = (126.997, 37.5824, 127.006, 37.5891)
CENTER = (127.0012, 37.58565)


def write_json(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def write_xml(path, root):
    ET.indent(root)
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


def binary(name):
    local = Path(sys.executable).parent / name
    result = str(local) if local.exists() else shutil.which(name)
    if not result:
        raise RuntimeError(f"{name} not found. Install requirements-hyehwa.txt in .venv-hyehwa.")
    return result


def download(refresh=False):
    target = OUT / "hyehwa.osm.xml"
    if target.exists() and not refresh:
        metadata = json.loads((OUT / "source.json").read_text())
        if hashlib.sha256(target.read_bytes()).hexdigest() != metadata["sha256"]:
            raise RuntimeError("OSM snapshot hash differs from source.json")
        return metadata
    url = "https://api.openstreetmap.org/api/0.6/map?bbox=" + ",".join(map(str, SOURCE_BBOX))
    request = urllib.request.Request(url, headers={"User-Agent": "HyehwaSUMOCourseProject/1.0"})
    data = urllib.request.urlopen(request, timeout=60).read()
    if ET.fromstring(data).tag != "osm":
        raise RuntimeError("Download is not an OSM XML document")
    target.write_bytes(data)
    meta = dict(source_url=url, bbox_west_south_east_north=SOURCE_BBOX,
                downloaded_at_utc=datetime.now(timezone.utc).isoformat(),
                sha256=hashlib.sha256(data).hexdigest(),
                copyright="© OpenStreetMap contributors", license="ODbL 1.0",
                license_url="https://www.openstreetmap.org/copyright")
    write_json(OUT / "source.json", meta)
    return meta


def convert():
    import sumolib
    types = ET.Element("types")
    for roadtype, speed, lanes, priority in [
        ("primary", 13.89, 2, 13), ("secondary", 13.89, 2, 12),
        ("tertiary", 8.33, 1, 11),
    ]:
        for suffix in ("", "_link"):
            ET.SubElement(types, "type", id=f"highway.{roadtype}{suffix}",
                          priority=str(priority), numLanes=str(1 if suffix else lanes),
                          speed=str(speed), oneway="false", allow="passenger bus taxi delivery emergency")
    write_xml(OUT / "hyehwa.typ.xml", types)
    stage = OUT / "hyehwa.import.net.xml"
    cmd = [binary("netconvert"), "--osm-files", str(OUT / "hyehwa.osm.xml"),
           "--type-files", str(OUT / "hyehwa.typ.xml"), "--output-file", str(stage),
           "--keep-edges.in-geo-boundary", ",".join(map(str, CORE_BBOX)),
           "--keep-edges.by-type", ",".join(t.get("id") for t in types),
           "--keep-edges.by-vclass", "passenger", "--geometry.remove",
           "--junctions.join", "--roundabouts.guess", "--no-turnarounds",
           "--junctions.corner-detail", "5", "--output.street-names",
           "--keep-edges.components", "1"]
    commands = [cmd]
    logs = []
    result = subprocess.run(cmd, text=True, capture_output=True, check=True)
    logs.append(result.stdout + result.stderr)
    net = sumolib.net.readNet(str(stage))
    # Three nodes form the central non-standard junction, plus the southwest
    # Seonggyungwan-ro junction. OSM does not encode their signal programs.
    signals = []
    for node in net.getNodes():
        lon, lat = net.convertXY2LonLat(*node.getCoord())
        central = 127.0006 < lon < 127.0017 and 37.5849 < lat < 37.5863
        southwest = 126.9980 < lon < 126.9995 and 37.5825 < lat < 37.5840
        if (central or southwest) and len(node.getIncoming()) >= 2 and len(node.getOutgoing()) >= 2:
            signals.append(node.getID())
    if len(signals) < 3:
        raise RuntimeError(f"Unexpected junction topology: only {signals} signal candidates")
    cmd = [binary("netconvert"), "--sumo-net-file", str(stage),
           "--output-file", str(OUT / "hyehwa.net.xml"),
           "--tls.set", ",".join(signals), "--tls.join", "--tls.join-dist", "120",
           "--tls.uncontrolled-within",
           "--tls.default-type", "static", "--tls.cycle.time", "75",
           "--no-turnarounds"]
    commands.append(cmd)
    result = subprocess.run(cmd, text=True, capture_output=True, check=True)
    logs.append(result.stdout + result.stderr)
    # No repeated stop bars within the same joined controller: vehicles that
    # entered on green clear the interior under normal SUMO junction rules.
    (OUT / "build.log").write_text("\n".join(logs))
    return sumolib.net.readNet(str(OUT / "hyehwa.net.xml")), commands


def routes_and_traffic(net, vehicles_per_hour=90.):
    # IDs refer to the archived snapshot. Fail on changed data rather than
    # silently train on a different junction after --refresh.
    ports = {
        "south": ("1455534425", "-1455534430"),
        "southwest": ("26299045#0", "1455534439#1"),
        "north": ("219073855", "-1455534441"),
        "northeast": ("37400220#1", "1069458463"),
    }
    pairs = [("southwest", "northeast"), ("northeast", "southwest"),
             ("south", "northeast"), ("northeast", "south"),
             ("north", "south"), ("south", "north"),
             ("north", "northeast"), ("northeast", "north")]
    route_dict, details = {}, []
    for origin, destination in pairs:
        path, cost = net.getShortestPath(net.getEdge(ports[origin][0]),
                                        net.getEdge(ports[destination][1]), vClass="passenger")
        if path is None:
            raise RuntimeError(f"No legal passenger route: {origin} -> {destination}")
        route_id = f"{origin}_to_{destination}"
        route_dict[route_id] = [e.getID() for e in path]
        for a, b in zip(path, path[1:]):
            if not any(c.getFromLane().allows("passenger") and c.getToLane().allows("passenger")
                       for c in a.getConnections(b)):
                raise RuntimeError(f"No legal lane connection: {a.getID()} -> {b.getID()}")
        # Every task must cross the central Hyehwa junction, not just its vicinity.
        central_nodes = {e.getToNode().getID() for e in path}
        if not any("1939144085" in node for node in central_nodes):
            raise RuntimeError(f"Route {route_id} misses Hyehwa center")
        details.append(dict(id=route_id, origin=origin, destination=destination,
                            edge_ids=route_dict[route_id], length_m=round(sum(e.getLength() for e in path), 2)))
    write_json(OUT / "routes.json", route_dict)
    root = ET.Element("routes")
    distribution = ET.SubElement(root, "vTypeDistribution", id="traffic_mix")
    for name, model, probability, color in [
        ("hv_idm", "IDM", "0.6", "0.3,0.6,0.9"),
        ("hv_krauss", "Krauss", "0.4", "0.9,0.7,0.2"),
    ]:
        ET.SubElement(distribution, "vType", id=name, probability=probability,
                      vClass="passenger", carFollowModel=model, accel="2.0", decel="4.5",
                      emergencyDecel="9", tau="1.2", minGap="2.5", length="4.8",
                      speedFactor="normc(0.95,0.08,0.7,1.1)", color=color)
    for route_id, edges in route_dict.items():
        ET.SubElement(root, "route", id="bg_" + route_id, edges=" ".join(edges))
    for i, route_id in enumerate(route_dict):
        ET.SubElement(root, "flow", id="flow_" + route_id, type="traffic_mix",
                      route="bg_" + route_id, begin=str(i * 0.25), end="3600",
                      probability=str(vehicles_per_hour / 3600), departLane="best", departSpeed="max")
    write_xml(OUT / "traffic.rou.xml", root)
    return details


def configuration():
    root = ET.Element("configuration")
    inputs = ET.SubElement(root, "input")
    ET.SubElement(inputs, "net-file", value="hyehwa.net.xml")
    ET.SubElement(inputs, "route-files", value="traffic.rou.xml")
    time = ET.SubElement(root, "time")
    ET.SubElement(time, "step-length", value="0.5")
    processing = ET.SubElement(root, "processing")
    ET.SubElement(processing, "time-to-teleport", value="-1")
    ET.SubElement(processing, "collision.action", value="remove")
    ET.SubElement(processing, "collision.check-junctions", value="true")
    report = ET.SubElement(root, "report")
    ET.SubElement(report, "no-step-log", value="true")
    ET.SubElement(report, "duration-log.disable", value="true")
    write_xml(OUT / "hyehwa.sumocfg", root)


def plot_network(net, route_details):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
    fig, ax = plt.subplots(figsize=(10, 10))
    shapes = [e.getShape() for e in net.getEdges()]
    widths = [0.7 + 0.5 * e.getLaneNumber() for e in net.getEdges()]
    ax.add_collection(LineCollection(shapes, colors="#c7cdd4", linewidths=widths))
    colors = ["#2166ac", "#b2182b", "#1b7837", "#762a83", "#e08214", "#008b8b", "#884422", "#cc44aa"]
    for route, color in zip(route_details, colors):
        points = [p for eid in route["edge_ids"] for p in net.getEdge(eid).getShape()]
        xs, ys = zip(*points)
        ax.plot(xs, ys, color=color, linewidth=1.2, alpha=0.7, label=route["id"].replace("_", " "))
        start = points[0]
        ax.scatter(*start, s=25, color=color, zorder=5)
    for node in net.getNodes():
        if node.getType().startswith("traffic_light"):
            ax.scatter(*node.getCoord(), color="black", marker="s", s=24, zorder=6)
    cx, cy = net.convertLonLat2XY(*CENTER)
    ax.annotate("Hyehwa junction", (cx, cy), xytext=(cx + 120, cy - 80),
                arrowprops={"arrowstyle": "->"}, fontsize=12)
    ax.autoscale(); ax.set_aspect("equal")
    ax.set(xlabel="SUMO local x (m)", ylabel="SUMO local y (m)", title="Hyehwa: archived OSM roads and 8 evaluation routes")
    ax.legend(loc="upper left", fontsize=8)
    fig.text(0.1, 0.015, "Road geometry: © OpenStreetMap contributors (ODbL). Signals, speeds and demand: experimental assumptions.", fontsize=8)
    fig.tight_layout(rect=(0, .03, 1, 1))
    fig.savefig(OUT / "network.png", dpi=160)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--refresh", action="store_true", help="Explicitly replace archived source snapshot")
    parser.add_argument("--vehicles-per-hour-per-route", type=float, default=90.)
    args = parser.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    if not 0 < args.vehicles_per_hour_per_route <= 3600:
        parser.error("traffic demand must be in (0, 3600]")
    source = download(args.refresh)
    net, commands = convert()
    routes = routes_and_traffic(net, args.vehicles_per_hour_per_route)
    configuration()
    plot_network(net, routes)
    files = ["hyehwa.osm.xml", "hyehwa.net.xml", "traffic.rou.xml", "routes.json", "hyehwa.sumocfg"]
    manifest = dict(name="Hyehwa junction, Seoul", center_lon_lat=CENTER,
                    core_bbox=CORE_BBOX, source=source,
                    build_time_utc=datetime.now(timezone.utc).isoformat(),
                    sumo_version=subprocess.check_output([binary("sumo"), "--version"], text=True).splitlines()[0],
                    external_edges=len(net.getEdges()), junctions=len(net.getNodes()),
                    traffic_light_controllers=len(net.getTrafficLights()),
                    signal_cycle_seconds={t.get("id"): sum(float(p.get("duration")) for p in t.findall("phase"))
                                          for t in ET.parse(OUT / "hyehwa.net.xml").getroot().findall("tlLogic")},
                    routes=routes, expected_background_vehicles_per_hour=len(routes) * args.vehicles_per_hour_per_route,
                    assumptions=[
                        "OSM geometry and one-way/lane tags retained; incomplete tags are not a surveyed lane-level digital twin.",
                        "Primary/secondary speed fallback 50 km/h; tertiary 30 km/h. Not verified legal speed limits.",
                        "Signals configured from junction topology; generated 75s programs, internal joined-controller links uncontrolled. Not measured Seoul timing.",
                        "Only largest connected principal-road component retained; minor alleys, pedestrian/bicycle traffic and bus operations excluded.",
                        "Background demand is synthetic: independent probabilistic flows, 60% IDM / 40% Krauss passenger vehicles.",
                        "Routes are preassigned; the RL policy controls acceleration and lane changes, not navigation or steering.",
                    ],
                    commands=commands,
                    sha256={f: hashlib.sha256((OUT / f).read_bytes()).hexdigest() for f in files})
    write_json(OUT / "manifest.json", manifest)
    (OUT / "ATTRIBUTION.md").write_text(
        "# Data attribution\n\nRoad data © OpenStreetMap contributors, licensed under ODbL 1.0.\n"
        "https://www.openstreetmap.org/copyright\n\nThe archived OSM snapshot and the derived SUMO road database retain this attribution.\n"
        "See source.json for acquisition URL/time/hash and manifest.json for all modeling assumptions.\n", encoding="utf-8")
    print(json.dumps({k: manifest[k] for k in ["external_edges", "junctions", "traffic_light_controllers", "expected_background_vehicles_per_hour"]}, indent=2))
    for route in routes:
        print(route["id"], route["length_m"], "m")


if __name__ == "__main__":
    main()
