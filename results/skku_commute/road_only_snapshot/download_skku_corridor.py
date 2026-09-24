"""Archive an OSM road corridor between SKKU's Seoul and Suwon public gates.

This downloads source data only. Road conversion is handled by
build_skku_commute.py. Successful Overpass tiles are cached and validated so
interrupted runs can resume without repeating the whole corridor download.
"""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime, timezone
import gzip
import hashlib
import json
import math
from pathlib import Path
import time
import urllib.error
import urllib.parse
import urllib.request
import xml.etree.ElementTree as ET

from pyproj import Transformer


ROOT = Path(__file__).resolve().parent
SOURCE = ROOT / "env" / "skku_commute" / "source"
OVERPASS = "https://overpass-api.de/api/interpreter"
OSRM = "https://router.project-osrm.org/route/v1/driving/"
START = (126.9972919, 37.5847826)
DESTINATION = (126.9738000, 37.2906472)
ROAD_TYPES = ("motorway", "motorway_link", "trunk", "trunk_link", "primary",
              "primary_link", "secondary", "secondary_link", "tertiary",
              "tertiary_link", "unclassified", "residential", "living_street",
              "service", "road")
USER_AGENT = "SKKU-SUMO-corridor-research/1.0"


def now():
    return datetime.now(timezone.utc).isoformat()


def sha(data):
    return hashlib.sha256(data).hexdigest()


def save_json(path, value):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + ".part")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)


def request_bytes(url, *, data=None, timeout=100, maximum=32 * 1024 * 1024):
    headers = {"User-Agent": USER_AGENT, "Accept-Encoding": "gzip"}
    if data is not None:
        headers["Content-Type"] = "application/x-www-form-urlencoded"
    request = urllib.request.Request(url, data=data, headers=headers)
    with urllib.request.urlopen(request, timeout=timeout) as response:
        body = response.read(maximum + 1)
        if len(body) > maximum:
            raise ValueError("Response exceeds the per-request archive limit")
        if response.headers.get("Content-Encoding") == "gzip":
            body = gzip.decompress(body)
        if len(body) > maximum:
            raise ValueError("Decompressed response exceeds the per-request archive limit")
        return body


def route_source(refresh=False):
    path = SOURCE / "route.json"
    metadata_path = SOURCE / "route_source.json"
    coords = ";".join(",".join(map(str, pair)) for pair in (START, DESTINATION))
    url = OSRM + coords + "?" + urllib.parse.urlencode(
        dict(alternatives="true", overview="full", geometries="geojson", steps="true"))
    if path.exists() and metadata_path.exists() and not refresh:
        metadata = json.loads(metadata_path.read_text())
        payload = path.read_bytes()
        if metadata.get("request_url") == url and metadata.get("sha256") == sha(payload):
            print("[route] reuse verified OSRM archive", flush=True)
            return json.loads(payload), metadata
    print(f"[route] request {url}", flush=True)
    body = request_bytes(url, timeout=60)
    route = json.loads(body)
    if route.get("code") != "Ok" or not route.get("routes"):
        raise ValueError(f"OSRM did not return a route: {route.get('code')}")
    if len(route["routes"][0]["geometry"]["coordinates"]) < 2:
        raise ValueError("OSRM returned an empty route geometry")
    temporary = path.with_suffix(".json.part")
    temporary.write_bytes(body)
    temporary.replace(path)
    metadata = dict(request_url=url, downloaded_at_utc=now(), sha256=sha(body),
                    selected_route_index=0, endpoints_lon_lat=[START, DESTINATION],
                    policy="First OSRM driving route; motorways permitted. No routing exclusions.")
    save_json(metadata_path, metadata)
    print(f"[route] {route['routes'][0]['distance'] / 1000:.2f} km; "
          f"{len(route['routes'])} returned alternatives", flush=True)
    return route, metadata


def route_tiles(coordinates, segment_meters, buffer_meters):
    project = Transformer.from_crs(4326, 32652, always_xy=True)
    inverse = Transformer.from_crs(32652, 4326, always_xy=True)
    projected = [project.transform(*p) for p in coordinates]
    chunks, current, distance = [], [projected[0]], 0.0
    for previous, point in zip(projected, projected[1:]):
        # Long edges are interpolated so no tile spans an unbounded road segment.
        length = math.dist(previous, point)
        count = max(1, math.ceil(length / 300.0))
        for index in range(1, count + 1):
            part = tuple(previous[k] + (point[k] - previous[k]) * index / count for k in (0, 1))
            distance += math.dist(current[-1], part)
            current.append(part)
            if distance >= segment_meters:
                chunks.append(current)
                current, distance = [part], 0.0
    if len(current) > 1:
        chunks.append(current)
    result = []
    for index, chunk in enumerate(chunks):
        xmin = min(p[0] for p in chunk) - buffer_meters
        xmax = max(p[0] for p in chunk) + buffer_meters
        ymin = min(p[1] for p in chunk) - buffer_meters
        ymax = max(p[1] for p in chunk) + buffer_meters
        corners = [inverse.transform(x, y) for x in (xmin, xmax) for y in (ymin, ymax)]
        bbox = (min(p[0] for p in corners), min(p[1] for p in corners),
                max(p[0] for p in corners), max(p[1] for p in corners))
        result.append((f"tile_{index:03d}", tuple(round(v, 7) for v in bbox)))
    return result


def query_for(bbox):
    west, south, east, north = bbox
    types = "|".join(ROAD_TYPES)
    return ("[out:xml][timeout:75][maxsize:268435456];\n"
            f'way["highway"~"^({types})$"]({south},{west},{north},{east})->.roads;\n'
            '(.roads;rel(bw.roads)["type"="restriction"];);\n'
            "(._;>>;);\nout meta;\n")


def inspect_osm(body):
    root = ET.fromstring(body)
    if root.tag != "osm":
        raise ValueError("Response is not an OSM XML archive")
    for tag in ("remark", "error"):
        error = root.find(tag)
        if error is not None:
            raise ValueError(f"Overpass {tag}: {''.join(error.itertext())[:400]}")
    counts = {kind: len(root.findall(kind)) for kind in ("node", "way", "relation")}
    if not counts["way"] or not counts["node"]:
        raise ValueError(f"Empty road tile: {counts}")
    meta = root.find("meta")
    return counts, dict(meta.attrib) if meta is not None else {}


def split_bbox(bbox):
    west, south, east, north = bbox
    width = (east - west) * math.cos(math.radians((south + north) / 2))
    if width > north - south:
        middle = (west + east) / 2
        return [(west, south, middle, north), (middle, south, east, north)]
    middle = (south + north) / 2
    return [(west, south, east, middle), (west, middle, east, north)]


def download_tile(name, bbox, refresh=False, depth=0):
    folder = SOURCE / "tiles"
    path = folder / f"{name}.osm.xml"
    metadata_path = folder / f"{name}.json"
    query = query_for(bbox)
    if path.exists() and metadata_path.exists() and not refresh:
        metadata = json.loads(metadata_path.read_text())
        body = path.read_bytes()
        if metadata.get("query") == query and metadata.get("sha256") == sha(body):
            inspect_osm(body)
            print(f"[tile] cached {name} {len(body) / 1e6:.2f} MB", flush=True)
            return [metadata]
    if not refresh and not path.exists():
        # Reuse a completed adaptive subtree, including after an interrupted run.
        children_present = [any(folder.glob(f"{name}_{index}*.osm.xml")) for index in (0, 1)]
        if all(children_present):
            children = []
            for index, child in enumerate(split_bbox(bbox)):
                children.extend(download_tile(f"{name}_{index}", tuple(round(v, 7) for v in child),
                                              False, depth + 1))
            return children
    (folder / f"{name}.query.txt").write_text(query, encoding="utf-8")
    print(f"[tile] request {name} bbox={bbox}", flush=True)
    started = time.monotonic()
    try:
        for attempt in range(5):
            try:
                body = request_bytes(OVERPASS, data=urllib.parse.urlencode({"data": query}).encode(), timeout=95)
                break
            except urllib.error.HTTPError as error:
                if error.code != 429 or attempt == 4:
                    raise
                delay = min(60, 10 * (2 ** attempt))
                print(f"[tile] rate limited {name}; retry same request in {delay}s", flush=True)
                time.sleep(delay)
            except urllib.error.URLError as error:
                if attempt >= 2:
                    raise RuntimeError(f"Overpass transport unavailable at {OVERPASS}: {error}") from error
                delay = 10 * (attempt + 1)
                print(f"[tile] transport error {name}; retry same request in {delay}s: {error}", flush=True)
                time.sleep(delay)
        counts, osm_metadata = inspect_osm(body)
    except Exception as error:
        print(f"[tile] failed {name}: {type(error).__name__}: {error}", flush=True)
        if isinstance(error, urllib.error.HTTPError) and error.code == 429:
            raise RuntimeError(f"Overpass rate limit persists for {name}; rerun later to resume caches") from error
        if isinstance(error, RuntimeError) and "transport unavailable" in str(error):
            raise
        if depth >= 3:
            raise RuntimeError(f"Tile {name} failed after adaptive splits") from error
        # Split a dense/slow tile instead of repeatedly submitting a large query.
        print(f"[tile] split {name} into two smaller boxes", flush=True)
        children = []
        time.sleep(2)
        for index, child in enumerate(split_bbox(bbox)):
            child = tuple(round(v, 7) for v in child)
            children.extend(download_tile(f"{name}_{index}", child, refresh, depth + 1))
        return children
    temporary = path.with_suffix(".xml.part")
    temporary.write_bytes(body)
    temporary.replace(path)
    metadata = dict(name=name, path=str(path.relative_to(SOURCE)), request_url=OVERPASS,
                    method="POST", query=query, bbox_west_south_east_north=bbox,
                    downloaded_at_utc=now(), osm_metadata=osm_metadata, counts=counts,
                    bytes=len(body), sha256=sha(body), elapsed_seconds=time.monotonic() - started)
    save_json(metadata_path, metadata)
    print(f"[tile] saved {name}: {counts['way']} ways, {counts['node']} nodes, "
          f"{len(body) / 1e6:.2f} MB in {metadata['elapsed_seconds']:.1f}s", flush=True)
    time.sleep(3)  # Leave room for the public server's per-user cooldown.
    return [metadata]


def merge_tiles(tiles):
    elements = {kind: {} for kind in ("node", "way", "relation")}
    duplicates = replacements = 0
    for tile in sorted(tiles, key=lambda t: t["name"]):
        for _, element in ET.iterparse(SOURCE / tile["path"], events=("end",)):
            if element.tag not in elements:
                continue
            kind, key = element.tag, int(element.attrib["id"])
            version = int(element.attrib.get("version", 0))
            existing = elements[kind].get(key)
            if existing:
                duplicates += 1
            if existing is None or version > existing[0]:
                replacements += int(existing is not None)
                elements[kind][key] = (version, ET.tostring(element, encoding="utf-8"))
            element.clear()
    # Ensure full-way closure before producing an archive that netconvert reads.
    missing_nodes = set()
    for _, body in elements["way"].values():
        way = ET.fromstring(body)
        missing_nodes.update(int(nd.attrib["ref"]) for nd in way.findall("nd")
                             if int(nd.attrib["ref"]) not in elements["node"])
    if missing_nodes:
        raise RuntimeError(f"Merged OSM has {len(missing_nodes)} missing way nodes")
    target = SOURCE / "merged.osm.xml"
    temporary = target.with_suffix(".xml.part")
    with temporary.open("wb") as output:
        output.write(b'<?xml version="1.0" encoding="UTF-8"?>\n')
        output.write(b'<osm version="0.6" generator="SKKU-corridor-source-archive" '
                     b'copyright="OpenStreetMap contributors" '
                     b'attribution="https://www.openstreetmap.org/copyright" '
                     b'license="https://opendatacommons.org/licenses/odbl/1-0/">\n')
        for kind in ("node", "way", "relation"):
            for key in sorted(elements[kind]):
                output.write(elements[kind][key][1])
                output.write(b"\n")
        output.write(b"</osm>\n")
    temporary.replace(target)
    counts = {kind: len(values) for kind, values in elements.items()}
    print(f"[merge] {counts}; removed {duplicates} duplicates; "
          f"{target.stat().st_size / 1e6:.2f} MB", flush=True)
    return dict(path="merged.osm.xml", sha256=sha(target.read_bytes()), bytes=target.stat().st_size,
                counts=counts, duplicate_elements=duplicates, higher_version_replacements=replacements,
                missing_way_nodes=0, duplicate_policy="Highest OSM version for each (type, id).")


def main():
    global OVERPASS
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--refresh", action="store_true", help="Replace cached requests explicitly")
    parser.add_argument("--segment-meters", type=float, default=3000)
    parser.add_argument("--buffer-meters", type=float, default=650)
    parser.add_argument("--workers", type=int, choices=(1, 2), default=1)
    parser.add_argument("--overpass-url", default=OVERPASS,
                        help="Overpass interpreter endpoint; each tile records its actual source")
    parser.add_argument("--maximum-source-mb", type=float, default=256)
    args = parser.parse_args()
    OVERPASS = args.overpass_url
    if not 500 <= args.segment_meters <= 5000 or not 300 <= args.buffer_meters <= 1500:
        parser.error("Use segment 500..5000m and buffer 300..1500m")
    SOURCE.mkdir(parents=True, exist_ok=True)
    (SOURCE / "tiles").mkdir(exist_ok=True)
    route, route_metadata = route_source(args.refresh)
    plans = route_tiles(route["routes"][0]["geometry"]["coordinates"],
                        args.segment_meters, args.buffer_meters)
    save_json(SOURCE / "tile_plan.json", dict(segment_meters=args.segment_meters,
              buffer_meters=args.buffer_meters, tiles=[dict(name=n, bbox=b) for n, b in plans]))
    print(f"[plan] {len(plans)} route tiles; {args.workers} concurrent requests maximum", flush=True)
    tiles = []
    if args.workers == 1:
        for name, bbox in plans:
            tiles.extend(download_tile(name, bbox, args.refresh))
            if sum(tile["bytes"] for tile in tiles) > args.maximum_source_mb * 1024 * 1024:
                raise RuntimeError("Downloaded road source exceeds configured disk budget")
    else:
        pool = ThreadPoolExecutor(max_workers=args.workers)
        futures = {pool.submit(download_tile, name, bbox, args.refresh): name for name, bbox in plans}
        try:
            for future in as_completed(futures):
                tiles.extend(future.result())
                if sum(tile["bytes"] for tile in tiles) > args.maximum_source_mb * 1024 * 1024:
                    raise RuntimeError("Downloaded road source exceeds configured disk budget")
        finally:
            pool.shutdown(wait=True, cancel_futures=True)
    merged = merge_tiles(tiles)
    manifest = dict(
        created_at_utc=now(), selected_route_index=0, route_source=route_metadata,
        osrm_route_url=route_metadata["request_url"],
        route_distance_m=route["routes"][0]["distance"], route_duration_s=route["routes"][0]["duration"],
        start=dict(name="SKKU Seoul public-road gate approach", lon_lat=START, osm_way_id=37401405,
                   official_gate_anchor_lon_lat=(126.997131, 37.584865)),
        destination=dict(name="SKKU Suwon public-road gate approach", lon_lat=DESTINATION,
                         osm_way_id=561229641, official_gate_anchor_lon_lat=(126.974074, 37.290649)),
        official_gate_sources=[
            "https://www.skku.edu/skku/about/campusInfo/campusMap.do?mode=buildList&campusCd=1",
            "https://www.skku.edu/skku/about/campusInfo/campusMap.do?mode=buildList&campusCd=2"],
        bbox_order="west,south,east,north", segment_meters=args.segment_meters,
        buffer_meters=args.buffer_meters, query_road_types=ROAD_TYPES,
        preserves="Full highway ways; restriction relations and their member ways/nodes; all original tags.",
        tiles=sorted(tiles, key=lambda t: t["name"]), merged=merged,
        sha256={"route.json": sha((SOURCE / "route.json").read_bytes()), "merged.osm.xml": merged["sha256"]},
        copyright="© OpenStreetMap contributors", license="ODbL 1.0",
        license_url="https://www.openstreetmap.org/copyright",
        assumptions=["The first OSRM driving alternative is used; motorways are permitted.",
                     "Endpoints are on public roads immediately outside the university gates, not inside campus.",
                     "Route-aligned overlapping bounding boxes approximate a corridor; full retained ways may extend beyond it.",
                     "Public routing duration is an estimate, not a measured journey time.",
                     "OSM tile timestamps are archived individually; separate downloads are not a transactional snapshot."])
    save_json(SOURCE / "manifest.json", manifest)
    print(f"[complete] {SOURCE}; {len(tiles)} archived tiles; route + merged OSM + manifest", flush=True)


if __name__ == "__main__":
    main()
