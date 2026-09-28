"""Download existing OSM building footprints along the archived SKKU road corridor.

Only env/skku_commute/buildings_source is written. Building ways and building
multipolygon relations retain original members, inner rings, tags and nodes.
Successful empty tiles are cached too. The existing road source is read-only.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import gzip
import hashlib
import json
import math
from pathlib import Path
import socket
import time
import urllib.error
import urllib.parse
import urllib.request
import xml.etree.ElementTree as ET


from Road_Environment.paths import ROOT
ROAD_SOURCE = ROOT / "env" / "skku_commute" / "source"
OUTPUT = ROOT / "env" / "skku_commute" / "buildings_source"
SERVER = "https://osm.hpi.de/overpass/api/interpreter"


def now():
    return datetime.now(timezone.utc).isoformat()


def digest(data):
    return hashlib.sha256(data).hexdigest()


def save_json(path, value):
    temporary = path.with_suffix(path.suffix + ".part")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)


class QueryTimeout(RuntimeError):
    """A query was too slow; this is the only reason to subdivide a tile."""


def building_query(bbox):
    west, south, east, north = bbox
    bounds = f"({south},{west},{north},{east})"
    return ('[out:xml][timeout:60][maxsize:268435456];\n(\n'
            f'  way["building"]["building"!="no"]{bounds};\n'
            f'  relation["type"="multipolygon"]["building"]["building"!="no"]{bounds};\n'
            ');\n(._;>>;);\nout meta;\n')


def parse_response(body):
    root = ET.fromstring(body)
    if root.tag != "osm":
        raise ValueError("Response is not OSM XML")
    for kind in ("remark", "error"):
        element = root.find(kind)
        if element is not None:
            message = "".join(element.itertext())
            if "timed out" in message.lower() or "timeout" in message.lower():
                raise QueryTimeout(message[:500])
            raise ValueError(f"Overpass {kind}: {message[:500]}")
    meta = root.find("meta")
    return ({kind: len(root.findall(kind)) for kind in ("node", "way", "relation")},
            dict(meta.attrib) if meta is not None else {})


def request_tile(server, query):
    payload = urllib.parse.urlencode({"data": query}).encode()
    maximum = 96 * 1024 * 1024
    for attempt in range(5):
        request = urllib.request.Request(
            server, data=payload,
            headers={"User-Agent": "SKKU-SUMO-building-footprint-research/1.0",
                     "Accept-Encoding": "gzip", "Content-Type": "application/x-www-form-urlencoded"})
        try:
            with urllib.request.urlopen(request, timeout=85) as response:
                body = response.read(maximum + 1)
                if len(body) > maximum:
                    raise ValueError("Building tile exceeds 96 MiB response limit")
                if response.headers.get("Content-Encoding") == "gzip":
                    body = gzip.decompress(body)
                if len(body) > maximum:
                    raise ValueError("Building tile exceeds 96 MiB decompressed limit")
                return body
        except urllib.error.HTTPError as error:
            if error.code in (408, 504):
                raise QueryTimeout(f"HTTP {error.code}") from error
            if error.code not in (429, 500, 502, 503) or attempt == 4:
                raise
            delay = min(60, 10 * (2 ** attempt))
            print(f"[retry] HTTP {error.code}; same query after {delay}s", flush=True)
            time.sleep(delay)
        except urllib.error.URLError as error:
            if attempt >= 2:
                raise RuntimeError(f"Transport unavailable at {server}: {error}") from error
            delay = 10 * (attempt + 1)
            print(f"[retry] transport error; same query after {delay}s: {error}", flush=True)
            time.sleep(delay)
        except (TimeoutError, socket.timeout) as error:
            if attempt >= 2:
                raise RuntimeError(f"Transport timed out at {server}: {error}") from error
            delay = 10 * (attempt + 1)
            print(f"[retry] transport timeout; same query after {delay}s", flush=True)
            time.sleep(delay)
    raise RuntimeError("Unreachable retry exhaustion")


def split_bbox(bbox):
    west, south, east, north = bbox
    if (east - west) * math.cos(math.radians((north + south) / 2)) > north - south:
        middle = round((west + east) / 2, 7)
        return [(west, south, middle, north), (middle, south, east, north)]
    middle = round((south + north) / 2, 7)
    return [(west, south, east, middle), (west, middle, east, north)]


def download(name, bbox, server, refresh=False, depth=0):
    folder = OUTPUT / "tiles"
    path, meta_path = folder / f"{name}.osm.xml", folder / f"{name}.json"
    query = building_query(bbox)
    if path.exists() and meta_path.exists() and not refresh:
        metadata = json.loads(meta_path.read_text())
        body = path.read_bytes()
        if metadata.get("query") == query and metadata.get("sha256") == digest(body):
            parse_response(body)
            print(f"[cache] {name}: {len(body) / 1e6:.2f} MB", flush=True)
            return [metadata]
    subdivision = folder / f"{name}.split.json"
    if subdivision.exists() and not refresh:
        plan = json.loads(subdivision.read_text())
        if plan.get("query") == query:
            result = []
            for child in plan["children"]:
                result.extend(download(child["name"], child["bbox"], server, False, depth + 1))
            return result
    (folder / f"{name}.query.txt").write_text(query, encoding="utf-8")
    print(f"[download] {name}: {bbox}", flush=True)
    start = time.monotonic()
    try:
        body = request_tile(server, query)
        counts, osm_meta = parse_response(body)
    except QueryTimeout as error:
        if depth >= 4:
            raise RuntimeError(f"Tile {name} still times out after four subdivisions") from error
        children = [dict(name=f"{name}_{i}", bbox=box) for i, box in enumerate(split_bbox(bbox))]
        save_json(subdivision, dict(query=query, reason=str(error), children=children))
        print(f"[split] {name}: query timeout; two smaller tiles", flush=True)
        time.sleep(3)
        result = []
        for child in children:
            result.extend(download(child["name"], child["bbox"], server, refresh, depth + 1))
        return result
    temporary = path.with_suffix(".xml.part")
    temporary.write_bytes(body)
    temporary.replace(path)
    metadata = dict(name=name, path=str(path.relative_to(OUTPUT)), request_url=server,
                    method="POST", query=query, bbox_west_south_east_north=bbox,
                    downloaded_at_utc=now(), osm_metadata=osm_meta, bytes=len(body),
                    sha256=digest(body), counts=counts, elapsed_seconds=time.monotonic() - start,
                    empty=not any(counts.values()))
    save_json(meta_path, metadata)
    print(f"[saved] {name}: {counts}, {len(body) / 1e6:.2f} MB, "
          f"{metadata['elapsed_seconds']:.1f}s", flush=True)
    time.sleep(3)
    return [metadata]


def merge(tiles):
    elements = {kind: {} for kind in ("node", "way", "relation")}
    duplicates = replacements = 0
    for tile in sorted(tiles, key=lambda item: item["name"]):
        for _, element in ET.iterparse(OUTPUT / tile["path"], events=("end",)):
            if element.tag not in elements:
                continue
            kind, ident = element.tag, int(element.attrib["id"])
            version = int(element.attrib.get("version", 0))
            old = elements[kind].get(ident)
            if old is not None:
                duplicates += 1
            if old is None or version > old[0]:
                elements[kind][ident] = (version, ET.tostring(element, encoding="utf-8"))
                replacements += int(old is not None)
            element.clear()
    missing_nodes, missing_members = [], []
    selected, multipolygons, inner_members = [], 0, 0
    unclosed = []
    for kind in ("way", "relation"):
        for ident, (_, body) in elements[kind].items():
            element = ET.fromstring(body)
            tags = {tag.attrib["k"]: tag.attrib["v"] for tag in element.findall("tag")}
            if kind == "way":
                refs = [int(nd.attrib["ref"]) for nd in element.findall("nd")]
                missing_nodes.extend((ident, ref) for ref in refs if ref not in elements["node"])
                if tags.get("building") and tags["building"] != "no" and (len(refs) < 4 or refs[0] != refs[-1]):
                    unclosed.append(ident)
            else:
                for member in element.findall("member"):
                    target_kind, target = member.attrib["type"], int(member.attrib["ref"])
                    if target not in elements[target_kind]:
                        missing_members.append((ident, target_kind, target))
                if tags.get("building") and tags["building"] != "no" and tags.get("type") == "multipolygon":
                    multipolygons += 1
                    inner_members += sum(member.attrib.get("role") == "inner" for member in element.findall("member"))
            if tags.get("building") and tags["building"] != "no":
                selected.append((kind, ident, tags))
    if missing_nodes or missing_members:
        raise RuntimeError(f"Incomplete building geometry: {len(missing_nodes)} way nodes and "
                           f"{len(missing_members)} relation members missing")
    target = OUTPUT / "merged.osm.xml"
    temporary = target.with_suffix(".xml.part")
    with temporary.open("wb") as stream:
        stream.write(b'<?xml version="1.0" encoding="UTF-8"?>\n')
        stream.write(b'<osm version="0.6" generator="SKKU-building-source-archive" '
                     b'copyright="OpenStreetMap contributors" '
                     b'attribution="https://www.openstreetmap.org/copyright" '
                     b'license="https://opendatacommons.org/licenses/odbl/1-0/">\n')
        for kind in ("node", "way", "relation"):
            for ident in sorted(elements[kind]):
                stream.write(elements[kind][ident][1])
                stream.write(b"\n")
        stream.write(b"</osm>\n")
    temporary.replace(target)
    stats = dict(
        counts={kind: len(items) for kind, items in elements.items()},
        building_tagged_objects=len(selected),
        building_tagged_ways=sum(kind == "way" for kind, _, _ in selected),
        building_multipolygon_relations=multipolygons,
        inner_ring_members_preserved=inner_members,
        objects_with_height=sum(bool(tags.get("height")) for _, _, tags in selected),
        objects_with_building_levels=sum(bool(tags.get("building:levels")) for _, _, tags in selected),
        objects_with_name=sum(any(tags.get(key) for key in ("name", "name:ko", "name:en")) for _, _, tags in selected),
        unclosed_tagged_building_ways=sorted(unclosed),
        missing_way_nodes=0, missing_relation_members=0,
        duplicate_elements=duplicates, higher_version_replacements=replacements)
    print(f"[merged] {len(selected)} building-tagged objects; {target.stat().st_size / 1e6:.2f} MB", flush=True)
    return dict(path="merged.osm.xml", bytes=target.stat().st_size,
                sha256=digest(target.read_bytes()), statistics=stats)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--refresh", action="store_true")
    parser.add_argument("--overpass-url", default=SERVER)
    parser.add_argument("--maximum-source-mb", type=float, default=350)
    args = parser.parse_args()
    tile_plan_path, route_path = ROAD_SOURCE / "tile_plan.json", ROAD_SOURCE / "route.json"
    plan = json.loads(tile_plan_path.read_text())
    route = json.loads(route_path.read_text())
    OUTPUT.mkdir(parents=True, exist_ok=True)
    (OUTPUT / "tiles").mkdir(exist_ok=True)
    print(f"[plan] {len(plan['tiles'])} existing road-corridor tiles; building geometry only; sequential", flush=True)
    tiles = []
    for tile in plan["tiles"]:
        tiles.extend(download(tile["name"], tile["bbox"], args.overpass_url, args.refresh))
        if sum(item["bytes"] for item in tiles) > args.maximum_source_mb * 1024 * 1024:
            raise RuntimeError("Building source exceeds configured disk budget; completed tiles are cached")
    merged = merge(tiles)
    manifest = dict(
        created_at_utc=now(), purpose="Existing OSM building footprints around the Seoul-to-Suwon SUMO corridor",
        corridor_route_distance_m=route["routes"][0]["distance"],
        corridor_source_files={"source/tile_plan.json": digest(tile_plan_path.read_bytes()),
                               "source/route.json": digest(route_path.read_bytes())},
        road_source_modified=False, selection="building-tagged ways and building multipolygon relations, excluding building=no",
        recursion="Downward members only (>>); no upward parent-relation recursion.",
        original_tile_plan=plan, tiles=sorted(tiles, key=lambda item: item["name"]), merged=merged,
        sha256={"merged.osm.xml": merged["sha256"]},
        copyright="© OpenStreetMap contributors", license="ODbL 1.0",
        license_url="https://www.openstreetmap.org/copyright",
        assumptions=[
            "Footprints and tags are archived OSM data, not synthetic buildings; unmapped buildings remain absent.",
            "Original node coordinates, way references and multipolygon inner/outer member roles are preserved.",
            "building=no is excluded from footprint selection; referenced members needed for a selected relation remain intact.",
            "Missing height, levels or names are not invented in this source archive.",
            "Road-corridor bounding boxes include surrounding neighborhoods; complete buildings may extend outside them.",
            "Overpass source timestamps are recorded per tile; this archive does not assert a simultaneous real-world survey.",
            "Object statistics count tagged ways and relations, not guaranteed unique physical buildings."])
    save_json(OUTPUT / "manifest.json", manifest)
    print(json.dumps(merged["statistics"], ensure_ascii=False, indent=2), flush=True)
    print(f"[complete] {OUTPUT}", flush=True)


if __name__ == "__main__":
    main()
