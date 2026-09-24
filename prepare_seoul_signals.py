"""Normalize archived Seoul signal mounts near the existing SUMO commute route.

Reads the official CSV, definition PDF metadata and ZIP PRJ/SHP/DBF. Writes only
signals_source/seoul/{normalized_signals,audit}.json. No live-status, individual
lamp positions, traffic headings, phase timings or missing facilities are inferred.
"""
from __future__ import annotations

import argparse
from collections import Counter
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re
import struct
import xml.etree.ElementTree as ET
import zipfile

import numpy as np
from pyproj import CRS, Transformer


ROOT = Path(__file__).resolve().parent
ENV = ROOT / "env" / "skku_commute"
SOURCE = ENV / "signals_source" / "seoul"
SOURCE_PAGE = "https://data.seoul.go.kr/dataList/OA-15546/F/1/datasetView.do"
TYPE_LABELS = {
    "001": "측주식횡형경보(2)", "002": "3색등", "003": "4색등", "004": "2색등(종형)",
    "005": "3색등(종형)", "006": "4색등(종형)", "007": "보행등", "008": "가변형가변등",
    "009": "차량보조등2색", "010": "차량보조등3색", "011": "신호유도2색",
    "012": "시선유도형경보(2)", "013": "시선유도형경보(1)", "014": "머릿돌경보(2)",
    "015": "톨게이트등", "016": "측주식횡형경보(3)", "017": "머릿돌경보(3)",
    "018": "시선유도형경보(3)", "019": "측주식종형경보(2)", "020": "측주식종형경보(3)",
    "021": "2색등", "022": "버스전용3색등", "023": "자전거 신호 종형 3색등",
    "024": "자전거 신호 종형 2색등",
}
STATUS_LABELS = {"001": "양호", "002": "파손", "003": "도색", "004": "노후"}
MOUNT_LABELS = {"001": "현수식", "002": "측주식종형", "003": "측주식횡형", "004": "중앙식", "005": "문형식"}
WARNING_CODES = {"001", "012", "013", "014", "016", "017", "018", "019", "020"}
GENERAL_VEHICLE_CODES = {"002", "003", "005", "006"}
OTHER_VEHICLE_CODES = {"004", "008", "009", "010", "011", "015", "021", "022"}


def sha(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def write_json(path, value):
    temporary = path.with_suffix(".json.part")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    temporary.replace(path)


def number(value):
    try:
        result = float(value)
        return result if math.isfinite(result) else None
    except (ValueError, TypeError):
        return None


def kind(code):
    if code == "007":
        return "pedestrian"
    if code in WARNING_CODES:
        return "warning"
    if code in GENERAL_VEHICLE_CODES | OTHER_VEHICLE_CODES:
        return "vehicle"
    return "unknown"


def percentiles(values):
    if not len(values):
        return None
    return dict(zip(("p50", "p90", "p99", "maximum"), map(float, np.percentile(values, [50, 90, 99, 100]))))


def verify_archive(rows):
    """Use matching IDs and actual SHP points to link the older PRJ to the CSV."""
    current = {row["부착대관리번호"]: row for row in rows}
    changes, shape_differences = [], []
    with zipfile.ZipFile(SOURCE / "mounted_signals_20210809.zip") as archive:
        wkt = archive.read("A057_L.prj").decode("utf-8")
        with archive.open("A057_L.dbf") as dbf, archive.open("A057_L.shp") as shp:
            header = dbf.read(32)
            count = struct.unpack_from("<I", header, 4)[0]
            header_size, record_size = struct.unpack_from("<HH", header, 8)
            field_bytes = dbf.read(header_size - 32)
            fields, offset = {}, 1
            for pos in range(0, len(field_bytes) - 1, 32):
                descriptor = field_bytes[pos:pos + 32]
                if descriptor[0] == 13:
                    break
                name = descriptor[:11].split(b"\0")[0].decode()
                fields[name] = (offset, descriptor[16])
                offset += descriptor[16]
            shape_header = shp.read(100)
            if struct.unpack_from("<i", shape_header, 32)[0] != 1:
                raise RuntimeError("Expected official archive point geometry")
            for _ in range(count):
                record = dbf.read(record_size)
                shape_record = shp.read(8)
                if len(record) != record_size or len(shape_record) != 8:
                    raise RuntimeError("Truncated SHP/DBF archive")
                content = shp.read(struct.unpack_from(">i", shape_record, 4)[0] * 2)
                def field(name):
                    start, length = fields[name]
                    return record[start:start + length].decode("euc-kr", errors="replace").strip()
                x, y = number(field("XCE")), number(field("YCE"))
                if x is None or y is None:
                    continue
                if struct.unpack_from("<i", content, 0)[0] == 1:
                    shape_x, shape_y = struct.unpack_from("<dd", content, 4)
                    shape_differences.append(math.hypot(x - shape_x, y - shape_y))
                newer = current.get(field("MGRNU"))
                if newer:
                    nx, ny = number(newer["X좌표"]), number(newer["Y좌표"])
                    if nx is not None and ny is not None:
                        changes.append(math.hypot(nx - x, ny - y))
    if not changes or np.median(changes) > 1 or max(shape_differences, default=math.inf) > 0.001:
        raise RuntimeError("CSV/ZIP coordinate provenance check failed; do not assume a CRS")
    crs = CRS.from_wkt(wkt)
    to_lonlat = Transformer.from_crs(crs, "EPSG:4326", always_xy=True)
    epsg_transform = Transformer.from_crs("EPSG:5186", "EPSG:4326", always_xy=True)
    check_xy = [(179594.75, 537233.22), (200000, 550000), (216156.54, 565846.56)]
    equivalent_error = max(max(abs(a - b) for a, b in zip(to_lonlat.transform(x, y), epsg_transform.transform(x, y)))
                           for x, y in check_xy)
    if equivalent_error > 1e-9:
        raise RuntimeError("PRJ differs from official EPSG:5186 metadata")
    return crs, dict(
        prj_archive_member="mounted_signals_20210809.zip!A057_L.prj", prj_wkt=wkt,
        epsg_operation_equivalent="EPSG:5186", epsg_equivalence_max_error_degrees=equivalent_error,
        archive_point_records=count, shp_dbf_compared=len(shape_differences),
        shp_dbf_maximum_difference_m=max(shape_differences), csv_zip_common_ids=len(changes),
        csv_zip_coordinate_change_m=percentiles(changes), csv_zip_within_1m=sum(v <= 1 for v in changes),
        note="The CSV is newer than the ZIP. Changed facility coordinates are retained from the CSV; no snapping or correction is applied.")


def route_segments(crs):
    route_path, net_path = ENV / "route.json", ENV / "skku_commute.net.xml"
    route = json.loads(route_path.read_text())
    wanted, shapes, location = set(route["edge_ids"]), {}, None
    for _, element in ET.iterparse(net_path, events=("end",)):
        if element.tag == "location":
            location = dict(element.attrib)
        elif element.tag == "edge":
            if element.get("id") in wanted:
                lane = element.find("lane")
                if lane is None:
                    raise RuntimeError("Route edge without a lane")
                shapes[element.get("id")] = [tuple(map(float, point.split(","))) for point in lane.get("shape").split()]
            element.clear()
    if wanted - shapes.keys() or not location:
        raise RuntimeError("Route geometry is missing from the SUMO network")
    offset = np.array(list(map(float, location["netOffset"].split(","))))
    transform = Transformer.from_crs(CRS.from_user_input(location["projParameter"]), crs, always_xy=True)
    starts, ends, ids = [], [], []
    for ident in route["edge_ids"]:
        points = np.array(shapes[ident]) - offset
        points = np.column_stack(transform.transform(points[:, 0], points[:, 1]))
        for start, end in zip(points[:-1], points[1:]):
            if np.linalg.norm(end - start) > 1e-6:
                starts.append(start); ends.append(end); ids.append(ident)
    return np.array(starts), np.array(ends), ids


def nearest_segments(points, starts, ends):
    vector = ends - starts
    lengths = np.sum(vector * vector, axis=1)
    distances, indexes = [], []
    for offset in range(0, len(points), 128):
        delta = points[offset:offset + 128, None, :] - starts[None, :, :]
        fraction = np.clip(np.sum(delta * vector[None, :, :], axis=2) / lengths, 0, 1)
        squared = np.sum((delta - fraction[:, :, None] * vector) ** 2, axis=2)
        nearest = np.argmin(squared, axis=1)
        indexes.extend(nearest.tolist())
        distances.extend(np.sqrt(squared[np.arange(len(nearest)), nearest]).tolist())
    return np.array(distances), indexes


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--buffer-m", type=float, default=150)
    args = parser.parse_args()
    if not 0 < args.buffer_m <= 1000:
        parser.error("--buffer-m must be positive and at most 1000 m")
    with (SOURCE / "mounted_signals.csv").open(encoding="cp949", newline="") as stream:
        rows = list(csv.DictReader(stream))
    crs, coordinate_audit = verify_archive(rows)
    to_lonlat = Transformer.from_crs(crs, "EPSG:4326", always_xy=True)
    from_lonlat = Transformer.from_crs("EPSG:4326", crs, always_xy=True)
    starts, ends, edge_ids = route_segments(crs)
    minimum, maximum = np.minimum(starts.min(axis=0), ends.min(axis=0)) - args.buffer_m, np.maximum(starts.max(axis=0), ends.max(axis=0)) + args.buffer_m
    nearby, valid, invalid = [], [], []
    for row in rows:
        x, y = number(row["X좌표"]), number(row["Y좌표"])
        if x is None or y is None or not (100000 < x < 300000 and 400000 < y < 700000):
            invalid.append(row["부착대관리번호"])
            continue
        valid.append((row, x, y))
        if minimum[0] <= x <= maximum[0] and minimum[1] <= y <= maximum[1]:
            nearby.append((row, x, y))
    distances, indexes = nearest_segments(np.array([[x, y] for _, x, y in nearby]), starts, ends)
    records, direction_differences = [], []
    for (row, x, y), distance, segment in zip(nearby, distances, indexes):
        if distance > args.buffer_m:
            continue
        lon, lat = to_lonlat.transform(x, y)
        code, status = row["신호등종류"].strip(), row["상태"].strip()
        direction = number(row["부착대방향"])
        front_quantity = number(row["신호등수량"])
        geometry = re.search(r"SDO_ORDINATE_ARRAY\(([^)]+)\)", row["공간데이터"])
        mount_geometry = None
        if geometry:
            ordinates = [float(value) for value in geometry.group(1).split(",")]
            if len(ordinates) % 2 == 0:
                mount_geometry = [list(to_lonlat.transform(*ordinates[i:i + 2])) for i in range(0, len(ordinates), 2)]
                if len(ordinates) >= 4 and direction is not None:
                    dx, dy = ordinates[2] - ordinates[0], ordinates[3] - ordinates[1]
                    if math.hypot(dx, dy) > 0.1:
                        math_angle = math.degrees(math.atan2(dy, dx)) % 360
                        direction_differences.append(abs((math_angle - direction + 180) % 360 - 180))
        records.append(dict(
            id=row["부착대관리번호"], lon=lon, lat=lat, signal_kind=kind(code), type_code=code,
            type_label=TYPE_LABELS.get(code, "미정의 코드"), status=status, status_label=STATUS_LABELS.get(status, "미기재/미정의"),
            direction_deg=direction, direction_semantics="Source mount orientation; not a vehicle approach heading",
            eligible_vehicle_signal=code in GENERAL_VEHICLE_CODES and front_quantity is not None and front_quantity > 0,
            eligible_vehicle_3_or_4_color=code in GENERAL_VEHICLE_CODES,
            operational_status_known=False, mount_type_code=row["부착방식"],
            mount_type_label=MOUNT_LABELS.get(row["부착방식"], "미기재/미정의"),
            front_signal_quantity=front_quantity, rear_signal_quantity=number(row["배면등수량"]),
            rear_type_code=row["배면등종류"], rear_type_label=TYPE_LABELS.get(row["배면등종류"], "미기재/미정의"),
            pole_id=row["지주관리번호"], display_code=row["표출구분"], construction_code=row["공사형태"],
            work_code=row["작업구분"], installation_date_raw=row["설치일"], replacement_date_raw=row["교체일"],
            x_source=x, y_source=y, mount_geometry_lon_lat=mount_geometry,
            distance_to_route_m=float(distance), nearest_route_edge_id=edge_ids[segment],
            source="Seoul OA-15546 signal mount CSV", source_url=SOURCE_PAGE,
            coordinate_represents="Official signal-mount reference point, not each individual signal head",
            coordinate_confidence="Provider PRJ verified against SHP/DBF and matching CSV IDs; original coordinates retained"))
    records.sort(key=lambda item: item["id"])
    # Independently compare nearby 2024 T-data intersection centers with these physical mounts.
    with (SOURCE / "v2xCrossroadMapInformation_20241114.csv").open(encoding="utf-8-sig", newline="") as stream:
        centers = list(csv.DictReader(stream))
    centers_xy = np.array([from_lonlat.transform(float(c["mapCtptIntLot"]), float(c["mapCtptIntLat"])) for c in centers])
    center_distances, _ = nearest_segments(centers_xy, starts, ends)
    vehicle = [(row, x, y) for row, x, y in valid
               if row["신호등종류"] in GENERAL_VEHICLE_CODES and (number(row["신호등수량"]) or 0) > 0]
    vehicle_xy = np.array([[x, y] for _, x, y in vehicle])
    comparisons = []
    for center, xy, distance in zip(centers, centers_xy, center_distances):
        if distance > args.buffer_m:
            continue
        nearby_distances = np.linalg.norm(vehicle_xy - xy, axis=1)
        nearest = int(np.argmin(nearby_distances))
        comparisons.append(dict(intersection_id=center["itstId"], name=center["itstNm"],
                                lon=float(center["mapCtptIntLot"]), lat=float(center["mapCtptIntLat"]),
                                distance_to_route_m=float(distance), nearest_vehicle_mount_id=vehicle[nearest][0]["부착대관리번호"],
                                nearest_vehicle_mount_distance_m=float(nearby_distances[nearest])))
    source_manifest = json.loads((SOURCE / "mounted_signals_manifest.json").read_text())
    source_paths = [SOURCE / name for name in ("mounted_signals.csv", "mounted_signals_definition.pdf", "mounted_signals_20210809.zip", "v2xCrossroadMapInformation_20241114.csv")]
    hashes = {str(path.relative_to(ROOT)): sha(path) for path in source_paths + [ENV / "route.json", ENV / "skku_commute.net.xml"]}
    metadata = dict(
        created_at_utc=datetime.now(timezone.utc).isoformat(), source_url=SOURCE_PAGE,
        provider="서울특별시 교통운영과", license=source_manifest["license"], attribution_required=True,
        file_listing_date="2023-06-23", definition_listing_date="2022-11-15", coordinate_archive_listing_date="2021-08-09",
        listing_date_is_not_field_survey_date=True, downloaded_at_utc=source_manifest["downloaded_at_utc"],
        source_encoding="CP949", source_crs="Korea_2000_Korea_Central_Belt_2010 (official ZIP PRJ; operation equivalent to EPSG:5186)",
        target_crs="EPSG:4326, longitude/latitude", oracle_sdo_srid="2093 is the source Oracle identifier, not an EPSG code",
        coordinate_evidence=coordinate_audit, filter=dict(buffer_m=args.buffer_m, geometry="First SUMO lane shape of each actual route edge; minimum 2D projected distance", no_snapping=True),
        source_records=len(rows), selected_records=len(records), source_sha256=hashes,
        field_definitions_source="mounted_signals_definition.pdf, physical pages 2-6",
        type_code_labels=TYPE_LABELS, status_code_labels=STATUS_LABELS,
        eligible_vehicle_type_codes=sorted(GENERAL_VEHICLE_CODES),
        eligible_vehicle_filter="Ordinary 3/4-color front signal type and front_signal_quantity > 0; operational status remains unknown",
        limitations=[
            "Each record is a signal mount and may carry multiple front/back heads; records are not unique intersections or individual lamps.",
            "Only archived facilities within the route buffer are included. This does not assert complete, current, operational signal coverage.",
            "Status 001 means good physical condition, not active operational status; the source has no real-time operating state.",
            "Mount orientation is retained as provided and must not be treated as a vehicle approach heading.",
            "Vehicle eligibility selects ordinary 3/4-color types 002/003/005/006; warning, pedestrian, bus-only, auxiliary and bicycle types remain auditable.",
            "A nearby facility may control a cross street, opposing direction, or different road level. Network control assignment needs a separate audit.",
            "No traffic-light phases, timings, signal-group-to-lane mapping, lamp offsets or missing signals are invented."])
    comparison_distances = [c["nearest_vehicle_mount_distance_m"] for c in comparisons]
    audit = dict(
        source_records=len(rows), valid_coordinates=len(valid), invalid_coordinate_ids=invalid,
        source_type_counts=dict(Counter(r["신호등종류"] for r in rows)), source_status_counts=dict(Counter(r["상태"] for r in rows)),
        selected_records=len(records), selected_kind_counts=dict(Counter(r["signal_kind"] for r in records)),
        selected_type_counts=dict(Counter(r["type_code"] for r in records)), selected_status_counts=dict(Counter(r["status"] for r in records)),
        eligible_vehicle_records=sum(r["eligible_vehicle_signal"] for r in records),
        eligible_vehicle_type_records=sum(r["eligible_vehicle_3_or_4_color"] for r in records),
        selected_zero_front_quantity_ids=[r["id"] for r in records if r["front_signal_quantity"] == 0],
        selected_hidden_display_ids=[r["id"] for r in records if r["display_code"] == "001"],
        selected_removal_construction_ids=[r["id"] for r in records if r["construction_code"] == "003"],
        duplicate_source_ids=[ident for ident, count in Counter(row["부착대관리번호"] for row in rows).items() if count > 1],
        distance_to_route_m=percentiles([r["distance_to_route_m"] for r in records]),
        coordinate_evidence=coordinate_audit,
        direction_vs_mount_geometry_degrees=dict(compared=len(direction_differences), absolute_difference=percentiles(direction_differences),
            interpretation="Diagnostic only: raw mount direction often matches a mathematical angle from east; it is not a traffic heading."),
        tdata_cross_validation=dict(source_records=len(centers), route_buffer_m=args.buffer_m,
            nearby_centers=len(comparisons), nearest_general_vehicle_mount_distance_m=percentiles(comparison_distances),
            centers_with_mount_within_50m=sum(d <= 50 for d in comparison_distances),
            centers_with_mount_within_100m=sum(d <= 100 for d in comparison_distances), records=comparisons,
            note="2024 independent intersection-center coordinates compared with 2023 physical mounts. Separation is expected and does not prove approach ownership."))
    write_json(SOURCE / "normalized_signals.json", dict(records=records, metadata=metadata))
    write_json(SOURCE / "audit.json", audit)
    print(json.dumps({key: audit[key] for key in ("source_records", "valid_coordinates", "selected_records", "selected_kind_counts", "eligible_vehicle_records", "distance_to_route_m")}, ensure_ascii=False, indent=2))
    print(json.dumps({key: audit["tdata_cross_validation"][key] for key in ("nearby_centers", "nearest_general_vehicle_mount_distance_m", "centers_with_mount_within_50m", "centers_with_mount_within_100m")}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
