"""Normalize archived official Gwacheon/Uiwang signal datasets (offline).

Run from repository root with .venv-skku/bin/python. Coordinates are source
facility points, not inferred intersection centres. Unknown codes remain unknown.
"""
from collections import Counter
import csv
import hashlib
import json
from pathlib import Path

from pyproj import Transformer
import sumolib
from sumolib.geomhelper import polygonOffsetAndDistanceToPoint

HERE = Path(__file__).resolve().parent / "env/skku_commute/signals_source"
NET = HERE.parent / "skku_commute.net.xml"
ROUTE = HERE.parent / "route.json"
CODE_SOURCE = "https://gis.gjcity.go.kr/arcgis/rest/services/GJUIS/SGMapDynamic/MapServer/42"


def gis_kind(code):
    # Common UIS code definitions independently published by Gwangju city.
    # Unverified codes are deliberately left unknown.
    return {"SGB001": "pedestrian", "SGB002": "vehicle"}.get(code, "unknown")


def standard_kind(props):
    # Use the literal operating sequence, not an assumed numeric code meaning.
    sequence = props.get("SGNASP_ORDR", "") or ""
    if "점멸" in sequence and not any(x in sequence for x in ("녹색", "녹등", "초록")):
        return "warning"
    if ("녹색" in sequence or "녹등" in sequence) and "황색" in sequence and "적색" in sequence:
        return "vehicle"
    return "unknown"


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    net = sumolib.net.readNet(str(NET))
    shapes = [net.getEdge(e).getShape() for e in json.loads(ROUTE.read_text())["edge_ids"]]

    def distance(lon, lat):
        point = net.convertLonLat2XY(lon, lat)
        return round(min(polygonOffsetAndDistanceToPoint(point, s)[1] for s in shapes), 3)

    records = []
    for filename, kind in [("gwacheon_signal_heads.geojson", "unknown"),
                           ("gwacheon_signal_controllers.geojson", "controller")]:
        path = HERE / filename
        data = json.loads(path.read_text())
        for feature in data["features"]:
            props = {k: v.strip(" \x00") if isinstance(v, str) else v
                     for k, v in feature["properties"].items()}
            feature["properties"] = props
            lon, lat = feature["geometry"]["coordinates"]
            records.append({
                "id": f"gwacheon:{props['FTR_CDE']}:{props['FTR_IDN']}",
                "lon": lon, "lat": lat, "signal_kind": kind if kind == "controller" else gis_kind(props.get("SGB_CDE")),
                "kind_evidence": "controller layer" if kind == "controller" else "common UIS SGB_CDE domain from official Gwangju GIS; other codes unknown",
                "kind_code_source": CODE_SOURCE,
                "source_dataset": props["source_dataset"],
                "source_city": "과천시", "source_date": "2026-02-04",
                "source_crs": "EPSG:5186", "crs_evidence": "explicit official portal description",
                "route_distance_m": distance(lon, lat), "properties": props,
            })
        path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n")

    transformer = Transformer.from_crs(5186, 4326, always_xy=True)
    uiwang_features = []
    with (HERE / "uiwang_signals.csv").open(encoding="utf-8-sig", newline="") as source:
        for index, props in enumerate(csv.DictReader(source)):
            x, y = float(props["위도"]), float(props["경도"])
            lon, lat = transformer.transform(x, y)
            # Official metadata says TM central GRS80; the numeric field labels
            # are wrong. 5186 is inferred: 5181 yields latitude38.2, outside city.
            assert 126.90 < lon < 127.10 and 37.25 < lat < 37.45, (index, lon, lat)
            record = {
                "id": f"uiwang:{props['공간지리식별번호']}:{index}",
                "lon": lon, "lat": lat, "signal_kind": gis_kind(props.get("신호등제식")),
                "kind_evidence": "common UIS SGB_CDE domain from official Gwangju GIS; other codes unknown",
                "kind_code_source": CODE_SOURCE,
                "source_dataset": "https://www.data.go.kr/data/15108891/fileData.do",
                "source_city": "의왕시", "source_date": "2022-12-02",
                "source_crs": "EPSG:5186", "source_x": x, "source_y": y,
                "crs_evidence": "inferred from official TM central GRS80 description and Uiwang coordinate extent; false northing600000, not500000",
                "route_distance_m": distance(lon, lat), "properties": props,
            }
            records.append(record)
            uiwang_features.append({"type": "Feature", "geometry": {
                "type": "Point", "coordinates": [lon, lat]}, "properties": record})
    (HERE / "uiwang_signals.geojson").write_text(json.dumps({
        "type": "FeatureCollection", "features": uiwang_features}, ensure_ascii=False, indent=2) + "\n")
    national_files = sorted(HERE.glob("national_standard_page[1-5].json"))
    national_count = 0
    national_kept = []
    for path in national_files:
        rows = json.loads(path.read_text())
        national_count += len(rows)
        for index, props in enumerate(rows):
            if props.get("CTPRVN_NM") != "경기도" or props.get("SIGNGU_NM") not in ("수원시", "안양시", "과천시", "의왕시"):
                continue
            try:
                lon, lat = float(props["LONGITUDE"]), float(props["LATITUDE"])
            except (KeyError, ValueError, TypeError):
                continue
            assert 126.8 < lon < 127.2 and 37.1 < lat < 37.6, (path, index, lon, lat)
            record = {
                "id": f"national:{path.stem}:{index}", "lon": lon, "lat": lat,
                "signal_kind": standard_kind(props),
                "kind_evidence": "literal SGNASP_ORDR green/yellow/red cycle => vehicle; flash without green => warning; otherwise unknown",
                "source_dataset": "https://www.data.go.kr/data/15028198/standard.do",
                "source_city": props["SIGNGU_NM"], "source_date": props.get("REFERENCE_DATE"),
                "source_crs": "EPSG:4326", "crs_evidence": "official longitude/latitude decimal-degree fields",
                "route_distance_m": distance(lon, lat), "properties": props,
            }
            records.append(record)
            national_kept.append(record)
    (HERE / "national_corridor_city_records.json").write_text(json.dumps(national_kept, ensure_ascii=False, indent=2) + "\n")
    counts = dict(Counter(r["source_city"] for r in records))
    nearby = {str(threshold): dict(Counter(r["source_city"] for r in records
                                          if r["route_distance_m"] <= threshold))
              for threshold in (50, 100, 200)}
    data = {"records": records, "metadata": {
        "retrieved_date": "2026-09-23", "coordinate_crs": "EPSG:4326",
        "total_records": len(records), "records_by_city": counts,
        "records_within_route_distance_m": nearby,
        "route_reference": {"file": "../route.json", "sha256": sha(ROUTE),
                            "net_file": "../skku_commute.net.xml", "net_sha256": sha(NET)},
        "license": {"gwacheon": "이용허락범위 제한 없음", "uiwang": "이용허락범위 제한 없음",
                    "national_standard": "Public official standard-data download; separate licence text not found in archived detail page"},
        "signal_kinds": dict(Counter(r["signal_kind"] for r in records)),
        "kind_code_source": CODE_SOURCE,
        "coverage_limitations": [
            "Facility points can include pedestrian and vehicle signal heads; SGB001/002 mapped using shared municipal UIS domain; other codes remain unknown.",
            "Controller cabinets are labelled controller and must not be treated as exact stop-line or intersection-centre positions.",
            "Uiwang is a one-time2022 survey; the source explicitly notes missing observations.",
            "Gwacheon publication2026-02-04 does not imply each feature was resurveyed; many LOD_YMD values are20230217.",
            "National standard website downloads are capped at50000 records; this is not a complete national export.",
            "Current corridor coverage is not guaranteed; source dates and raw observations differ, and nearby points may belong to side roads.",
        ],
        "sources": [
            {"url": "https://www.data.go.kr/data/15129910/fileData.do",
             "title": "경기도 과천시_공간정보(교통시설물)_20260204",
             "raw_file": "gwacheon_transport_20260204.zip", "sha256": sha(HERE / "gwacheon_transport_20260204.zip"),
             "crs_status": "EPSG:5186 explicitly documented", "counts": {"signal_heads":388,"controllers":60}},
            {"url": "https://www.data.go.kr/data/15108891/fileData.do",
             "title": "경기도 의왕시_신호등 설치 현황(보행안전지수)_20221202",
             "raw_file": "uiwang_signals.csv", "sha256": sha(HERE / "uiwang_signals.csv"),
             "crs_status": "EPSG:5186 inferred; original 위도 field=easting and 경도 field=northing", "count":2533},
            {"url": "https://www.data.go.kr/data/15028198/standard.do",
             "title": "전국신호등표준데이터", "raw_downloaded_records": national_count,
             "selected_records": len(national_kept), "selected_city_counts": dict(Counter(r["source_city"] for r in national_kept)),
             "download_limit":50000, "raw_files":[{"file":p.name,"sha256":sha(p)} for p in national_files],
             "download_implementation": "national_std_download.js; public /download/columList.json and /download/standard.json website endpoints"},
        ],
    }}
    temporary = HERE / "gyeonggi_normalized.tmp.json"
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n")
    temporary.replace(HERE / "gyeonggi_normalized.json")
    print(json.dumps({"total_records": len(records), "counts": counts, "nearby": nearby}, ensure_ascii=False))


if __name__ == "__main__":
    main()
