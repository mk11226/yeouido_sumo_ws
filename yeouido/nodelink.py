"""국가표준노드링크(국토교통부) 전국 shapefile → 우리 범위의 노드·링크·회전제한 (region.json).

    python -m yeouido.nodelink

원본: 국가교통정보센터 표준노드링크 (env/yeouido/source/nodelink/raw/)
  MOCT_LINK  : 링크 = 한 방향 도로 구간. 차로 수, 도로 등급·유형, 제한속도, 도로명
  MOCT_NODE  : 노드 = 교차로·도로 시종점·시설물 변환점 등
  TURNINFO   : 교차로별 회전 제한 (좌회전 금지, U턴 등)
좌표는 중부원점 TM(GRS80) → 위경도로 바꿔 저장한다.

코드 정의 (지능형교통체계 표준노드·링크 구축기준, 국토교통부고시 제2023-22호)
"""
import json

from yeouido.config import BBOX, SOURCE, south_of_river

RAW = SOURCE / "nodelink" / "raw"
REGION = SOURCE / "nodelink" / "region.json"
MARGIN_M = 300   # 범위보다 조금 넓게 읽고, 정확한 자르기는 build_net 단계에서 한다

ROAD_RANK = {"101": "고속국도", "102": "도시고속국도", "103": "일반국도", "104": "특별광역시도",
             "105": "국가지원지방도", "106": "지방도", "107": "시군도", "108": "기타"}
ROAD_TYPE = {"000": "일반도로", "001": "고가차도", "002": "지하차도", "003": "교량", "004": "터널"}
NODE_TYPE = {"101": "교차로", "102": "도로시종점", "103": "속성변화점", "104": "도로시설물",
             "105": "행정경계", "106": "연결로접속부", "107": "최소노드배치점", "109": "기타"}
TURN_TYPE = {"001": "비보호회전", "002": "버스만회전", "003": "회전금지", "011": "U-TURN",
             "012": "P-TURN", "101": "좌회전금지", "102": "직진금지", "103": "우회전금지"}


def _transformers():
    from pyproj import CRS, Transformer
    tm = CRS.from_wkt((RAW / "MOCT_LINK.prj").read_text())
    to_tm = Transformer.from_crs("EPSG:4326", tm, always_xy=True)
    to_geo = Transformer.from_crs(tm, "EPSG:4326", always_xy=True)
    return to_tm, to_geo


def main():
    import shapefile
    if not (RAW / "MOCT_LINK.shp").exists():
        raise RuntimeError(
            f"표준노드링크 원본이 없습니다: {RAW}\n"
            "  국가교통정보센터(its.go.kr) → 교통데이터 → 표준노드링크에서 최신 zip을 받아\n"
            "  위 폴더에 압축을 풀어 주세요 (MOCT_LINK.*, MOCT_NODE.*, TURNINFO.dbf).")
    to_tm, to_geo = _transformers()
    b = BBOX
    xs, ys = zip(*[to_tm.transform(lon, lat) for lon in (b["west"], b["east"])
                   for lat in (b["south"], b["north"])])
    box = [min(xs) - MARGIN_M, min(ys) - MARGIN_M, max(xs) + MARGIN_M, max(ys) + MARGIN_M]

    def geo(points):
        return [[round(v, 7) for v in to_geo.transform(x, y)] for x, y in points]

    links = []
    with shapefile.Reader(str(RAW / "MOCT_LINK"), encoding="cp949") as r:
        names = [f[0] for f in r.fields[1:]]
        for sr in r.iterShapeRecords(bbox=box):
            rec = dict(zip(names, sr.record))
            if rec["ROAD_USE"] != "0":          # 0 = 사용, 1 = 미사용
                continue
            pts = geo(sr.shape.points)
            # 한강 북쪽(마포구)만 지나는 링크는 버린다. 다리는 남쪽 끝이 범위 안이라 남는다
            if not any(south_of_river(lon, lat) for lon, lat in pts):
                continue
            links.append(dict(
                id=rec["LINK_ID"], from_node=rec["F_NODE"], to_node=rec["T_NODE"],
                lanes=int(rec["LANES"] or 0), rank=rec["ROAD_RANK"], type=rec["ROAD_TYPE"],
                name=rec["ROAD_NAME"], connect=rec["CONNECT"] == "1",
                max_speed=int(rec["MAX_SPD"] or 0), length=round(float(rec["LENGTH"] or 0), 1),
                shape=pts))
    used = {n for l in links for n in (l["from_node"], l["to_node"])}

    nodes = {}
    with shapefile.Reader(str(RAW / "MOCT_NODE"), encoding="cp949") as r:
        names = [f[0] for f in r.fields[1:]]
        for sr in r.iterShapeRecords(bbox=box):
            rec = dict(zip(names, sr.record))
            if rec["NODE_ID"] in used:
                (lon, lat), = geo(sr.shape.points)
                nodes[rec["NODE_ID"]] = dict(type=rec["NODE_TYPE"], name=rec["NODE_NAME"],
                                             lon=lon, lat=lat)

    turns = []
    with shapefile.Reader(dbf=open(RAW / "TURNINFO.dbf", "rb"), encoding="cp949") as r:
        names = [f[0] for f in r.fields[1:]]
        for rec in r.iterRecords():
            rec = dict(zip(names, rec))
            if rec["NODE_ID"] in nodes:
                turns.append(dict(node=rec["NODE_ID"], from_link=rec["ST_LINK"],
                                  to_link=rec["ED_LINK"], type=rec["TURN_TYPE"],
                                  all_day=rec["TURN_OPER"] == "0"))

    REGION.write_text(json.dumps(dict(links=links, nodes=nodes, turns=turns), ensure_ascii=False))
    print(f"[nodelink] 링크 {len(links)}개, 노드 {len(nodes)}개, 회전제한 {len(turns)}개 → {REGION}")


if __name__ == "__main__":
    main()
