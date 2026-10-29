"""1단계: 원본 데이터 다운로드 — TOPIS 교통량 엑셀 + OSM 도로 + OSM 주변 환경.

    python -m yeouido.download            # 없는 파일만 받는다
    python -m yeouido.download --refresh  # 전부 다시 받는다

TOPIS 자료실 웹페이지가 내부적으로 쓰는 요청을 그대로 재현한다.
(파일 목록 조회 → 파일 다운로드, 로그인·인증키 불필요)
"""
import argparse
import http.cookiejar
import json
import time
import urllib.error
import urllib.parse
import urllib.request

from yeouido.config import (BBOX, HIGHWAY_CLASSES, SOURCE, TOPIS_MONTHS,
                            TOPIS_YEAR)

TOPIS = "https://topis.seoul.go.kr"
# 공개 Overpass 서버는 자주 바쁘므로 여러 곳을 차례로 시도한다.
OVERPASS = [
    "https://overpass-api.de/api/interpreter",
    "https://overpass.kumi.systems/api/interpreter",
    "https://osm.hpi.de/overpass/api/interpreter",
]


def topis_path(month: int):
    return SOURCE / "topis" / f"{TOPIS_YEAR}_{month:02d}.xlsx"


def _opener():
    # TOPIS는 세션 쿠키가 있어야 목록 API가 응답한다.
    jar = http.cookiejar.CookieJar()
    opener = urllib.request.build_opener(urllib.request.HTTPCookieProcessor(jar))
    opener.open(f"{TOPIS}/refRoom/openRefRoom_2.do", timeout=30).read()
    return opener


def _post(opener, path, data, timeout=60):
    body = urllib.parse.urlencode(data).encode()
    return opener.open(f"{TOPIS}{path}", body, timeout=timeout).read()


def download_topis(refresh=False):
    todo = [m for m in TOPIS_MONTHS if refresh or not topis_path(m).exists()]
    if not todo:
        print("[topis] 모든 월 파일이 이미 있음")
        return
    opener = _opener()
    # blbdDivCd=08: "지점별 일자별 교통량" 게시판
    rows = json.loads(_post(opener, "/refroom/selectRefRoomListASC.do", {
        "blbdDivCd": "08", "bdwrDivCd": str(TOPIS_YEAR), "mainBdwrRowNum": "12",
    }))["rows"]
    files = {int(r["monthsTxt"].rstrip("월")): r for r in rows if r.get("apndFilePathNm")}
    topis_path(todo[0]).parent.mkdir(parents=True, exist_ok=True)
    for m in todo:
        if m not in files:
            raise RuntimeError(f"TOPIS에 {TOPIS_YEAR}년 {m}월 파일이 아직 없습니다")
        r = files[m]
        data = _post(opener, "/downloadFileRefRoom.do", {
            "apndFileNm": r["apndFileNm"], "apndFilePathNm": r["apndFilePathNm"],
            "bdwrSeq": r["bdwrSeq"], "blbdDivCd": r["blbdDivCd"],
        }, timeout=180)
        if not data.startswith(b"PK"):  # xlsx는 zip 형식
            raise RuntimeError(f"{m}월 파일이 엑셀이 아닙니다 (응답 {len(data)} bytes)")
        topis_path(m).write_bytes(data)
        print(f"[topis] {r['apndFileNm']} → {topis_path(m)}")


def osm_path():
    return SOURCE / "yeouido.osm.xml"


def context_path():
    return SOURCE / "context.osm.xml"


def _overpass(query, timeout=240):
    """공개 Overpass 서버들에 차례로 질의해 OSM XML을 돌려준다."""
    body = urllib.parse.urlencode({"data": query}).encode()
    errors = []
    for attempt in range(2):
        for server in OVERPASS:
            req = urllib.request.Request(server, body,
                                         headers={"User-Agent": "skku-av-course-project"})
            try:
                data = urllib.request.urlopen(req, timeout=timeout).read()
            except (urllib.error.URLError, TimeoutError) as e:
                errors.append(f"{server}: {e}")
                continue
            if b"<osm" in data[:500] and (b"<way" in data or b"<node" in data):
                return data
            errors.append(f"{server}: 비정상 응답 {data[:120]!r}")
        time.sleep(10)
    raise RuntimeError("Overpass 다운로드 실패:\n  " + "\n  ".join(errors))


def _fresh(path, refresh):
    """파일이 있고, 같은 범위(BBOX)로 받은 것이면 다시 받지 않는다."""
    stamp = path.with_suffix(".bbox.json")
    if refresh or not path.exists() or not stamp.exists():
        return False
    return json.loads(stamp.read_text()) == BBOX


def _save(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(data)
    path.with_suffix(".bbox.json").write_text(json.dumps(BBOX))
    print(f"[osm] {len(data) / 1e6:.1f} MB → {path}")


def _box():
    b = BBOX
    return f"{b['south']},{b['west']},{b['north']},{b['east']}"


def download_osm(refresh=False):
    if _fresh(osm_path(), refresh):
        print(f"[osm] 이미 있음: {osm_path()}")
        return
    # 도로(way)와 그 도로를 이루는 모든 점(node)을 받는다.
    # 범위 밖으로 이어지는 도로는 build_net 단계에서 범위에 맞게 자른다.
    box, classes = _box(), "|".join(HIGHWAY_CLASSES)
    _save(osm_path(), _overpass(
        f'[out:xml][timeout:180];'
        f'(way["highway"~"^({classes})$"]({box});'
        f' node["highway"="traffic_signals"]({box}););'
        f'(._;>;);out body;'))


def download_context(refresh=False):
    """도로 밖 주변 환경: 건물, 공원·녹지, 물(한강·샛강), 토지 이용, 이름 있는 장소.

    화면 표시(build_context)와 "어느 장소에서 어느 장소로" 목적지 지정(places)에 쓴다.
    주행·교통 시뮬레이션에는 영향이 없다.
    """
    if _fresh(context_path(), refresh):
        print(f"[osm] 이미 있음: {context_path()}")
        return
    box = _box()
    _save(context_path(), _overpass(
        f'[out:xml][timeout:300];('
        f'way["building"]({box});'
        f'way["landuse"]({box});relation["landuse"]({box});'
        f'way["leisure"]({box});relation["leisure"]({box});'
        f'way["natural"]({box});relation["natural"="water"]({box});'
        f'way["waterway"="riverbank"]({box});'
        f'way["amenity"]({box});'
        f'node["railway"="station"]({box});node["public_transport"="station"]({box});'
        f'node["amenity"]["name"]({box});node["tourism"]["name"]({box});'
        f'node["shop"="mall"]({box});node["office"]["name"]({box});'
        f');(._;>;);out body;', timeout=360))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--refresh", action="store_true", help="이미 받은 파일도 다시 받기")
    args = ap.parse_args(argv)
    download_topis(args.refresh)
    download_osm(args.refresh)
    download_context(args.refresh)


if __name__ == "__main__":
    main()
