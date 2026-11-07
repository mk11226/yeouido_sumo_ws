"""검수 도구를 로컬 웹 서버로 띄운다.

    python -m inspection.serve              # http://127.0.0.1:8765 를 브라우저로 연다
    python -m inspection.serve --port 9000 --no-browser

화면
  /              검수 카드 (cards.html): 왼쪽 SUMO 그림 · 오른쪽 항공사진을 나란히 놓고
                 교차로 카드 (진입로·차로 수·차로별 회전 방향·신호)와
                 도로 구간 카드 (차로 수·모양·앞뒤 연결)를 비교·기록
  /viewer.html   전체 지도 (항공사진 + SUMO 도로망 + 국가표준노드링크)

- VWorld API 키는 저장소 루트 .env 의 VWORLD_API_KEY 를 읽어 /config.js 로만 내보낸다
  (HTML 파일에 키를 적지 않으므로 커밋해도 키가 새지 않는다).
- inspection/out/ 의 자료가 없거나 도로망·노드링크보다 오래됐으면 먼저 다시 만든다.
- 수정 목록(yeouido/corrections.json) 반영본을 inspection/out/compare/new.net.xml 에 만들어, 카드 화면
  왼쪽에서 "원래 / 수정 후" 를 바꿔 볼 수 있다. 카드의 버튼·키로 차로 수, 회전 포켓, 차로별 방향을 고치면
  목록을 고치고 반영본을 바로 다시 만든다 (env/yeouido 도로망은 그대로).
- yeouido/corrections.json 을 편집기로 직접 고쳐 저장해도 1~2초 안에 알아채고 반영본을 다시 만든다
  (열려 있는 카드 화면도 자동으로 새로 그린다. JSON 오류·도로망 실패는 화면 위쪽 줄에 나온다).
- 검수 기록은 inspection/records/{junctions,roads}.json 에 저장한다 (팀 공유용, git 으로 관리).
- "문제"·"보류" 로 판정한 카드는 메모를 inspection/issues.md 에 따로 모아 적는다 (기록할 때마다 다시 씀).
- 이 PC 안(127.0.0.1)에서만 열린다.
"""
import argparse
import functools
import json
import os
import subprocess
import sys
import threading
import time
import webbrowser
from datetime import date
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from inspection import export_cards, export_net
from yeouido import build_net, corrections
from yeouido.config import NET, ROOT
from yeouido.nodelink import REGION

HERE = Path(__file__).resolve().parent
RECORDS = {kind: HERE / "records" / f"{kind}.json" for kind in ("junctions", "roads")}
NEW_NET = HERE / "out" / "compare" / "new.net.xml"     # yeouido/corrections.json 반영본
NEW_GEOJSON = HERE / "out" / "net_new.geojson"
ISSUES = HERE / "issues.md"       # 문제·보류 카드 메모 모음
_lock = threading.Lock()
# 수정 목록 상태 (화면이 /api/corrections/status 로 2초마다 본다). version 이 오르면 화면이 다시 불러온다.
_watch = {"version": 0, "state": "ok", "message": "", "skipped": [], "mtime": None}


def vworld_key():
    key = os.environ.get("VWORLD_API_KEY", "")
    env = ROOT / ".env"
    if not key and env.exists():
        for line in env.read_text().splitlines():
            name, _, value = line.partition("=")
            if name.strip() == "VWORLD_API_KEY":
                key = value.strip().strip("\"'")
    return key


def refresh():
    """도로망·노드링크·변환 코드가 결과물보다 새로우면 다시 만든다."""
    for module, result in ((export_net, export_net.GEOJSON), (export_cards, export_cards.CARDS)):
        sources = [NET, REGION, Path(module.__file__)]
        newest = max(p.stat().st_mtime for p in sources if p.exists())
        if not result.exists() or result.stat().st_mtime < newest:
            module.export()
    # 수정 후 도로망 → 왼쪽 화면 "수정 후" 전환용. 수정 목록이나 만드는 코드가 바뀌었으면 다시 만든다.
    sources = [corrections.CORRECTIONS, Path(corrections.__file__), Path(build_net.__file__), REGION]
    newest = max(p.stat().st_mtime for p in sources if p.exists())
    if any(corrections.load().values()) and (not NEW_NET.exists() or NEW_NET.stat().st_mtime < newest):
        print("[serve] 수정 목록 반영본 다시 만드는 중...")
        rebuild_fixed()
    elif NEW_NET.exists():
        newest = max(NEW_NET.stat().st_mtime, Path(export_net.__file__).stat().st_mtime)
        if not NEW_GEOJSON.exists() or NEW_GEOJSON.stat().st_mtime < newest:
            export_net.export(NEW_NET, NEW_GEOJSON)
    elif NEW_GEOJSON.exists():
        NEW_GEOJSON.unlink()


def rebuild_fixed():
    """yeouido/corrections.json 반영본을 inspection/out/compare/new.net.xml 에 만들고 화면용으로 바꾼다.
    반환: build_net 이 건너뛴 항목 (dict(type, edge, side|lane, why))."""
    r = subprocess.run([sys.executable, "-m", "yeouido.build_net", "--out", str(NEW_NET)],
                       capture_output=True, text=True, cwd=ROOT)
    if r.returncode:
        raise RuntimeError((r.stdout + r.stderr)[-2000:])
    export_net.export(NEW_NET, NEW_GEOJSON)
    return json.loads((NEW_NET.parent / f"{NEW_NET.stem}_plain" / "skipped.json").read_text())


def orphan_note():
    """도로 삭제 때문에 끊겨서 같이 빠진 도로 → 안내 문구 (없으면 빈 문자열)."""
    path = NEW_NET.parent / f"{NEW_NET.stem}_plain" / "removed.json"
    orphans = json.loads(path.read_text())["orphans"] if path.exists() else []
    if not orphans:
        return ""
    return f"삭제로 끊겨서 같이 빠진 도로 {len(orphans)}개: {', '.join(orphans[:6])}" + (" …" if len(orphans) > 6 else "")


def _key(kind, c):
    """수정 항목을 찾는 열쇠: 차로 수 = 도로, 포켓 = 도로·쪽, 차로 방향 = 도로·차로, 회전 제한 = 진입·나감."""
    if kind == "remove_turns":
        return (kind, c["from"], c["to"])
    return (kind, c["edge"], c.get("side", "left") if kind == "turn_pockets" else c.get("lane"))


def _mtime():
    return corrections.CORRECTIONS.stat().st_mtime_ns if corrections.CORRECTIONS.exists() else None


def _done(state="ok", message="", skipped=()):
    """반영본이 바뀌었음을 화면에 알린다 (지금 파일 시각을 기억해 같은 저장을 두 번 반영하지 않는다)."""
    _watch.update(version=_watch["version"] + 1, state=state, message=message,
                  skipped=list(skipped), mtime=_mtime())


def watch_corrections():
    """yeouido/corrections.json 을 직접 고쳐 저장하면 반영본을 다시 만든다 (1.5초마다 파일 시각 확인).
    직접 고친 항목은 반영이 안 돼도 목록에서 빼지 않고 이유만 알린다."""
    while True:
        time.sleep(1.5)
        if _mtime() == _watch["mtime"] or _lock.locked():   # 버튼으로 다시 만드는 중이면 끝난 뒤에 본다
            continue
        _watch.update(state="building", message="corrections.json 바뀜 → 도로망 다시 만드는 중… (5~10초)")
        with _lock:
            if _mtime() == _watch["mtime"]:     # 화면 버튼으로 저장한 것 (이미 반영됨)
                _watch.update(state="ok", message="")
                continue
            mtime = _mtime()
            try:
                data = corrections.load()
            except ValueError as e:
                where = f"{e.lineno}째 줄 {e.colno}칸: {e.msg}" if hasattr(e, "lineno") else str(e)
                _watch.update(state="error", mtime=mtime,
                              message=f"corrections.json 을 읽지 못함 ({where}) — 고쳐서 다시 저장하세요")
                print(f"[serve] {_watch['message']}")
                continue
            try:
                if any(data.values()):
                    skipped = rebuild_fixed()
                else:
                    skipped = []
                    NEW_GEOJSON.unlink(missing_ok=True)
            except Exception as e:
                last = str(e).strip().splitlines()[-1:] or [""]
                _watch.update(state="error", mtime=mtime,
                              message=f"도로망을 만들지 못함 (왼쪽 '수정 후'는 예전 그대로): {last[0]}")
                print(f"[serve] {_watch['message']}")
                continue
            for s in skipped:
                s["label"] = corrections.label(s)
            n = sum(map(len, data.values()))
            note = orphan_note() if skipped is not None and any(data.values()) else ""
            _done(message=f"corrections.json 반영됨 (수정 {n}개)" + (f" · {note}" if note else ""), skipped=skipped)
            print(f"[serve] {_watch['message']}" + "".join(f"\n  건너뜀 {s['label']}: {s['why']}" for s in skipped))


def change_corrections(msg):
    """검수 화면 버튼·키: 수정 항목 여러 개를 한 번에 넣고 빼기 → 수정 목록 저장 → 반영본 다시 만들기.

    msg = {"changes": [{"type": "remove_turns", "from", "to", "remove": true}
                       | {"type": "remove_links", "edge", "remove": true}
                       | {"type": "lane_counts", "edge", "lanes"}
                       | {"type": "turn_pockets", "edge", "side", "length"}
                       | {"type": "lane_moves", "edge", "lane", "moves"}],   값이 null 이면 그 항목을 지움
           "reviewer"}   (항목마다 "card", "road" 를 붙이면 근거 기록에 남는다)
    - 차로 수나 포켓이 바뀐 도로는 차로 번호가 달라지므로 그 도로의 차로 방향 수정을 지운다.
    - 넣을 수 없는 항목(도로가 짧음 등)은 이번에 넣은 것만 빼고 다시 만든다.
    - 다시 만들다 실패하면 목록을 원래대로 돌린다.
    """
    before = corrections.load()
    data = json.loads(json.dumps(before))
    who = msg.get("reviewer") or "이름 없음"
    added, notes = set(), []
    for ch in msg["changes"]:
        kind = ch["type"]
        if kind not in corrections.KINDS:
            raise ValueError(f"알 수 없는 수정 종류 {kind}")
        value = {"remove_turns": "remove", "remove_links": "remove", "lane_counts": "lanes", "turn_pockets": "length",
                 "lane_moves": "moves"}[kind]
        if kind == "turn_pockets" and ch.get("side") not in corrections.SIDES:
            raise ValueError("side 는 left / right")
        key = _key(kind, ch)
        data[kind] = [c for c in data[kind] if _key(kind, c) != key]
        if ch.get(value) is not None:
            item = {"from": ch["from"], "to": ch["to"]} if kind == "remove_turns" else dict(edge=ch["edge"])
            if kind == "turn_pockets":
                item.update(side=ch["side"], length=float(ch["length"]))
            elif kind == "lane_moves":
                item.update(lane=int(ch["lane"]), moves=[m for m in "slr" if m in ch["moves"]])
            elif kind == "lane_counts":
                item.update(lanes=int(ch["lanes"]))
            item.update(why=f"검수 화면에서 수정 — {ch.get('road', '')}".rstrip(" —"),
                        checked=f"inspection 카드 {ch.get('card', '')} · {who} · {date.today().isoformat()}")
            data[kind].append(item)
            added.add(key)
        if kind not in ("lane_moves", "remove_turns"):
            gone = [c for c in data["lane_moves"] if c["edge"] == ch["edge"]]
            if gone:
                data["lane_moves"] = [c for c in data["lane_moves"] if c["edge"] != ch["edge"]]
                notes.append(f"{ch['edge']}: 차로 번호가 바뀌어 차로 방향 수정 {len(gone)}개를 지움")
    corrections.save(data)
    try:
        skipped = rebuild_fixed()
        rejected = [s for s in skipped if _key(s["type"], s) in added]
        if rejected:
            for s in rejected:
                data[s["type"]] = [c for c in data[s["type"]] if _key(s["type"], c) != _key(s["type"], s)]
            corrections.save(data)
            skipped = rebuild_fixed()
    except Exception:
        corrections.save(before)
        try:
            rebuild_fixed()
        except Exception:
            pass
        _done()
        raise
    for s in skipped + rejected:
        s["label"] = corrections.label(s)
    note = orphan_note()
    if note:
        notes.append(note)
    _done(skipped=skipped)
    return dict(ok=not rejected, corrections=data, skipped=skipped, rejected=rejected, notes=notes,
                version=_watch["version"])


def load_records(kind):
    path = RECORDS[kind]
    return json.loads(path.read_text()) if path.exists() else {}


# ── 문제·보류 카드 메모 (inspection/issues.md) ─────────────────
def write_issues():
    """문제·보류로 판정한 카드마다 이름·판정·검수자·메모를 inspection/issues.md 에 모은다."""
    cards = json.loads(export_cards.CARDS.read_text()) if export_cards.CARDS.exists() else {}
    names = {"junctions": {c["id"]: " × ".join(c["names"]) for c in cards.get("cards", [])},
             "roads": {c["id"]: c["name"] for c in cards.get("roads", [])}}
    lines = ["# 검수 문제·보류 목록", "",
             "검수 카드에서 **문제** 나 **보류** 로 판정한 곳의 메모 (`python -m inspection.serve` 가 기록할 때마다 다시 씀).", ""]
    for kind, label in (("junctions", "교차로"), ("roads", "도로 구간")):
        recs = load_records(kind)
        for status, word in (("bad", "문제"), ("hold", "보류")):
            picked = {k: v for k, v in sorted(recs.items()) if v.get("status") == status}
            lines += [f"## {label} · {word} ({len(picked)})", ""]
            for cid, r in picked.items():
                memo = (r.get("memo") or "").strip().replace("\n", "\n  ") or "(메모 없음)"
                lines += [f"- **{names[kind].get(cid) or '(이름 없음)'}** `{cid}` · "
                          f"{r.get('reviewer') or '이름 없음'} · {r.get('updated', '')}",
                          f"  {memo}"]
            lines.append("")
    ISSUES.write_text("\n".join(lines))


def records_kind(path):
    """/api/records/<종류> → 종류 (RECORDS 의 키), 아니면 None."""
    prefix = "/api/records/"
    kind = path[len(prefix):] if path.startswith(prefix) else None
    return kind if kind in RECORDS else None


class Handler(SimpleHTTPRequestHandler):
    key = ""

    def end_headers(self):
        # 카드·도로망 파일(out/*.json, *.geojson)과 화면 파일도 캐시하지 않는다.
        # 캐시하면 도로망을 다시 만든 뒤에도 브라우저가 예전 카드 목록·도로망을 보여 준다
        # (예: 교차로를 합쳐 ID가 바뀌었는데 예전 카드가 열림).
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def _send(self, body, ctype, code=200):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _json(self, obj, code=200):
        self._send(json.dumps(obj, ensure_ascii=False).encode(), "application/json; charset=utf-8", code)

    def do_GET(self):
        if self.path == "/config.js":
            self._send(f"window.VWORLD_KEY = {json.dumps(self.key)};\n".encode(),
                       "application/javascript; charset=utf-8")
            return
        path, _, query = self.path.partition("?")
        if path == "/api/corrections/status":     # 잠금 없이 (다시 만드는 중에도 바로 답한다)
            self._json(dict(_watch, mtime=None, has_fixed=NEW_GEOJSON.exists()))
            return
        if path == "/api/corrections":
            with _lock:
                self._json(corrections.load())
            return
        kind = records_kind(path)
        if kind:
            with _lock:
                self._json(load_records(kind))
            return
        if path in ("/", ""):   # 주소 뒤 ?v=base 같은 값은 화면(cards.html)이 읽는다
            self.path = "/cards.html" + (f"?{query}" if query else "")
        super().do_GET()

    def do_POST(self):
        if self.path == "/api/corrections":
            # 수정 항목 넣기·빼기 (change_corrections 설명 참고)
            try:
                msg = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))))
                with _lock:
                    self._json(change_corrections(msg))
            except (ValueError, KeyError) as e:
                self._json({"error": f"잘못된 요청: {e}"}, 400)
            except RuntimeError as e:
                self._json({"error": f"도로망을 만들지 못해 수정 목록을 되돌렸습니다: {e}"}, 500)
            return
        # 카드 하나의 기록을 저장 ({"id": ..., "record": {...}}, record 가 null 이면 지움)
        kind = records_kind(self.path)
        if not kind:
            self._json({"error": "not found"}, 404)
            return
        try:
            msg = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))))
            jid, record = msg["id"], msg.get("record")
        except (ValueError, KeyError):
            self._json({"error": "bad request"}, 400)
            return
        with _lock:
            records = load_records(kind)
            if record:
                records[jid] = record
            else:
                records.pop(jid, None)
            path = RECORDS[kind]
            path.parent.mkdir(parents=True, exist_ok=True)
            tmp = path.with_suffix(".tmp")
            tmp.write_text(json.dumps(dict(sorted(records.items())), ensure_ascii=False, indent=1) + "\n")
            tmp.replace(path)
            write_issues()
        self._json({"ok": True, "count": len(records)})

    def log_message(self, *args):   # 요청마다 찍히는 로그는 끈다
        pass


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--no-browser", action="store_true", help="브라우저를 자동으로 열지 않음")
    args = ap.parse_args(argv)

    refresh()
    _watch["mtime"] = _mtime()
    threading.Thread(target=watch_corrections, daemon=True).start()
    Handler.key = vworld_key()
    if not Handler.key:
        print("[serve] .env 에 VWORLD_API_KEY 가 없습니다 → 항공사진 없이 OSM 지도만 표시합니다")

    server = ThreadingHTTPServer(("127.0.0.1", args.port),
                                 functools.partial(Handler, directory=str(HERE)))
    url = f"http://127.0.0.1:{args.port}/"
    print(f"[serve] 검수 카드 {url}   전체 지도 {url}viewer.html   (끝내려면 Ctrl+C)")
    print(f"[serve] 검수 기록 → {RECORDS['junctions'].parent}/   문제·보류 메모 → {ISSUES}")
    write_issues()
    if not args.no_browser:
        webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
