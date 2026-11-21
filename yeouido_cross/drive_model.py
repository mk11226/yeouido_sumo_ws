"""학습한 모델(최종 DAgger, PPO 등)이 sumo-gui 에서 운전하는 것을 보여 주고, 원하면 영상(mp4)으로 녹화한다 (5단계 추론 · 제출 영상).

    python -m yeouido_cross.drive_model                                   # 최종 모델, 출근 시간대 무작위 출발·도착
    python -m yeouido_cross.drive_model --origin 5 --destination 3 --level rush --reroute
    python -m yeouido_cross.drive_model --problem 21 --reroute            # 시험지 문제 그대로 (시간대·출발·도착·워밍업·SUMO 시드)
    python -m yeouido_cross.drive_model --problem 21 --reroute --change-dest 63빌딩 --at 120 \\
        --record results/video/demo.mp4                                   # 120초에 목적지 변경, 영상 저장

화면
  파란 차로  출발할 때의 경로 (drive_route.py 와 같음)
  주황 선    주행 중 바뀐 경로 (경로 재탐색 --reroute, 목적지 변경 --change-dest). 파란 경로는 그대로 둔다
  핀         랜드마크 1~7, 게이트 A~F
  위 띠      (녹화 영상에만) 시간 · 속도 · 남은 거리 · 주변 일반 차량 수 · 목적지 · 재탐색 횟수

운전은 평가(test.py)와 똑같다: 같은 환경 · 같은 경로(SUMO 기본) · 안전 쉴드 · 모델의 결정적 행동(가감속 평균, 차로 argmax).
녹화: 매 스텝(0.5 s) sumo-gui 화면을 PNG 로 저장 → 위 띠를 그려 넣고 → ffmpeg 로 mp4 (기본 10 fps = 5배속).
기록: --record 와 같은 이름의 .csv 에 스텝마다 시간 · 위치 · 속도 · 남은 거리 · 주변 차량 수 · 쉴드 · 경로 변경.
GUI 이므로 LIBSUMO_AS_TRACI 를 붙이지 않는다.
"""
import argparse
import csv
import glob
import json
import os
import shutil
import subprocess
import sys

from yeouido_cross import drive_route as dr
from env.road_config import ROAD, EGO, TRAFFIC
from env.mdp_config import SIMULATION, OBSERVATION, ACTION
from env import road_builder
from env.yeouido_env import make_env, place_name, traci
from train import REWARD
from yeouido.config import COUNT_POINTS

BASE = dr.BASE
DEFAULT_MODEL = "results/ppo_v1/best/model.pt"   # 최종 모델 PPO (2026-11-21 새 도로망 재평가: 도착 44/60, 충돌 0)
LEVEL_KO = {"night": "심야", "day": "주간", "rush": "출근"}
# 화면·영상에는 게이트 글자 대신 실제 이름 ("게이트 B" → "마포대교"). "영등포로(오목교)" 처럼 괄호가 있으면 괄호 안 이름
GATE_NAMES = {f"게이트 {v['gate']}": (v["name"].split("(")[1].rstrip(")") if "(" in v["name"] else v["name"])
              for v in COUNT_POINTS.values() if v["gate"]}


def show(place):
    """출발·도착지의 화면 표시 이름."""
    return GATE_NAMES.get(place, place)


def find_font():
    """영상 상태 띠용 한글 글꼴 (macOS · Windows · Linux). 없으면 None → PIL 기본 글꼴 (한글은 깨짐)."""
    cands = ["/System/Library/Fonts/Supplemental/AppleGothic.ttf",          # macOS
             "/System/Library/Fonts/AppleSDGothicNeo.ttc",
             "C:/Windows/Fonts/malgun.ttf",                                 # Windows 맑은 고딕
             "/usr/share/fonts/truetype/nanum/NanumGothic.ttf",            # Linux (fonts-nanum)
             "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"]     # Linux (fonts-noto-cjk)
    try:
        from matplotlib import font_manager
        for name in ("NanumGothic", "Nanum Gothic", "Malgun Gothic", "AppleGothic", "Noto Sans CJK KR", "Noto Sans CJK JP"):
            path = font_manager.findfont(name, fallback_to_default=False)
            cands.append(path)
    except Exception:
        pass
    return next((c for c in cands if c and os.path.exists(c)), None)


def find_ffmpeg():
    """시스템 ffmpeg, 없으면 pip 의 imageio-ffmpeg 가 가진 실행 파일."""
    for cand in (shutil.which("ffmpeg"), "/opt/homebrew/bin/ffmpeg", "/usr/local/bin/ffmpeg"):
        if cand and os.path.exists(cand):
            return cand
    try:
        import imageio_ffmpeg
        return imageio_ffmpeg.get_ffmpeg_exe()
    except Exception:
        return None


def _font(size):
    from PIL import ImageFont
    path = find_font()
    if path is None:
        return ImageFont.load_default()
    return ImageFont.truetype(path, size)


def _draw_banner(job):
    """프레임 한 장에 상태 띠 (annotate 의 작업 단위, 프로세스마다 따로 실행)."""
    from PIL import Image, ImageDraw
    path, row, title, event = job
    big = _font(26)
    small = _font(20)
    im = Image.open(path).convert("RGB")
    w, _ = im.size
    d = ImageDraw.Draw(im, "RGBA")
    d.rectangle([0, 0, w, 78], fill=(20, 22, 26, 215))
    d.text((16, 8), title, font=big, fill=(243, 198, 35))
    t = row["t"]
    rem = f"{float(row['remaining']):,.0f} m" if row["remaining"] not in ("", None) else "-"
    line = (f"{int(t // 60):02d}:{int(t % 60):02d}   속도 {float(row['speed_kmh']):.0f} km/h   남은 거리 {rem}   "
            f"주변 차량(100 m) {row['hv_near']}대 · 도로망 {row['hv_total']}대   목적지 {row['destination']}"
            f"   재탐색 {row['reroutes']}회")
    d.text((16, 44), line, font=small, fill=(235, 235, 230))
    if event:
        d.rectangle([0, 78, w, 112], fill=(255, 140, 0, 220))
        d.text((16, 83), event, font=small, fill=(20, 22, 26))
    im.save(path)


def annotate(frames, rows, title, hold=30):
    """프레임마다 위에 상태 띠를 그린다 (PIL). 사건(목적지 변경·재탐색) 띠는 hold 프레임 동안 유지.
    프레임이 수천 장이라 (30분 주행 = 3,600장) 여러 프로세스로 나눠 그린다."""
    jobs, event, left = [], "", 0
    for path, row in zip(frames, rows):
        if row.get("event"):
            event, left = row["event"], hold
        jobs.append((path, row, title, event if left > 0 else ""))
        left = max(left - 1, 0)
    workers = max(1, min(8, (os.cpu_count() or 2) - 2))
    if workers == 1 or len(jobs) < 200:
        for j in jobs:
            _draw_banner(j)
        return
    import multiprocessing as mp
    with mp.get_context("spawn").Pool(workers) as pool:
        for _ in pool.imap_unordered(_draw_banner, jobs, chunksize=32):
            pass


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model", default=DEFAULT_MODEL, help=f"운전할 모델 (기본 {DEFAULT_MODEL})")
    ap.add_argument("--problem", type=int, default=None, help="시험지 문제 번호 (--eval-set 의 id) 그대로 재현")
    ap.add_argument("--eval-set", default="eval_set/random_60_s1.json")
    ap.add_argument("--origin", default=None, help="출발지 (1~7, A~F, 이름). 없으면 무작위")
    ap.add_argument("--destination", default=None, help="도착지")
    ap.add_argument("--level", choices=["night", "day", "rush"], default=None,
                    help="시간대 (기본 rush). --problem 과 함께 쓰면 같은 출발·도착·시드로 시간대만 바꿈")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--reroute", action="store_true", help="경로 재탐색 켜기 (바뀐 경로는 주황색)")
    ap.add_argument("--change-dest", default=None, metavar="장소", help="주행 중 목적지를 이곳으로 바꿈")
    ap.add_argument("--at", type=float, default=120.0, help="--change-dest 를 할 시각 (출발 후 초)")
    ap.add_argument("--record", default=None, metavar="MP4", help="영상 저장 경로 (.mp4). 같은 이름 .csv 에 주행 기록")
    ap.add_argument("--fps", type=int, default=10, help="영상 초당 프레임 (스텝 0.5 s → 10 fps = 5배속)")
    ap.add_argument("--size", default="1280x720", help="녹화 해상도")
    ap.add_argument("--max-seconds", type=float, default=None, help="이 시간(출발 후 초)이 지나면 멈춤")
    ap.add_argument("--zoom", type=float, default=None, help="화면 확대 배율 (기본 ROAD gui_zoom)")
    ap.add_argument("--delay", type=float, default=0.0, help="sumo-gui 재생 지연 ms (녹화할 때는 0)")
    ap.add_argument("--keep-frames", action="store_true", help="영상을 만든 뒤 PNG 프레임을 지우지 않음")
    ap.add_argument("--hold", action="store_true", help="끝난 뒤 창을 닫지 않음 (직접 닫으면 끝)")
    ap.add_argument("--label", default=None, help="화면·영상 제목의 모델 이름 (기본: 경로에서 DAgger/PPO/BC)")
    ap.add_argument("--check", default=None, metavar="RESULTS_JSON",
                    help="--problem 과 함께: 평가 결과 JSON 의 같은 문제와 결과·스텝 수가 같은지 확인 (추론 = 평가 검증)")
    args = ap.parse_args()
    if os.environ.get("LIBSUMO_AS_TRACI"):
        ap.error("sumo-gui 는 libsumo 로 띄울 수 없습니다. LIBSUMO_AS_TRACI 없이 실행하세요.")
    if args.check and args.problem is None:
        ap.error("--check 는 --problem 과 함께 쓰세요.")
    if args.check and args.change_dest:
        ap.error("--check 는 평가와 같은 조건이어야 하므로 --change-dest 와 함께 쓸 수 없습니다.")

    # 시작 조건
    options = {"level": args.level or "rush"}
    if args.problem is not None:
        with open(os.path.join(BASE, args.eval_set), encoding="utf-8") as f:
            prob = {p["id"]: p for p in json.load(f)}[args.problem]
        options = {k: prob[k] for k in ("level", "origin", "destination", "snapshot_time", "sumo_seed") if k in prob}
        if args.level:   # 같은 문제를 다른 시간대 교통으로 (교통량 비교 영상)
            options["level"] = args.level
    if args.origin:
        options["origin"] = args.origin
    if args.destination:
        options["destination"] = args.destination

    dr.write_gui_files()
    road = {**ROAD, "gui_files": ["yeouido.poly.xml", dr.PINS], "gui_settings": dr.VIEW}
    if args.reroute:
        road["reroute"] = {**ROAD.get("reroute", {}), "enabled": True}
    cfg = road_builder.build(road, EGO, TRAFFIC, os.path.join(BASE, "env", "sumo"))
    env = make_env(cfg_path=cfg, road=road, ego=EGO, mdp_sim=SIMULATION, mdp_obs=OBSERVATION,
                   action=ACTION, reward=REWARD, traffic=TRAFFIC, gui=True, gui_autostart=True)
    env.gui_delay = args.delay
    if args.zoom:
        env.gui_zoom = args.zoom

    import test
    agent = test.load_agent(os.path.join(BASE, args.model) if not os.path.isabs(args.model) else args.model,
                            env.observation_space.shape[0], env.action_space.shape[0])

    rec_dir = csv_path = None
    width, height = (int(x) for x in args.size.lower().split("x"))
    if args.record:
        out = os.path.abspath(args.record)
        os.makedirs(os.path.dirname(out), exist_ok=True)
        rec_dir = os.path.splitext(out)[0] + "_frames"
        shutil.rmtree(rec_dir, ignore_errors=True)
        os.makedirs(rec_dir)
        csv_path = os.path.splitext(out)[0] + ".csv"

    rows, info, step, done = [], {}, 0, False
    dest_changed = False
    try:
        # 시험지 문제는 평가(utils/evaluator.py)와 똑같이 seed 없이 리셋 (결과가 평가와 같아야 한다)
        obs, info0 = env.reset(seed=None if args.problem is not None else args.seed, options=options)
        e = env.ego_id
        dr.highlight(dr.route_lanes(list(traci.vehicle.getRoute(e))))
        watch = dr.RouteWatch(e)
        dr.fit_pins()
        view = traci.gui.getIDList()[0]
        low = args.model.lower()
        label = args.label or ("DAgger" if "dagger" in low else "PPO" if "ppo" in low else "BC" if "bc" in low else "모델")
        title = (f"{label} 자율주행 · {LEVEL_KO[env.level]} · {show(env.origin)} 출발"
                 + (" · 경로 재탐색 켬" if args.reroute else ""))   # 지금 목적지는 아래 줄에 (목적지 변경 반영)
        print(f"[model] {title} → {show(env.destination)}  (경로 {env.route_length / 1000:.1f} km, 일반 차량 {traci.vehicle.getIDCount() - 1}대)",
              flush=True)
        while not done:
            event = ""
            t = step * env.step_length
            if args.change_dest and not dest_changed and t >= args.at:
                old = env.destination
                if env.change_destination(args.change_dest):   # 교차로 안 등에서 실패하면 다음 스텝에 다시 (30초까지)
                    dest_changed = True
                    event = f"목적지 변경: {show(old)} → {show(env.destination)}"
                    print(f"[model] {t:.0f}초 {event}", flush=True)
                elif t >= args.at + 30:
                    dest_changed = True
                    event = f"목적지 변경 실패 ({show(args.change_dest)}: 지금 위치에서 갈 수 없음)"
                    print(f"[model] {t:.0f}초 {event}", flush=True)
            obs, _, term, trunc, info = env.step(agent.predict(obs, deterministic=True))
            step += 1
            done = term or trunc
            if not done and watch.update() and not event:
                event = "경로 재탐색: 막혀서 더 빠른 길로 (주황색)"
                print(f"[model] {step * env.step_length:.0f}초 {event}, 남은 거리 {env._remaining or 0:.0f} m", flush=True)
            if rec_dir and not done:
                traci.gui.screenshot(view, os.path.join(rec_dir, f"f_{step:05d}.png"), width, height)
            rows.append(dict(
                t=round(step * env.step_length, 1), speed_kmh=round((info.get("speed") or 0) * 3.6, 1),
                remaining="" if info.get("remaining") is None else round(info["remaining"], 1),
                hv_total=info.get("hv_total"), hv_near=info.get("hv_near"), destination=show(env.destination),
                reroutes=info.get("reroutes", 0), shield=int(bool(info.get("shield"))),
                red_light=int(bool(info.get("red_light"))), event=event,
                road=traci.vehicle.getRoadID(e) if e in traci.vehicle.getIDList() else ""))
            if step % 100 == 0:
                print(f"[model] {step * env.step_length:.0f}초, 남은 거리 {env._remaining or 0:.0f} m", flush=True)
            if args.max_seconds and step * env.step_length >= args.max_seconds:
                break
        result = ("도착" if info.get("arrived") else "충돌" if info.get("collided")
                  else "순간이동" if info.get("teleported") else "시간초과" if done else "중단(--max-seconds)")
        near = [r["hv_near"] for r in rows if r["hv_near"] is not None]
        tot = [r["hv_total"] for r in rows if r["hv_total"] is not None]
        print(f"[model] {result} — {step * env.step_length:.0f}초, 재탐색 {info.get('reroutes', 0)}회, "
              f"빨간불 {info.get('red_lights', 0)}, 주변 차량 평균 {sum(near) / max(len(near), 1):.1f}대 "
              f"(도로망 {sum(tot) / max(len(tot), 1):.0f}대)", flush=True)
        if args.check:
            with open(os.path.join(BASE, args.check), encoding="utf-8") as f:
                ref = {r["id"]: r for r in json.load(f)["records"]}[args.problem]
            ended = {"도착": "arrival", "충돌": "collision", "순간이동": "teleport", "시간초과": "timeout"}.get(result)
            same = ended == ref["ended_by"] and step == ref["length"]
            print(f"[model] 평가와 비교: 평가 {ref['ended_by']} {ref['length']}스텝 / 추론 {ended} {step}스텝 → "
                  f"{'같음 (추론 = 평가)' if same else '다름!'}", flush=True)
        if args.hold:
            print("[model] 창을 닫으면 종료됩니다.", flush=True)
            import time
            try:
                while True:
                    traci.simulation.getTime()
                    time.sleep(0.5)
            except (traci.TraCIException, traci.FatalTraCIError, OSError):
                env._started = False
        else:
            try:
                traci.simulationStep()           # 마지막 스크린샷이 저장되도록 한 번 더 그림
            except (traci.TraCIException, traci.FatalTraCIError):
                pass
    finally:
        env.close()

    if csv_path and not rows:
        print("[model] 주행 기록이 없어 영상을 만들지 않았습니다.", flush=True)
        return
    if csv_path:
        with open(csv_path, "w", newline="", encoding="utf-8") as f:
            wr = csv.DictWriter(f, fieldnames=list(rows[0]))
            wr.writeheader()
            wr.writerows(rows)
        frames = sorted(glob.glob(os.path.join(rec_dir, "f_*.png")))
        by_step = {int(os.path.basename(p)[2:7]): p for p in frames}
        pairs = [(by_step[i + 1], rows[i]) for i in range(len(rows)) if (i + 1) in by_step]
        print(f"[model] 프레임 {len(pairs)}장 → 상태 띠 그리는 중...", flush=True)
        annotate([p for p, _ in pairs], [r for _, r in pairs], title, hold=3 * args.fps)
        # 빠진 번호 없이 이어 붙이도록 다시 번호를 매긴다
        for k, (p, _) in enumerate(pairs):
            os.replace(p, os.path.join(rec_dir, f"v_{k:05d}.png"))
        ff = find_ffmpeg()
        if not ff:
            print(f"[model] ffmpeg 를 찾지 못해 영상은 만들지 않았습니다. 프레임: {rec_dir}", flush=True)
            return
        cmd = [ff, "-y", "-loglevel", "error", "-framerate", str(args.fps), "-i", os.path.join(rec_dir, "v_%05d.png"),
               "-vf", "pad=ceil(iw/2)*2:ceil(ih/2)*2", "-c:v", "libx264", "-pix_fmt", "yuv420p", out]
        subprocess.run(cmd, check=True)
        secs = len(pairs) / args.fps
        print(f"[model] 영상 {out} ({secs:.0f}초, {args.fps} fps), 기록 {csv_path}", flush=True)
        if not args.keep_frames:
            shutil.rmtree(rec_dir, ignore_errors=True)


if __name__ == "__main__":
    sys.exit(main())
