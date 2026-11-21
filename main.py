"""자율주행 주행 화면 실행 (추론): 최종 모델이 sumo-gui에서 운전한다.

    python main.py                                         # 선택 창: 시간대 · 출발지 · 목적지 · 재탐색을 고르고 [주행 시작]
    python main.py --level rush --origin 6 --destination B # 바로 주행 (출근, 당산역 → 마포대교)
    python main.py --level day --origin 국회의사당 --destination 당산역 --reroute
    python main.py --level night --origin 1 --destination 3 --record results/video/demo.mp4   # 영상도 저장
    python main.py --level rush --origin 6 --destination B --speed 4     # 4배속 (max = 컴퓨터가 낼 수 있는 최대 속도)

--origin · --destination 둘 다 주면 선택 창 없이 바로 주행한다. 하나라도 없으면 선택 창이 뜬다 (주어진 값은 미리 선택됨).
출발지 · 목적지: 랜드마크 1~7, 게이트 A~F · ㄱ~ㄷ, 또는 이름 (python main.py --list).
운전은 inference/drive_model.py 와 같다 (같은 환경 · 안전 쉴드 · 결정적 행동). 파란색 = 처음 경로, 주황색 = 재탐색으로 바뀐 경로.
"""
import argparse
import os
import sys

ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, ROOT)
os.chdir(ROOT)                      # 상대 경로(results/ 등)가 저장소 최상위 기준이 되도록

SPEEDS = ["1", "2", "4", "8", "max"]
LEVELS = [("rush", "출근 08시"), ("day", "주간 12시"), ("night", "심야 03시")]
LANDMARKS = ["국회의사당", "IFC 서울", "63빌딩", "여의도공원", "영등포역", "당산역", "노량진수산시장"]
GATES = [("A", "서강대교"), ("B", "마포대교"), ("C", "원효대교"), ("D", "양화대교"), ("E", "오목교"),
         ("F", "거리공원"), ("ㄱ", "여의교"), ("ㄴ", "서울교"), ("ㄷ", "여의2교")]
DEFAULT_MODEL = "results/ppo_v1/best/model.pt"


def places():
    """선택 창 목록: (화면 이름, drive_model 에 넘길 값)."""
    out = [(f"{i}  {name}", str(i)) for i, name in enumerate(LANDMARKS, 1)]
    out += [(f"{g}  {name}", g) for g, name in GATES]
    return out


def to_code(value):
    """번호 · 글자 · 이름(실제 이름 포함)을 drive_model 이 받는 값으로."""
    if value is None:
        return None
    v = value.strip()
    if v in LANDMARKS:
        return str(LANDMARKS.index(v) + 1)
    for g, name in GATES:
        if v in (name, f"게이트 {g}"):
            return g
    return v


def choose(args):
    """Tk 선택 창. [주행 시작]을 누르면 args 를 채워 돌려준다. 창을 닫으면 None."""
    import tkinter as tk
    from tkinter import ttk
    font = "AppleGothic" if sys.platform == "darwin" else ("Malgun Gothic" if os.name == "nt" else "NanumGothic")
    items = places()
    labels = [lab for lab, _ in items]
    code_to_label = {code: lab for lab, code in items}
    result = {}

    root = tk.Tk()
    root.title("여의도 자율주행 · 주행 설정")
    root.geometry("+80+80")
    root.attributes("-topmost", True)

    tk.Label(root, text="여의도 자율주행 (최종 모델로 주행)", font=(font, 15, "bold")).pack(padx=16, pady=(12, 6))

    tk.Label(root, text="교통 시간대", font=(font, 12, "bold")).pack(anchor="w", padx=16)
    level = tk.StringVar(value=args.level or "rush")
    row = tk.Frame(root)
    row.pack(padx=16, pady=(2, 8), anchor="w")
    for lv, label in LEVELS:
        tk.Radiobutton(row, text=label, value=lv, variable=level, font=(font, 12)).pack(side=tk.LEFT, padx=4)

    form = tk.Frame(root)
    form.pack(padx=16, pady=4, anchor="w")
    tk.Label(form, text="출발지", font=(font, 12, "bold")).grid(row=0, column=0, sticky="w", pady=3)
    origin = ttk.Combobox(form, values=labels, state="readonly", width=18, font=(font, 12))
    origin.grid(row=0, column=1, padx=8, pady=3)
    tk.Label(form, text="목적지", font=(font, 12, "bold")).grid(row=1, column=0, sticky="w", pady=3)
    dest = ttk.Combobox(form, values=labels, state="readonly", width=18, font=(font, 12))
    dest.grid(row=1, column=1, padx=8, pady=3)
    origin.set(code_to_label.get(to_code(args.origin), labels[5]))       # 기본 6 당산역
    dest.set(code_to_label.get(to_code(args.destination), labels[1]))    # 기본 2 IFC 서울

    reroute = tk.BooleanVar(value=args.reroute)
    tk.Checkbutton(root, text="경로 재탐색 켜기 (막히면 더 빠른 길로, 주황색 표시)", variable=reroute,
                   font=(font, 12)).pack(anchor="w", padx=16, pady=(6, 2))
    srow = tk.Frame(root)
    srow.pack(padx=16, pady=(2, 4), anchor="w")
    tk.Label(srow, text="재생 속도", font=(font, 12, "bold")).pack(side=tk.LEFT)
    speed = tk.StringVar(value=args.speed)
    for sp in SPEEDS:
        tk.Radiobutton(srow, text="최대" if sp == "max" else f"{sp}배", value=sp, variable=speed,
                       font=(font, 12)).pack(side=tk.LEFT, padx=3)
    record = tk.BooleanVar(value=bool(args.record))
    tk.Checkbutton(root, text="영상 저장 (results/video/main_demo.mp4)", variable=record,
                   font=(font, 12)).pack(anchor="w", padx=16, pady=(0, 6))
    msg = tk.Label(root, text="", fg="#c0392b", font=(font, 11))
    msg.pack(padx=16)

    def start():
        o = items[labels.index(origin.get())][1]
        d = items[labels.index(dest.get())][1]
        if o == d:
            msg.config(text="출발지와 목적지가 같습니다.")
            return
        result.update(level=level.get(), origin=o, destination=d, reroute=reroute.get(), speed=speed.get(),
                      record=args.record or ("results/video/main_demo.mp4" if record.get() else None))
        root.destroy()

    tk.Button(root, text="주행 시작", font=(font, 13, "bold"), width=14, command=start).pack(pady=(4, 14))
    root.mainloop()
    if not result:
        return None
    for k, v in result.items():
        setattr(args, k, v)
    return args


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--level", choices=["night", "day", "rush"], default=None,
                    help="교통 시간대: night 심야 03시 · day 주간 12시 · rush 출근 08시 (기본 rush)")
    ap.add_argument("--origin", default=None, help="출발지 (1~7, A~F, ㄱ~ㄷ 또는 이름)")
    ap.add_argument("--destination", default=None, help="목적지 (형식은 --origin 과 같음)")
    ap.add_argument("--reroute", action="store_true", help="경로 재탐색 켜기 (막히면 같은 목적지로 더 빠른 길, 주황색)")
    ap.add_argument("--model", default=DEFAULT_MODEL, help=f"운전할 모델 (기본: 최종 PPO {DEFAULT_MODEL})")
    ap.add_argument("--seed", type=int, default=0, help="SUMO 시드 (같은 값이면 같은 교통)")
    ap.add_argument("--record", default=None, metavar="MP4", help="영상 저장 경로 (.mp4). 같은 이름 .csv 에 주행 기록")
    ap.add_argument("--speed", choices=SPEEDS, default="2",
                    help="재생 배속: 실제 시간 대비 1·2·4·8배, max = 지연 없이 최대 (기본 2). 차가 많으면 계산 때문에 max 에 가까워질 수 있음")
    ap.add_argument("--delay", type=float, default=None, help="sumo-gui 스텝 지연 ms 를 직접 지정 (주면 --speed 무시)")
    ap.add_argument("--hold", action="store_true", help="주행이 끝나도 창을 닫지 않음")
    ap.add_argument("--list", action="store_true", help="출발지 · 목적지 목록 출력")
    args = ap.parse_args()

    if args.list:
        for label, _ in places():
            print(label)
        return
    if os.environ.get("LIBSUMO_AS_TRACI"):
        ap.error("sumo-gui 는 libsumo 로 띄울 수 없습니다. LIBSUMO_AS_TRACI 없이 실행하세요.")
    if args.origin is None or args.destination is None:
        if choose(args) is None:
            print("주행을 취소했습니다.")
            return
    argv = ["drive_model", "--model", args.model, "--level", args.level or "rush",
            "--origin", to_code(args.origin), "--destination", to_code(args.destination),
            "--seed", str(args.seed)]
    if args.reroute:
        argv.append("--reroute")
    if args.record:
        argv += ["--record", args.record]      # 녹화할 때는 지연 없이 (drive_model 기본 0)
    else:
        # 한 스텝 = 0.5초. N배속 = 스텝마다 500/N ms 지연 (계산 시간이 그보다 길면 그만큼 느려짐)
        delay = args.delay if args.delay is not None else (0.0 if args.speed == "max" else 500.0 / float(args.speed))
        argv += ["--delay", f"{delay:.0f}"]
    if args.hold:
        argv.append("--hold")

    from inference import drive_model
    sys.argv = argv
    drive_model.main()


if __name__ == "__main__":
    main()
