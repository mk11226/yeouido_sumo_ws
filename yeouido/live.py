"""sumo-gui + 시간대 전환 버튼 (심야 / 주간 / 출근).

    python -m yeouido.live                # 출근 교통으로 시작
    python -m yeouido.live --level night

sumo-gui 옆에 작은 조절 창이 뜬다. 버튼을 누르면 그 순간부터 새로 들어오는 일반 차량이
그 시간대의 실측 기반 일정표(env/yeouido/<level>.rou.xml)를 따른다. 이미 도로에 있는 차는
그대로 달려 나가므로, 출근 → 심야로 바꾸면 정체가 서서히 풀리는 모습을 볼 수 있다.

동작 방식: 이 창이 TraCI로 시뮬레이션을 한 스텝씩 진행하면서, 선택된 시간대의 차량을
출발 시각에 맞춰 traci.vehicle.add 로 넣는다. 세 시간대 일정표는 같은 길이(END 초)라서
현재 시각을 END 로 나눈 나머지로 어느 시간대든 바로 이어 붙일 수 있다.
재생 속도는 sumo-gui 상단의 Delay(ms) 로 조절한다.
"""
import argparse
import tkinter as tk
import xml.etree.ElementTree as ET
from bisect import bisect_right

from yeouido.build_context import POLY
from yeouido.build_demand import VTYPES
from yeouido.config import END, LEVELS, NET, OUT, sumo_home
from yeouido.view import GUI_SETTINGS, MARKERS, gui_binary, write_gui_settings, write_markers
from yeouido.visualization import VISUAL_NET
from yeouido.tdata_signals import activate_program, additional_files, controlled_ids

STEP_LENGTH = 0.5        # env/mdp_config.py SIMULATION["step_length"] 와 동일
STEPS_PER_TICK = 2       # 조절 창이 한 번 갱신될 때마다 진행할 스텝 수
STATS_EVERY = 20         # 이 스텝마다 대수·평균 속도 표시 갱신
COLORS = {"night": "#34495e", "day": "#f39c12", "rush": "#c0392b"}


def load_schedule(level):
    """<level>.rou.xml → 출발 시각 순으로 정렬된 [(depart, edges)]."""
    root = ET.parse(OUT / f"{level}.rou.xml").getroot()
    named = {r.get("id"): r.get("edges") for r in root.findall("route")}
    trips = []
    for v in root.iter("vehicle"):
        r = v.find("route")
        edges = r.get("edges") if r is not None else named.get(v.get("route"))
        if edges:
            trips.append((float(v.get("depart")), edges))
    trips.sort()
    return trips


class Live:
    def __init__(self, level):
        sumo_home()
        import traci
        self.traci = traci
        self.schedules = {lv: load_schedule(lv) for lv in LEVELS}
        self.departs = {lv: [t for t, _ in s] for lv, s in self.schedules.items()}
        self.level = level
        self.routes = {}
        self.count = 0
        self.last_phase = 0.0
        self.paused = False
        self.pending_signals = set()
        self.signal_transitions = {}
        self.measured_signals = {lv: set(controlled_ids(lv)) for lv in LEVELS}

        write_markers()
        write_gui_settings()
        traci.start([gui_binary("sumo-gui"), "-n", str(VISUAL_NET), "-r", str(VTYPES),
                     "--additional-files", additional_files(POLY, MARKERS),
                     "--gui-settings-file", str(GUI_SETTINGS),
                     "--step-length", str(STEP_LENGTH), "--start",
                     "--time-to-teleport", "300", "--ignore-route-errors",
                     "--collision.action", "remove", "--no-warnings", "--no-step-log"])
        for tid in set().union(*self.measured_signals.values()):
            activate_program(traci, tid, level, 0)
        self._build_window()

    # ── 조절 창 ────────────────────────────────────────────
    def _build_window(self):
        self.root = tk.Tk()
        self.root.title("여의도 교통 시간대")
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        tk.Label(self.root, text="교통 시간대 (TOPIS 평일 실측 기반)",
                 font=("AppleGothic", 13, "bold")).pack(padx=12, pady=(10, 4))
        row = tk.Frame(self.root)
        row.pack(padx=12, pady=4)
        self.buttons = {}
        for lv, spec in LEVELS.items():
            b = tk.Button(row, text=spec["label"], width=10, font=("AppleGothic", 12),
                          command=lambda lv=lv: self.set_level(lv))
            b.pack(side=tk.LEFT, padx=3)
            self.buttons[lv] = b
        self.pause_btn = tk.Button(self.root, text="⏸ 일시정지", command=self.toggle_pause)
        self.pause_btn.pack(pady=4)
        self.status = tk.Label(self.root, justify=tk.LEFT, font=("Menlo", 12))
        self.status.pack(padx=12, pady=(4, 12), anchor="w")
        self._refresh_buttons()

    def _refresh_buttons(self):
        for lv, b in self.buttons.items():
            active = lv == self.level
            b.config(relief=tk.SUNKEN if active else tk.RAISED,
                     fg=COLORS[lv] if active else "black",
                     font=("AppleGothic", 12, "bold" if active else "normal"))

    def set_level(self, level):
        if level != self.level:
            self.pending_signals = set().union(*self.measured_signals.values())
        self.level = level
        self._refresh_buttons()

    def _switch_signals(self, t):
        """Finish clearance before changing even an overlapping green program."""
        tls = self.traci.trafficlight
        for tid in tuple(self.pending_signals):
            transition = self.signal_transitions.get(tid)
            if transition is not None:
                stage, deadline = transition
                if t < deadline:
                    continue
                if stage == 'yellow':
                    tls.setRedYellowGreenState(tid, 'r' * len(tls.getRedYellowGreenState(tid)))
                    # Conservative clearance for a synthetic program change;
                    # this does not alter the observed steady-state cycle.
                    self.signal_transitions[tid] = ('red', t + 8)
                else:
                    activate_program(self.traci, tid, self.level, t)
                    self.pending_signals.remove(tid)
                    del self.signal_transitions[tid]
                continue
            state = tls.getRedYellowGreenState(tid)
            if set(state) == {'r'} and tls.getNextSwitch(tid) - t <= STEP_LENGTH:
                activate_program(self.traci, tid, self.level, t)
                self.pending_signals.remove(tid)
                continue
            logic = next(p for p in tls.getAllProgramLogics(tid)
                         if p.programID == tls.getProgram(tid))
            if not any(set(p.state) == {'r'} for p in logic.phases):
                yellow = max([3.0] + [p.duration for p in logic.phases if 'y' in p.state])
                tls.setRedYellowGreenState(tid, ''.join('y' if c in 'Ggy' else 'r' for c in state))
                self.signal_transitions[tid] = ('yellow', t + yellow)

    def toggle_pause(self):
        self.paused = not self.paused
        self.pause_btn.config(text="▶ 계속" if self.paused else "⏸ 일시정지")

    # ── 차량 투입 ──────────────────────────────────────────
    def _route_id(self, edges):
        rid = self.routes.get(edges)
        if rid is None:
            rid = f"live_r{len(self.routes)}"
            self.traci.route.add(rid, edges.split())
            self.routes[edges] = rid
        return rid

    def _inject(self, t):
        """직전 스텝 이후 ~ 지금 사이에 출발해야 할 차량을 현재 시간대 일정표에서 넣는다."""
        phase = t % END
        ranges = ([(self.last_phase, phase)] if phase >= self.last_phase
                  else [(self.last_phase, END), (0.0, phase)])
        self.last_phase = phase
        sched, departs = self.schedules[self.level], self.departs[self.level]
        for lo, hi in ranges:
            for i in range(bisect_right(departs, lo), bisect_right(departs, hi)):
                self.count += 1
                try:
                    self.traci.vehicle.add(f"{self.level}_{self.count}",
                                           self._route_id(sched[i][1]), typeID="hv",
                                           departLane="best", departSpeed="max")
                except self.traci.TraCIException:
                    pass   # 경로가 지금 도로망과 맞지 않으면 건너뛴다

    # ── 진행 ───────────────────────────────────────────────
    def tick(self):
        traci = self.traci
        try:
            if not self.paused:
                for _ in range(STEPS_PER_TICK):
                    traci.simulationStep()
                    t = traci.simulation.getTime()
                    self._switch_signals(t)
                    self._inject(t)
                    if int(t / STEP_LENGTH) % STATS_EVERY == 0:
                        self._update_status(t)
        except traci.FatalTraCIError:   # sumo-gui 창을 닫은 경우
            self.root.destroy()
            return
        self.root.after(1, self.tick)

    def _update_status(self, t):
        traci = self.traci
        ids = traci.vehicle.getIDList()
        speed = sum(traci.vehicle.getSpeed(v) for v in ids) / len(ids) * 3.6 if ids else 0.0
        halting = sum(1 for v in ids if traci.vehicle.getSpeed(v) < 0.1)
        self.status.config(text=(f"현재 시간대  {LEVELS[self.level]['label']}\n"
                                 f"시뮬 시각    {int(t // 60):3d}분 {int(t % 60):02d}초\n"
                                 f"도로 위 차량  {len(ids):5d}대 (정지 {halting}대)\n"
                                 f"평균 속도    {speed:5.1f} km/h"))

    def run(self):
        self.root.after(1, self.tick)
        self.root.mainloop()

    def close(self):
        try:
            self.traci.close()
        except Exception:
            pass
        self.root.destroy()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--level", choices=list(LEVELS), default="rush", help="시작 시간대")
    args = ap.parse_args(argv)
    Live(args.level).run()


if __name__ == "__main__":
    main()
