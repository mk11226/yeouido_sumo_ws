"""sumo-gui + 조절 창: 교통 시간대 전환 버튼과 경로 주행 입력.

    python -m yeouido_cross.drive_panel                 # 주간 교통으로 시작
    python -m yeouido_cross.drive_panel --level rush
    python -m yeouido_cross.drive_panel --start 1 B     # 켜자마자 1 → B 주행

sumo-gui 옆에 조절 창이 뜬다.
  교통 시간대   [심야 03시] [주간 12시] [출근 08시]
                주행 중이 아니면 누르는 즉시 그 시간대의 워밍업 교통(env/yeouido/states/)으로 바뀐다.
                주행 중에 누르면 그때부터 새로 들어오는 일반 차량만 그 시간대 일정표를 따른다
                (yeouido/live.py 와 같은 방식, 주행 중인 ego는 그대로).
  출발 / 도착   1~7 (랜드마크), A~F (게이트) 또는 이름. [주행 시작]을 누르면 그 시간대 교통에서
                routes.json 의 최단거리 경로로 주행한다. 경로 차로 색칠·핀 표시는 drive_route.py 와 같다.
  일시정지      시뮬레이션을 멈추고 다시 진행.

SUMO는 하나만 띄워 둔다. 도로망 + 차량 종류만 읽고 시작해서, 일반 차량은 이 창이 선택된 시간대의
일정표(env/yeouido/<level>.rou.xml)에서 출발 시각에 맞춰 넣는다. 그래서 시간대를 바꿔도 창이 다시 뜨지 않는다.
"""
import argparse
import json
import os
import tkinter as tk
from bisect import bisect_right

from yeouido_cross import drive_route as dr
from env.road_config import ROAD, EGO, TRAFFIC
from env.mdp_config import SIMULATION, OBSERVATION, ACTION
from env import road_builder
from env.yeouido_env import CONTEXT_RADIUS, DATA, make_env, place_name, traci
import traci.constants as tc
from training.train import REWARD
from yeouido.config import END, LEVELS
from yeouido.live import load_schedule

PANEL_CFG = os.path.join(dr.GUI_DIR, "panel.sumocfg")
SNAPSHOT = 1200.0        # 시간대를 바꿀 때 불러올 워밍업 시점 (초)
STATS_EVERY = 20         # 이 스텝마다 상태 표시 갱신
COLORS = {"night": "#34495e", "day": "#f39c12", "rush": "#c0392b"}
FONT = "AppleGothic"


def write_panel_cfg():
    """도로망 + 차량 종류만 읽는 설정 (일반 차량은 조절 창이 넣는다)."""
    with open(PANEL_CFG, "w") as f:
        f.write('<configuration>\n'
                '    <input>\n'
                f'        <net-file value="{os.path.join(DATA, "yeouido.net.xml")}"/>\n'
                f'        <route-files value="{os.path.join(DATA, "yeouido.vtypes.xml")}"/>\n'
                '    </input>\n'
                '    <time><begin value="0"/><end value="1000000"/></time>\n'
                '    <processing>\n'
                '        <time-to-teleport value="300"/>\n'
                '        <ignore-route-errors value="true"/>\n'
                '    </processing>\n'
                '</configuration>\n')


class Panel:
    def __init__(self, level, delay, auto=None, shot=None):
        self.auto, self.shot, self.ticks = auto, shot, 0
        self.routes = {(r["origin"], r["destination"]): r for r in json.load(open(dr.ROUTES))}
        self.places = {n: None for n in dict.fromkeys(o for o, _ in self.routes)}   # 13곳
        self.schedules = {lv: load_schedule(lv) for lv in LEVELS}
        self.departs = {lv: [t for t, _ in s] for lv, s in self.schedules.items()}
        self.level = level
        self.route_ids = {}
        self.count = 0
        self.last_phase = 0.0
        self.paused = False
        self.driving = False
        self.selected = []
        self.obs = None
        self.drive_text = "출발·도착을 입력하고 [주행 시작]"

        dr.write_gui_files()
        write_panel_cfg()
        road = {**ROAD, "gui_files": ["yeouido.poly.xml", dr.PINS], "gui_settings": dr.VIEW}
        cfg = road_builder.build(road, EGO, TRAFFIC, os.path.join(dr.BASE, "env", "sumo"))
        self.env = make_env(cfg_path=cfg, road=road, ego=EGO, mdp_sim=SIMULATION, mdp_obs=OBSERVATION,
                            action=ACTION, reward=REWARD, traffic=TRAFFIC, gui=True, gui_autostart=True)
        env = self.env
        env._cfg = lambda _level: PANEL_CFG       # 시간대와 상관없이 같은 SUMO 하나를 쓴다
        env.gui_delay, env.gui_track_ego = delay, True
        from training.algorithms.sumo_expert import SumoExpert
        self.expert = SumoExpert(env)

        env.level = level
        env._start_sumo()
        # sumo-gui가 응답하지 않으면(창이 멈추거나 닫힘) 조절 창이 같이 멈추지 않도록 30초에서 끊는다
        traci.getConnection()._socket.settimeout(30)
        self._load_level(level)
        dr.highlight([])                          # 화면 설정(경로 색칠 방식) 적용
        dr.fit_pins()
        self._build_window()

    # ── 조절 창 ────────────────────────────────────────────
    def _build_window(self):
        self.root = tk.Tk()
        self.root.title("여의도 교통 · 경로 주행")
        self.root.geometry("+20+60")
        self.root.attributes("-topmost", True)    # sumo-gui 창에 가려지지 않게 항상 위
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        tk.Label(self.root, text="교통 시간대 (TOPIS 평일 실측 기반)",
                 font=(FONT, 13, "bold")).pack(padx=12, pady=(10, 4))
        row = tk.Frame(self.root)
        row.pack(padx=12, pady=4)
        self.buttons = {}
        for lv, spec in LEVELS.items():
            b = tk.Button(row, text=spec["label"], width=10, font=(FONT, 12),
                          command=lambda lv=lv: self.set_level(lv))
            b.pack(side=tk.LEFT, padx=3)
            self.buttons[lv] = b
        self.pause_btn = tk.Button(self.root, text="⏸ 일시정지", command=self.toggle_pause)
        self.pause_btn.pack(pady=4)

        tk.Frame(self.root, height=1, bg="#bbbbbb").pack(fill=tk.X, padx=12, pady=6)
        tk.Label(self.root, text="경로 주행  (랜드마크 1~7 · 게이트 A~F)",
                 font=(FONT, 13, "bold")).pack(padx=12, pady=(2, 4))
        form = tk.Frame(self.root)
        form.pack(padx=12, pady=2)
        tk.Label(form, text="출발", font=(FONT, 12)).grid(row=0, column=0, padx=(0, 4))
        self.origin = tk.Entry(form, width=6, font=(FONT, 13), justify="center")
        self.origin.grid(row=0, column=1)
        tk.Label(form, text="→  도착", font=(FONT, 12)).grid(row=0, column=2, padx=6)
        self.dest = tk.Entry(form, width=6, font=(FONT, 13), justify="center")
        self.dest.grid(row=0, column=3)
        tk.Button(form, text="주행 시작", font=(FONT, 12, "bold"),
                  command=self.start_drive).grid(row=0, column=4, padx=(10, 0))
        self.dest.bind("<Return>", lambda _e: self.start_drive())
        self.origin.bind("<Return>", lambda _e: self.dest.focus_set())

        self.status = tk.Label(self.root, justify=tk.LEFT, font=(FONT, 12), anchor="w")
        self.status.pack(padx=12, pady=(8, 12), anchor="w")
        self._refresh_buttons()
        self._update_status()

    def _refresh_buttons(self):
        for lv, b in self.buttons.items():
            active = lv == self.level
            b.config(relief=tk.SUNKEN if active else tk.RAISED,
                     fg=COLORS[lv] if active else "black",
                     font=(FONT, 12, "bold" if active else "normal"))

    # ── 시간대 ─────────────────────────────────────────────
    def _load_level(self, level):
        """그 시간대 워밍업 교통을 바로 불러온다 (주행 중이 아닐 때만)."""
        traci.simulation.loadState(self.env._snapshot(level, SNAPSHOT))
        self.env.level = self.env.loaded_level = level
        self.last_phase = traci.simulation.getTime() % END
        self._clear_route()

    def set_level(self, level):
        self.level = level
        if not self.driving:
            self._load_level(level)
        self._refresh_buttons()
        self._update_status()

    def toggle_pause(self):
        self.paused = not self.paused
        self.pause_btn.config(text="▶ 계속" if self.paused else "⏸ 일시정지")

    # ── 일반 차량 투입 (yeouido/live.py 와 같은 방식) ───────
    def _route_id(self, edges):
        rid = self.route_ids.get(edges)
        if rid is None:
            rid = f"panel_r{len(self.route_ids)}"
            traci.route.add(rid, edges.split())
            self.route_ids[edges] = rid
        return rid

    def _inject(self):
        phase = traci.simulation.getTime() % END
        ranges = ([(self.last_phase, phase)] if phase >= self.last_phase
                  else [(self.last_phase, END), (0.0, phase)])
        self.last_phase = phase
        sched, departs = self.schedules[self.level], self.departs[self.level]
        for lo, hi in ranges:
            for i in range(bisect_right(departs, lo), bisect_right(departs, hi)):
                self.count += 1
                try:
                    traci.vehicle.add(f"panel_{self.level}_{self.count}", self._route_id(sched[i][1]),
                                      typeID="hv", departLane="best", departSpeed="max")
                except traci.TraCIException:
                    pass

    # ── 경로 주행 ──────────────────────────────────────────
    def _clear_route(self):
        dr.clear_overlay()
        for lane in self.selected:
            try:
                if traci.gui.isSelected(lane, "lane"):
                    traci.gui.toggleSelection(lane, "lane")
            except traci.TraCIException:
                pass
        self.selected = []

    def start_drive(self):
        try:
            o = place_name(self.places, self.origin.get().strip())
            d = place_name(self.places, self.dest.get().strip())
        except KeyError:
            self.drive_text = "출발·도착을 1~7, A~F 또는 이름으로 입력하세요"
            return self._update_status()
        if not o or not d or o == d:
            self.drive_text = "출발과 도착을 서로 다르게 입력하세요"
            return self._update_status()
        env, r = self.env, self.routes[(o, d)]
        self._clear_route()
        env.loaded_level = self.level             # 시간대가 달라도 SUMO를 다시 켜지 않는다
        self.obs, _ = env.reset(options={"level": self.level, "origin": o, "destination": d,
                                         "snapshot_time": SNAPSHOT, **dr.route_points(r)})
        self.last_phase = traci.simulation.getTime() % END
        here = traci.vehicle.getRoadID(env.ego_id)
        if here in r["edges"]:
            traci.vehicle.setRoute(env.ego_id, r["edges"][r["edges"].index(here):])
            env.route_length = r["length_m"]
            env._remaining = env._prev_remaining = env.remaining_distance()
        self.selected = dr.route_lanes(list(traci.vehicle.getRoute(env.ego_id)))
        dr.highlight(self.selected)
        self.watch = dr.RouteWatch(env.ego_id)    # 주행 중 경로가 바뀌면 주황색으로 겹쳐 그림
        dr.fit_pins()
        via = f" via {r['bridge']}" if r["bridge"] else ""
        self.route_text = f"{o} → {d}{via}  {r['length_m'] / 1000:.1f} km"
        self.drive_text = f"주행 중: {self.route_text}"
        self.driving, self.steps = True, 0
        self._update_status()

    def _drive_step(self):
        env = self.env
        self.obs, _, term, trunc, info = env.step(self.expert.predict(self.obs))
        self.steps += 1
        if not (term or trunc) and self.watch.update():
            print(f"[panel] 경로 바뀜 ({self.watch.changes}번째) → 주황색으로 표시", flush=True)
        left = env._remaining
        self.drive_text = f"주행 중: {self.route_text}" + (f"\n남은 거리    {left:.0f} m" if left is not None else "")
        if term or trunc:
            result = "도착" if info.get("arrived") else "충돌" if info.get("collided") else "시간 초과/중단"
            self.drive_text = f"{result}: {self.route_text}  ({self.steps * env.step_length:.0f}초)"
            self.driving = False
            print(f"[panel] {self.drive_text}", flush=True)
            # 목적지에 닿으면 env가 ego를 지우는데, ego에 걸어 둔 주변 차량 구독이 남아 다음 스텝에
            # SUMO가 "Vehicle 'ego' is not known" 오류로 시뮬레이션을 끝낸다("Simulation ended" 창).
            # 주행이 끝나도 교통은 계속 돌리므로 여기서 구독을 지운다.
            try:
                traci.vehicle.unsubscribeContext(env.ego_id, tc.CMD_GET_VEHICLE_VARIABLE, CONTEXT_RADIUS)
            except traci.TraCIException:
                pass

    # ── 진행 ───────────────────────────────────────────────
    def tick(self):
        """한 번 부를 때마다 시뮬레이션을 한 스텝 진행한다. sumo-gui 창이 닫히면 이 창도 닫는다."""
        try:
            # sumo-gui 창을 닫으면 macOS에서는 창(view)만 사라지고 프로세스는 TraCI 연결을 붙든 채 남는다.
            # 그래서 연결 오류만 기다리지 않고 view가 남아 있는지 직접 확인한다 (멈춰 있을 때도).
            if self.ticks % 10 == 0 and (not traci.gui.getIDList() or self._sumo_exited()):
                self.close()
                return
            if not self.paused:
                if self.driving:
                    try:
                        self._drive_step()
                    except traci.TraCIException as err:   # 주행 한 스텝 실패: 주행만 끝내고 교통은 계속
                        self.driving = False
                        self.drive_text = f"주행 중단: {err}"
                else:
                    traci.simulationStep()
                self._inject()
                step = int(traci.simulation.getTime() / self.env.step_length)
                if step % 4 == 0:
                    dr.fit_pins()
                if step % STATS_EVERY == 0:
                    self._update_status()
        except (traci.FatalTraCIError, ConnectionError, OSError):   # sumo-gui 창이 닫힘
            self.close()
            return
        self.ticks += 1
        if self.auto and self.ticks == 10:
            self.origin.insert(0, self.auto[0]); self.dest.insert(0, self.auto[1]); self.start_drive()
        if self.shot and self.ticks == 80:
            traci.gui.screenshot(traci.gui.getIDList()[0], self.shot)
        self.root.after(1, self.tick)

    def _update_status(self):
        try:
            t = traci.simulation.getTime()
            ids = traci.vehicle.getIDList()
            speed = sum(traci.vehicle.getSpeed(v) for v in ids) / len(ids) * 3.6 if ids else 0.0
            halting = sum(1 for v in ids if traci.vehicle.getSpeed(v) < 0.1)
        except (traci.FatalTraCIError, traci.TraCIException):
            return
        self.status.config(text=(f"현재 시간대  {LEVELS[self.level]['label']}\n"
                                 f"시뮬 시각    {int(t // 60):3d}분 {int(t % 60):02d}초\n"
                                 f"도로 위 차량  {len(ids):5d}대 (정지 {halting}대)\n"
                                 f"평균 속도    {speed:5.1f} km/h\n\n"
                                 f"{self.drive_text}"))

    def run(self):
        self.root.after(1, self.tick)
        self.root.mainloop()

    @staticmethod
    def _process():
        try:
            return traci.getConnection()._process     # traci.start 가 띄운 sumo-gui
        except Exception:
            return None

    def _sumo_exited(self):
        proc = self._process()
        return proc is not None and proc.poll() is not None

    def close(self):
        """조절 창을 닫거나 sumo-gui 창이 닫히면 둘 다 정리한다 (창 없이 남은 sumo-gui 프로세스도 끝낸다)."""
        proc = self._process()
        try:
            traci.close(wait=False)
        except Exception:
            pass
        if proc is not None and proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=3)
            except Exception:
                proc.kill()
        self.env._started = False
        try:
            self.root.destroy()
        except tk.TclError:
            pass


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--level", choices=list(LEVELS), default="day", help="시작 시간대 (기본 day)")
    ap.add_argument("--delay", type=float, default=20.0, help="sumo-gui 재생 지연 ms (작을수록 빠름)")
    ap.add_argument("--start", nargs=2, metavar=("출발", "도착"), help="켜자마자 이 경로 주행 시작 (예: --start 1 B)")
    ap.add_argument("--screenshot", help=argparse.SUPPRESS)
    args = ap.parse_args()
    Panel(args.level, args.delay, args.start, args.screenshot and os.path.abspath(args.screenshot)).run()


if __name__ == "__main__":
    main()
