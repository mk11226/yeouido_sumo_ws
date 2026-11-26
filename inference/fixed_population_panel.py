"""차량을 맵 전체에 분산 배치하고 목표 대수까지 보충하는 여의도 실행창.

    python -m inference.fixed_population_panel
    python -m inference.fixed_population_panel --validate --seconds 180
"""
import argparse
import json
import os
from pathlib import Path
import xml.etree.ElementTree as ET

from env.fixed_population import (make_population, OUTPUT, LEVELS, prepare,
                                  config_path, snapshot_path)
from env.yeouido_env import YeouidoEnv, traci
from yeouido_cross import drive_panel as panel
from yeouido_cross import drive_route as dr


def reveal_windows(app):
    """Restore the actual SUMO window, ignoring FOX tooltip/helper windows."""
    app.root.deiconify()
    width, height = app.root.winfo_screenwidth(), app.root.winfo_screenheight()
    app.root.geometry(f"440x650+{max(10, width - 450)}+40")
    app.root.lift()
    app.root.update()
    records = []
    if os.name == "nt":
        import ctypes
        from ctypes import wintypes as w
        user = ctypes.WinDLL("user32", use_last_error=True)
        callback_type = ctypes.WINFUNCTYPE(w.BOOL, w.HWND, w.LPARAM)
        user.EnumWindows.argtypes = (callback_type, w.LPARAM)
        user.GetWindowThreadProcessId.argtypes = (w.HWND, ctypes.POINTER(w.DWORD))
        user.GetWindowTextW.argtypes = (w.HWND, w.LPWSTR, ctypes.c_int)
        user.ShowWindow.argtypes = (w.HWND, ctypes.c_int)
        user.SetWindowPos.argtypes = (w.HWND, w.HWND, ctypes.c_int, ctypes.c_int,
                                      ctypes.c_int, ctypes.c_int, w.UINT)
        user.IsWindowVisible.argtypes = (w.HWND,)
        user.SetForegroundWindow.argtypes = (w.HWND,)
        process = traci.getConnection()._process

        @callback_type
        def visit(handle, data):
            owner = w.DWORD()
            user.GetWindowThreadProcessId(handle, ctypes.byref(owner))
            if owner.value != process.pid:
                return True
            title = ctypes.create_unicode_buffer(512)
            user.GetWindowTextW(handle, title, 512)
            if "SUMO" not in title.value:
                return True
            user.ShowWindow(handle, 9)
            user.SetWindowPos(handle, None, 5, 30, max(600, width - 460), max(500, height - 90), 0x40)
            user.SetForegroundWindow(handle)
            records.append({"pid": owner.value, "title": title.value,
                            "visible": bool(user.IsWindowVisible(handle))})
            return True

        user.EnumWindows(visit, 0)
    app.root.lift()
    app.root.attributes("-topmost", True)
    record = {"panel_pid": os.getpid(), "panel_mapped": bool(app.root.winfo_ismapped()),
              "panel_geometry": app.root.geometry(), "sumo_windows": records}
    (OUTPUT / "window_status.json").write_text(json.dumps(record, ensure_ascii=False, indent=2), encoding="utf-8")
    print("WINDOW_STATUS: " + json.dumps(record, ensure_ascii=False), flush=True)


class PopulationEnv(YeouidoEnv):
    def _ensure_snapshots(self):
        if not all(snapshot_path(lv).exists() for lv in LEVELS):
            raise FileNotFoundError("Fixed-population snapshots are missing")

    def _cfg(self, level):
        return str(config_path(level))

    def _snapshot(self, level, t):
        return str(snapshot_path(level))


class PopulationPanel(panel.Panel):
    def __init__(self, level, delay, targets):
        self.targets = targets
        self.manager = None
        super().__init__(level, delay)
        self.root.title("여의도 · 전체 맵 분산 배치 · 차량 보충 유지")
        # Clearly distinguish this population scenario from measured inflow.
        for widget in self.root.winfo_children():
            if widget.winfo_class() == "Label" and widget.cget("text") == "교통 시간대 (TOPIS 평일 실측 기반)":
                widget.config(text="교통 시간대 · 유지 목표 대수")
        for lv, button in self.buttons.items():
            button.config(text=f"{panel.LEVELS[lv]['label']}\n{self.targets[lv]:,}대", width=13)
        self.origin.insert(0, "1")
        self.dest.insert(0, "6")

    def _load_level(self, level):
        super()._load_level(level)
        self.manager = make_population(self.targets[level])
        self.manager.attach(traci)
        self.env.population_manager = self.manager

    def set_level(self, level):
        super().set_level(level)
        self.env.level = level
        self.manager.set_target(self.targets[level])
        self._update_status()

    def start_drive(self):
        super().start_drive()
        if self.driving:
            self.manager.attach(traci)
            self.manager.set_target(self.targets[self.level])
            self.manager.tick(traci)

    def _inject(self):
        self.manager.tick(traci)

    def _update_status(self):
        if self.manager is None:
            return
        try:
            s = self.manager.stats(traci)
            now = traci.simulation.getTime()
            self.status.config(text=(f"현재 시간대  {panel.LEVELS[self.level]['label']}\n"
                                     f"시뮬 시각    {now:.1f}초\n"
                                     f"목표 대수    {self.manager.target:,}대\n"
                                     f"전체 관리 대수 {s['expected_population']:,}대 · 부족분 보충\n"
                                     f"도로 위      {s['on_road']:,}대\n"
                                     f"삽입 대기    {s['pending']}대\n"
                                     f"순간이동 중  {s['in_transit']}대 (관리 대수에 포함)\n"
                                     f"평균 속도    {s['mean_speed_kmh']:.1f} km/h\n"
                                     f"차량이 있는 배치 도로  {s['occupied_spawn_edges']} / {s['eligible_spawn_edges']}개"
                                     f"\n\n{self.drive_text}"))
            previous = getattr(self, "_last_status_log", -60)
            if now - previous >= 60 or now < previous:
                print("UI_STATE: " + json.dumps({"time_s": now, "level": self.level, **s}), flush=True)
                self._last_status_log = now
        except (traci.TraCIException, traci.FatalTraCIError):
            pass


def configure_ui():
    panel.ROAD = {**panel.ROAD, "demand": "uniform", "snapshot_times": [1]}
    panel.SNAPSHOT = 1.0
    panel.PANEL_CFG = str(OUTPUT / "panel.sumocfg")
    panel.make_env = lambda **kwargs: PopulationEnv(**kwargs)
    panel.load_schedule = lambda level: []
    panel.FONT = "Malgun Gothic" if os.name == "nt" else panel.FONT
    dr.VIEW = str(OUTPUT / "balanced.view.xml")
    dr.SCHEME = "population_view"
    write_gui_files = dr.write_gui_files

    def visible_traffic_view():
        write_gui_files()
        tree = ET.parse(dr.VIEW)
        for vehicles in tree.getroot().iter("vehicles"):
            vehicles.set("vehicle_minSize", "0.00")
            vehicles.set("vehicle_constantSize", "0")
            vehicles.set("vehicle_exaggeration", "1.50")
            vehicles.set("vehicleQuality", "1")
        tree.write(dr.VIEW, encoding="utf-8", xml_declaration=True)

    dr.write_gui_files = visible_traffic_view
    # Reuse established pin bitmaps on machines without a plotting installation.
    render = dr._pin_png

    def cached_pin(path, *args, **kwargs):
        if not all(Path(path + suffix).exists() for suffix in ("_icon.png", "_label.png")):
            render(path, *args, **kwargs)

    dr._pin_png = cached_pin


def validate(seconds, targets):
    from yeouido.config import sumo_bin
    report = {"seconds_per_level": seconds, "targets": targets, "levels": {}}
    for level in LEVELS:
        traci.start([sumo_bin("sumo"), "-c", str(config_path(level)),
                     "--load-state", str(snapshot_path(level)), "--step-length", "0.5",
                     "--seed", "7", "--no-step-log", "true", "--no-warnings", "true"])
        try:
            manager = make_population(targets[level])
            manager.attach(traci)
            initial = manager.stats(traci)
            created = manager.created
            start = traci.simulation.getTime()
            low = high = initial["on_road"]
            arrivals = collisions = teleports = max_pending = 0
            while traci.simulation.getTime() < start + seconds:
                traci.simulationStep()
                arrivals += len(traci.simulation.getArrivedIDList())
                collisions += traci.simulation.getCollidingVehiclesNumber()
                teleports += traci.simulation.getStartingTeleportNumber()
                manager.tick(traci)
                active = sum(v.startswith("fp_") for v in traci.vehicle.getIDList())
                low, high = min(low, active), max(high, active)
                max_pending = max(max_pending, len(manager.pending))
                assert manager.expected_population(traci) <= targets[level]
                assert traci.simulation.getMinExpectedNumber() <= targets[level], (level, active, len(manager.pending))
            final = manager.stats(traci)
            assert final["expected_population"] <= targets[level], final
            assert final["expected_population"] == initial["expected_population"] - arrivals + manager.created - created - manager.cancelled_pending, final
            assert collisions == 0, (level, collisions)
            report["levels"][level] = {"initial": initial, "final": final,
                                       "min_on_road": low, "max_on_road": high,
                                       "max_pending": max_pending, "arrivals": arrivals,
                                       "colliding_vehicles": collisions, "teleports": teleports}
            print(json.dumps({"level": level, **report["levels"][level]}, ensure_ascii=False), flush=True)
        finally:
            traci.close()
    (OUTPUT / "validation_replenishment_restored.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--level", choices=LEVELS, default="rush")
    parser.add_argument("--delay", type=float, default=20)
    parser.add_argument("--validate", action="store_true")
    parser.add_argument("--seconds", type=float, default=180)
    parser.add_argument("--smoke", action="store_true")
    args = parser.parse_args()
    os.chdir(Path(__file__).resolve().parent.parent)
    manifest = prepare()
    targets = manifest["signature"]["targets"]
    if args.validate:
        validate(args.seconds, targets)
        return
    configure_ui()
    app = PopulationPanel(args.level, args.delay, targets)
    if args.smoke:
        try:
            for level in LEVELS:
                app.set_level(level)
                assert len([v for v in traci.vehicle.getIDList() if v.startswith("fp_")]) == targets[level]
            app.start_drive()
            assert app.driving, app.drive_text
            for _ in range(40):
                app.tick()
            assert "ego" in traci.vehicle.getIDList()
            # Switching mid-drive must preserve ego and avoid deleting traffic.
            app.set_level("night")
            assert "ego" in traci.vehicle.getIDList()
            assert app.manager.target == targets["night"]
            app.set_level("rush")
            print("POPULATION_GUI_SMOKE_PASSED", flush=True)
        finally:
            app.close()
        return
    reveal_windows(app)
    app.root.after(250, lambda: traci.gui.screenshot(traci.gui.getIDList()[0], str(OUTPUT / "current_map.png")))
    print("PANEL_READY_FIXED_POPULATION: " + json.dumps(targets) + "; running; policy=balanced; replenishment=on; relief=off", flush=True)
    app.run()


if __name__ == "__main__":
    main()
