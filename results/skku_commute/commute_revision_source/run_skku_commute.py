"""Preview or validate the Seoul-to-Suwon SKKU route using SUMO's own driver.

    .venv-skku/bin/python run_skku_commute.py
    .venv-skku/bin/python run_skku_commute.py --background
    .venv-skku/bin/python run_skku_commute.py --gui --keep-open

The default isolates vehicle ``ego`` to test route connectivity. --background
keeps the configuration's original traffic. This script does not load/train PPO.
Outputs include summary.json, trajectory.csv, tripinfo.xml, statistics.xml and
SUMO logs. Exit status is 0 only for an ego arrival without collision/teleport.
"""
from __future__ import annotations

import argparse
import copy
import csv
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys
import time
import uuid
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parent
DEFAULT_SCENARIO = ROOT / "env/skku_commute"


def prepare_runtime():
    # Debian/Ubuntu SUMO packages keep Python bindings outside site-packages.
    for directory in (Path(os.environ.get("SUMO_HOME", "/usr/share/sumo")) / "tools",):
        if directory.is_dir() and str(directory) not in sys.path:
            sys.path.append(str(directory))
    candidates = [Path(sys.executable).parent, ROOT / ".venv-skku/bin"]
    os.environ["PATH"] = os.pathsep.join(map(str, candidates)) + os.pathsep + os.environ.get("PATH", "")
    if shutil.which("sumo") is None:
        raise RuntimeError("SUMO is unavailable; use .venv-skku/bin/python or put SUMO on PATH")


def configured_files(config_path, element):
    setting = ET.parse(config_path).find(f"./input/{element}")
    if setting is None:
        return []
    return [(config_path.parent / item.strip()).resolve()
            for item in setting.get("value", "").split(",") if item.strip()]


def isolate_ego(route_files, target):
    """Keep type/route definitions and ego, removing all other demand."""
    kept = {name: [] for name in ("vType", "vTypeDistribution", "route", "routeDistribution")}
    ego = None
    for path in route_files:
        for element in ET.parse(path).getroot():
            if element.tag in kept:
                kept[element.tag].append(copy.deepcopy(element))
            elif element.tag == "vehicle" and element.get("id") == "ego":
                if ego is not None:
                    raise ValueError("Multiple vehicles with id='ego' occur in configured routes")
                ego = copy.deepcopy(element)
                ego.set("depart", "0")
    if ego is None:
        raise ValueError("Expected an explicit <vehicle id='ego'> in the configured route files")
    root = ET.Element("routes")
    for definitions in kept.values():
        root.extend(definitions)
    root.append(ego)
    ET.indent(root)
    ET.ElementTree(root).write(target, encoding="utf-8", xml_declaration=True)
    return target


def save_json(path, data):
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)


def parse_tripinfo(path):
    if not path.is_file():
        return None
    try:
        ego = ET.parse(path).find("./tripinfo[@id='ego']")
        return dict(ego.attrib) if ego is not None else None
    except ET.ParseError:
        # A GUI kept open may still be holding an unfinished output document.
        # Individual tripinfo tags are already flushed when the ego arrives.
        import re
        text = path.read_text(encoding="utf-8")
        match = re.search(r'<tripinfo\s[^>]*\bid="ego"[^>]*>', text)
        if match:
            return dict(ET.fromstring(match.group(0).rstrip(">/") + "/>").attrib)
        return None


def configure_gui(conn, state, extent_m):
    conn.gui.setSchema("View #0", "SKKU campus commute")
    # A fixed SUMO zoom percentage depends on the full network's 40+ km extent.
    # Instead request a local viewport in meters, then follow the ego.
    x, y = state
    conn.gui.setBoundary("View #0", x - extent_m / 2, y - extent_m / 2,
                         x + extent_m / 2, y + extent_m / 2)
    conn.gui.trackVehicle("View #0", "ego")


def run(args):
    prepare_runtime()
    import traci
    import traci.constants as tc
    import sumolib

    scenario = args.scenario_dir.resolve()
    cfg = scenario / "skku_commute.sumocfg"
    if not cfg.is_file():
        raise FileNotFoundError(f"Scenario is not built: {cfg}")
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    output = (args.output_dir or ROOT / "results/skku_commute" /
              f"{'traffic' if args.background else 'isolated'}_{timestamp}_seed{args.seed}").resolve()
    output.mkdir(parents=True, exist_ok=True)
    route_files = configured_files(cfg, "route-files")
    net_files = configured_files(cfg, "net-file")
    additional_files = configured_files(cfg, "additional-files")
    if not route_files or not net_files:
        raise ValueError("Configuration needs net-file and route-files")
    isolated = None if args.background else isolate_ego(route_files, output / "isolated.rou.xml")
    gui_settings = scenario / "gui.view.xml"
    if args.gui and not gui_settings.is_file():
        raise FileNotFoundError(f"GUI settings are missing: {gui_settings}")
    tripinfo_path, statistics_path = output / "tripinfo.xml", output / "statistics.xml"
    cmd = [sumolib.checkBinary("sumo-gui" if args.gui else "sumo"), "-c", str(cfg),
           "--seed", str(args.seed), "--step-length", str(args.step_length),
           "--begin", "0", "--end", str(args.max_time + args.step_length),
           "--collision.action", "remove", "--collision.check-junctions", "true",
           "--time-to-teleport", str(args.teleport_after),
           "--tripinfo-output", str(tripinfo_path),
           "--tripinfo-output.write-unfinished", "true",
           "--tripinfo-output.write-undeparted", "true",
           "--statistic-output", str(statistics_path),
           "--duration-log.statistics", "true", "--no-step-log", "true",
           "--log", str(output / "sumo.log"), "--error-log", str(output / "sumo_errors.log")]
    if isolated:
        cmd += ["--route-files", str(isolated)]
    if args.gui:
        cmd += ["--gui-settings-file", str(gui_settings), "--start",
                "--quit-on-end", "false" if args.keep_open else "true",
                "--delay", str(args.delay_ms)]
    label = "skku_commute_" + uuid.uuid4().hex[:10]
    conn = None
    started = time.monotonic()
    report = {
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "controller": "SUMO baseline; no learned policy", "config": str(cfg),
        "background_enabled": args.background, "seed": args.seed,
        "step_length_s": args.step_length, "max_time_s": args.max_time,
        "teleport_after_s": args.teleport_after,
        "collision_checks": "SUMO vehicle and junction checks; colliding vehicles removed",
        "input_sha256": {str(path): hashlib.sha256(path.read_bytes()).hexdigest()
                         for path in [cfg, *net_files, *route_files, *additional_files, gui_settings]
                         if path.is_file()},
        "runner_source_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "command": cmd, "outcome": "not_started", "arrived": False,
        "ego_collision": False, "ego_teleported": False, "ego_departed": False,
        "ego_departure_time_s": None, "ego_arrival_time_s": None,
        "route_edges": [], "visited_edges": [], "collision_reports": [],
        "teleport_events": [], "total_departed_vehicles": 0, "total_arrived_vehicles": 0,
        "ego_speed_samples": 0, "ego_speed_sum_mps": 0.0,
        "ego_max_speed_mps": 0.0, "ego_observed_distance_m": 0.0,
        "ego_stopped_time_s": 0.0, "maximum_visited_road_speed_mps": 0.0,
        "ego_lane_changes": 0,
        "maximum_visited_allowed_speed_mps": 0.0,
        "last_simulation_time_s": 0.0,
    }
    lane_speed_cache, street_cache = {}, {}
    previous_road, previous_lane = None, None
    screenshot_requested = False
    collision_ids, teleport_ids = set(), set()
    trace_fields = ["time_s", "elapsed_since_departure_s", "x_m", "y_m", "speed_mps",
                    "acceleration_mps2", "distance_m", "road_id", "road_name", "lane_id",
                    "posted_speed_mps", "allowed_speed_mps", "waiting_time_s"]
    interrupted = None
    try:
        with (output / "sumo_stdout.log").open("w", encoding="utf-8") as stdout, \
                (output / "trajectory.csv").open("w", newline="", encoding="utf-8") as trace:
            writer = csv.DictWriter(trace, trace_fields)
            writer.writeheader()
            traci.start(cmd, label=label, doSwitch=False, verbose=False, stdout=stdout)
            conn = traci.getConnection(label)
            if args.gui:
                for type_id in conn.vehicletype.getIDList():
                    if conn.vehicletype.getVehicleClass(type_id) == "passenger":
                        conn.vehicletype.setShapeClass(type_id, "passenger/sedan")
            report["outcome"] = "running"
            next_progress = 300.0
            while conn.simulation.getTime() < args.max_time:
                conn.simulationStep()
                now = float(conn.simulation.getTime())
                report["last_simulation_time_s"] = now
                departed = conn.simulation.getDepartedIDList()
                arrived = conn.simulation.getArrivedIDList()
                teleported = conn.simulation.getStartingTeleportIDList()
                colliding = conn.simulation.getCollidingVehiclesIDList()
                report["total_departed_vehicles"] += len(departed)
                report["total_arrived_vehicles"] += len(arrived)
                collision_ids.update(colliding)
                teleport_ids.update(teleported)
                for event in conn.simulation.getCollisions():
                    report["collision_reports"].append({
                        "time_s": now, "collider": event.collider, "victim": event.victim,
                        "type": event.type, "lane": event.lane, "position_m": event.pos})
                for vehicle_id in teleported:
                    report["teleport_events"].append({"time_s": now, "vehicle_id": vehicle_id})
                report["ego_collision"] |= "ego" in colliding
                report["ego_teleported"] |= "ego" in teleported
                if "ego" in departed:
                    report["ego_departed"] = True
                    report["ego_departure_time_s"] = now - args.step_length
                    report["route_edges"] = list(conn.vehicle.getRoute("ego"))
                    conn.vehicle.subscribe("ego", [tc.VAR_POSITION, tc.VAR_SPEED,
                        tc.VAR_ACCELERATION, tc.VAR_DISTANCE, tc.VAR_ROAD_ID,
                        tc.VAR_LANE_ID, tc.VAR_ALLOWED_SPEED, tc.VAR_WAITING_TIME])
                    if args.gui:
                        conn.vehicle.setColor("ego", (255, 40, 40, 255))
                        conn.vehicle.setShapeClass("ego", "passenger/sedan")
                        configure_gui(conn, conn.vehicle.getPosition("ego"), args.viewport_m)
                # Collision removal can also appear in SUMO's arrived IDs.
                if report["ego_collision"]:
                    report["outcome"] = "collision"
                    break
                if "ego" in arrived:
                    report["arrived"] = True
                    report["ego_arrival_time_s"] = now
                    report["outcome"] = "arrived"
                    break
                if report["ego_teleported"]:
                    report["outcome"] = "teleport"
                    break
                if report["ego_departed"]:
                    state = conn.vehicle.getSubscriptionResults("ego")
                    if not state:
                        report["outcome"] = "removed_without_arrival"
                        break
                    speed = float(state[tc.VAR_SPEED])
                    lane_id, road_id = state[tc.VAR_LANE_ID], state[tc.VAR_ROAD_ID]
                    if (previous_road == road_id and previous_lane != lane_id
                            and not road_id.startswith(":")):
                        report["ego_lane_changes"] += 1
                    previous_road, previous_lane = road_id, lane_id
                    if lane_id not in lane_speed_cache:
                        lane_speed_cache[lane_id] = float(conn.lane.getMaxSpeed(lane_id))
                    if road_id not in street_cache:
                        street_cache[road_id] = conn.edge.getStreetName(road_id)
                    posted = lane_speed_cache[lane_id]
                    allowed = float(state[tc.VAR_ALLOWED_SPEED])
                    report["ego_speed_samples"] += 1
                    report["ego_speed_sum_mps"] += speed
                    report["ego_max_speed_mps"] = max(report["ego_max_speed_mps"], speed)
                    report["ego_observed_distance_m"] = max(0.0, float(state[tc.VAR_DISTANCE]))
                    report["ego_stopped_time_s"] += args.step_length * (speed < 0.1)
                    report["maximum_visited_road_speed_mps"] = max(report["maximum_visited_road_speed_mps"], posted)
                    report["maximum_visited_allowed_speed_mps"] = max(report["maximum_visited_allowed_speed_mps"], allowed)
                    if not road_id.startswith(":") and (not report["visited_edges"] or report["visited_edges"][-1] != road_id):
                        report["visited_edges"].append(road_id)
                    x, y = state[tc.VAR_POSITION]
                    writer.writerow({"time_s": now,
                        "elapsed_since_departure_s": now - report["ego_departure_time_s"],
                        "x_m": x, "y_m": y, "speed_mps": speed,
                        "acceleration_mps2": state[tc.VAR_ACCELERATION],
                        "distance_m": state[tc.VAR_DISTANCE], "road_id": road_id,
                        "road_name": street_cache[road_id], "lane_id": lane_id,
                        "posted_speed_mps": posted, "allowed_speed_mps": allowed,
                        "waiting_time_s": state[tc.VAR_WAITING_TIME]})
                if now >= next_progress:
                    trace.flush()
                    print(f"[commute] t={now:.0f}s distance={report['ego_observed_distance_m']/1000:.2f}km "
                          f"stopped={report['ego_stopped_time_s']:.0f}s", flush=True)
                    next_progress += 300
                if (args.screenshot and report["ego_departed"] and not screenshot_requested
                        and now >= 30.0):
                    conn.gui.screenshot("View #0", str(output / "gui.png"))
                    screenshot_requested = True
                if conn.simulation.getMinExpectedNumber() == 0:
                    report["outcome"] = "ego_never_departed" if not report["ego_departed"] else "simulation_empty"
                    break
            else:
                report["outcome"] = "timeout" if report["ego_departed"] else "ego_departure_timeout"
    except KeyboardInterrupt:
        report["outcome"] = "interrupted"
        interrupted = "KeyboardInterrupt"
    except Exception as error:
        report["outcome"] = "error"
        report["error"] = f"{type(error).__name__}: {error}"
        interrupted = report["error"]
    finally:
        if conn is not None:
            try:
                conn.close(wait=not (args.gui and args.keep_open))
            except Exception as error:
                report["close_error"] = f"{type(error).__name__}: {error}"
        report["wall_time_s"] = time.monotonic() - started
        trip = parse_tripinfo(tripinfo_path)
        report["ego_tripinfo"] = trip
        # Tripinfo is authoritative for the final step after the ego disappears.
        report["ego_distance_m"] = float(trip["routeLength"]) if trip and "routeLength" in trip else report["ego_observed_distance_m"]
        report["ego_duration_s"] = float(trip["duration"]) if trip and "duration" in trip else (
            report["last_simulation_time_s"] - report["ego_departure_time_s"]
            if report["ego_departure_time_s"] is not None else None)
        report["ego_average_speed_mps"] = (report["ego_distance_m"] / report["ego_duration_s"]
                                             if report["ego_duration_s"] else 0.0)
        report["ego_sample_mean_speed_mps"] = report["ego_speed_sum_mps"] / max(1, report["ego_speed_samples"])
        report["ego_queue_wait_s"] = float(trip["waitingTime"]) if trip and "waitingTime" in trip else report["ego_stopped_time_s"]
        report["collision_report_count"] = len(report["collision_reports"])
        report["colliding_vehicle_ids"] = sorted(collision_ids)
        report["teleport_event_count"] = len(report["teleport_events"])
        report["teleported_vehicle_ids"] = sorted(teleport_ids)
        report["route_first_edge"] = report["route_edges"][0] if report["route_edges"] else None
        report["route_last_edge"] = report["route_edges"][-1] if report["route_edges"] else None
        report["route_edge_count"] = len(report["route_edges"])
        report["visited_road_names"] = list(dict.fromkeys(name for name in street_cache.values() if name))
        report["validation_passed"] = bool(report["arrived"] and not report["ego_collision"] and not report["ego_teleported"])
        report["artifacts"] = ["trajectory.csv", "tripinfo.xml", "statistics.xml", "sumo.log", "sumo_errors.log"]
        if (output / "gui.png").is_file():
            report["artifacts"].append("gui.png")
            report["gui_screenshot"] = "gui.png"
        save_json(output / "summary.json", report)
    print(f"[commute] {report['outcome']} | {report['ego_distance_m']/1000:.2f}km | "
          f"{report['ego_duration_s']}s | collision={report['ego_collision']} "
          f"teleport={report['ego_teleported']}\n[saved] {output / 'summary.json'}", flush=True)
    if interrupted:
        print(f"[commute] {interrupted}", file=sys.stderr)
    return 0 if report["validation_passed"] else (130 if interrupted == "KeyboardInterrupt" else 2)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--scenario-dir", type=Path, default=DEFAULT_SCENARIO)
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--screenshot", action="store_true", help="Save one GUI screenshot at simulation time 30 s")
    parser.add_argument("--keep-open", action="store_true", help="Keep the GUI window after the run ends")
    parser.add_argument("--background", action="store_true", help="Use original traffic; default isolates ego")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--max-time", type=float, default=10800.0)
    parser.add_argument("--step-length", type=float, default=0.5)
    parser.add_argument("--teleport-after", type=float, default=-1,
                        help="Seconds stuck before SUMO teleports; -1 disables teleportation (default)")
    parser.add_argument("--delay-ms", type=float, default=20.0)
    parser.add_argument("--viewport-m", type=float, default=400.0)
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()
    if args.keep_open and not args.gui:
        parser.error("--keep-open requires --gui")
    if args.screenshot and not args.gui:
        parser.error("--screenshot requires --gui")
    if min(args.max_time, args.step_length, args.viewport_m) <= 0 or args.delay_ms < 0:
        parser.error("Time/step/viewport must be positive and delay nonnegative")
    raise SystemExit(run(args))


if __name__ == "__main__":
    main()
