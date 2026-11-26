"""Distributed HV population, replenished up to a fixed live inventory."""
from collections import Counter, defaultdict
from pathlib import Path
import gzip
import hashlib
import heapq
import json
import math
import random
import xml.etree.ElementTree as ET

import sumolib
from sumolib.geomhelper import positionAtShapeOffset
import traci.constants as tc

ROOT = Path(__file__).resolve().parent.parent
DATA = ROOT / "env" / "yeouido"
OUTPUT = DATA / "fixed_population"
LEVELS = ("night", "day", "rush")
PREFIX = "fp_"
VERSION = 4
MARGIN = 15.0
GAP = 15.0
BALANCED_POLICY = {"route_edges": 10}
EGO_ROUTE_PROTECTION_M = 100.0


def reference_targets():
    """Count on-road IDs, excluding queued/loaded vehicles in the saved states."""
    targets, sources = {}, {}
    for level in LEVELS:
        path = DATA / "states" / f"{level}_uniform_600.xml.gz"
        raw = path.read_bytes()
        state = ET.fromstring(gzip.decompress(raw))
        active = {vid for lane in state.findall("lane") for group in lane.findall("vehicles")
                  for vid in group.get("value", "").split() if vid != "ego"}
        targets[level] = len(active)
        if not active:
            raise ValueError(f"No active reference vehicles: {path}")
        sources[level] = {"path": str(path), "sha256": hashlib.sha256(raw).hexdigest(),
                          "reference_time_s": 600, "on_road": len(active)}
    return targets, sources


def quotas(total, weights):
    """Hamilton allocation: integer quotas sum exactly to the population."""
    size = sum(weights.values())
    exact = {key: total * weight / size for key, weight in weights.items()}
    result = {key: math.floor(value) for key, value in exact.items()}
    order = sorted(exact, key=lambda key: (-(exact[key] - result[key]), key))
    for key in order[:total - sum(result.values())]:
        result[key] += 1
    return result


def free_position(length, positions):
    """Find a gap at rest; SUMO still performs all normal insertion checks."""
    candidates = [MARGIN, length - MARGIN]
    positions = sorted(positions)
    bounds = [MARGIN - GAP, *positions, length - MARGIN + GAP]
    candidates += [(a + b) / 2 for a, b in zip(bounds, bounds[1:])]
    candidates = [p for p in candidates if MARGIN <= p <= length - MARGIN]
    if not candidates:
        return None
    clearance = lambda p: min((abs(p - x) for x in positions), default=length)
    best = max(candidates, key=clearance)
    return best if clearance(best) >= GAP else None


class FixedPopulation:
    def __init__(self, target, seed=7, route_edges=None):
        self.target = int(target)
        self.initial_population = int(target)
        self.route_edges = route_edges
        self.rng = random.Random(seed)
        self.net = sumolib.net.readNet(str(DATA / "yeouido.net.xml"))
        self.lanes = {}
        self.next_edges = {}
        for edge in self.net.getEdges():
            if edge.getFunction() == "internal":
                continue
            for lane in edge.getLanes():
                next_edges = sorted({c.getTo().getID() for c in lane.getOutgoing()
                                     if c.getToLane().allows("passenger")})
                if lane.allows("passenger") and lane.getLength() >= 50 and next_edges:
                    self.lanes[lane.getID()] = lane
                    self.next_edges[lane.getID()] = next_edges
        self.weights = {key: lane.getLength() - 2 * MARGIN for key, lane in self.lanes.items()}
        self.set_target(target)
        self.pool = defaultdict(list)
        # Keep the established route mix, while choosing the departure location
        # explicitly. A departure lane must support the first turning movement.
        for vehicle in ET.parse(DATA / "rush_uniform.rou.xml").getroot().findall("vehicle"):
            node = vehicle.find("route")
            route = tuple(node.get("edges").split()) if node is not None else ()
            if len(route) >= 2:
                self.pool[(route[0], route[1])].append(route)
        self.route_candidates = {
            key: tuple(r for nxt in self.next_edges[key]
                       for r in self.pool[(lane.getEdge().getID(), nxt)])
            for key, lane in self.lanes.items()}
        self.route_cache = {}
        self.pending = {}
        self.subscribed = set()
        self.serial = self.route_serial = 0
        self.created = self.cancelled_pending = 0

    def set_target(self, target):
        if target < 0:
            raise ValueError("Population must be nonnegative")
        self.target = int(target)
        by_edge = defaultdict(dict)
        for key, weight in self.weights.items():
            by_edge[self.lanes[key].getEdge().getID()][key] = weight
        # Give every eligible road at least one car when the population allows
        # it, then share the rest by lane length. Within each road, use all lanes.
        minimum = 1 if self.target >= len(by_edge) else 0
        edge_quota = quotas(self.target - minimum * len(by_edge),
                            {edge: sum(w.values()) for edge, w in by_edge.items()})
        self.quota = {}
        for edge, weights in by_edge.items():
            self.quota.update(quotas(edge_quota[edge] + minimum, weights))
        if any(n and (self.lanes[key].getLength() - 2 * MARGIN) / n < GAP
               for key, n in self.quota.items()):
            raise ValueError("Population exceeds safe placement capacity")

    def route_for(self, lane_id):
        lane = self.lanes[lane_id]
        source = lane.getEdge().getID()
        candidates = self.route_candidates[lane_id]
        if candidates:
            route = self.rng.choice(candidates)
            return route[:self.route_edges] if self.route_edges else route
        # Some short-lived source roads were not sampled by randomTrips.
        # Explicitly preserve an allowed first lane-to-lane movement.
        nxt = self.rng.choice(self.next_edges[lane_id])
        if self.route_edges == 2:
            return (source, nxt)
        for _ in range(20):
            dest = self.rng.choice(list(self.lanes.values())).getEdge()
            path, _ = self.net.getShortestPath(self.net.getEdge(nxt), dest, vClass="passenger")
            if path and source not in [e.getID() for e in path]:
                route = (source, *(e.getID() for e in path))
                return route[:self.route_edges] if self.route_edges else route
        return (source, nxt)

    def add(self, api, lane_id, position, depart_speed=None):
        if not self.spawn_allowed(api, lane_id, position):
            return False
        route = self.route_for(lane_id)
        if route not in self.route_cache:
            self.route_serial += 1
            rid = f"fp_route_{self.route_serial}"
            api.route.add(rid, route)
            self.route_cache[route] = rid
        self.serial += 1
        vid = f"{PREFIX}{self.serial}"
        lane = self.lanes[lane_id]
        api.vehicle.add(vid, self.route_cache[route], typeID="hv", depart="now",
                        departLane=str(lane.getIndex()), departPos=f"{position:.3f}",
                        departSpeed="0" if depart_speed is None else depart_speed,
                        arrivalPos="random")
        self.pending[vid] = (lane_id, position, api.simulation.getTime())
        self.created += 1
        return True

    def seed(self, api):
        """Evenly space the first cohort, before allowing vehicles to move."""
        for lane_id, n in self.quota.items():
            length = self.lanes[lane_id].getLength()
            for i in range(n):
                position = MARGIN + (i + 0.5) * (length - 2 * MARGIN) / n
                self.add(api, lane_id, position, depart_speed="0")

    def attach(self, api):
        """Reattach after snapshot reload; IDs/routes must not be reused."""
        ids = [v for v in api.vehicle.getIDList() if v.startswith(PREFIX)]
        self.serial = max((int(v[len(PREFIX):]) for v in ids), default=0)
        self.route_serial = max((int(r[len("fp_route_"):]) for r in api.route.getIDList()
                                 if r.startswith("fp_route_")), default=0)
        self.route_cache.clear()
        self.pending.clear()
        self.subscribed.clear()
        self.subscribe(api, ids)

    def subscribe(self, api, ids):
        for vid in set(ids) - self.subscribed:
            api.vehicle.subscribe(vid, (tc.VAR_LANE_ID, tc.VAR_LANEPOSITION, tc.VAR_SPEED,
                                       tc.VAR_POSITION))
            self.subscribed.add(vid)

    def protect_near_ego(self, api, ego_id='ego', radius=EGO_ROUTE_PROTECTION_M):
        """Extend expiring HV routes before stepping; never move/delete a car.

        A 30m buffer covers vehicles entering the radius in the next 0.5s.
        Actual network exits with no legal continuation remain valid arrivals.
        """
        all_ids = set(api.vehicle.getIDList())
        if ego_id not in all_ids:
            return 0
        ex, ey = api.vehicle.getPosition(ego_id)
        self.subscribe(api, {vid for vid in all_ids if vid.startswith(PREFIX)})
        # A queued insertion can become nearby after ego moves. Cancel only
        # off-road requests, before SUMO gets another chance to insert them.
        for vid, (lane_id, pos, _) in list(self.pending.items()):
            if vid not in all_ids and not self.spawn_allowed(api, lane_id, pos, (ex, ey)):
                api.vehicle.remove(vid, reason=tc.REMOVE_VAPORIZED)
                del self.pending[vid]
                self.cancelled_pending += 1
        count = 0
        for vid, values in (api.vehicle.getAllSubscriptionResults() or {}).items():
            if not vid.startswith(PREFIX) or tc.VAR_POSITION not in values:
                continue
            x, y = values[tc.VAR_POSITION]
            if (x-ex)**2 + (y-ey)**2 > (radius+30.0)**2:
                continue
            current = api.vehicle.getRoadID(vid)
            if not current:
                continue
            route = api.vehicle.getRoute(vid)
            index = api.vehicle.getRouteIndex(vid)
            if index < 0:
                continue
            remaining = list(route[index:])
            # Many short edges can be crossed in one step. Check distance as
            # well as edge count, including vehicles currently in a junction.
            if index < len(route)-2 and sum(self.net.getEdge(e).getLength()
                                           for e in remaining) > 400:
                continue
            tail = self.net.getEdge(remaining[-1])
            extension = []
            distance = 0.0
            for _ in range(32):
                outgoing = [edge for edge, connections in tail.getOutgoing().items()
                            if edge.allows('passenger') and any(
                                c.getFromLane().allows('passenger') and
                                c.getToLane().allows('passenger') for c in connections)]
                if not outgoing:
                    break
                # Prefer onward roads over immediate returns or dead ends.
                fresh = [edge for edge in outgoing
                         if edge.getID() not in remaining+extension]
                candidates = fresh or outgoing
                onward = [edge for edge in candidates if any(
                    nxt.allows('passenger') for nxt in edge.getOutgoing())]
                tail = self.rng.choice(sorted(onward or candidates, key=lambda e: e.getID()))
                extension.append(tail.getID())
                distance += tail.getLength()
                if len(extension) >= 4 and distance >= 500:
                    break
            if extension:
                api.vehicle.setRoute(vid, remaining+extension)
                count += 1
        self.near_ego_extensions = getattr(self, 'near_ego_extensions', 0)+count
        return count

    def spawn_allowed(self, api, lane_id, position, ego_position=None):
        """Keep new traffic outside ego's 100m area plus a movement buffer."""
        if ego_position is None:
            if 'ego' not in api.vehicle.getIDList():
                return True
            ego_position = api.vehicle.getPosition('ego')
        x, y = positionAtShapeOffset(self.lanes[lane_id].getShape(), position)
        ex, ey = ego_position
        return (x-ex)**2+(y-ey)**2 > (EGO_ROUTE_PROTECTION_M+30.0)**2

    def expected_population(self, api, all_ids=None):
        """Reserve slots for all loaded HVs, including teleporting vehicles.

        This scenario loads vehicle types only: there are no future scheduled
        flows or persons. SUMO's expected count is therefore its complete live
        inventory, including off-road and queued vehicles. Ego is excluded.
        """
        all_ids = set(api.vehicle.getIDList()) if all_ids is None else all_ids
        ego_in_transit = getattr(self, "_ego_in_transit", False)
        if "ego" in api.simulation.getStartingTeleportIDList():
            ego_in_transit = True
        if ("ego" in all_ids or "ego" in api.simulation.getEndingTeleportIDList()
                or "ego" in api.simulation.getArrivedIDList()):
            ego_in_transit = False
        self._ego_in_transit = ego_in_transit
        loaded = api.simulation.getMinExpectedNumber() - int("ego" in all_ids or ego_in_transit)
        # Also reserve requests made in this TraCI frame before the next step.
        reserved = sum(v.startswith(PREFIX) for v in all_ids) + len(self.pending)
        return max(reserved, loaded)

    def tick(self, api, limit=500):
        """Replace departed HVs on underfilled lanes without moving live cars."""
        all_ids = set(api.vehicle.getIDList())
        ids = {v for v in all_ids if v.startswith(PREFIX)}
        now = api.simulation.getTime()
        arrived = set(api.simulation.getArrivedIDList())
        for vid, (_, _, since) in list(self.pending.items()):
            if vid in ids or vid in arrived:
                del self.pending[vid]
            elif now - since > 10:
                # Only cancel a request that has not entered the road.
                api.vehicle.remove(vid, reason=tc.REMOVE_VAPORIZED)
                del self.pending[vid]
                self.cancelled_pending += 1
        self.subscribed.intersection_update(ids)
        self.subscribe(api, ids)
        budget = max(0, self.target - self.expected_population(api, all_ids))
        budget = min(budget, 500 if limit is None else max(0, int(limit)))
        if not budget:
            return 0
        occupied = defaultdict(list)
        results = api.vehicle.getAllSubscriptionResults() or {}
        for vid in ids:
            value = results.get(vid)
            if value is None:
                lane_id, pos = api.vehicle.getLaneID(vid), api.vehicle.getLanePosition(vid)
            else:
                lane_id, pos = value.get(tc.VAR_LANE_ID), value.get(tc.VAR_LANEPOSITION)
            if lane_id in self.lanes and pos is not None and pos >= 0:
                occupied[lane_id].append(pos)
        for lane_id, pos, _ in self.pending.values():
            occupied[lane_id].append(pos)
        candidates = [(len(occupied[key]) / quota, self.rng.random(), key)
                      for key, quota in self.quota.items() if quota > len(occupied[key])]
        heapq.heapify(candidates)
        added = 0
        while candidates and added < budget:
            _, _, lane_id = heapq.heappop(candidates)
            pos = free_position(self.lanes[lane_id].getLength(), occupied[lane_id])
            if pos is None:
                continue
            if not self.spawn_allowed(api, lane_id, pos):
                continue
            if self.add(api, lane_id, pos, depart_speed="max") is False:
                continue
            occupied[lane_id].append(pos)
            added += 1
            quota = self.quota[lane_id]
            if len(occupied[lane_id]) < quota:
                heapq.heappush(candidates, (len(occupied[lane_id]) / quota, self.rng.random(), lane_id))
        return added

    def stats(self, api):
        ids = [v for v in api.vehicle.getIDList() if v.startswith(PREFIX)]
        results = api.vehicle.getAllSubscriptionResults()
        values = [results[key] for key in ids if key in results]
        on_road = [v for v in values if v.get(tc.VAR_LANE_ID)]
        expected = self.expected_population(api)
        in_transit = max(0, expected - len(ids) - len(self.pending))
        counts = Counter(v[tc.VAR_LANE_ID] for v in values if tc.VAR_LANE_ID in v)
        edges = {self.lanes[lane].getEdge().getID() for lane in counts if lane in self.lanes}
        return {"target": self.target, "registered": len(ids), "expected_population": expected,
                "on_road": len(on_road),
                "in_transit": in_transit, "pending": len(self.pending),
                "occupied_spawn_edges": len(edges),
                "eligible_spawn_edges": len({l.getEdge().getID() for l in self.lanes.values()}),
                "eligible_lanes": len(self.lanes), "created": self.created,
                "cancelled_pending": self.cancelled_pending,
                "mean_speed_kmh": round(sum(v.get(tc.VAR_SPEED, 0) for v in on_road) * 3.6 / max(1, len(on_road)), 2)}


def config_path(level):
    return OUTPUT / f"{level}.sumocfg"


def make_population(target, seed=7):
    """Replenished ambient traffic; ego keeps its full origin/destination route."""
    return FixedPopulation(target, seed, **BALANCED_POLICY)


def snapshot_path(level):
    return OUTPUT / "states" / f"{level}_1.xml.gz"


def prepare(force=False):
    """Build separate snapshots with no scheduled inflow and no HV removal."""
    from yeouido.config import sumo_bin
    import traci
    targets, sources = reference_targets()
    OUTPUT.mkdir(exist_ok=True)
    (OUTPUT / "states").mkdir(exist_ok=True)
    inputs = [DATA / "yeouido.net.xml", DATA / "yeouido.vtypes.xml", DATA / "rush_uniform.rou.xml",
              *(DATA / f"signals_{lv}.add.xml" for lv in LEVELS)]
    signature = {"version": VERSION, "targets": targets, "policy": BALANCED_POLICY,
                 "controller_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                 "inputs": {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}}
    manifest_path = OUTPUT / "manifest.json"
    if not force and manifest_path.exists():
        saved = json.loads(manifest_path.read_text(encoding="utf-8"))
        if saved["signature"] == signature and all(snapshot_path(lv).exists() for lv in LEVELS):
            return saved
    manifest = {"signature": signature, "reference_sources": sources, "initial": {}}
    for level in LEVELS:
        cfg = ET.Element("configuration")
        inp = ET.SubElement(cfg, "input")
        ET.SubElement(inp, "net-file", value=str(DATA / "yeouido.net.xml"))
        ET.SubElement(inp, "route-files", value=str(DATA / "yeouido.vtypes.xml"))
        ET.SubElement(inp, "additional-files", value=str(DATA / f"signals_{level}.add.xml"))
        processing = ET.SubElement(cfg, "processing")
        ET.SubElement(processing, "time-to-teleport", value="300")
        ET.ElementTree(cfg).write(config_path(level), encoding="utf-8", xml_declaration=True)
        print(f"[fixed population] {level}: distributing {targets[level]} vehicles", flush=True)
        traci.start([sumo_bin("sumo"), "-c", str(config_path(level)), "--step-length", "0.5",
                     "--seed", "7", "--no-step-log", "true", "--no-warnings", "true"])
        try:
            manager = make_population(targets[level])
            manager.seed(traci)
            traci.simulationStep()
            manager.tick(traci, limit=0)
            traci.simulationStep()
            manager.tick(traci, limit=0)
            initial = manager.stats(traci)
            if initial["on_road"] != targets[level] or initial["pending"]:
                raise RuntimeError(f"Initial distribution incomplete: {initial}")
            if traci.simulation.getCollidingVehiclesNumber():
                raise RuntimeError("Collision in initial distribution")
            traci.simulation.saveState(str(snapshot_path(level)))
            manifest["initial"][level] = initial
            print(json.dumps(initial), flush=True)
        finally:
            traci.close()
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")
    return manifest
