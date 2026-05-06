#!/usr/bin/env python3
from __future__ import annotations

import argparse
import contextlib
import json
import math
import os
import re
import shutil
import subprocess
import sys
from collections import Counter, defaultdict
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
MERGE_CONFIG = REPO_ROOT / "map" / "merge.sumocfg"
AVEIRO_CONFIG = REPO_ROOT / "aveiro_map" / "aveiro.sumocfg"
LOG_DIR = REPO_ROOT / "logs"
MERGE_X = 300.0
MERGE_POINT = (MERGE_X, 0.0)

CAM_PERIOD = 0.5
REQUEST_PERIOD = 0.5
REQUEST_ZONE_M = 135.0
MESSAGE_TIMEOUT_S = 1.5
MIN_TIME_GAP_S = 2.5
SAFE_WAIT_SPEED = 2.5
STOP_BEFORE_MERGE_M = 18.0
DEMO_MARKER_LAYER = 250
DEMO_MARKER_ALPHA = 90
DEMO_BADGE_LAYER = 260
DEMO_BADGE_ALPHA = 230


def configure_sumo_tools() -> None:
    candidates = []
    if "SUMO_HOME" in os.environ:
        candidates.append(Path(os.environ["SUMO_HOME"]) / "tools")
    candidates.extend(
        [
            Path("/usr/share/sumo/tools"),
            Path("/usr/local/share/sumo/tools"),
        ]
    )

    for tools_dir in candidates:
        if (tools_dir / "traci").exists():
            sys.path.append(str(tools_dir))
            return

    sys.exit(
        "Could not import TraCI. Set SUMO_HOME or install SUMO tools "
        "(expected e.g. /usr/share/sumo/tools)."
    )


try:
    import traci  # type: ignore
except ModuleNotFoundError:
    configure_sumo_tools()
    import traci  # type: ignore


@dataclass(frozen=True)
class VehicleSpec:
    vehicle_id: str
    route_id: str
    depart: float
    depart_speed: float
    color: tuple[int, int, int, int]


@dataclass(frozen=True)
class Scenario:
    description: str
    vehicles: tuple[VehicleSpec, ...]
    cooperative: bool = True
    loss_window: tuple[float, float] | None = None


@dataclass(frozen=True)
class MapProfile:
    description: str
    sumo_config: Path
    route_edges: dict[str, tuple[str, ...]]
    main_edge: str
    ramp_edge: str
    out_edge: str
    request_zone_m: float = REQUEST_ZONE_M
    stop_before_merge_m: float = STOP_BEFORE_MERGE_M


@dataclass
class VehicleState:
    vehicle_id: str
    station_id: int
    time: float
    road: str
    lane: str
    position: tuple[float, float]
    heading: float
    speed: float
    acceleration: float
    lane_position: float | None
    length: float
    width: float
    distance_to_merge: float | None
    eta_at_merge: float | None


@dataclass
class Reservation:
    target_eta: float
    accepted_at: float
    reason: str


@dataclass
class PlanDecision:
    accepted: bool
    target_eta: float | None
    reason: str
    yield_targets: dict[str, float]


BLUE = (45, 112, 230, 255)
TEAL = (20, 160, 170, 255)
ORANGE = (240, 128, 32, 255)
GREEN = (70, 170, 75, 255)
GRAY = (120, 125, 130, 255)
YELLOW = (245, 190, 55, 255)
RED = (220, 65, 65, 255)
CYAN = (55, 185, 220, 255)
PURPLE = (145, 90, 220, 255)

STATUS_COLORS = {
    "drive": GRAY,
    "request": YELLOW,
    "accepted": GREEN,
    "released": GREEN,
    "cooperate": CYAN,
    "blocked": RED,
    "timeout": RED,
    "merged": PURPLE,
}


MAP_PROFILES: dict[str, MapProfile] = {
    "merge": MapProfile(
        description="small synthetic merge network",
        sumo_config=MERGE_CONFIG,
        route_edges={
            "main_route": ("main_in", "main_out"),
            "ramp_route": ("ramp_in", "main_out"),
        },
        main_edge="main_in",
        ramp_edge="ramp_in",
        out_edge="main_out",
    ),
    "aveiro": MapProfile(
        description="Aveiro OSM motorway-link merge",
        sumo_config=AVEIRO_CONFIG,
        route_edges={
            "main_route": ("560761994", "1331698336", "135424828"),
            "ramp_route": ("34126779", "1331698336", "135424828"),
        },
        main_edge="560761994",
        ramp_edge="34126779",
        out_edge="1331698336",
        request_zone_m=140.0,
        stop_before_merge_m=18.0,
    ),
}


MERGE_SCENARIOS: dict[str, Scenario] = {
    "base": Scenario(
        description=(
            "Two vehicles with a conflict near the merge point. The ramp "
            "vehicle should choose the safe default and enter after the main car."
        ),
        vehicles=(
            VehicleSpec("main_car_0", "main_route", depart=0.0, depart_speed=10.0, color=BLUE),
            VehicleSpec("ramp_car_0", "ramp_route", depart=7.2, depart_speed=7.5, color=ORANGE),
        ),
        cooperative=False,
    ),
    "gap": Scenario(
        description=(
            "Two main-lane vehicles create a temporal gap. The ramp vehicle "
            "reserves that gap and merges between them."
        ),
        vehicles=(
            VehicleSpec("main_front", "main_route", depart=0.0, depart_speed=11.0, color=BLUE),
            VehicleSpec("main_back", "main_route", depart=7.0, depart_speed=11.0, color=TEAL),
            VehicleSpec("ramp_car_0", "ramp_route", depart=8.0, depart_speed=7.5, color=ORANGE),
        ),
        cooperative=False,
    ),
    "adaptive": Scenario(
        description=(
            "The natural gap is too small. The following main-lane vehicle "
            "cooperates by slowing down, opening room for the ramp vehicle."
        ),
        vehicles=(
            VehicleSpec("main_front", "main_route", depart=0.0, depart_speed=10.9, color=BLUE),
            VehicleSpec("main_back", "main_route", depart=5.0, depart_speed=11.1, color=TEAL),
            VehicleSpec("ramp_car_0", "ramp_route", depart=7.3, depart_speed=7.5, color=ORANGE),
        ),
        cooperative=True,
    ),
    "loss": Scenario(
        description=(
            "Same shape as the adaptive scenario, but V2X messages involving "
            "the ramp vehicle are dropped while it is negotiating."
        ),
        vehicles=(
            VehicleSpec("main_front", "main_route", depart=0.0, depart_speed=10.9, color=BLUE),
            VehicleSpec("main_back", "main_route", depart=5.0, depart_speed=11.1, color=TEAL),
            VehicleSpec("ramp_car_0", "ramp_route", depart=7.3, depart_speed=7.5, color=ORANGE),
        ),
        cooperative=True,
        loss_window=(8.0, 32.0),
    ),
}


AVEIRO_SCENARIOS: dict[str, Scenario] = {
    "base": Scenario(
        description=(
            "Two vehicles on the Aveiro motorway merge. The ramp vehicle has "
            "to yield and enter after the main-lane vehicle."
        ),
        vehicles=(
            VehicleSpec("main_car_0", "main_route", depart=0.0, depart_speed=8.0, color=BLUE),
            VehicleSpec("ramp_car_0", "ramp_route", depart=6.0, depart_speed=12.0, color=ORANGE),
        ),
        cooperative=False,
    ),
    "gap": Scenario(
        description=(
            "Two main-lane vehicles create a safe temporal gap on the Aveiro "
            "merge; the ramp vehicle reserves and uses that gap."
        ),
        vehicles=(
            VehicleSpec("main_front", "main_route", depart=0.0, depart_speed=11.0, color=BLUE),
            VehicleSpec("main_back", "main_route", depart=8.0, depart_speed=11.0, color=TEAL),
            VehicleSpec("ramp_car_0", "ramp_route", depart=3.0, depart_speed=12.0, color=ORANGE),
        ),
        cooperative=False,
    ),
    "adaptive": Scenario(
        description=(
            "The Aveiro gap is initially too small. The following main-lane "
            "vehicle slows down to open the gap."
        ),
        vehicles=(
            VehicleSpec("main_front", "main_route", depart=0.0, depart_speed=11.0, color=BLUE),
            VehicleSpec("main_back", "main_route", depart=5.0, depart_speed=11.1, color=TEAL),
            VehicleSpec("ramp_car_0", "ramp_route", depart=3.0, depart_speed=12.0, color=ORANGE),
        ),
        cooperative=True,
    ),
    "loss": Scenario(
        description=(
            "Aveiro adaptive scenario with V2X outage around the negotiation "
            "window; the ramp vehicle waits at the manual gate."
        ),
        vehicles=(
            VehicleSpec("main_front", "main_route", depart=0.0, depart_speed=11.0, color=BLUE),
            VehicleSpec("main_back", "main_route", depart=5.0, depart_speed=11.1, color=TEAL),
            VehicleSpec("ramp_car_0", "ramp_route", depart=3.0, depart_speed=12.0, color=ORANGE),
        ),
        cooperative=True,
        loss_window=(8.0, 32.0),
    ),
}


SCENARIOS_BY_MAP: dict[str, dict[str, Scenario]] = {
    "merge": MERGE_SCENARIOS,
    "aveiro": AVEIRO_SCENARIOS,
}
SCENARIO_ORDER = ("base", "gap", "adaptive", "loss")


def station_id(vehicle_id: str) -> int:
    numbers = re.findall(r"\d+", vehicle_id)
    suffix = int(numbers[-1]) if numbers else 0
    return sum((index + 1) * ord(char) for index, char in enumerate(vehicle_id)) + suffix


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def euclidean(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


class LaneMergeDemo:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.profile = MAP_PROFILES[args.map]
        self.scenario = SCENARIOS_BY_MAP[args.map][args.scenario]
        self.pending: list[VehicleSpec] = []
        self.spawned: set[str] = set()
        self.ramp_ids: set[str] = set()
        self.pending_colors: dict[str, tuple[int, int, int, int]] = {}
        self.vehicle_type_id = "car"

        self.known_states: dict[str, dict[str, VehicleState]] = defaultdict(dict)
        self.reservations: dict[str, Reservation] = {}
        self.yield_targets: dict[str, float] = {}
        self.request_started_at: dict[str, float] = {}
        self.last_request_at: dict[str, float] = defaultdict(lambda: -999.0)
        self.last_timeout_log_at: dict[str, float] = defaultdict(lambda: -999.0)
        self.last_gate_log_at: dict[str, float] = defaultdict(lambda: -999.0)
        self.merge_crossing_times: dict[str, float] = {}
        self.merge_authorized_ids: set[str] = set()
        self.gate_holding_ids: set[str] = set()
        self.merged_ramp_ids: set[str] = set()
        self.controlled: set[str] = set()
        self.gui_view_id: str | None = None
        self.tracked_vehicle_id: str | None = None
        self.demo_marker_ids: set[str] = set()
        self.demo_badge_ids: set[str] = set()
        self.vehicle_status: dict[str, str] = {}
        self.cycles_started = 0
        self.next_cycle_at: float | None = None
        self.log_handle = None
        self.cam_log_handle = None

        self.messages = Counter()
        self.negotiation_times: list[float] = []
        self.collision_count = 0
        self.min_distance = math.inf
        self.min_eta_gap = math.inf
        self.min_speed_by_vehicle: dict[str, float] = {}
        self.max_abs_accel_by_vehicle: dict[str, float] = {}
        self.last_cam_at = -999.0

    def log(self, text: str) -> None:
        if not self.args.quiet:
            print(text, flush=True)
        if self.log_handle:
            print(text, file=self.log_handle, flush=True)

    def log_cam(self, message: dict) -> None:
        if not self.cam_log_handle:
            return
        print(json.dumps(message, separators=(",", ":"), sort_keys=True), file=self.cam_log_handle, flush=True)

    def run(self) -> None:
        if self.args.log_file:
            self.log_handle = open(self.args.log_file, "a", encoding="utf-8", buffering=1)
        if self.args.cam_log_file:
            self.cam_log_handle = open(self.args.cam_log_file, "a", encoding="utf-8", buffering=1)
        try:
            self.log(f"Map: {self.args.map} ({self.profile.description})")
            self.log(f"Scenario: {self.args.scenario}")
            self.log(f"  {self.scenario.description}")
            if self.args.log_file:
                self.log(f"Log file: {self.args.log_file}")
            if self.args.cam_log_file:
                self.log(f"CAM log file: {self.args.cam_log_file}")
            if self.args.loop:
                self.log("Loop mode: enabled, press Ctrl+C to stop.")
            elif self.args.repeat > 1:
                self.log(f"Repeat mode: {self.args.repeat} cycles.")
            if self.scenario.loss_window:
                start, end = self.scenario.loss_window
                self.log(f"  Communication loss window: t={start:.1f}s..{end:.1f}s")

            with open(os.devnull, "w", encoding="utf-8") as devnull:
                with contextlib.redirect_stdout(devnull):
                    traci.start(self.sumo_command(), stdout=subprocess.DEVNULL)
            try:
                self.ensure_vehicle_type_and_routes()
                self.configure_gui_view()
                self.start_cycle(0.0)
                while self.should_continue():
                    now = traci.simulation.getTime()
                    self.spawn_due_vehicles(now)
                    traci.simulationStep()
                    now = traci.simulation.getTime()

                    self.apply_pending_colors()
                    active_ids = list(traci.vehicle.getIDList())
                    self.broadcast_cams(now, active_ids)
                    self.update_controls(now, active_ids)
                    self.update_demo_view(active_ids)
                    self.update_metrics(now, active_ids)
                    self.maybe_start_next_cycle(now, active_ids)
            except KeyboardInterrupt:
                self.log("\nInterrupted by user.")
            finally:
                traci.close()

            self.print_summary()
        finally:
            if self.log_handle:
                self.log_handle.close()
                self.log_handle = None
            if self.cam_log_handle:
                self.cam_log_handle.close()
                self.cam_log_handle = None

    def sumo_command(self) -> list[str]:
        binary_name = "sumo-gui" if self.args.gui else "sumo"
        binary = shutil.which(binary_name)
        if binary is None:
            sys.exit(f"Could not find {binary_name} in PATH.")

        command = [
            binary,
            "-c",
            str(self.profile.sumo_config),
            "--step-length",
            str(self.args.step_length),
            "--begin",
            "0",
            "--end",
            str(1_000_000_000 if self.args.loop else self.args.end),
            "--quit-on-end",
            "--no-step-log",
            "true",
            "--duration-log.disable",
            "true",
            "--collision.action",
            "warn",
        ]
        if self.args.gui:
            command.extend(["--start", "--delay", str(self.args.delay)])
        return command

    def max_cycles(self) -> float:
        if self.args.loop:
            return math.inf
        return max(1, self.args.repeat)

    def cycle_vehicle_id(self, vehicle_id: str, cycle_index: int) -> str:
        if cycle_index == 0:
            return vehicle_id
        return f"{vehicle_id}_r{cycle_index + 1:02d}"

    def start_cycle(self, start_time: float) -> None:
        cycle_index = self.cycles_started
        specs = []
        for spec in self.scenario.vehicles:
            vehicle_id = self.cycle_vehicle_id(spec.vehicle_id, cycle_index)
            cycle_spec = VehicleSpec(
                vehicle_id=vehicle_id,
                route_id=spec.route_id,
                depart=start_time + spec.depart,
                depart_speed=spec.depart_speed,
                color=spec.color,
            )
            specs.append(cycle_spec)
            self.pending_colors[vehicle_id] = spec.color
            if spec.route_id == "ramp_route":
                self.ramp_ids.add(vehicle_id)

        self.pending.extend(sorted(specs, key=lambda item: item.depart))
        self.cycles_started += 1
        self.next_cycle_at = None
        self.log(f"[t={start_time:5.1f}s] CYCLE_START {self.args.scenario} #{self.cycles_started}")

    def maybe_start_next_cycle(self, now: float, active_ids: list[str]) -> None:
        if self.pending or active_ids:
            self.next_cycle_at = None
            return
        if self.cycles_started >= self.max_cycles():
            return

        if self.next_cycle_at is None:
            self.next_cycle_at = now + self.args.loop_pause
            self.log(
                f"[t={now:5.1f}s] CYCLE_DONE {self.args.scenario} #{self.cycles_started}; "
                f"next cycle at t={self.next_cycle_at:.1f}s"
            )
            return

        if now + 1e-9 >= self.next_cycle_at:
            self.start_cycle(now)

    def ensure_vehicle_type_and_routes(self) -> None:
        if "car" in traci.vehicletype.getIDList():
            self.vehicle_type_id = "car"
        else:
            self.vehicle_type_id = "DEFAULT_VEHTYPE"

        existing_routes = set(traci.route.getIDList())
        for route_id, edges in self.profile.route_edges.items():
            if route_id not in existing_routes:
                traci.route.add(route_id, list(edges))
        if self.ramp_hold_route_id not in set(traci.route.getIDList()):
            traci.route.add(self.ramp_hold_route_id, [self.profile.ramp_edge])

    @property
    def ramp_hold_route_id(self) -> str:
        return "ramp_route_hold"

    def route_id_for_spawn(self, spec: VehicleSpec) -> str:
        if spec.route_id == "ramp_route":
            return self.ramp_hold_route_id
        return spec.route_id

    def grant_full_ramp_route(self, vehicle_id: str) -> None:
        traci.vehicle.setRoute(vehicle_id, list(self.profile.route_edges["ramp_route"]))
        self.messages["ROUTE_GRANTED"] += 1

    def configure_gui_view(self) -> None:
        if not self.args.gui:
            return

        view_ids = list(traci.gui.getIDList())
        if not view_ids:
            return

        self.gui_view_id = view_ids[0]
        traci.gui.setZoom(self.gui_view_id, self.args.zoom)

    def should_continue(self) -> bool:
        now = traci.simulation.getTime()
        if self.args.loop:
            return True
        if now >= self.args.end:
            return False
        return (
            bool(self.pending)
            or traci.simulation.getMinExpectedNumber() > 0
            or bool(traci.vehicle.getIDList())
            or self.cycles_started < self.max_cycles()
        )

    def spawn_due_vehicles(self, now: float) -> None:
        ready = [spec for spec in self.pending if spec.depart <= now + 1e-9]
        self.pending = [spec for spec in self.pending if spec.depart > now + 1e-9]

        for spec in ready:
            route_id = self.route_id_for_spawn(spec)
            traci.vehicle.add(
                vehID=spec.vehicle_id,
                routeID=route_id,
                typeID=self.vehicle_type_id,
                depart="now",
                departLane="0",
                departPos="0",
                departSpeed=str(spec.depart_speed),
            )
            traci.vehicle.setSpeedMode(spec.vehicle_id, 7)
            traci.vehicle.setMaxSpeed(spec.vehicle_id, spec.depart_speed)
            self.spawned.add(spec.vehicle_id)
            self.set_vehicle_status(spec.vehicle_id, "drive")
            self.log(
                f"[t={now:5.1f}s] depart {spec.vehicle_id:<11} "
                f"route={route_id:<15} v0={spec.depart_speed:.1f} m/s"
            )
            if self.args.gui and spec.vehicle_id in self.ramp_ids:
                self.track_vehicle(spec.vehicle_id)

    def apply_pending_colors(self) -> None:
        active = set(traci.vehicle.getIDList())
        for vehicle_id in list(self.pending_colors):
            if vehicle_id in active:
                traci.vehicle.setColor(vehicle_id, self.pending_colors[vehicle_id])
                del self.pending_colors[vehicle_id]

    def track_vehicle(self, vehicle_id: str) -> None:
        if not self.args.gui or not self.gui_view_id:
            return
        if self.tracked_vehicle_id == vehicle_id:
            return

        traci.gui.trackVehicle(self.gui_view_id, vehicle_id)
        traci.gui.setZoom(self.gui_view_id, self.args.zoom)
        self.tracked_vehicle_id = vehicle_id

    def update_demo_view(self, active_ids: list[str]) -> None:
        if not self.args.gui or not self.args.demo_markers:
            return

        active = set(active_ids)
        for marker_id in list(self.demo_marker_ids):
            vehicle_id = marker_id.removeprefix("marker:")
            if vehicle_id not in active:
                traci.polygon.remove(marker_id, layer=DEMO_MARKER_LAYER)
                self.demo_marker_ids.remove(marker_id)
        for badge_id in list(self.demo_badge_ids):
            vehicle_id = badge_id.removeprefix("badge:")
            if vehicle_id not in active:
                traci.polygon.remove(badge_id, layer=DEMO_BADGE_LAYER)
                self.demo_badge_ids.remove(badge_id)
                self.vehicle_status.pop(vehicle_id, None)

        for vehicle_id in active_ids:
            position = traci.vehicle.getPosition(vehicle_id)
            color = traci.vehicle.getColor(vehicle_id)
            marker_color = (color[0], color[1], color[2], DEMO_MARKER_ALPHA)
            marker_id = f"marker:{vehicle_id}"
            shape = self.marker_shape(position)

            if marker_id in self.demo_marker_ids:
                traci.polygon.setShape(marker_id, shape)
                traci.polygon.setColor(marker_id, marker_color)
            else:
                traci.polygon.add(
                    marker_id,
                    shape,
                    marker_color,
                    fill=True,
                    polygonType="vehicle-marker",
                    layer=DEMO_MARKER_LAYER,
                    lineWidth=2,
                )
                self.demo_marker_ids.add(marker_id)

            status = self.vehicle_status.get(vehicle_id, "drive")
            badge_id = f"badge:{vehicle_id}"
            badge_color = self.status_color(status)
            badge_shape = self.badge_shape(position)
            if badge_id in self.demo_badge_ids:
                traci.polygon.setShape(badge_id, badge_shape)
                traci.polygon.setColor(badge_id, badge_color)
            else:
                traci.polygon.add(
                    badge_id,
                    badge_shape,
                    badge_color,
                    fill=True,
                    polygonType=f"status-{status}",
                    layer=DEMO_BADGE_LAYER,
                    lineWidth=1,
                )
                self.demo_badge_ids.add(badge_id)

    def marker_shape(self, center: tuple[float, float]) -> list[tuple[float, float]]:
        radius = self.args.marker_radius
        points = []
        for index in range(16):
            angle = 2.0 * math.pi * index / 16
            points.append(
                (
                    center[0] + math.cos(angle) * radius,
                    center[1] + math.sin(angle) * radius,
                )
            )
        return points

    def badge_shape(self, center: tuple[float, float]) -> list[tuple[float, float]]:
        size = self.args.badge_size
        offset = self.args.marker_radius + size * 1.5
        x = center[0] + offset
        y = center[1] + offset
        return [
            (x, y + size),
            (x + size, y),
            (x, y - size),
            (x - size, y),
        ]

    def status_color(self, status: str) -> tuple[int, int, int, int]:
        color = STATUS_COLORS.get(status, GRAY)
        return (color[0], color[1], color[2], DEMO_BADGE_ALPHA)

    def set_vehicle_status(self, vehicle_id: str, status: str) -> None:
        self.vehicle_status[vehicle_id] = status

    def broadcast_cams(self, now: float, active_ids: list[str]) -> None:
        if now - self.last_cam_at + 1e-9 < CAM_PERIOD:
            return
        self.last_cam_at = now

        states = {vehicle_id: self.read_state(vehicle_id, now) for vehicle_id in active_ids}
        delivered = 0
        delivered_by_sender: dict[str, list[str]] = {vehicle_id: [] for vehicle_id in active_ids}
        for sender_id, state in states.items():
            for receiver_id in active_ids:
                if sender_id == receiver_id:
                    continue
                if self.link_available(sender_id, receiver_id, states):
                    self.known_states[receiver_id][sender_id] = state
                    self.messages["CAM"] += 1
                    delivered += 1
                    delivered_by_sender[sender_id].append(receiver_id)

        for sender_id, state in states.items():
            self.log_cam(self.cam_message(state, delivered_by_sender[sender_id]))

        if self.args.trace_cam and delivered:
            self.log(f"[t={now:5.1f}s] CAM delivered={delivered}")

    def cam_message(self, state: VehicleState, delivered_to: list[str]) -> dict:
        lat, lon = self.geo_position(state.position)
        speed_value = self.etsi_speed_value(state.speed)
        heading_value = self.etsi_heading_value(state.heading)
        accel_value = self.etsi_acceleration_value(state.acceleration)

        return {
            "messageType": "CAM",
            "format": "etsi-cam-jsonl-v0",
            "source": "sumo-traci",
            "map": self.args.map,
            "scenario": self.args.scenario,
            "cycle": self.cycles_started,
            "timestamp": {
                "simulationTimeS": round(state.time, 3),
                "generationDeltaTimeMs": int(round(state.time * 1000)) % 65536,
            },
            "itsPduHeader": {
                "protocolVersion": 2,
                "messageID": 2,
                "stationID": state.station_id,
            },
            "cam": {
                "generationDeltaTime": int(round(state.time * 1000)) % 65536,
                "camParameters": {
                    "basicContainer": {
                        "stationType": 5,
                        "referencePosition": {
                            "latitude": None if lat is None else int(round(lat * 10_000_000)),
                            "longitude": None if lon is None else int(round(lon * 10_000_000)),
                            "latitudeDeg": lat,
                            "longitudeDeg": lon,
                            "altitude": {"altitudeValue": 800001, "altitudeConfidence": "unavailable"},
                        },
                    },
                    "highFrequencyContainer": {
                        "choice": "basicVehicleContainerHighFrequency",
                        "basicVehicleContainerHighFrequency": {
                            "heading": {
                                "headingValue": heading_value,
                                "headingConfidence": 127,
                                "headingDeg": round(state.heading, 2),
                            },
                            "speed": {
                                "speedValue": speed_value,
                                "speedConfidence": 127,
                                "speedMps": round(state.speed, 3),
                            },
                            "driveDirection": "forward",
                            "vehicleLength": {
                                "vehicleLengthValue": int(round(state.length * 10)),
                                "vehicleLengthConfidenceIndication": "noTrailerPresent",
                                "vehicleLengthM": round(state.length, 2),
                            },
                            "vehicleWidth": int(round(state.width * 10)),
                            "vehicleWidthM": round(state.width, 2),
                            "longitudinalAcceleration": {
                                "longitudinalAccelerationValue": accel_value,
                                "longitudinalAccelerationConfidence": 102,
                                "accelerationMps2": round(state.acceleration, 3),
                            },
                        },
                    },
                },
            },
            "sumo": {
                "vehicleID": state.vehicle_id,
                "role": self.vehicle_role(state.vehicle_id),
                "roadID": state.road,
                "laneID": state.lane,
                "lanePositionM": None if state.lane_position is None else round(state.lane_position, 3),
                "position": {"x": round(state.position[0], 3), "y": round(state.position[1], 3)},
            },
            "mergeApplication": {
                "status": self.vehicle_status.get(state.vehicle_id, "drive"),
                "distanceToMergeM": None
                if state.distance_to_merge is None
                else round(state.distance_to_merge, 3),
                "etaAtMergeS": None if state.eta_at_merge is None else round(state.eta_at_merge, 3),
                "hasReservation": state.vehicle_id in self.reservations,
                "reservationTargetEtaS": None
                if state.vehicle_id not in self.reservations
                else round(self.reservations[state.vehicle_id].target_eta, 3),
                "mergeAuthorized": state.vehicle_id in self.merge_authorized_ids,
                "gateHolding": state.vehicle_id in self.gate_holding_ids,
                "yieldTargetEtaS": None
                if state.vehicle_id not in self.yield_targets
                else round(self.yield_targets[state.vehicle_id], 3),
            },
            "channel": {
                "commRangeM": self.args.comm_range,
                "deliveredTo": delivered_to,
                "deliveredCount": len(delivered_to),
            },
        }

    def geo_position(self, position: tuple[float, float]) -> tuple[float | None, float | None]:
        try:
            lon, lat = traci.simulation.convertGeo(position[0], position[1])
        except Exception:
            return None, None
        return lat, lon

    def etsi_speed_value(self, speed_mps: float) -> int:
        return int(clamp(round(speed_mps * 100), 0, 16382))

    def etsi_heading_value(self, heading_deg: float) -> int:
        return int(round((heading_deg % 360.0) * 10)) % 3600

    def etsi_acceleration_value(self, acceleration_mps2: float) -> int:
        return int(clamp(round(acceleration_mps2 * 10), -160, 160))

    def vehicle_role(self, vehicle_id: str) -> str:
        if vehicle_id in self.ramp_ids:
            return "ramp"
        if vehicle_id.startswith("main"):
            return "main"
        return "unknown"

    def link_available(
        self,
        sender_id: str,
        receiver_id: str,
        states: dict[str, VehicleState],
    ) -> bool:
        sender = states[sender_id]
        receiver = states[receiver_id]

        if euclidean(sender.position, receiver.position) > self.args.comm_range:
            return False

        if self.scenario.loss_window and ("ramp" in sender_id or "ramp" in receiver_id):
            now = max(sender.time, receiver.time)
            start, end = self.scenario.loss_window
            if start <= now <= end:
                return False

        return True

    def update_controls(self, now: float, active_ids: list[str]) -> None:
        for vehicle_id in list(self.yield_targets):
            if vehicle_id not in active_ids or traci.vehicle.getRoadID(vehicle_id) != self.profile.main_edge:
                self.release_control(vehicle_id)
                self.yield_targets.pop(vehicle_id, None)
                continue
            self.set_speed_for_eta(vehicle_id, self.yield_targets[vehicle_id], now, min_speed=2.0)

        for vehicle_id in active_ids:
            if vehicle_id in self.ramp_ids:
                self.control_ramp_vehicle(vehicle_id, now)

        for vehicle_id in active_ids:
            if traci.vehicle.getRoadID(vehicle_id) == self.profile.out_edge:
                if vehicle_id not in self.merge_crossing_times:
                    self.merge_crossing_times[vehicle_id] = now
                    self.log(f"[t={now:5.1f}s] MERGE_POINT {vehicle_id} entered main_out")
                if vehicle_id in self.ramp_ids and vehicle_id not in self.merged_ramp_ids:
                    self.merged_ramp_ids.add(vehicle_id)
                    self.set_vehicle_status(vehicle_id, "merged")
                    self.release_control(vehicle_id)

    def control_ramp_vehicle(self, vehicle_id: str, now: float) -> None:
        if traci.vehicle.getRoadID(vehicle_id) != self.profile.ramp_edge:
            return

        state = self.read_state(vehicle_id, now)
        if state.distance_to_merge is None or state.distance_to_merge > self.profile.request_zone_m:
            return

        reservation = self.reservations.get(vehicle_id)
        if self.manual_merge_gate_holds(vehicle_id, now, state, reservation):
            return

        if reservation:
            self.set_speed_for_eta(vehicle_id, reservation.target_eta, now, min_speed=1.0)
            return

        if vehicle_id not in self.request_started_at:
            self.request_started_at[vehicle_id] = now
            self.messages["MERGE_REQUEST"] += 1
            self.set_vehicle_status(vehicle_id, "request")
            self.log(
                f"[t={now:5.1f}s] MERGE_REQUEST {vehicle_id} "
                f"eta={state.eta_at_merge:.1f}s d={state.distance_to_merge:.1f}m"
            )

        if now - self.last_request_at[vehicle_id] + 1e-9 < REQUEST_PERIOD:
            self.apply_safe_default(vehicle_id, state)
            return
        self.last_request_at[vehicle_id] = now

        main_states = self.fresh_main_states_for(vehicle_id, now)
        if not main_states:
            self.messages["MERGE_TIMEOUT"] += 1
            self.set_vehicle_status(vehicle_id, "timeout")
            self.apply_safe_default(vehicle_id, state)
            if now - self.last_timeout_log_at[vehicle_id] >= 5.0:
                self.last_timeout_log_at[vehicle_id] = now
                self.log(f"[t={now:5.1f}s] no fresh CAMs for {vehicle_id}; slowing before merge")
            return

        decision = self.plan_merge(state, main_states, now)
        if not decision.accepted or decision.target_eta is None:
            self.messages["MERGE_REJECT"] += 1
            self.apply_safe_default(vehicle_id, state)
            return

        self.messages["MERGE_ACCEPT"] += len(main_states)
        self.set_vehicle_status(vehicle_id, "accepted")
        self.reservations[vehicle_id] = Reservation(
            target_eta=decision.target_eta,
            accepted_at=now,
            reason=decision.reason,
        )
        self.negotiation_times.append(now - self.request_started_at[vehicle_id])

        for yielding_vehicle, target_eta in decision.yield_targets.items():
            self.yield_targets[yielding_vehicle] = target_eta
            self.set_vehicle_status(yielding_vehicle, "cooperate")
            self.messages["MERGE_COOP"] += 1

        yielding = ", ".join(f"{veh}->{eta:.1f}s" for veh, eta in decision.yield_targets.items())
        if not yielding:
            yielding = "none"
        self.log(
            f"[t={now:5.1f}s] MERGE_ACCEPT {vehicle_id} target_eta={decision.target_eta:.1f}s "
            f"reason={decision.reason}; cooperative_yield={yielding}"
        )

    def manual_merge_gate_holds(
        self,
        vehicle_id: str,
        now: float,
        state: VehicleState,
        reservation: Reservation | None,
    ) -> bool:
        if vehicle_id in self.merge_authorized_ids:
            return False
        if state.distance_to_merge is None or state.distance_to_merge > self.profile.stop_before_merge_m:
            return False

        if reservation:
            self.grant_full_ramp_route(vehicle_id)
            self.merge_authorized_ids.add(vehicle_id)
            self.gate_holding_ids.discard(vehicle_id)
            self.set_vehicle_status(vehicle_id, "released")
            self.messages["MANUAL_MERGE_RELEASE"] += 1
            self.log(
                f"[t={now:5.1f}s] MANUAL_MERGE_RELEASE {vehicle_id} "
                f"target_eta={reservation.target_eta:.1f}s"
            )
            return False

        traci.vehicle.setSpeed(vehicle_id, 0.0)
        self.controlled.add(vehicle_id)
        self.set_vehicle_status(vehicle_id, "blocked")

        if vehicle_id not in self.gate_holding_ids:
            self.gate_holding_ids.add(vehicle_id)
            self.last_gate_log_at[vehicle_id] = now
            self.messages["MANUAL_GATE_HOLD"] += 1
            reason = "waiting for accepted reservation"
            if reservation:
                reason = f"waiting for target_eta={reservation.target_eta:.1f}s"
            self.log(f"[t={now:5.1f}s] MANUAL_GATE_HOLD {vehicle_id}: {reason}")
        elif now - self.last_gate_log_at[vehicle_id] >= 5.0:
            self.last_gate_log_at[vehicle_id] = now
            self.log(f"[t={now:5.1f}s] MANUAL_GATE_HOLD {vehicle_id}: still blocked")

        return True

    def fresh_main_states_for(self, receiver_id: str, now: float) -> list[VehicleState]:
        fresh = []
        for state in self.known_states.get(receiver_id, {}).values():
            if not state.vehicle_id.startswith("main"):
                continue
            if state.eta_at_merge is None:
                continue
            if now - state.time > MESSAGE_TIMEOUT_S:
                continue
            fresh.append(state)
        return sorted(fresh, key=lambda state: state.eta_at_merge or math.inf)

    def plan_merge(
        self,
        ramp_state: VehicleState,
        main_states: list[VehicleState],
        now: float,
    ) -> PlanDecision:
        if ramp_state.eta_at_merge is None:
            return PlanDecision(False, None, "ramp already past merge", {})

        target_eta = ramp_state.eta_at_merge
        effective_main = [
            {
                "vehicle_id": state.vehicle_id,
                "station_id": state.station_id,
                "eta": state.eta_at_merge,
            }
            for state in main_states
            if state.eta_at_merge is not None
        ]
        yield_targets: dict[str, float] = {}
        ramp_yielded = False

        for _ in range(len(effective_main) + 3):
            effective_main.sort(key=lambda item: (item["eta"], item["station_id"]))
            changed = False

            for item in effective_main:
                main_eta = float(item["eta"])
                gap = main_eta - target_eta
                if abs(gap) + 1e-9 >= MIN_TIME_GAP_S:
                    continue

                ramp_has_priority = (
                    target_eta,
                    ramp_state.station_id,
                ) < (
                    main_eta,
                    int(item["station_id"]),
                )

                if ramp_has_priority and self.scenario.cooperative and gap >= 0:
                    new_main_eta = target_eta + MIN_TIME_GAP_S
                    if new_main_eta > main_eta + 1e-9:
                        item["eta"] = new_main_eta
                        yield_targets[str(item["vehicle_id"])] = new_main_eta
                        changed = True
                        break

                target_eta = main_eta + MIN_TIME_GAP_S
                ramp_yielded = True
                changed = True
                break

            if not changed:
                break

        target_speed = self.speed_needed_for_eta(ramp_state.vehicle_id, target_eta, now)
        if target_speed is None:
            return PlanDecision(False, None, "cannot compute target speed", {})

        reason = "reserved current gap"
        if yield_targets:
            reason = "main-lane vehicle opens gap"
        elif ramp_yielded:
            reason = "ramp yields to earlier ETA"

        return PlanDecision(True, target_eta, reason, yield_targets)

    def apply_safe_default(self, vehicle_id: str, state: VehicleState) -> None:
        if state.distance_to_merge is not None and state.distance_to_merge <= self.profile.stop_before_merge_m:
            traci.vehicle.setSpeed(vehicle_id, 0.0)
        else:
            traci.vehicle.setSpeed(vehicle_id, min(SAFE_WAIT_SPEED, max(0.0, state.speed)))
        self.controlled.add(vehicle_id)

    def set_speed_for_eta(self, vehicle_id: str, target_eta: float, now: float, min_speed: float) -> None:
        target_speed = self.speed_needed_for_eta(vehicle_id, target_eta, now)
        if target_speed is None:
            return

        max_speed = traci.vehicle.getAllowedSpeed(vehicle_id)
        target_speed = clamp(target_speed, min_speed, max_speed)
        traci.vehicle.setSpeed(vehicle_id, target_speed)
        self.controlled.add(vehicle_id)

    def speed_needed_for_eta(self, vehicle_id: str, target_eta: float, now: float) -> float | None:
        distance = self.distance_remaining_to_merge(vehicle_id)
        if distance is None:
            return None

        time_remaining = max(target_eta - now, self.args.step_length)
        return distance / time_remaining

    def release_control(self, vehicle_id: str) -> None:
        if vehicle_id in self.controlled and vehicle_id in set(traci.vehicle.getIDList()):
            traci.vehicle.setSpeed(vehicle_id, -1.0)
        self.controlled.discard(vehicle_id)

    def update_metrics(self, now: float, active_ids: list[str]) -> None:
        self.collision_count += len(traci.simulation.getCollisions())

        states = [self.read_state(vehicle_id, now) for vehicle_id in active_ids]
        for state in states:
            self.min_speed_by_vehicle[state.vehicle_id] = min(
                self.min_speed_by_vehicle.get(state.vehicle_id, math.inf),
                state.speed,
            )
            self.max_abs_accel_by_vehicle[state.vehicle_id] = max(
                self.max_abs_accel_by_vehicle.get(state.vehicle_id, 0.0),
                abs(state.acceleration),
            )

        for index, left in enumerate(states):
            for right in states[index + 1 :]:
                self.min_distance = min(self.min_distance, euclidean(left.position, right.position))

        etas = sorted(state.eta_at_merge for state in states if state.eta_at_merge is not None)
        for index, eta in enumerate(etas[:-1]):
            self.min_eta_gap = min(self.min_eta_gap, abs(etas[index + 1] - eta))

    def read_state(self, vehicle_id: str, now: float) -> VehicleState:
        position = traci.vehicle.getPosition(vehicle_id)
        speed = traci.vehicle.getSpeed(vehicle_id)
        lane_id = traci.vehicle.getLaneID(vehicle_id)
        lane_position = traci.vehicle.getLanePosition(vehicle_id) if lane_id else None
        distance = self.distance_remaining_to_merge(vehicle_id)
        eta = now + distance / max(speed, 0.1) if distance is not None else None
        return VehicleState(
            vehicle_id=vehicle_id,
            station_id=station_id(vehicle_id),
            time=now,
            road=traci.vehicle.getRoadID(vehicle_id),
            lane=lane_id,
            position=position,
            heading=traci.vehicle.getAngle(vehicle_id),
            speed=speed,
            acceleration=traci.vehicle.getAcceleration(vehicle_id),
            lane_position=lane_position,
            length=traci.vehicle.getLength(vehicle_id),
            width=traci.vehicle.getWidth(vehicle_id),
            distance_to_merge=distance,
            eta_at_merge=eta,
        )

    def distance_remaining_to_merge(self, vehicle_id: str) -> float | None:
        road = traci.vehicle.getRoadID(vehicle_id)
        lane_id = traci.vehicle.getLaneID(vehicle_id)

        if road not in {self.profile.main_edge, self.profile.ramp_edge} or not lane_id:
            return None

        lane_length = traci.lane.getLength(lane_id)
        lane_position = traci.vehicle.getLanePosition(vehicle_id)
        return max(0.0, lane_length - lane_position)

    def print_summary(self) -> None:
        min_distance = "n/a" if math.isinf(self.min_distance) else f"{self.min_distance:.2f} m"
        merge_gaps = [
            right - left
            for left, right in zip(
                sorted(self.merge_crossing_times.values()),
                sorted(self.merge_crossing_times.values())[1:],
            )
        ]
        min_merge_gap = "n/a" if not merge_gaps else f"{min(merge_gaps):.2f} s"
        avg_negotiation = (
            "n/a"
            if not self.negotiation_times
            else f"{sum(self.negotiation_times) / len(self.negotiation_times):.2f} s"
        )

        ramp_success = all(vehicle_id in self.merged_ramp_ids for vehicle_id in self.ramp_ids)
        unauthorized_merges = sorted(self.merged_ramp_ids - self.merge_authorized_ids)
        self.log("\n=== SUMO/V2X demo metrics ===")
        self.log(f"map                   : {self.args.map}")
        self.log(f"scenario              : {self.args.scenario}")
        self.log(f"cycles started        : {self.cycles_started}")
        self.log(f"ramp merge success    : {'yes' if ramp_success else 'no'}")
        self.log(f"unauthorized merges   : {len(unauthorized_merges)}")
        self.log(f"collisions            : {self.collision_count}")
        self.log(f"min vehicle distance  : {min_distance}")
        self.log(f"min actual merge gap  : {min_merge_gap}")
        self.log(f"avg negotiation time  : {avg_negotiation}")
        self.log(
            "messages              : "
            + ", ".join(f"{key}={value}" for key, value in sorted(self.messages.items()))
        )

        if self.min_speed_by_vehicle:
            speed_summary = ", ".join(
                f"{vehicle_id}={speed:.2f}m/s"
                for vehicle_id, speed in sorted(self.min_speed_by_vehicle.items())
            )
            self.log(f"min speeds            : {speed_summary}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run the SUMO/TraCI V2X lane-merge demo scenarios.",
    )
    parser.add_argument(
        "--map",
        choices=sorted(MAP_PROFILES),
        default="merge",
        help="SUMO map profile to use.",
    )
    parser.add_argument(
        "--scenario",
        choices=sorted(next(iter(SCENARIOS_BY_MAP.values()))),
        default="base",
        help="Scenario to run.",
    )
    parser.add_argument("--all", action="store_true", help="Run all scenarios for the selected map.")
    parser.add_argument(
        "--demo",
        action="store_true",
        help="Use presentation-friendly GUI defaults: slower playback, stronger zoom and visible markers.",
    )
    parser.add_argument("--repeat", type=int, default=1, help="Repeat the selected scenario N times in one SUMO run.")
    parser.add_argument("--loop", action="store_true", help="Repeat forever. Press Ctrl+C to stop and print a summary.")
    parser.add_argument("--loop-pause", type=float, default=2.0, help="Seconds to wait before spawning the next loop cycle.")
    parser.add_argument("--gui", action="store_true", help="Use sumo-gui instead of headless sumo.")
    parser.add_argument("--delay", type=int, default=80, help="GUI delay in ms between simulation steps.")
    parser.add_argument("--zoom", type=float, default=900.0, help="SUMO-GUI zoom used when following the ramp vehicle.")
    parser.add_argument(
        "--marker-radius",
        type=float,
        default=7.0,
        help="Radius in meters for GUI-only vehicle markers.",
    )
    parser.add_argument(
        "--badge-size",
        type=float,
        default=4.0,
        help="Size in meters for the real-time status badge next to each vehicle.",
    )
    parser.add_argument(
        "--no-demo-markers",
        dest="demo_markers",
        action="store_false",
        help="Disable the large GUI-only colored markers above vehicles.",
    )
    parser.set_defaults(demo_markers=True)
    parser.add_argument("--end", type=float, default=80.0, help="Maximum simulation time in seconds.")
    parser.add_argument("--step-length", type=float, default=0.1, help="SUMO step length in seconds.")
    parser.add_argument(
        "--comm-range",
        type=float,
        default=220.0,
        help="Maximum V2X delivery distance in meters.",
    )
    parser.add_argument("--trace-cam", action="store_true", help="Print CAM delivery ticks.")
    parser.add_argument("--quiet", action="store_true", help="Only print final metrics.")
    parser.add_argument("--log-file", default=None, help="Write logs to this file. Defaults to logs/<timestamp>_...log.")
    parser.add_argument("--no-log-file", action="store_true", help="Disable log file creation.")
    parser.add_argument(
        "--cam-log-file",
        default=None,
        help="Write ETSI-like CAM JSONL messages to this file. Defaults to logs/<timestamp>_...cam.jsonl.",
    )
    parser.add_argument("--no-cam-log", action="store_true", help="Disable CAM JSONL log creation.")
    return parser.parse_args()


def prepare_log_file(args: argparse.Namespace) -> None:
    if args.demo:
        args.gui = True
        args.delay = max(args.delay, 180)
        args.zoom = max(args.zoom, 1200)
        args.marker_radius = max(args.marker_radius, 9)
        args.badge_size = max(args.badge_size, 5)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    scenario = "all" if args.all else args.scenario

    if args.no_log_file:
        args.log_file = None
    elif args.log_file:
        path = Path(args.log_file)
        if not path.is_absolute():
            path = REPO_ROOT / path
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("", encoding="utf-8")
        args.log_file = str(path)
    else:
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        path = LOG_DIR / f"{timestamp}_{args.map}_{scenario}.log"
        path.write_text("", encoding="utf-8")
        args.log_file = str(path)

    if args.log_file:
        latest = LOG_DIR / "latest.log"
        update_latest_link(latest, Path(args.log_file))

    if not args.quiet:
        if args.log_file:
            print(f"Logs: {args.log_file}")
            print(f"Follow live logs with: tail -f {LOG_DIR / 'latest.log'}")

    if args.no_cam_log:
        args.cam_log_file = None
    elif args.cam_log_file:
        cam_path = Path(args.cam_log_file)
        if not cam_path.is_absolute():
            cam_path = REPO_ROOT / cam_path
        cam_path.parent.mkdir(parents=True, exist_ok=True)
        cam_path.write_text("", encoding="utf-8")
        args.cam_log_file = str(cam_path)
    else:
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        cam_path = LOG_DIR / f"{timestamp}_{args.map}_{scenario}.cam.jsonl"
        cam_path.write_text("", encoding="utf-8")
        args.cam_log_file = str(cam_path)

    if args.cam_log_file:
        update_latest_link(LOG_DIR / "latest.cam.jsonl", Path(args.cam_log_file))
        if not args.quiet:
            print(f"CAM JSONL: {args.cam_log_file}")
            print(f"Follow CAMs with: tail -f {LOG_DIR / 'latest.cam.jsonl'}")


def update_latest_link(latest: Path, target: Path) -> None:
    latest.parent.mkdir(parents=True, exist_ok=True)
    try:
        if latest.exists() or latest.is_symlink():
            latest.unlink()
        latest.symlink_to(target.resolve())
    except OSError:
        latest.write_text(f"Latest log: {target}\n", encoding="utf-8")


def main() -> None:
    args = parse_args()
    prepare_log_file(args)
    if args.scenario not in SCENARIOS_BY_MAP[args.map]:
        available = ", ".join(sorted(SCENARIOS_BY_MAP[args.map]))
        sys.exit(f"Scenario '{args.scenario}' is not available for map '{args.map}'. Available: {available}")

    if args.all:
        scenarios = [name for name in SCENARIO_ORDER if name in SCENARIOS_BY_MAP[args.map]]
        round_index = 0
        try:
            while args.loop or round_index < max(1, args.repeat):
                for index, scenario in enumerate(scenarios):
                    if index or round_index:
                        print()
                    scenario_args = argparse.Namespace(**vars(args))
                    scenario_args.scenario = scenario
                    scenario_args.loop = False
                    scenario_args.repeat = 1
                    LaneMergeDemo(scenario_args).run()
                round_index += 1
        except KeyboardInterrupt:
            print("\nInterrupted by user.")
        return

    LaneMergeDemo(args).run()


if __name__ == "__main__":
    main()
