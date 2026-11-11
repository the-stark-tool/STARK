"""Show one CapSel run in a terminal.

The road view is fixed. It does not follow the ego car.

Usage:
    python animate_simulation.py [csv_path] [--run ID | --first-crash]
"""

import argparse
import csv
from itertools import groupby
import math
from operator import itemgetter
import os
import sys
import time


ROAD_START = 0.0
ROAD_END = 4000.0
ROAD_WIDTH = 100
TOTAL_DURATION = 15.0
LEFT_LANE_FRAME_LINE = 5
OBJECT_SPOOFED = 3
SENSOR_RANGE = 450.0
LEFT_LANE_Y = 6.0

CLEAR_SCREEN = "\033[2J"
CURSOR_HOME = "\033[H"
HIDE_CURSOR = "\033[?25l"
SHOW_CURSOR = "\033[?25h"
RESET = "\033[0m"
BOLD = "\033[1m"
RED = "\033[31m"
YELLOW = "\033[33m"
GREEN = "\033[32m"

OBJECT_NAMES = {
    0: "NONE",
    1: "LEAD",
    2: "ONCOMING",
    3: "SPOOFED",
}

MODE_NAMES = {
    0: "FOLLOW",
    1: "MOVE LEFT",
    2: "PASS",
    3: "MOVE RIGHT",
    4: "ABORT",
    5: "CRUISE",
}


def enable_ansi_on_windows() -> None:
    """Enable ANSI colors in a Windows terminal when this is possible."""
    if os.name != "nt":
        return
    try:
        os.system("")
    except OSError:
        pass


def add_replay_arguments(parser: argparse.ArgumentParser) -> None:
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument("--run", type=int, help="Select a saved run by its zero-based ID.")
    selection.add_argument("--first-crash", action="store_true", help="Replay the first run with a crash.")


def select_batch_run(reader: csv.DictReader, run: int | None, first_crash: bool) -> list[dict]:
    """Select one trajectory from a batch ordered by run and step."""
    if run is None and not first_crash:
        raise ValueError("For a batch CSV, choose --run ID or --first-crash.")

    for run_id, frames in groupby(reader, key=itemgetter("run")):
        if first_crash:
            rows = list(frames)
            if any(float(row["crash"]) > 0.5 for row in rows):
                return rows
        elif int(run_id) == run:
            return list(frames)

    if first_crash:
        raise ValueError("No crashed run was found in the batch.")
    raise ValueError(f"Run {run} was not found in the batch.")


def load_data(path: str, *, run: int | None = None, first_crash: bool = False) -> list[dict]:
    """Read the preview or one saved trajectory from a batch CSV."""
    if run is not None and (run < 0 or first_crash):
        raise ValueError("Choose a nonnegative run ID or --first-crash.")

    rows = []
    with open(path, newline="", encoding="utf-8") as source:
        reader = csv.DictReader(source)
        if "run" in (reader.fieldnames or ()):
            raw_rows = select_batch_run(reader, run, first_crash)
        elif run is not None or first_crash:
            raise ValueError("--run and --first-crash require a batch CSV with a run column.")
        else:
            raw_rows = reader

        for raw in raw_rows:
            row = {key: float(value) for key, value in raw.items()
                   if key not in ("attack", "condition", "run")}
            if "run" in raw:
                row["run"] = int(raw["run"])
            attack = raw.get("attack", raw.get("condition", "none")).split("_", 1)[0].upper()
            row["attack"] = "NONE" if attack == "NOMINAL" else attack
            rows.append(row)

    if not rows:
        raise ValueError("The simulation file has no data rows.")

    previous_speed = rows[0]["ego_speed"]
    for row in rows:
        row["ego_speed_delta"] = row["ego_speed"] - previous_speed
        previous_speed = row["ego_speed"]
    return rows


def compute_viewport(_rows: list[dict]) -> tuple[float, float]:
    """Return the fixed world limits for every frame."""
    return ROAD_START, ROAD_END


def _padded_line(plain: str, colored: str | None = None) -> str:
    """Make one line with a fixed text width."""
    shown = plain if colored is None else colored
    padding = max(0, ROAD_WIDTH - len(plain))
    return f"| {shown}{' ' * padding} |"


def _object_text(row: dict, sector: str) -> str:
    """Return the object and distance for one LiDAR sector."""
    object_code = int(round(row[f"lidar_{sector}_type"]))
    distance = row[f"lidar_{sector}_distance"]
    return f"{OBJECT_NAMES.get(object_code, 'UNKNOWN'):8s} {distance:6.1f}"


def spoofed_vehicle_world_x(row: dict) -> float | None:
    """Return the displayed world position of the spoofed vehicle."""
    for sector in ("left", "centre", "right"):
        object_code = int(round(row[f"lidar_{sector}_type"]))
        if object_code == OBJECT_SPOOFED:
            return row["ego_x"] + row[f"lidar_{sector}_distance"]
    return None


def oncoming_vehicle_numbers(row: dict) -> list[int]:
    """Return all numbered oncoming-vehicle columns in one simulation row."""
    numbers = []
    for key in row:
        if not key.startswith("oncoming_") or not key.endswith("_x"):
            continue
        number_text = key[len("oncoming_") : -len("_x")]
        if number_text.isdigit():
            numbers.append(int(number_text))
    return sorted(numbers)


def removed_vehicle_number(row: dict) -> int | None:
    """Return the physical oncoming vehicle hidden by a removal attack."""
    if row["attack"] != "REMOVAL" or row["attack_success"] <= 0.5:
        return None

    candidates = []
    for number in oncoming_vehicle_numbers(row):
        vehicle_x = row[f"oncoming_{number}_x"]
        if vehicle_x < row["ego_x"]:
            continue
        distance = math.hypot(vehicle_x - row["ego_x"], LEFT_LANE_Y - row["ego_y"])
        if distance <= SENSOR_RANGE:
            candidates.append((distance, number))

    return min(candidates)[1] if candidates else None


def render_frame(
    row: dict,
    total_steps: int,
    view_left: float = ROAD_START,
    view_right: float = ROAD_END,
) -> str:
    """Build one frame with a fixed road view."""
    view_range = view_right - view_left

    def to_column(world_x: float) -> int | None:
        if world_x < view_left or world_x > view_right:
            return None
        column = int((world_x - view_left) / view_range * (ROAD_WIDTH - 1))
        return max(0, min(ROAD_WIDTH - 1, column))

    left_lane = [" "] * ROAD_WIDTH
    right_lane = [" "] * ROAD_WIDTH

    def put_car(
        lane: list[str],
        world_x: float,
        label: str,
        replace_existing: bool = False,
    ) -> None:
        column = to_column(world_x)
        if column is None:
            return
        if replace_existing or lane[column] == " ":
            lane[column] = label
        else:
            lane[column] = "X"

    removed_number = removed_vehicle_number(row)
    for number in oncoming_vehicle_numbers(row):
        marker = "R" if number == removed_number else "<"
        put_car(
            left_lane,
            row[f"oncoming_{number}_x"],
            marker,
            replace_existing=marker == "R",
        )

    spoofed_world_x = spoofed_vehicle_world_x(row)
    if spoofed_world_x is not None:
        # Keep the false vehicle visible when two markers use one column.
        put_car(left_lane, spoofed_world_x, "S", replace_existing=True)

    put_car(right_lane, row["lead_x"], "L")

    ego_y = row["ego_y"]
    if ego_y >= 5.0:
        put_car(left_lane, row["ego_x"], "E")
    elif ego_y <= 3.0:
        put_car(right_lane, row["ego_x"], "E")
    else:
        put_car(left_lane, row["ego_x"], "E")
        put_car(right_lane, row["ego_x"], "E")

    divider = "".join("-" if index % 4 < 2 else " " for index in range(ROAD_WIDTH))
    edge = "=" * ROAD_WIDTH
    left_lane_plain = "".join(left_lane)
    left_lane_colored = (
        left_lane_plain
        .replace("S", f"{YELLOW}S{RESET}")
        .replace("R", f"{YELLOW}R{RESET}")
    )

    crashed = row["crash"] > 0.5
    completed = row["overtake_complete"] > 0.5
    if crashed:
        status_plain = "CRASH"
        status_colored = f"{BOLD}{RED}CRASH{RESET}"
    elif completed:
        status_plain = "OVERTAKE COMPLETE"
        status_colored = f"{GREEN}OVERTAKE COMPLETE{RESET}"
    elif row["controller_mode"] == 4.0:
        status_plain = "ABORT OVERTAKE"
        status_colored = f"{YELLOW}ABORT OVERTAKE{RESET}"
    else:
        status_plain = "RUNNING"
        status_colored = "RUNNING"

    attack_name = row["attack"]
    attack_active = row["attack_success"] > 0.5
    if attack_name == "NONE":
        attack_plain = "NONE"
        attack_colored = f"{GREEN}NONE{RESET}"
    else:
        result = "SUCCESS" if attack_active else "FAILURE"
        attack_plain = f"{attack_name} {result}"
        color = RED if attack_active else GREEN
        attack_colored = f"{color}{attack_plain}{RESET}"

    step = int(round(row["step"]))
    elapsed = step / max(1, total_steps - 1) * TOTAL_DURATION
    mode = MODE_NAMES.get(int(round(row["controller_mode"])), "UNKNOWN")
    brake = row["ego_speed_delta"] < -2.0
    brake_plain = "  BRAKING" if brake else ""
    brake_colored = f"  {BOLD}{RED}BRAKING{RESET}" if brake else ""

    ego_plain = (
        f"Ego:  x={row['ego_x']:7.1f}  y={ego_y:3.1f}  "
        f"speed={row['ego_speed']:5.1f}{brake_plain}"
    )
    ego_colored = (
        f"Ego:  x={row['ego_x']:7.1f}  y={ego_y:3.1f}  "
        f"speed={row['ego_speed']:5.1f}{brake_colored}"
    )
    lidar_plain = (
        f"LiDAR L {_object_text(row, 'left')}  "
        f"C {_object_text(row, 'centre')}  "
        f"R {_object_text(row, 'right')}"
    )

    lines = [
        "+" + "=" * (ROAD_WIDTH + 2) + "+",
        "| " + "CapSel bidirectional LiDAR attack simulation".center(ROAD_WIDTH) + " |",
        "+" + "=" * (ROAD_WIDTH + 2) + "+",
        _padded_line(
            f"Fixed world view: {view_left:.0f} to {view_right:.0f}   "
            "E=ego  L=lead  <=real oncoming  S=spoofed  R=removed from LiDAR"
        ),
        "| " + edge + " |",
        "| " + left_lane_colored + " |",
        "| " + divider + " |",
        "| " + "".join(right_lane) + " |",
        "| " + edge + " |",
        "+" + "-" * (ROAD_WIDTH + 2) + "+",
        _padded_line(
            f"Step {step:3d}/{total_steps - 1:3d}  Time {elapsed:5.2f}s  Status: {status_plain}",
            f"Step {step:3d}/{total_steps - 1:3d}  Time {elapsed:5.2f}s  Status: {status_colored}",
        ),
        _padded_line(ego_plain, ego_colored),
        _padded_line(f"Lead: x={row['lead_x']:7.1f}  speed={row['lead_speed']:5.1f}  Controller: {mode}"),
        _padded_line(
            f"Oncoming TTC={row['oncoming_ttc']:6.2f}  RSS gap={row['rss_gap']:6.1f}  "
            f"Abort count={int(round(row['abort_count']))}"
        ),
        _padded_line(lidar_plain),
        _padded_line(
            f"Attack: {attack_plain}  bearing={row['attacker_bearing_degrees']:5.1f} deg  "
            f"probability={row['attack_success_probability']:.3f}",
            f"Attack: {attack_colored}  bearing={row['attacker_bearing_degrees']:5.1f} deg  "
            f"probability={row['attack_success_probability']:.3f}",
        ),
        "+" + "-" * (ROAD_WIDTH + 2) + "+",
    ]
    return "\n".join(lines)


def animate(rows: list[dict]) -> None:
    """Show all frames at a constant rate."""
    enable_ansi_on_windows()
    total_steps = len(rows)
    frame_interval = TOTAL_DURATION / total_steps
    view_left, view_right = compute_viewport(rows)
    start = time.time()

    sys.stdout.write(CLEAR_SCREEN + HIDE_CURSOR)
    sys.stdout.flush()
    try:
        for index, row in enumerate(rows):
            sys.stdout.write(CURSOR_HOME + render_frame(row, total_steps, view_left, view_right))
            sys.stdout.flush()
            remaining = start + (index + 1) * frame_interval - time.time()
            if remaining > 0.0:
                time.sleep(remaining)
    finally:
        sys.stdout.write(SHOW_CURSOR + "\n")
        sys.stdout.flush()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("csv_path", nargs="?", default="simulation_data.csv")
    add_replay_arguments(parser)
    args = parser.parse_args()
    try:
        simulation_rows = load_data(args.csv_path, run=args.run, first_crash=args.first_crash)
    except (OSError, ValueError) as error:
        parser.error(str(error))
    run_id = simulation_rows[0].get("run")
    run_label = f" for run {run_id}" if run_id is not None else ""
    print(f"Loaded {len(simulation_rows)} steps{run_label} from {args.csv_path}")
    time.sleep(0.5)
    animate(simulation_rows)

