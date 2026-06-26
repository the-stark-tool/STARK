"""Render a STARK two-car simulation CSV as an ANSI terminal animation.

Usage:
    python animate_simulation.py [csv_path]
"""

import csv
import os
import sys
import time

ROAD_WIDTH = 100
TOTAL_DURATION = 15.0
VIEW_MARGIN = 100.0
BRAKE_THRESHOLD = -2.0

CLEAR_SCREEN = "\033[2J"
CURSOR_HOME = "\033[H"
HIDE_CURSOR = "\033[?25l"
SHOW_CURSOR = "\033[?25h"
RESET = "\033[0m"
BOLD = "\033[1m"
YELLOW = "\033[33m"
RED = "\033[31m"
DIM = "\033[2m"


def enable_ansi_on_windows() -> None:
    """Enable ANSI escape-sequence handling in supported Windows terminals."""
    if os.name == "nt":
        os.system("")


def load_data(csv_path: str) -> list[dict]:
    """Load one simulation trajectory and calculate per-step speed changes."""
    with open(csv_path, "r", encoding="utf-8") as csv_file:
        reader = csv.DictReader(csv_file)
        rows = [
            {
                "step": int(row["step"]),
                "my_x": float(row["my_x"]),
                "my_y": float(row["my_y"]),
                "my_speed": float(row["my_speed"]),
                "other_x": float(row["other_x"]),
                "other_y": float(row["other_y"]),
                "other_speed": float(row["other_speed"]),
                "dist": float(row["dist"]),
                "rss_gap": float(row["RSS_gap"]),
                "crash": float(row["crash"]),
                "attack": row.get("attack", "None"),
            }
            for row in reader
        ]

    for index, row in enumerate(rows):
        previous = rows[index - 1] if index > 0 else row
        row["my_speed_delta"] = row["my_speed"] - previous["my_speed"]
        row["other_speed_delta"] = row["other_speed"] - previous["other_speed"]

    return rows


def compute_viewport(rows: list[dict]) -> tuple[float, float]:
    """Return fixed world-coordinate bounds spanning the complete trajectory."""
    min_x = min(min(row["my_x"], row["other_x"]) for row in rows)
    max_x = max(max(row["my_x"], row["other_x"]) for row in rows)
    return min_x - VIEW_MARGIN, max_x + VIEW_MARGIN


def get_attack_status(attack: str) -> tuple[str, str]:
    """Return plain and ANSI-coloured versions of the current attack label."""
    if attack == "Dazzle":
        return "[DAZZLE]", f"{BOLD}{YELLOW}[DAZZLE]{RESET}"
    if attack == "Blackout":
        return "[BLACKOUT]", f"{BOLD}{RED}[BLACKOUT]{RESET}"

    plain = "(none - LiDAR clean)"
    return plain, f"{DIM}{plain}{RESET}"


def padded_line(plain_text: str, coloured_text: str) -> str:
    """Return a fixed-width bordered line, padded according to visible text."""
    return f"| {coloured_text}{' ' * max(0, ROAD_WIDTH - len(plain_text))} |"


def render_frame(row: dict, total_steps: int, view_left: float, view_right: float) -> str:
    """Render one simulation state as a complete ANSI terminal frame."""
    view_range = view_right - view_left

    def to_column(world_x: float) -> int:
        column = int((world_x - view_left) / view_range * ROAD_WIDTH)
        return max(0, min(ROAD_WIDTH - 1, column))

    def lane_characters(label: str, y_position: float) -> tuple[str, str]:
        if y_position >= 5:
            return label, " "
        if y_position <= 3:
            return " ", label
        return label, label

    left_lane = [" "] * ROAD_WIDTH
    right_lane = [" "] * ROAD_WIDTH

    other_column = to_column(row["other_x"])
    other_left, other_right = lane_characters("O", row["other_y"])
    left_lane[other_column] = other_left
    right_lane[other_column] = other_right

    my_column = to_column(row["my_x"])
    my_left, my_right = lane_characters("M", row["my_y"])
    left_lane[my_column] = my_left
    right_lane[my_column] = my_right

    divider_shift = int(view_left / 5) % 4
    divider = "".join(
        "-" if (column + divider_shift) % 4 < 2 else " "
        for column in range(ROAD_WIDTH)
    )
    edge = "=" * ROAD_WIDTH

    if row["crash"] > 0.5:
        status = "*** CRASH ***"
    elif row["dist"] < row["rss_gap"] * 0.5:
        status = "DANGER"
    elif row["dist"] < row["rss_gap"]:
        status = "WARNING"
    else:
        status = "SAFE"

    time_fraction = row["step"] / max(total_steps - 1, 1)
    elapsed = time_fraction * TOTAL_DURATION
    attack_plain, attack_coloured = get_attack_status(row["attack"])

    def speed_line(label: str, x: float, y: float, speed: float, delta: float) -> str:
        prefix = f"{label} x = {x:10.2f}   y = {y:.2f}   speed: {speed:6.2f}"
        braking = delta < BRAKE_THRESHOLD
        plain_indicator = "   [BRAKING]" if braking else ""
        coloured_indicator = f"   {BOLD}{RED}[BRAKING]{RESET}" if braking else ""
        return padded_line(prefix + plain_indicator, prefix + coloured_indicator)

    status_text = (
        f"Step: {row['step']:3d} / {total_steps - 1:3d}"
        f"   Time: {elapsed:5.2f}s / {TOTAL_DURATION:5.2f}s"
        f"   Status: {status}"
    )
    metric_text = f"Distance: {row['dist']:8.2f} units   RSS gap: {row['rss_gap']:8.2f} units"

    return "\n".join(
        [
            "+" + "=" * (ROAD_WIDTH + 2) + "+",
            "| " + "STARK Two-Lane LiDAR Attack Simulation".center(ROAD_WIDTH) + " |",
            "+" + "=" * (ROAD_WIDTH + 2) + "+",
            "| " + edge + " |",
            "| " + "".join(left_lane) + " |",
            "| " + divider + " |",
            "| " + "".join(right_lane) + " |",
            "| " + edge + " |",
            "+" + "-" * (ROAD_WIDTH + 2) + "+",
            padded_line(status_text, status_text),
            speed_line("[M] My car:   ", row["my_x"], row["my_y"], row["my_speed"], row["my_speed_delta"]),
            speed_line(
                "[O] Other car:",
                row["other_x"],
                row["other_y"],
                row["other_speed"],
                row["other_speed_delta"],
            ),
            padded_line(metric_text, metric_text),
            padded_line(f"Active attack: {attack_plain}", f"Active attack: {attack_coloured}"),
            "+" + "-" * (ROAD_WIDTH + 2) + "+",
        ]
    )


def animate(rows: list[dict]) -> None:
    """Play the supplied trajectory at a fixed total duration."""
    if not rows:
        raise ValueError("The simulation CSV contains no data rows.")

    enable_ansi_on_windows()
    total_steps = len(rows)
    frame_interval = TOTAL_DURATION / total_steps
    view_left, view_right = compute_viewport(rows)

    sys.stdout.write(CLEAR_SCREEN + HIDE_CURSOR)
    sys.stdout.flush()

    start_time = time.perf_counter()
    try:
        for index, row in enumerate(rows):
            sys.stdout.write(CURSOR_HOME + render_frame(row, total_steps, view_left, view_right))
            sys.stdout.flush()

            target_time = start_time + (index + 1) * frame_interval
            remaining = target_time - time.perf_counter()
            if remaining > 0:
                time.sleep(remaining)
    finally:
        sys.stdout.write(SHOW_CURSOR + "\n")
        sys.stdout.flush()


def main() -> None:
    csv_path = sys.argv[1] if len(sys.argv) > 1 else "simulation_data.csv"
    if not os.path.exists(csv_path):
        raise SystemExit(f"Error: file not found: {csv_path}")

    rows = load_data(csv_path)
    if not rows:
        raise SystemExit(f"Error: no simulation rows found in {csv_path}")

    print(f"Loaded {len(rows)} simulation steps from {csv_path}")
    print(f"Animating over {TOTAL_DURATION:.1f} seconds...")
    time.sleep(1)
    animate(rows)
    print(f"Animation complete. Final distance: {rows[-1]['dist']:.2f}")


if __name__ == "__main__":
    main()
