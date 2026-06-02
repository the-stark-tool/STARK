"""
animate_simulation.py

Reads a STARK simulation CSV and renders an ASCII animation of the two-car scenario,
compressing the full run into a desired length of playback.

Usage:
    python animate_simulation.py [csv_path]

Defaults to simulation_data.csv in the current directory.
"""

import csv
import os
import sys
import time

# Display configuration 
ROAD_WIDTH = 100         # columns of road to render
TOTAL_DURATION = 15.0    # seconds for the full simulation
VIEW_MARGIN = 100.0      # world-unit margin on each side of the viewport

# ANSI escape codes (work in Windows Terminal, PowerShell 7+, modern cmd.exe,
# and all unix terminals)
CLEAR_SCREEN = "\033[2J"
CURSOR_HOME = "\033[H"
HIDE_CURSOR = "\033[?25l"
SHOW_CURSOR = "\033[?25h"
RESET = "\033[0m"
BOLD = "\033[1m"
YELLOW = "\033[33m"
RED = "\033[31m"
GREEN = "\033[32m"
DIM = "\033[2m"


def enable_ansi_on_windows() -> None:
    """A no-op os.system call enables ANSI codes in Windows 10+ terminals."""
    if os.name == "nt":
        os.system("")


def load_data(csv_path: str) -> list[dict]:
    """Load CSV rows as list of dicts with float values."""
    with open(csv_path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        rows = []
        for r in reader:
            rows.append({
                "step": int(r["step"]),
                "my_x": float(r["my_x"]),
                "my_y": float(r["my_y"]),
                "my_speed": float(r["my_speed"]),
                "other_x": float(r["other_x"]),
                "other_y": float(r["other_y"]),
                "other_speed": float(r["other_speed"]),
                "dist": float(r["dist"]),
                "rss_gap": float(r["RSS_gap"]),
                "crash": float(r["crash"]),
                "attack": r.get("attack", "None"), 
            })

    # Compute per-step speed deltas so the animation can highlight rapid braking events
    for i, row in enumerate(rows):
        if i == 0:
            row["my_speed_delta"] = 0.0
            row["other_speed_delta"] = 0.0
        else:
            row["my_speed_delta"] = row["my_speed"] - rows[i - 1]["my_speed"]
            row["other_speed_delta"] = row["other_speed"] - rows[i - 1]["other_speed"]
    return rows


def compute_viewport(rows: list[dict]) -> tuple[float, float]:
    """Return the world-x bounds spanning the whole run, with margin.

    The viewport is fixed for the entire animation so both cars travel across
    the road with their relative spacing preserved."""
    min_x = min(min(r["my_x"], r["other_x"]) for r in rows)
    max_x = max(max(r["my_x"], r["other_x"]) for r in rows)
    return min_x - VIEW_MARGIN, max_x + VIEW_MARGIN


def get_attack_status(attack: str):
    """Return (plain_text, colored_text) for the attack label coming from the CSV.

    Plain text is needed for accurate padding. ANSI codes are invisible
    but still count toward string length, so we pad on plain then swap
    in the colored version. Simply add a new attack to have it displayed."""
    if attack == "Dazzle":
        plain = "[DAZZLE]"
        colored = f"{BOLD}{YELLOW}[DAZZLE]{RESET}"
    elif attack == "Blackout":
        plain = "[BLACKOUT]"
        colored = f"{BOLD}{RED}[BLACKOUT]{RESET}"
    else:
        plain = "(none - LiDAR clean)"
        colored = f"{DIM}{plain}{RESET}"
    return plain, colored


def padded_line(content_plain: str, content_colored: str) -> str:
    """Build a bordered status line, padding based on visible (plain) length."""
    padding = max(0, ROAD_WIDTH - len(content_plain))
    return f"| {content_colored}{' ' * padding} |"


def render_frame(row: dict, total_steps: int, view_left: float, view_right: float) -> str:
    """Return the full ASCII frame for this simulation step as a single string."""
    my_x, my_y = row["my_x"], row["my_y"]
    other_x, other_y = row["other_x"], row["other_y"]
    view_range = view_right - view_left

    def to_col(world_x: float) -> int:
        """World x-coordinate -> column index in the road strip."""
        col = int((world_x - view_left) / view_range * ROAD_WIDTH)
        return max(0, min(ROAD_WIDTH - 1, col))

    # Build the two-lane road. y >= 4 is the left lane (top of display),
    # y < 4 is the right lane (bottom of display). Cars near the divider
    # (y between 3 and 5) are drawn straddling both rows.
    def lane_chars(car_label: str, car_y: float):
        if car_y >= 5:
            return (car_label, " ")
        if car_y <= 3:
            return (" ", car_label)
        return (car_label, car_label)  # mid-lane-change: show in both

    left_lane = [" "] * ROAD_WIDTH
    right_lane = [" "] * ROAD_WIDTH

    my_col = to_col(my_x)
    other_col = to_col(other_x)

    other_left, other_right = lane_chars("O", other_y)
    if other_left != " ":
        left_lane[other_col] = other_left
    if other_right != " ":
        right_lane[other_col] = other_right

    my_left, my_right = lane_chars("M", my_y)
    if my_left != " ":
        left_lane[my_col] = my_left
    if my_right != " ":
        right_lane[my_col] = my_right

    # Lane divider with dashed pattern shifted by view position for a
    # subtle scrolling effect as the cars move forward.
    divider_shift = int(view_left / 5) % 4
    divider = "".join("-" if (i + divider_shift) % 4 < 2 else " "
                       for i in range(ROAD_WIDTH))

    edge = "=" * ROAD_WIDTH

    # Status text
    crashed = row["crash"] > 0.5
    if crashed:
        status = "*** CRASH ***"
    elif row["dist"] < row["rss_gap"] * 0.5:
        status = "DANGER"
    elif row["dist"] < row["rss_gap"]:
        status = "WARNING"
    else:
        status = "SAFE"

    elapsed = row["step"] / total_steps * TOTAL_DURATION
    attacks_plain, attacks_colored = get_attack_status(row["attack"])

    # Compose the frame.
    status_line = padded_line(
        f"Step: {row['step']:3d} / {total_steps - 1:3d}"
        f"   Time: {elapsed:5.2f}s / {TOTAL_DURATION:5.2f}s"
        f"   Status: {status}",
        f"Step: {row['step']:3d} / {total_steps - 1:3d}"
        f"   Time: {elapsed:5.2f}s / {TOTAL_DURATION:5.2f}s"
        f"   Status: {status}",
    )
    attack_line = padded_line(
        f"Active attacks: {attacks_plain}",
        f"Active attacks: {attacks_colored}",
    )

    # Speed lines with optional [BRAKING] indicator when speed drops sharply
    # between steps. Threshold -2.0 catches min-brake (-3) and max-brake (-5).
    BRAKE_THRESHOLD = -2.0

    def speed_line(label: str, x: float, y: float, speed: float, delta: float) -> str:
        is_braking = delta < BRAKE_THRESHOLD
        prefix = f"{label} x = {x:10.2f}   y = {y:.2f}   speed: {speed:6.2f}"
        plain_indicator = "   [BRAKING]" if is_braking else ""
        colored_indicator = f"   {BOLD}{RED}[BRAKING]{RESET}" if is_braking else ""
        return padded_line(prefix + plain_indicator, prefix + colored_indicator)

    my_speed_line = speed_line(
        "[M] My car:   ", my_x, my_y, row["my_speed"], row["my_speed_delta"]
    )
    other_speed_line = speed_line(
        "[O] Other car:", other_x, other_y, row["other_speed"], row["other_speed_delta"]
    )

    lines = [
        "+" + "=" * (ROAD_WIDTH + 2) + "+",
        "| " + "STARK Two-Lane LiDAR Attack Simulation".center(ROAD_WIDTH) + " |",
        "+" + "=" * (ROAD_WIDTH + 2) + "+",
        "| " + edge + " |",
        "| " + "".join(left_lane) + " |",
        "| " + divider + " |",
        "| " + "".join(right_lane) + " |",
        "| " + edge + " |",
        "+" + "-" * (ROAD_WIDTH + 2) + "+",
        status_line,
        my_speed_line,
        other_speed_line,
        f"| Distance: {row['dist']:8.2f} units"
        f"   RSS gap: {row['rss_gap']:8.2f} units".ljust(ROAD_WIDTH + 2) + " |",
        attack_line,
        "+" + "-" * (ROAD_WIDTH + 2) + "+",
    ]
    return "\n".join(lines)


def animate(rows: list[dict]) -> None:
    enable_ansi_on_windows()
    total_steps = len(rows)
    frame_interval = TOTAL_DURATION / total_steps
    view_left, view_right = compute_viewport(rows)

    sys.stdout.write(CLEAR_SCREEN + HIDE_CURSOR)
    sys.stdout.flush()

    start = time.time()
    try:
        for i, row in enumerate(rows):
            frame = render_frame(row, total_steps, view_left, view_right)
            sys.stdout.write(CURSOR_HOME + frame)
            sys.stdout.flush()

            target = start + (i + 1) * frame_interval
            remaining = target - time.time()
            if remaining > 0:
                time.sleep(remaining)
    finally:
        sys.stdout.write(SHOW_CURSOR + "\n")
        sys.stdout.flush()


if __name__ == "__main__":
    csv_path = sys.argv[1] if len(sys.argv) > 1 else "simulation_data.csv"
    if not os.path.exists(csv_path):
        print(f"Error: file not found: {csv_path}")
        sys.exit(1)
    rows = load_data(csv_path)
    print(f"Loaded {len(rows)} simulation steps from {csv_path}")
    print(f"Animating over {TOTAL_DURATION:.1f} seconds...")
    time.sleep(1)
    animate(rows)
    print(f"Animation complete. Final distance: {rows[-1]['dist']:.2f}")