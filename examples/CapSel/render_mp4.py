"""Render one CapSel simulation as an MP4 file.

The video uses the same fixed road view as the terminal animation.

Usage:
    python render_mp4.py [simulation_data.csv] [simulation.mp4]
"""

import os
import re
import sys

import imageio.v2 as imageio
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from animate_simulation import (
    LEFT_LANE_FRAME_LINE,
    TOTAL_DURATION,
    compute_viewport,
    load_data,
    render_frame,
)


FONT_SIZE = 16
LINE_SPACING = 4
PAD_X = 20
PAD_Y = 20
BACKGROUND = (15, 15, 25)
FOREGROUND = (220, 220, 230)
SPOOF_COLOR = (255, 190, 60)
REMOVAL_COLOR = (255, 80, 80)
DIM_COLOR = (130, 130, 145)
ANSI_PATTERN = re.compile(r"\x1b\[[0-9;]*m")


def find_monospace_font(size: int):
    """Find a common monospace font."""
    for name in (
        "Consolas.ttf",
        "consola.ttf",
        "DejaVuSansMono.ttf",
        "Menlo.ttc",
        "Courier New.ttf",
        "cour.ttf",
    ):
        try:
            return ImageFont.truetype(name, size)
        except (OSError, IOError):
            continue
    print("Warning: a TrueType monospace font was not found.")
    return ImageFont.load_default()


def strip_ansi(text: str) -> str:
    """Remove terminal color codes."""
    return ANSI_PATTERN.sub("", text)


def color_for_line(line: str) -> tuple[int, int, int]:
    """Select a color for an important status line."""
    if "Attack: SPOOF" in line:
        return SPOOF_COLOR
    if "Attack: REMOVAL" in line:
        return REMOVAL_COLOR
    if "Attack: NONE" in line:
        return DIM_COLOR
    return FOREGROUND


def render_image(row: dict, total_steps: int, view_left: float, view_right: float, font):
    """Convert one text frame to one image."""
    frame = render_frame(row, total_steps, view_left, view_right)
    lines = [strip_ansi(line) for line in frame.splitlines()]
    box = font.getbbox("M")
    line_height = box[3] - box[1] + LINE_SPACING
    width = max(font.getbbox(line)[2] for line in lines) + 2 * PAD_X
    height = line_height * len(lines) + 2 * PAD_Y

    image = Image.new("RGB", (width, height), BACKGROUND)
    draw = ImageDraw.Draw(image)
    for index, line in enumerate(lines):
        line_x = PAD_X
        line_y = PAD_Y + index * line_height
        draw.text(
            (line_x, line_y),
            line,
            fill=color_for_line(line),
            font=font,
        )
        if index == LEFT_LANE_FRAME_LINE:
            marker_index = line.find("S")
            if marker_index >= 0:
                marker_x = line_x + font.getlength(line[:marker_index])
                draw.text((marker_x, line_y), "S", fill=SPOOF_COLOR, font=font)
    return image


def build_mp4(csv_path: str, output_path: str) -> None:
    """Write all simulation frames to an MP4 file."""
    rows = load_data(csv_path)
    total_steps = len(rows)
    view_left, view_right = compute_viewport(rows)
    frames_per_second = total_steps / TOTAL_DURATION
    font = find_monospace_font(FONT_SIZE)

    first = render_image(rows[0], total_steps, view_left, view_right, font)
    width, height = first.size
    width += width % 2
    height += height % 2

    writer = imageio.get_writer(
        output_path,
        fps=frames_per_second,
        codec="libx264",
        quality=8,
        pixelformat="yuv420p",
        macro_block_size=1,
    )
    try:
        for index, row in enumerate(rows):
            image = render_image(row, total_steps, view_left, view_right, font)
            if image.size != (width, height):
                canvas = Image.new("RGB", (width, height), BACKGROUND)
                canvas.paste(image, (0, 0))
                image = canvas
            writer.append_data(np.asarray(image))
            if (index + 1) % 40 == 0:
                print(f"Rendered {index + 1} of {total_steps} frames.")
    finally:
        writer.close()

    size_mb = os.path.getsize(output_path) / (1024 * 1024)
    print(f"Wrote {output_path} ({size_mb:.2f} MB).")


if __name__ == "__main__":
    source_path = sys.argv[1] if len(sys.argv) > 1 else "simulation_data.csv"
    target_path = sys.argv[2] if len(sys.argv) > 2 else "simulation.mp4"
    if not os.path.exists(source_path):
        print(f"Error: file not found: {source_path}")
        sys.exit(1)
    build_mp4(source_path, target_path)

