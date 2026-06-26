"""
render_mp4.py

Renders an MP4 of the STARK two-car simulation, mirroring the layout of the
terminal animation in animate_simulation.py. All 300 simulation steps are
compressed into desired duration of playback.

Requires:
    pip install pillow imageio imageio-ffmpeg

Usage:
    python render_mp4.py [csv_path] [output_path]

Defaults:
    csv_path = simulation_data.csv
    output_path = simulation.mp4
"""

import os
import sys

import imageio.v2 as imageio
import numpy as np
from PIL import Image, ImageDraw, ImageFont

# The frame text builder still lives in animate_simulation
from animate_simulation import (
    load_data,
    compute_viewport,
    render_frame as _render_frame_ansi,
    TOTAL_DURATION,
)

# Image configuration
FONT_SIZE = 16
LINE_SPACING = 4
PAD_X = 20
PAD_Y = 20
BG_COLOR = (15, 15, 25)
FG_COLOR = (220, 220, 230)
DAZZLE_COLOR = (255, 215, 0)
BLACKOUT_COLOR = (255, 70, 70)
DIM_COLOR = (110, 110, 130)


def find_monospace_font(size: int) -> ImageFont.FreeTypeFont:
    """Try common monospace fonts across platforms. Falls back to default."""
    candidates = [
        "Consolas.ttf",          # Windows
        "consola.ttf",
        "DejaVuSansMono.ttf",    # Linux
        "Menlo.ttc",             # macOS
        "Courier New.ttf",
        "cour.ttf",
    ]
    for name in candidates:
        try:
            return ImageFont.truetype(name, size)
        except (OSError, IOError):
            continue
    print("Warning: no TTF monospace font found, using PIL default (may render small)")
    return ImageFont.load_default()


def strip_ansi(text: str) -> str:
    """Remove ANSI escape sequences for plain text rendering."""
    out = []
    i = 0
    while i < len(text):
        if text[i] == "\033":
            # Skip until 'm' (end of SGR sequence)
            while i < len(text) and text[i] != "m":
                i += 1
            i += 1  # skip the 'm' itself
        else:
            out.append(text[i])
            i += 1
    return "".join(out)


def color_for_line(plain_line: str):
    """Map a status line to a color based on which attack it refers to."""
    if "[DAZZLE]" in plain_line:
        return DAZZLE_COLOR
    if "[BLACKOUT]" in plain_line:
        return BLACKOUT_COLOR
    if "(none - LiDAR clean)" in plain_line:
        return DIM_COLOR
    return FG_COLOR


def render_image_frame(row: dict, total_steps: int, view_left: float, view_right: float, font) -> Image.Image:
    # Build the same ASCII frame the terminal version uses, then strip ANSI
    ansi_frame = _render_frame_ansi(row, total_steps, view_left, view_right)
    lines = [strip_ansi(line) for line in ansi_frame.split("\n")]

    # Measure once using the widest line to set canvas size
    bbox = font.getbbox("M")
    char_h = bbox[3] - bbox[1]
    line_height = char_h + LINE_SPACING

    width = max(font.getbbox(l)[2] for l in lines) + 2 * PAD_X
    height = line_height * len(lines) + 2 * PAD_Y

    img = Image.new("RGB", (width, height), BG_COLOR)
    draw = ImageDraw.Draw(img)

    for idx, line in enumerate(lines):
        y = PAD_Y + idx * line_height
        color = color_for_line(line)
        draw.text((PAD_X, y), line, fill=color, font=font)

    return img


def build_mp4(csv_path: str, output_path: str) -> None:
    rows = load_data(csv_path)
    total_steps = len(rows)
    view_left, view_right = compute_viewport(rows)
    fps = total_steps / TOTAL_DURATION
    print(f"Rendering {total_steps} frames at {fps:.1f} fps "
          f"({TOTAL_DURATION:.0f}s total)...")

    font = find_monospace_font(FONT_SIZE)

    # Render the first frame to lock in dimensions. MP4 (H.264) requires
    # even width and height, so round up if needed.
    first = render_image_frame(rows[0], total_steps, view_left, view_right, font)
    width, height = first.size
    width += width % 2
    height += height % 2

    writer = imageio.get_writer(
        output_path,
        fps=fps,
        codec="libx264",
        quality=8,            # 1 (worst) .. 10 (best). 8 is high-quality
        pixelformat="yuv420p", # broadest player compatibility
        macro_block_size=1,    # don't auto-resize to multiples of 16
    )
    try:
        for i, row in enumerate(rows):
            img = render_image_frame(row, total_steps, view_left, view_right, font)
            if img.size != (width, height):
                # Pad to the locked-in even dimensions
                padded = img.crop((0, 0, width, height))
                img = padded
            writer.append_data(np.asarray(img))
            if (i + 1) % 30 == 0:
                print(f"  ...rendered {i + 1} / {total_steps}")
    finally:
        writer.close()

    size_mb = os.path.getsize(output_path) / (1024 * 1024)
    print(f"Wrote {output_path} ({size_mb:.2f} MB)")


if __name__ == "__main__":
    csv_path = sys.argv[1] if len(sys.argv) > 1 else "simulation_data.csv"
    output_path = sys.argv[2] if len(sys.argv) > 2 else "simulation.mp4"
    if not os.path.exists(csv_path):
        print(f"Error: file not found: {csv_path}")
        sys.exit(1)
    build_mp4(csv_path, output_path)