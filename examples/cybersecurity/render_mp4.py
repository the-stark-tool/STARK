"""Render a STARK two-car simulation CSV as an MP4 animation.

Requires:
    pip install pillow imageio imageio-ffmpeg

Usage:
    python render_mp4.py [csv_path] [output_path]
"""

import os
import sys

import imageio.v2 as imageio
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from animate_simulation import TOTAL_DURATION, compute_viewport, load_data, render_frame

FONT_SIZE = 16
LINE_SPACING = 4
PAD_X = 20
PAD_Y = 20
BG_COLOR = (15, 15, 25)
FG_COLOR = (220, 220, 230)
DAZZLE_COLOR = (255, 215, 0)
BLACKOUT_COLOR = (255, 70, 70)
DIM_COLOR = (110, 110, 130)


def find_monospace_font(size: int) -> ImageFont.ImageFont:
    """Return a commonly available monospace font, with a PIL fallback."""
    candidates = [
        "Consolas.ttf",
        "consola.ttf",
        "DejaVuSansMono.ttf",
        "Menlo.ttc",
        "Courier New.ttf",
        "cour.ttf",
    ]
    for candidate in candidates:
        try:
            return ImageFont.truetype(candidate, size)
        except OSError:
            continue

    print("Warning: no TrueType monospace font found; using the PIL default font.")
    return ImageFont.load_default()


def strip_ansi(text: str) -> str:
    """Remove ANSI SGR escape sequences from one rendered terminal line."""
    characters = []
    index = 0
    while index < len(text):
        if text[index] == "\033":
            while index < len(text) and text[index] != "m":
                index += 1
            index += 1
        else:
            characters.append(text[index])
            index += 1
    return "".join(characters)


def color_for_line(line: str) -> tuple[int, int, int]:
    """Select the output colour for a line mentioning the active attack."""
    if "[DAZZLE]" in line:
        return DAZZLE_COLOR
    if "[BLACKOUT]" in line:
        return BLACKOUT_COLOR
    if "(none - LiDAR clean)" in line:
        return DIM_COLOR
    return FG_COLOR


def render_image_frame(
        row: dict,
        total_steps: int,
        view_left: float,
        view_right: float,
        font: ImageFont.ImageFont,
) -> Image.Image:
    """Render one terminal frame to an RGB image."""
    lines = [
        strip_ansi(line)
        for line in render_frame(row, total_steps, view_left, view_right).split("\n")
    ]

    character_box = font.getbbox("M")
    line_height = character_box[3] - character_box[1] + LINE_SPACING
    width = max(font.getbbox(line)[2] for line in lines) + 2 * PAD_X
    height = line_height * len(lines) + 2 * PAD_Y

    image = Image.new("RGB", (width, height), BG_COLOR)
    draw = ImageDraw.Draw(image)
    for index, line in enumerate(lines):
        draw.text((PAD_X, PAD_Y + index * line_height), line, fill=color_for_line(line), font=font)

    return image


def pad_frame(image: Image.Image, width: int, height: int) -> Image.Image:
    """Pad a frame to the locked video dimensions without cropping content."""
    if image.size == (width, height):
        return image

    padded = Image.new("RGB", (width, height), BG_COLOR)
    padded.paste(image, (0, 0))
    return padded


def build_mp4(csv_path: str, output_path: str) -> None:
    """Render the simulation CSV into an H.264-compatible MP4 file."""
    rows = load_data(csv_path)
    if not rows:
        raise ValueError(f"No simulation rows found in {csv_path}")

    total_steps = len(rows)
    view_left, view_right = compute_viewport(rows)
    fps = total_steps / TOTAL_DURATION
    font = find_monospace_font(FONT_SIZE)

    first_frame = render_image_frame(rows[0], total_steps, view_left, view_right, font)
    width, height = first_frame.size
    width += width % 2
    height += height % 2

    print(f"Rendering {total_steps} frames at {fps:.1f} fps ({TOTAL_DURATION:.0f}s total)...")
    writer = imageio.get_writer(
        output_path,
        fps=fps,
        codec="libx264",
        quality=8,
        pixelformat="yuv420p",
        macro_block_size=1,
    )
    try:
        for index, row in enumerate(rows):
            frame = render_image_frame(row, total_steps, view_left, view_right, font)
            writer.append_data(np.asarray(pad_frame(frame, width, height)))
            if (index + 1) % 30 == 0:
                print(f"  ...rendered {index + 1} / {total_steps}")
    finally:
        writer.close()

    size_mb = os.path.getsize(output_path) / (1024 * 1024)
    print(f"Wrote {output_path} ({size_mb:.2f} MB)")


def main() -> None:
    csv_path = sys.argv[1] if len(sys.argv) > 1 else "simulation_data.csv"
    output_path = sys.argv[2] if len(sys.argv) > 2 else "simulation.mp4"
    if not os.path.exists(csv_path):
        raise SystemExit(f"Error: file not found: {csv_path}")

    build_mp4(csv_path, output_path)


if __name__ == "__main__":
    main()
