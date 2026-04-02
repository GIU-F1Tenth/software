#!/usr/bin/env python3

from __future__ import annotations

import argparse
import binascii
import os
from pathlib import Path
import struct
import subprocess
import sys
import zlib
import socket

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Save the current Nav2 map and convert the resulting PGM to grayscale PNG."
    )
    parser.add_argument(
        "-o",
        "--output",
        default="map",
        help=(
            "Base output path without extension. "
            "Example: ~/maps/my_map -> my_map.pgm, my_map.yaml, my_map.png"
        ),
    )
    return parser.parse_args()


def run_map_saver(output_base: Path) -> None:
    """
    Run Nav2 map saver CLI.

    Nav2 map_saver_cli uses '-f' with a base filename/path, not the final '.pgm' name.
    """
    output_base.parent.mkdir(parents=True, exist_ok=True)

    cmd = [
        "ros2",
        "run",
        "nav2_map_server",
        "map_saver_cli",
        "-f",
        str(output_base),
    ]

    print(f"[INFO] Saving map to base path: {output_base}")
    print(f"[INFO] Running: {' '.join(cmd)}")

    try:
        result = subprocess.run(
            cmd,
            check=True,
            text=True,
            capture_output=True,
        )
    except FileNotFoundError:
        raise RuntimeError(
            "Could not find 'ros2'. Make sure your ROS 2 / Nav2 environment is sourced."
        ) from None
    except subprocess.CalledProcessError as exc:
        stderr = exc.stderr.strip() if exc.stderr else ""
        stdout = exc.stdout.strip() if exc.stdout else ""
        details = "\n".join(part for part in [stdout, stderr] if part)
        raise RuntimeError(f"map_saver_cli failed.\n{details}") from exc

    if result.stdout.strip():
        print(result.stdout.strip())
    if result.stderr.strip():
        print(result.stderr.strip(), file=sys.stderr)


def _read_token(stream) -> bytes:
    """
    Read the next non-comment token from a PGM file.
    Supports comments starting with '#'.
    """
    token = bytearray()

    while True:
        ch = stream.read(1)
        if not ch:
            break

        if ch in b" \t\r\n":
            continue

        if ch == b"#":
            stream.readline()
            continue

        token.extend(ch)
        break

    while True:
        ch = stream.read(1)
        if not ch or ch in b" \t\r\n":
            break
        if ch == b"#":
            stream.readline()
            break
        token.extend(ch)

    if not token:
        raise ValueError("Unexpected end of file while reading PGM header.")

    return bytes(token)


def read_pgm(pgm_path: Path) -> tuple[int, int, bytes]:
    """
    Read a PGM image and return (width, height, 8-bit grayscale pixels).

    Supports:
    - P2 (ASCII)
    - P5 (binary)

    If maxval != 255, pixel values are scaled to 0..255.
    """
    with pgm_path.open("rb") as f:
        magic = _read_token(f)
        if magic not in (b"P2", b"P5"):
            raise ValueError(f"Unsupported PGM format: {magic!r}. Expected P2 or P5.")

        width = int(_read_token(f))
        height = int(_read_token(f))
        maxval = int(_read_token(f))

        if width <= 0 or height <= 0:
            raise ValueError("Invalid PGM dimensions.")
        if maxval <= 0:
            raise ValueError("Invalid PGM max value.")

        pixel_count = width * height

        if magic == b"P5":
            if maxval < 256:
                raw = f.read(pixel_count)
                if len(raw) != pixel_count:
                    raise ValueError("PGM pixel data is incomplete.")
                pixels = raw
            else:
                raw = f.read(pixel_count * 2)
                if len(raw) != pixel_count * 2:
                    raise ValueError("16-bit PGM pixel data is incomplete.")

                values = []
                for i in range(0, len(raw), 2):
                    value = (raw[i] << 8) | raw[i + 1]
                    values.append((value * 255) // maxval)
                pixels = bytes(values)

        else:  # P2
            values = []
            for _ in range(pixel_count):
                token = _read_token(f)
                value = int(token)
                values.append((value * 255) // maxval)

            pixels = bytes(values)

    return width, height, pixels


def _png_chunk(chunk_type: bytes, data: bytes) -> bytes:
    crc = binascii.crc32(chunk_type)
    crc = binascii.crc32(data, crc) & 0xFFFFFFFF
    return (
        struct.pack(">I", len(data))
        + chunk_type
        + data
        + struct.pack(">I", crc)
    )


def write_grayscale_png(png_path: Path, width: int, height: int, pixels: bytes) -> None:
    """
    Write an 8-bit grayscale PNG using only the Python standard library.
    """
    if len(pixels) != width * height:
        raise ValueError("Pixel buffer size does not match image dimensions.")

    signature = b"\x89PNG\r\n\x1a\n"

    ihdr = struct.pack(
        ">IIBBBBB",
        width,
        height,
        8,  # bit depth
        0,  # color type: grayscale
        0,  # compression method
        0,  # filter method
        0,  # interlace method
    )

    # PNG scanlines need a filter byte at the start of each row.
    scanlines = bytearray()
    row_size = width
    for y in range(height):
        start = y * row_size
        end = start + row_size
        scanlines.append(0)  # filter type 0 = None
        scanlines.extend(pixels[start:end])

    compressed = zlib.compress(bytes(scanlines), level=9)

    png_bytes = (
        signature
        + _png_chunk(b"IHDR", ihdr)
        + _png_chunk(b"IDAT", compressed)
        + _png_chunk(b"IEND", b"")
    )

    with png_path.open("wb") as f:
        f.write(png_bytes)


def main() -> int:
    args = parse_args()

    output_base = Path(os.path.expanduser(args.output)).resolve()
    pgm_path = output_base.with_suffix(".pgm")
    png_path = output_base.with_suffix(".png")
    yaml_path = output_base.with_suffix(".yaml")

    try:
        run_map_saver(output_base)

        if not pgm_path.exists():
            raise FileNotFoundError(
                f"Map saver finished, but expected PGM file was not found: {pgm_path}"
            )

        width, height, pixels = read_pgm(pgm_path)
        write_grayscale_png(png_path, width, height, pixels)

        print(f"[INFO] PGM map saved at : {pgm_path}")
        print(f"[INFO] PNG map saved at : {png_path}")
        print(f"[INFO] YAML map saved at : {yaml_path}")
        print(f"[INFO] Image size      : {width} x {height}")
        print(f"[INFO] To download the images locally, run this command:")
        
        user = os.getenv("USER") or "user"
        hostname = socket.gethostname()
        ip = socket.gethostbyname(hostname)
        
        print(f"    scp {user}@{ip}:{pgm_path} ~/path/to/save/")
        print(f"    scp {user}@{ip}:{png_path} ~/path/to/save/")
        print(f"    scp {user}@{ip}:{yaml_path} ~/path/to/save/")
        return 0

    except Exception as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())