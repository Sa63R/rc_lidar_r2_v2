#!/usr/bin/env python3
"""Convert a PCD map into a simple Nav2-compatible 2D occupancy map."""

import argparse
from pathlib import Path
import sys

import numpy as np
from PIL import Image


PCD_TYPES = {
    ("F", 4): "<f4",
    ("F", 8): "<f8",
    ("I", 1): "i1",
    ("I", 2): "<i2",
    ("I", 4): "<i4",
    ("U", 1): "u1",
    ("U", 2): "<u2",
    ("U", 4): "<u4",
}


def parse_args():
    parser = argparse.ArgumentParser(
        description="Project a PCD point cloud into a trinary Nav2 occupancy map."
    )
    parser.add_argument("--input", required=True, help="Input .pcd file.")
    parser.add_argument(
        "--output-prefix",
        required=True,
        help="Output path prefix. Writes <prefix>.pgm and <prefix>.yaml.",
    )
    parser.add_argument(
        "--resolution",
        type=float,
        default=0.10,
        help="Map resolution in meters per pixel.",
    )
    parser.add_argument(
        "--z-min",
        type=float,
        default=0.15,
        help="Minimum point height to mark as obstacle.",
    )
    parser.add_argument(
        "--z-max",
        type=float,
        default=2.20,
        help="Maximum point height to mark as obstacle.",
    )
    parser.add_argument(
        "--padding",
        type=float,
        default=1.0,
        help="Padding around the point cloud bounds in meters.",
    )
    parser.add_argument(
        "--occupied-radius",
        type=float,
        default=0.15,
        help="Obstacle dilation radius in meters.",
    )
    parser.add_argument(
        "--min-points-per-cell",
        type=int,
        default=1,
        help="Minimum filtered points needed before a cell is occupied.",
    )
    parser.add_argument(
        "--unknown-background",
        action="store_true",
        help="Use unknown background instead of free background.",
    )
    return parser.parse_args()


def read_pcd(path):
    with Path(path).open("rb") as stream:
        header_lines = []
        while True:
            line = stream.readline()
            if not line:
                raise ValueError("PCD header ended before DATA line")
            decoded = line.decode("ascii", errors="replace").strip()
            header_lines.append(decoded)
            if decoded.startswith("DATA"):
                payload = stream.read()
                break

    header = {}
    for line in header_lines:
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        header[parts[0]] = parts[1:]

    required = ["FIELDS", "SIZE", "TYPE", "COUNT", "POINTS", "DATA"]
    missing = [key for key in required if key not in header]
    if missing:
        raise ValueError(f"PCD header is missing: {', '.join(missing)}")

    fields = header["FIELDS"]
    sizes = [int(value) for value in header["SIZE"]]
    types = header["TYPE"]
    counts = [int(value) for value in header["COUNT"]]
    points = int(header["POINTS"][0])
    data_kind = header["DATA"][0].lower()

    if not {"x", "y", "z"}.issubset(fields):
        raise ValueError("PCD must contain x, y and z fields")

    dtype_fields = []
    for field, size, pcd_type, count in zip(fields, sizes, types, counts):
        fmt = PCD_TYPES.get((pcd_type, size))
        if fmt is None:
            raise ValueError(f"Unsupported PCD field type {pcd_type}{size}: {field}")
        if count == 1:
            dtype_fields.append((field, fmt))
        else:
            dtype_fields.append((field, fmt, (count,)))

    dtype = np.dtype(dtype_fields)
    if data_kind == "binary":
        cloud = np.frombuffer(payload, dtype=dtype, count=points)
    elif data_kind == "ascii":
        values = np.loadtxt(payload.splitlines(), dtype=np.float32)
        cloud = np.zeros(values.shape[0], dtype=dtype)
        for index, field in enumerate(fields):
            cloud[field] = values[:, index]
    else:
        raise ValueError(f"Unsupported PCD DATA kind: {data_kind}")

    xyz = np.column_stack((cloud["x"], cloud["y"], cloud["z"])).astype(np.float64)
    return xyz[np.isfinite(xyz).all(axis=1)]


def dilate(mask, radius_cells):
    if radius_cells <= 0 or not mask.any():
        return mask

    occupied = np.argwhere(mask)
    result = mask.copy()
    offsets = []
    for dy in range(-radius_cells, radius_cells + 1):
        for dx in range(-radius_cells, radius_cells + 1):
            if dx * dx + dy * dy <= radius_cells * radius_cells:
                offsets.append((dy, dx))

    height, width = result.shape
    for dy, dx in offsets:
        rows = occupied[:, 0] + dy
        cols = occupied[:, 1] + dx
        valid = (rows >= 0) & (rows < height) & (cols >= 0) & (cols < width)
        result[rows[valid], cols[valid]] = True
    return result


def write_map(points, args):
    resolution = args.resolution
    if resolution <= 0.0:
        raise ValueError("--resolution must be positive")
    if args.z_max <= args.z_min:
        raise ValueError("--z-max must be greater than --z-min")

    xyz_min = points.min(axis=0)
    xyz_max = points.max(axis=0)
    origin_x = float(xyz_min[0] - args.padding)
    origin_y = float(xyz_min[1] - args.padding)
    max_x = float(xyz_max[0] + args.padding)
    max_y = float(xyz_max[1] + args.padding)

    width = int(np.ceil((max_x - origin_x) / resolution)) + 1
    height = int(np.ceil((max_y - origin_y) / resolution)) + 1

    z_mask = (points[:, 2] >= args.z_min) & (points[:, 2] <= args.z_max)
    obstacle_points = points[z_mask]
    if obstacle_points.size == 0:
        raise ValueError("No points left after z filtering")

    cols = np.floor((obstacle_points[:, 0] - origin_x) / resolution).astype(np.int64)
    rows_from_bottom = np.floor(
        (obstacle_points[:, 1] - origin_y) / resolution
    ).astype(np.int64)
    rows = height - 1 - rows_from_bottom

    valid = (rows >= 0) & (rows < height) & (cols >= 0) & (cols < width)
    rows = rows[valid]
    cols = cols[valid]

    counts = np.zeros((height, width), dtype=np.uint16)
    np.add.at(counts, (rows, cols), 1)
    occupied = counts >= max(1, args.min_points_per_cell)
    occupied = dilate(
        occupied,
        int(np.ceil(max(0.0, args.occupied_radius) / resolution)),
    )

    background = 205 if args.unknown_background else 254
    image = np.full((height, width), background, dtype=np.uint8)
    image[occupied] = 0

    output_prefix = Path(args.output_prefix)
    output_prefix.parent.mkdir(parents=True, exist_ok=True)
    pgm_path = output_prefix.with_suffix(".pgm")
    yaml_path = output_prefix.with_suffix(".yaml")

    Image.fromarray(image, mode="L").save(pgm_path)
    yaml_path.write_text(
        "\n".join(
            [
                f"image: {pgm_path.name}",
                "mode: trinary",
                f"resolution: {resolution:.6f}",
                f"origin: [{origin_x:.6f}, {origin_y:.6f}, 0.000000]",
                "negate: 0",
                "occupied_thresh: 0.65",
                "free_thresh: 0.25",
                "",
            ]
        ),
        encoding="utf-8",
    )

    total = width * height
    occupied_count = int(occupied.sum())
    print(f"input_points: {len(points)}")
    print(f"filtered_obstacle_points: {len(obstacle_points)}")
    print(f"resolution: {resolution}")
    print(f"size: {width} x {height}")
    print(f"origin: [{origin_x:.3f}, {origin_y:.3f}, 0.000]")
    print(f"occupied_cells: {occupied_count} ({occupied_count / total:.2%})")
    print(f"wrote: {pgm_path}")
    print(f"wrote: {yaml_path}")


def main():
    args = parse_args()
    try:
        points = read_pcd(args.input)
        if points.size == 0:
            raise ValueError("PCD has no finite xyz points")
        write_map(points, args)
    except Exception as exc:
        print(f"pcd_to_occupancy: error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
