#!/usr/bin/env python3
"""Create a simple hand-authored Nav2 occupancy map for the red field."""

import argparse
from pathlib import Path
import sys

import numpy as np
from PIL import Image


def parse_args():
    parser = argparse.ArgumentParser(
        description="Generate a simple Nav2 PGM/YAML map from fixed field dimensions."
    )
    parser.add_argument(
        "--output-prefix",
        required=True,
        help="Output path prefix. Writes <prefix>.pgm, <prefix>.yaml and <prefix>_preview.png.",
    )
    parser.add_argument(
        "--resolution",
        type=float,
        default=0.05,
        help="Map resolution in meters per pixel.",
    )
    parser.add_argument(
        "--wall-thickness",
        type=float,
        default=0.15,
        help="Outer boundary thickness in meters.",
    )
    return parser.parse_args()


def cell_centers(min_value, max_value, count):
    resolution = (max_value - min_value) / count
    return min_value + (np.arange(count) + 0.5) * resolution


def write_yaml(path, image_name, resolution, origin):
    path.write_text(
        "\n".join(
            [
                f"image: {image_name}",
                "mode: trinary",
                f"resolution: {resolution:.6f}",
                f"origin: [{origin[0]:.6f}, {origin[1]:.6f}, 0.000000]",
                "negate: 0",
                "occupied_thresh: 0.65",
                "free_thresh: 0.25",
                "",
            ]
        ),
        encoding="utf-8",
    )


def main():
    args = parse_args()
    if args.resolution <= 0.0:
        print("create_manual_field_map: --resolution must be positive", file=sys.stderr)
        return 1
    if args.wall_thickness < 0.0:
        print("create_manual_field_map: --wall-thickness must be non-negative", file=sys.stderr)
        return 1

    # Map convention:
    #   +X points toward the top of the paper field.
    #   +Y points toward the left of the paper field.
    #   The field's top-left boundary corner is approximately (x=0.0, y=1.0).
    field_x_min = -12.10
    field_x_max = 0.00
    field_y_min = -11.15
    field_y_max = 1.00

    # Red side navigation area. The bottom 2.70 m field section stays occupied.
    free_x_min = -9.40
    free_x_max = 0.00
    free_y_min = -5.00
    free_y_max = 1.00

    # Red Merlin green-block area, simplified as one rectangular obstacle.
    green_x_min = -8.00
    green_x_max = -3.20
    green_y_min = -3.80
    green_y_max = -0.20

    width = int(round((field_x_max - field_x_min) / args.resolution))
    height = int(round((field_y_max - field_y_min) / args.resolution))
    if width <= 0 or height <= 0:
        print("create_manual_field_map: invalid map dimensions", file=sys.stderr)
        return 1

    actual_x_max = field_x_min + width * args.resolution
    actual_y_max = field_y_min + height * args.resolution
    if (
        abs(actual_x_max - field_x_max) > 1e-6
        or abs(actual_y_max - field_y_max) > 1e-6
    ):
        print(
            "create_manual_field_map: field dimensions must be divisible by resolution",
            file=sys.stderr,
        )
        return 1

    xs = cell_centers(field_x_min, field_x_max, width)
    # Image row 0 is the map's maximum Y edge.
    ys = field_y_max - (np.arange(height) + 0.5) * args.resolution
    x_grid, y_grid = np.meshgrid(xs, ys)

    occupied_value = 0
    free_value = 254
    image = np.full((height, width), occupied_value, dtype=np.uint8)

    free_mask = (
        (x_grid >= free_x_min)
        & (x_grid < free_x_max)
        & (y_grid >= free_y_min)
        & (y_grid < free_y_max)
    )
    image[free_mask] = free_value

    wall_mask = (
        (x_grid < field_x_min + args.wall_thickness)
        | (x_grid >= field_x_max - args.wall_thickness)
        | (y_grid < field_y_min + args.wall_thickness)
        | (y_grid >= field_y_max - args.wall_thickness)
    )
    image[wall_mask] = occupied_value

    green_mask = (
        (x_grid >= green_x_min)
        & (x_grid < green_x_max)
        & (y_grid >= green_y_min)
        & (y_grid < green_y_max)
    )
    image[green_mask] = occupied_value

    output_prefix = Path(args.output_prefix)
    output_prefix.parent.mkdir(parents=True, exist_ok=True)
    pgm_path = output_prefix.with_suffix(".pgm")
    yaml_path = output_prefix.with_suffix(".yaml")
    preview_path = output_prefix.with_name(output_prefix.name + "_preview.png")

    Image.fromarray(image, mode="L").save(pgm_path)
    Image.fromarray(image, mode="L").save(preview_path)
    write_yaml(
        yaml_path,
        pgm_path.name,
        args.resolution,
        (field_x_min, field_y_min),
    )

    total_cells = width * height
    free_cells = int((image == free_value).sum())
    print(f"size: {width} x {height}")
    print(f"resolution: {args.resolution}")
    print(f"origin: [{field_x_min:.3f}, {field_y_min:.3f}, 0.000]")
    print(f"free_cells: {free_cells} ({free_cells / total_cells:.2%})")
    print(f"occupied_cells: {total_cells - free_cells} ({1.0 - free_cells / total_cells:.2%})")
    print(f"wrote: {pgm_path}")
    print(f"wrote: {yaml_path}")
    print(f"preview: {preview_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
