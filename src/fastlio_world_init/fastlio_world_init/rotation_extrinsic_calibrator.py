#!/usr/bin/env python3
"""Record rotation data and estimate LiDAR-to-base XY extrinsic error."""

import argparse
import csv
import json
import math
import os
import re
import sys
import time
from dataclasses import asdict
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


DEFAULT_STATIC_X = -0.240
DEFAULT_STATIC_Y = 0.315
DEFAULT_STATIC_Z = -0.760
DEFAULT_STATIC_YAW = math.pi

FIT_YAW_SOURCES = (
    'init_body_highrate',
    'init_base_link_highrate',
    'camera_init_body',
    'world_body',
    'odom',
)


@dataclass
class StaticExtrinsic:
    """Static transform values used by the current TF launch files."""

    x: float
    y: float
    z: float
    yaw: float
    source: str


@dataclass
class CalibrationSample:
    """One synchronized TF sample used by the planar calibration fit."""

    elapsed_sec: float
    stamp_sec: float
    fit_yaw: float
    body_x: float
    body_y: float
    body_yaw: float
    base_x: float
    base_y: float
    base_yaw: float
    static_x: float
    static_y: float
    static_yaw: float
    camera_init_body_x: float
    camera_init_body_y: float
    camera_init_body_yaw: float
    world_body_x: float
    world_body_y: float
    world_body_yaw: float
    odom_x: float
    odom_y: float
    odom_yaw: float
    odom_age_sec: float


def quaternion_to_yaw(q):
    """Return planar yaw from a geometry_msgs Quaternion."""

    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def unwrap_angles(angles):
    """Unwrap a sequence of radians without depending on numpy unwrap."""

    if len(angles) == 0:
        return []

    result = [float(angles[0])]
    offset = 0.0
    previous = float(angles[0])

    for angle in angles[1:]:
        angle = float(angle)
        delta = angle - previous
        if delta > math.pi:
            offset -= 2.0 * math.pi
        elif delta < -math.pi:
            offset += 2.0 * math.pi

        result.append(angle + offset)
        previous = angle

    return result


def is_finite(value):
    """Return True when value is a finite number."""

    return value is not None and math.isfinite(float(value))


def summarize_yaw_values(values):
    """Summarize raw and unwrapped yaw values in degrees."""

    finite_values = [float(value) for value in values if is_finite(value)]
    if not finite_values:
        return {
            'sample_count': 0,
            'raw_min_deg': None,
            'raw_max_deg': None,
            'unwrapped_start_deg': None,
            'unwrapped_end_deg': None,
            'unwrapped_min_deg': None,
            'unwrapped_max_deg': None,
            'span_deg': None,
            'net_deg': None,
            'abs_travel_deg': None,
            'wrap_jump_count': 0,
            'max_step_deg': None,
        }

    unwrapped = unwrap_angles(finite_values)
    steps = [
        unwrapped[index] - unwrapped[index - 1]
        for index in range(1, len(unwrapped))
    ]
    raw_steps = [
        finite_values[index] - finite_values[index - 1]
        for index in range(1, len(finite_values))
    ]
    wrap_jump_count = sum(abs(step) > math.pi for step in raw_steps)

    return {
        'sample_count': len(finite_values),
        'raw_min_deg': math.degrees(min(finite_values)),
        'raw_max_deg': math.degrees(max(finite_values)),
        'unwrapped_start_deg': math.degrees(unwrapped[0]),
        'unwrapped_end_deg': math.degrees(unwrapped[-1]),
        'unwrapped_min_deg': math.degrees(min(unwrapped)),
        'unwrapped_max_deg': math.degrees(max(unwrapped)),
        'span_deg': math.degrees(max(unwrapped) - min(unwrapped)),
        'net_deg': math.degrees(unwrapped[-1] - unwrapped[0]),
        'abs_travel_deg': math.degrees(sum(abs(step) for step in steps)),
        'wrap_jump_count': int(wrap_jump_count),
        'max_step_deg': (
            math.degrees(max(abs(step) for step in steps))
            if steps else 0.0
        ),
    }


def yaw_source_map(samples):
    """Return yaw sequences used to diagnose where rotation is lost."""

    return {
        'init_body_highrate': [sample.body_yaw for sample in samples],
        'init_base_link_highrate': [sample.base_yaw for sample in samples],
        'camera_init_body': [
            sample.camera_init_body_yaw for sample in samples
        ],
        'world_body': [sample.world_body_yaw for sample in samples],
        'odom': [sample.odom_yaw for sample in samples],
    }


def fit_rotation_error(yaws, observed_xy):
    """Fit observed_xy = center + R(yaw) * delta_xy."""

    yaws = np.asarray(yaws, dtype=float)
    observed_xy = np.asarray(observed_xy, dtype=float)

    rows = []
    values = []
    for yaw, observed in zip(yaws, observed_xy):
        c = math.cos(float(yaw))
        s = math.sin(float(yaw))
        rows.append([1.0, 0.0, c, -s])
        values.append(float(observed[0]))
        rows.append([0.0, 1.0, s, c])
        values.append(float(observed[1]))

    solution, residuals, rank, singular_values = np.linalg.lstsq(
        np.asarray(rows, dtype=float),
        np.asarray(values, dtype=float),
        rcond=None,
    )

    center = solution[0:2]
    delta = solution[2:4]

    corrected = []
    predicted = []
    for yaw, observed in zip(yaws, observed_xy):
        c = math.cos(float(yaw))
        s = math.sin(float(yaw))
        rotated_delta = np.asarray(
            [
                c * delta[0] - s * delta[1],
                s * delta[0] + c * delta[1],
            ],
            dtype=float,
        )
        predicted_xy = center + rotated_delta
        predicted.append(predicted_xy)
        corrected.append(observed - rotated_delta)

    corrected = np.asarray(corrected, dtype=float)
    predicted = np.asarray(predicted, dtype=float)
    residual_vectors = observed_xy - predicted

    observed_center = np.mean(observed_xy, axis=0)
    before_vectors = observed_xy - observed_center
    after_center = np.mean(corrected, axis=0)
    after_vectors = corrected - after_center

    before_rms = float(np.sqrt(np.mean(np.sum(before_vectors ** 2, axis=1))))
    after_rms = float(np.sqrt(np.mean(np.sum(after_vectors ** 2, axis=1))))
    model_rms = float(np.sqrt(np.mean(np.sum(residual_vectors ** 2, axis=1))))

    return {
        'center_x': float(center[0]),
        'center_y': float(center[1]),
        'delta_x': float(delta[0]),
        'delta_y': float(delta[1]),
        'rank': int(rank),
        'singular_values': [float(v) for v in singular_values],
        'lstsq_residuals': [float(v) for v in residuals],
        'before_rms': before_rms,
        'after_rms': after_rms,
        'model_rms': model_rms,
    }


def bin_samples_by_yaw(yaws, observed_xy, bin_deg):
    """Average samples by yaw bins so pauses do not overweight one angle."""

    yaws = np.asarray(yaws, dtype=float)
    observed_xy = np.asarray(observed_xy, dtype=float)
    if bin_deg <= 0.0 or len(yaws) == 0:
        return yaws, observed_xy

    bin_rad = math.radians(bin_deg)
    if bin_rad <= 0.0:
        return yaws, observed_xy

    bins = {}
    for yaw, observed in zip(yaws, observed_xy):
        key = int(round(float(yaw) / bin_rad))
        bucket = bins.setdefault(key, {'yaw': [], 'xy': []})
        bucket['yaw'].append(float(yaw))
        bucket['xy'].append(np.asarray(observed, dtype=float))

    binned_yaws = []
    binned_xy = []
    for key in sorted(bins):
        bucket = bins[key]
        binned_yaws.append(float(np.mean(bucket['yaw'])))
        binned_xy.append(
            np.mean(np.asarray(bucket['xy'], dtype=float), axis=0)
        )

    return (
        np.asarray(binned_yaws, dtype=float),
        np.asarray(binned_xy, dtype=float),
    )


def parse_static_extrinsic(path):
    """Parse x/y/z/yaw constants from a static TF launch file."""

    path = Path(path)
    text = path.read_text(encoding='utf-8')

    def read_arg(name, default):
        pattern = rf'"--{name}"\s*,\s*"([^"]+)"'
        match = re.search(pattern, text)
        if not match:
            return default
        return float(match.group(1))

    yaw = DEFAULT_STATIC_YAW
    yaw_match = re.search(r'YAW_180_DEG_RAD\s*=\s*"([^"]+)"', text)
    if yaw_match:
        yaw = float(yaw_match.group(1))
    else:
        yaw = read_arg('yaw', yaw)

    return StaticExtrinsic(
        x=read_arg('x', DEFAULT_STATIC_X),
        y=read_arg('y', DEFAULT_STATIC_Y),
        z=read_arg('z', DEFAULT_STATIC_Z),
        yaw=yaw,
        source=str(path),
    )


def resolve_workspace():
    """Resolve workspace for default output and static TF files."""

    env_workspace = os.environ.get('RC_LIDAR_WORKSPACE')
    if env_workspace:
        return Path(env_workspace).expanduser().resolve()

    cwd = Path.cwd().resolve()
    if (cwd / 'src' / 'my_static_tf').exists():
        return cwd

    for parent in Path(__file__).resolve().parents:
        if (parent / 'src' / 'my_static_tf').exists():
            return parent

    return cwd


def default_output_dir(workspace):
    """Return the default calibration output directory."""

    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    return workspace / 'logs' / 'calibration' / f'rotation_extrinsic_{stamp}'


def summarize_samples(
    samples,
    static_extrinsic,
    min_samples,
    min_yaw_span_deg,
    yaw_bin_deg,
    fit_yaw_source,
):
    """Build a JSON-serializable calibration summary."""

    yaw_sources = {
        name: summarize_yaw_values(values)
        for name, values in yaw_source_map(samples).items()
    }

    selected_rows = []
    for sample in samples:
        if is_finite(sample.fit_yaw):
            selected_rows.append(sample)

    raw_yaws = [sample.fit_yaw for sample in selected_rows]
    unwrapped_yaws = unwrap_angles(raw_yaws)
    observed_xy = np.asarray(
        [[sample.base_x, sample.base_y] for sample in selected_rows],
        dtype=float,
    )

    warnings = []
    if len(samples) < min_samples:
        warnings.append(
            f'sample_count {len(samples)} is below minimum {min_samples}'
        )
    if len(selected_rows) < min_samples:
        warnings.append(
            f'fit_yaw_sample_count {len(selected_rows)} is below '
            f'minimum {min_samples}'
        )

    if len(selected_rows) < 4:
        warnings.append(
            f'not enough finite yaw samples from fit_yaw_source='
            f'{fit_yaw_source}'
        )
        return {
            'sample_count': len(samples),
            'fit_yaw_source': fit_yaw_source,
            'fit_yaw_sample_count': len(selected_rows),
            'fit_sample_count': 0,
            'yaw_bin_deg': float(yaw_bin_deg),
            'duration_sec': (
                samples[-1].elapsed_sec - samples[0].elapsed_sec
                if len(samples) >= 2 else 0.0
            ),
            'yaw_span_deg': None,
            'yaw_sources': yaw_sources,
            'xy_peak_to_peak': {
                'x': None,
                'y': None,
                'norm': None,
            },
            'fitted_error_xy': {
                'x': None,
                'y': None,
            },
            'current_static_xy': {
                'x': static_extrinsic.x,
                'y': static_extrinsic.y,
            },
            'suggested_static_xy': {
                'x': None,
                'y': None,
            },
            'current_static_full': asdict(static_extrinsic),
            'residual_before_after': {
                'before_rms': None,
                'after_rms': None,
                'model_rms': None,
            },
            'fit': None,
            'quality': {
                'ok': False,
                'warnings': warnings,
                'min_samples': int(min_samples),
                'min_yaw_span_deg': float(min_yaw_span_deg),
            },
        }

    fit_yaws, fit_observed_xy = bin_samples_by_yaw(
        unwrapped_yaws,
        observed_xy,
        yaw_bin_deg,
    )
    fit = fit_rotation_error(fit_yaws, fit_observed_xy)

    yaw_span_rad = max(unwrapped_yaws) - min(unwrapped_yaws)
    yaw_span_deg = math.degrees(abs(yaw_span_rad))
    xy_min = np.min(observed_xy, axis=0)
    xy_max = np.max(observed_xy, axis=0)
    xy_peak_to_peak = xy_max - xy_min

    if len(fit_yaws) < 12:
        warnings.append(
            f'fit_sample_count {len(fit_yaws)} is low; rotate more slowly '
            'or cover more angles'
        )
    if yaw_span_deg < min_yaw_span_deg:
        warnings.append(
            f'yaw_span_deg {yaw_span_deg:.1f} is below '
            f'minimum {min_yaw_span_deg:.1f}'
        )
        for source, stats in yaw_sources.items():
            if source == fit_yaw_source:
                continue
            source_span = stats.get('span_deg')
            if source_span is not None and source_span >= min_yaw_span_deg:
                warnings.append(
                    f'fit_yaw_source={fit_yaw_source} has low span, but '
                    f'{source} span is {source_span:.1f} deg; compare yaw '
                    'sources before applying calibration'
                )
    if fit['rank'] < 4:
        warnings.append('least-squares matrix rank is below 4')

    suggested_x = static_extrinsic.x - fit['delta_x']
    suggested_y = static_extrinsic.y - fit['delta_y']

    return {
        'sample_count': len(samples),
        'fit_yaw_source': fit_yaw_source,
        'fit_yaw_sample_count': len(selected_rows),
        'fit_sample_count': int(len(fit_yaws)),
        'yaw_bin_deg': float(yaw_bin_deg),
        'duration_sec': (
            samples[-1].elapsed_sec - samples[0].elapsed_sec
            if len(samples) >= 2 else 0.0
        ),
        'yaw_span_deg': yaw_span_deg,
        'yaw_sources': yaw_sources,
        'xy_peak_to_peak': {
            'x': float(xy_peak_to_peak[0]),
            'y': float(xy_peak_to_peak[1]),
            'norm': float(np.linalg.norm(xy_peak_to_peak)),
        },
        'fitted_error_xy': {
            'x': fit['delta_x'],
            'y': fit['delta_y'],
        },
        'current_static_xy': {
            'x': static_extrinsic.x,
            'y': static_extrinsic.y,
        },
        'suggested_static_xy': {
            'x': suggested_x,
            'y': suggested_y,
        },
        'current_static_full': asdict(static_extrinsic),
        'residual_before_after': {
            'before_rms': fit['before_rms'],
            'after_rms': fit['after_rms'],
            'model_rms': fit['model_rms'],
        },
        'fit': fit,
        'quality': {
            'ok': not warnings,
            'warnings': warnings,
            'min_samples': int(min_samples),
            'min_yaw_span_deg': float(min_yaw_span_deg),
        },
    }


def write_samples_csv(path, samples):
    """Write raw calibration samples to CSV."""

    fieldnames = [
        'elapsed_sec',
        'stamp_sec',
        'fit_yaw',
        'body_x',
        'body_y',
        'body_yaw',
        'base_x',
        'base_y',
        'base_yaw',
        'static_x',
        'static_y',
        'static_yaw',
        'camera_init_body_x',
        'camera_init_body_y',
        'camera_init_body_yaw',
        'world_body_x',
        'world_body_y',
        'world_body_yaw',
        'odom_x',
        'odom_y',
        'odom_yaw',
        'odom_age_sec',
    ]
    with Path(path).open('w', newline='', encoding='utf-8') as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        for sample in samples:
            writer.writerow(asdict(sample))


def write_report(path, summary):
    """Write a human-readable Markdown report."""

    quality = summary['quality']

    def fmt(value):
        if value is None:
            return 'n/a'
        return f'{float(value):.6f}'

    def fmt_deg(value):
        if value is None:
            return 'n/a'
        return f'{float(value):.3f}'

    lines = [
        '# Rotation Extrinsic Calibration Report',
        '',
        '## Result',
        '',
        f"- quality_ok: `{quality['ok']}`",
        f"- sample_count: `{summary['sample_count']}`",
        f"- fit_yaw_source: `{summary.get('fit_yaw_source', 'n/a')}`",
        f"- fit_yaw_sample_count: "
        f"`{summary.get('fit_yaw_sample_count', 0)}`",
        f"- fit_sample_count: `{summary.get('fit_sample_count', 0)}`",
        f"- yaw_bin_deg: `{fmt_deg(summary.get('yaw_bin_deg'))}`",
        f"- yaw_span_deg: `{fmt_deg(summary.get('yaw_span_deg'))}`",
        '',
        '## Yaw Diagnostics',
        '',
        '| source | samples | span_deg | net_deg | abs_travel_deg | wraps |',
        '| --- | ---: | ---: | ---: | ---: | ---: |',
    ]

    for source, stats in summary.get('yaw_sources', {}).items():
        lines.append(
            f"| `{source}` | "
            f"{stats.get('sample_count', 0)} | "
            f"{fmt_deg(stats.get('span_deg'))} | "
            f"{fmt_deg(stats.get('net_deg'))} | "
            f"{fmt_deg(stats.get('abs_travel_deg'))} | "
            f"{stats.get('wrap_jump_count', 0)} |"
        )

    lines.extend([
        '',
        '## Suggested Static XY',
        '',
        f"- current_x: `{fmt(summary['current_static_xy']['x'])}`",
        f"- current_y: `{fmt(summary['current_static_xy']['y'])}`",
        f"- fitted_error_x: `{fmt(summary['fitted_error_xy']['x'])}`",
        f"- fitted_error_y: `{fmt(summary['fitted_error_xy']['y'])}`",
        f"- suggested_x: `{fmt(summary['suggested_static_xy']['x'])}`",
        f"- suggested_y: `{fmt(summary['suggested_static_xy']['y'])}`",
        '',
        '## Residual',
        '',
        '- before_rms: '
        f"`{fmt(summary['residual_before_after']['before_rms'])}`",
        '- after_rms: '
        f"`{fmt(summary['residual_before_after']['after_rms'])}`",
        '- model_rms: '
        f"`{fmt(summary['residual_before_after']['model_rms'])}`",
        '',
        '## XY Peak To Peak',
        '',
        f"- x: `{fmt(summary['xy_peak_to_peak']['x'])}`",
        f"- y: `{fmt(summary['xy_peak_to_peak']['y'])}`",
        f"- norm: `{fmt(summary['xy_peak_to_peak']['norm'])}`",
        '',
    ])

    if quality['warnings']:
        lines.extend(['## Warnings', ''])
        lines.extend(f"- {warning}" for warning in quality['warnings'])
        lines.append('')

    lines.extend([
        '## Apply Manually After Review',
        '',
        'Update these three static TF launch files with the suggested x/y:',
        '',
        '- `src/my_static_tf/launch/map_to_init.launch.py`',
        '- `src/my_static_tf/launch/lidar_to_base.launch.py`',
        '- `src/my_static_tf/launch/lidar_to_base_highrate.launch.py`',
        '',
        'Keep `z`, `roll`, `pitch`, and `yaw` unchanged for this calibration.',
        '',
    ])

    Path(path).write_text('\n'.join(lines), encoding='utf-8')


class RotationExtrinsicCalibrator(Node):
    """Collect TF samples for planar rotation extrinsic calibration."""

    def __init__(self, args, output_dir, static_extrinsic):
        super().__init__('rotation_extrinsic_calibrator')

        self.args = args
        self.output_dir = Path(output_dir)
        self.static_extrinsic = static_extrinsic
        self.samples = []
        self.done = False
        self.start_monotonic = time.monotonic()
        self.last_warn = 0.0
        self.latest_odom = None
        self.latest_odom_monotonic = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.odom_sub = self.create_subscription(
            Odometry,
            args.odometry_topic,
            self._odom_callback,
            50,
        )

        self.timer = self.create_timer(1.0 / args.rate, self._tick)

        self.get_logger().info(
            'Recording rotation calibration samples for '
            f'{args.duration:.1f}s at {args.rate:.1f}Hz'
        )
        self.get_logger().info(f'Output directory: {self.output_dir}')
        self.get_logger().info(
            f'Current static x/y: {static_extrinsic.x:.6f}, '
            f'{static_extrinsic.y:.6f}'
        )
        self.get_logger().info(
            f'Fit yaw source: {args.fit_yaw_source}; diagnostics include '
            f'{", ".join(FIT_YAW_SOURCES)}'
        )

    def _odom_callback(self, msg):
        self.latest_odom = msg
        self.latest_odom_monotonic = time.monotonic()

    def _warn_throttled(self, message):
        now = time.monotonic()
        if now - self.last_warn >= self.args.warn_period:
            self.last_warn = now
            self.get_logger().warn(message)

    def _lookup(self, target_frame, source_frame):
        return self.tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            Time(),
        )

    def _lookup_optional(self, target_frame, source_frame):
        try:
            return self._lookup(target_frame, source_frame)
        except TransformException:
            return None

    @staticmethod
    def _xy_yaw_from_tf(tf_msg):
        if tf_msg is None:
            return float('nan'), float('nan'), float('nan')

        t = tf_msg.transform.translation
        return (
            float(t.x),
            float(t.y),
            quaternion_to_yaw(tf_msg.transform.rotation),
        )

    def _latest_odom_xy_yaw(self):
        if self.latest_odom is None:
            return (
                float('nan'),
                float('nan'),
                float('nan'),
                float('nan'),
            )

        p = self.latest_odom.pose.pose.position
        q = self.latest_odom.pose.pose.orientation
        age = (
            time.monotonic() - self.latest_odom_monotonic
            if self.latest_odom_monotonic is not None else float('nan')
        )
        return (
            float(p.x),
            float(p.y),
            quaternion_to_yaw(q),
            float(age),
        )

    @staticmethod
    def _choose_fit_yaw(args, values):
        source = args.fit_yaw_source
        if source not in values:
            raise ValueError(f'Unknown fit yaw source: {source}')
        return values[source]

    def _tick(self):
        elapsed = time.monotonic() - self.start_monotonic
        if elapsed >= self.args.duration:
            self.done = True
            return

        try:
            body_tf = self._lookup(
                self.args.target_frame,
                self.args.body_frame,
            )
            base_tf = self._lookup(
                self.args.target_frame,
                self.args.base_frame,
            )
            static_tf = self._lookup(
                self.args.body_frame,
                self.args.base_frame,
            )
        except TransformException as exc:
            self._warn_throttled(f'Waiting for required TFs: {exc}')
            return

        now = self.get_clock().now()
        body_t = body_tf.transform.translation
        base_t = base_tf.transform.translation
        static_t = static_tf.transform.translation
        camera_init_body_tf = self._lookup_optional(
            self.args.fastlio_parent_frame,
            self.args.fastlio_body_frame,
        )
        world_body_tf = self._lookup_optional(
            self.args.world_frame,
            self.args.fastlio_body_frame,
        )
        (
            camera_init_body_x,
            camera_init_body_y,
            camera_init_body_yaw,
        ) = self._xy_yaw_from_tf(camera_init_body_tf)
        (
            world_body_x,
            world_body_y,
            world_body_yaw,
        ) = self._xy_yaw_from_tf(world_body_tf)
        odom_x, odom_y, odom_yaw, odom_age_sec = self._latest_odom_xy_yaw()
        yaw_values = {
            'init_body_highrate': quaternion_to_yaw(
                body_tf.transform.rotation
            ),
            'init_base_link_highrate': quaternion_to_yaw(
                base_tf.transform.rotation
            ),
            'camera_init_body': camera_init_body_yaw,
            'world_body': world_body_yaw,
            'odom': odom_yaw,
        }

        sample = CalibrationSample(
            elapsed_sec=float(elapsed),
            stamp_sec=float(now.nanoseconds) / 1e9,
            fit_yaw=self._choose_fit_yaw(self.args, yaw_values),
            body_x=float(body_t.x),
            body_y=float(body_t.y),
            body_yaw=yaw_values['init_body_highrate'],
            base_x=float(base_t.x),
            base_y=float(base_t.y),
            base_yaw=yaw_values['init_base_link_highrate'],
            static_x=float(static_t.x),
            static_y=float(static_t.y),
            static_yaw=quaternion_to_yaw(static_tf.transform.rotation),
            camera_init_body_x=camera_init_body_x,
            camera_init_body_y=camera_init_body_y,
            camera_init_body_yaw=camera_init_body_yaw,
            world_body_x=world_body_x,
            world_body_y=world_body_y,
            world_body_yaw=world_body_yaw,
            odom_x=odom_x,
            odom_y=odom_y,
            odom_yaw=odom_yaw,
            odom_age_sec=odom_age_sec,
        )
        self.samples.append(sample)

    def write_outputs(self):
        """Write CSV, JSON, and Markdown outputs."""

        self.output_dir.mkdir(parents=True, exist_ok=True)
        samples_csv = self.output_dir / 'samples.csv'
        summary_json = self.output_dir / 'summary.json'
        report_md = self.output_dir / 'report.md'

        write_samples_csv(samples_csv, self.samples)

        if len(self.samples) >= 4:
            summary = summarize_samples(
                self.samples,
                self.static_extrinsic,
                self.args.min_samples,
                self.args.min_yaw_span_deg,
                self.args.yaw_bin_deg,
                self.args.fit_yaw_source,
            )
        else:
            yaw_sources = {
                name: summarize_yaw_values(values)
                for name, values in yaw_source_map(self.samples).items()
            }
            summary = {
                'sample_count': len(self.samples),
                'fit_yaw_source': self.args.fit_yaw_source,
                'fit_yaw_sample_count': 0,
                'fit_sample_count': 0,
                'yaw_bin_deg': float(self.args.yaw_bin_deg),
                'duration_sec': 0.0,
                'yaw_span_deg': None,
                'yaw_sources': yaw_sources,
                'xy_peak_to_peak': {
                    'x': None,
                    'y': None,
                    'norm': None,
                },
                'fitted_error_xy': {
                    'x': None,
                    'y': None,
                },
                'current_static_xy': {
                    'x': self.static_extrinsic.x,
                    'y': self.static_extrinsic.y,
                },
                'suggested_static_xy': {
                    'x': None,
                    'y': None,
                },
                'residual_before_after': {
                    'before_rms': None,
                    'after_rms': None,
                    'model_rms': None,
                },
                'fit': None,
                'quality': {
                    'ok': False,
                    'warnings': ['not enough samples to fit calibration'],
                    'min_samples': int(self.args.min_samples),
                    'min_yaw_span_deg': float(self.args.min_yaw_span_deg),
                },
                'current_static_full': asdict(self.static_extrinsic),
            }

        summary['frames'] = {
            'target_frame': self.args.target_frame,
            'body_frame': self.args.body_frame,
            'base_frame': self.args.base_frame,
            'fastlio_parent_frame': self.args.fastlio_parent_frame,
            'fastlio_body_frame': self.args.fastlio_body_frame,
            'world_frame': self.args.world_frame,
        }
        summary['odometry_topic'] = self.args.odometry_topic
        summary['output_dir'] = str(self.output_dir)
        summary['files'] = {
            'samples_csv': str(samples_csv),
            'summary_json': str(summary_json),
            'report_md': str(report_md),
        }

        summary_json.write_text(
            json.dumps(summary, indent=2, ensure_ascii=False),
            encoding='utf-8',
        )
        write_report(report_md, summary)

        self.get_logger().info(f'Wrote samples: {samples_csv}')
        self.get_logger().info(f'Wrote summary: {summary_json}')
        self.get_logger().info(f'Wrote report: {report_md}')

        if not summary['quality']['ok']:
            for warning in summary['quality']['warnings']:
                self.get_logger().warn(warning)

        return summary


def parse_args(argv=None):
    """Parse command line arguments."""

    workspace = resolve_workspace()
    default_static_file = (
        workspace
        / 'src'
        / 'my_static_tf'
        / 'launch'
        / 'lidar_to_base_highrate.launch.py'
    )

    parser = argparse.ArgumentParser(
        description='Record self-rotation TF data and estimate XY extrinsic.'
    )
    parser.add_argument('--duration', type=float, default=40.0)
    parser.add_argument('--rate', type=float, default=50.0)
    parser.add_argument('--warn-period', type=float, default=2.0)
    parser.add_argument('--target-frame', default='init')
    parser.add_argument('--body-frame', default='body_highrate')
    parser.add_argument('--base-frame', default='base_link_highrate')
    parser.add_argument('--fastlio-parent-frame', default='camera_init')
    parser.add_argument('--fastlio-body-frame', default='body')
    parser.add_argument('--world-frame', default='world')
    parser.add_argument('--odometry-topic', default='/Odometry')
    parser.add_argument(
        '--fit-yaw-source',
        default='init_body_highrate',
        choices=FIT_YAW_SOURCES,
        help=(
            'Yaw source used for fitting. All sources are still recorded '
            'for diagnostics.'
        ),
    )
    parser.add_argument('--min-samples', type=int, default=200)
    parser.add_argument('--min-yaw-span-deg', type=float, default=270.0)
    parser.add_argument('--yaw-bin-deg', type=float, default=2.0)
    parser.add_argument('--output-dir', default='')
    parser.add_argument(
        '--static-launch-file',
        default=str(default_static_file),
    )
    parser.add_argument('--current-x', type=float, default=None)
    parser.add_argument('--current-y', type=float, default=None)
    return parser.parse_known_args(argv)


def resolve_static_extrinsic(args):
    """Resolve current static extrinsic from launch file and CLI overrides."""

    static_path = Path(args.static_launch_file).expanduser()
    if static_path.exists():
        extrinsic = parse_static_extrinsic(static_path)
    else:
        extrinsic = StaticExtrinsic(
            x=DEFAULT_STATIC_X,
            y=DEFAULT_STATIC_Y,
            z=DEFAULT_STATIC_Z,
            yaw=DEFAULT_STATIC_YAW,
            source='built-in defaults',
        )

    if args.current_x is not None:
        extrinsic.x = float(args.current_x)
        extrinsic.source += ' + --current-x'
    if args.current_y is not None:
        extrinsic.y = float(args.current_y)
        extrinsic.source += ' + --current-y'

    return extrinsic


def main(argv=None):
    """Run the calibration recorder."""

    cli_args, ros_args = parse_args(sys.argv[1:] if argv is None else argv)
    workspace = resolve_workspace()
    output_dir = (
        Path(cli_args.output_dir).expanduser()
        if cli_args.output_dir
        else default_output_dir(workspace)
    )
    static_extrinsic = resolve_static_extrinsic(cli_args)

    if cli_args.duration <= 0.0:
        raise SystemExit('--duration must be positive')
    if cli_args.rate <= 0.0:
        raise SystemExit('--rate must be positive')

    rclpy.init(args=[sys.argv[0], *ros_args])
    node = RotationExtrinsicCalibrator(cli_args, output_dir, static_extrinsic)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
        summary = node.write_outputs()
    except KeyboardInterrupt:
        node.get_logger().warn(
            'Interrupted; writing partial calibration data.'
        )
        summary = node.write_outputs()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if summary['sample_count'] < 4:
        return 3
    return 0


if __name__ == '__main__':
    sys.exit(main())
