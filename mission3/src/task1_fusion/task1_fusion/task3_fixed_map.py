"""Persistent field origin tied to an immutable saved map, never to a boot pose.

This module does not perform registration. Its source must be the existing
accepted map-localizer output, map -> IMU/body (not raw FAST-LIO odometry).
"""
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import time

import numpy as np

from .task3_config import prepare, relative_start_config, sha256


def reference_id(parameters):
    # Content identity survives reboot and moving the identical PCD to a new path.
    keys = ('expected_map_sha256', 'input_parent_frame', 'input_child_frame',
            'output_frame', 'output_from_input_xyz', 'output_from_input_rpy',
            'source_to_body_xyz', 'source_to_body_rpy', 'body_to_lidar_xyz',
            'body_to_lidar_rpy', 'lidar_to_diagram_rpy', 'lidar_to_corner_diagram_xyz')
    value = json.dumps({k: parameters[k] for k in keys}, sort_keys=True, allow_nan=False)
    return 'task3_field_'+hashlib.sha256(value.encode()).hexdigest()[:24]


def bind_origin(base, map_file, map_body_xyz, map_body_quaternion, *,
                output_frame='task3_field', localizer_node='/map_localizer'):
    """ONE-TIME: corner on formal O, diagram axes aligned with formal field axes.

    Z remains map height; this is horizontal origin/heading calibration only.
    Calling this deliberately defines a new reference, never a daily startup.
    """
    if 'task3_fixed_map' in base:
        raise ValueError('refusing to overwrite an existing fixed-map reference')
    clean = deepcopy(base)
    p, _ = prepare(clean['task3_corner']['ros__parameters'])
    if p['map_file'] or not np.allclose(p['output_from_input_xyz'], 0.) or not np.allclose(
            p['output_from_input_rpy'], 0.):
        raise ValueError('use the untransformed car mechanical configuration, not an old start frame')
    if not output_frame or output_frame in ('map', 'camera_init', 'task3_start', p['output_child_frame']):
        raise ValueError('use a distinct fixed field output frame')
    if not localizer_node.startswith('/') or localizer_node == '/':
        raise ValueError('localizer_node must be a fully qualified ROS node name')
    # Explicitly change the INPUT contract; changing a frame name on raw LIO
    # messages would not achieve map localization.
    p.update(input_topic='/map_lidar/odom', input_parent_frame='map', input_child_frame='body',
             output_frame='', calibration_confirmed=False)
    clean['task3_corner']['ros__parameters'] = p
    result = relative_start_config(clean, map_body_xyz, map_body_quaternion)
    p = result['task3_corner']['ros__parameters']
    p.update(output_frame=output_frame, map_file=str(Path(map_file).resolve()),
             expected_map_sha256=sha256(map_file))
    vision = result.setdefault('task3_vision', {}).setdefault('ros__parameters', {})
    vision.update(reference_id=reference_id(p), allow_unverified_extrinsics=True)
    result['task3_fixed_map'] = {
        'schema_version': 1, 'localizer_node': localizer_node,
        'quality_topic': '/map_lidar/quality',
        'origin_definition': 'same rigid corner on chosen formal O; diagram axes aligned with field',
        'z_valid': False,
    }
    validate(result)
    return result


def validate(config):
    p, projector = prepare(config['task3_corner']['ros__parameters'])
    meta = config.get('task3_fixed_map', {})
    if meta.get('schema_version') != 1:
        raise ValueError('missing fixed-map reference; bind the formal origin once first')
    if (p['input_parent_frame'], p['input_child_frame'], p['input_topic']) != (
            'map', 'body', '/map_lidar/odom'):
        raise ValueError('fixed-map mode requires accepted /map_lidar/odom in map -> body')
    if not p['map_file'] or not p['expected_map_sha256']:
        raise ValueError('fixed-map mode requires a saved map and its SHA256')
    if not p['output_frame'] or p['output_frame'] in ('map', 'camera_init', 'task3_start'):
        raise ValueError('fixed-map mode requires a distinct persistent field frame')
    if config.get('task3_vision', {}).get('ros__parameters', {}).get('reference_id') != reference_id(p):
        raise ValueError('reference identity mismatch: map, geometry or origin changed')
    if meta.get('quality_topic') != '/map_lidar/quality' or not meta.get('localizer_node', '').startswith('/'):
        raise ValueError('invalid map-localizer identity/quality contract')
    return p, projector


class MapGate:
    """Fail closed until map identity and accepted localization are both fresh.

    Quality is a receipt-time status (the upstream array has no stamp). It is
    never used to restamp or manufacture odometry; the source stamp is checked
    separately by CornerNode and during the one-time capture.
    """
    def __init__(self, p, *, now=time.monotonic):
        self.p, self.now = p, now
        self.identity_until = self.quality_until = -math.inf
        self.reason = 'waiting_for_map_localization'

    def identity(self, parameters):
        try:
            path = Path(parameters['map_path'])
            correct = (path.is_absolute() and path.resolve() == Path(self.p['map_file']).resolve()
                       and sha256(path) == self.p['expected_map_sha256']
                       and parameters['map_frame'] == 'map' and parameters['body_frame'] == 'body'
                       and parameters['output_odom_topic'] == '/map_lidar/odom'
                       and parameters['output_quality_topic'] == '/map_lidar/quality'
                       and parameters['auto_initialize_identity'] is False)
        except (KeyError, OSError, TypeError, ValueError):
            correct = False
        self.identity_until = self.now()+3. if correct else -math.inf
        if not correct:
            self.reason = 'map_identity_unconfirmed'
        return correct

    def quality(self, values):
        # The existing localizer also reports timer ticks with no NEW match:
        # duplicate scan (7), or a registration already running (8). These are
        # neither a new accepted observation nor a failed geometric match.
        # Keep the previous accepted lease WITHOUT renewing it; it still times
        # out normally. Otherwise harmless duplicate ticks destroy every stable
        # origin window and make the UI flicker between valid/invalid.
        if len(values) > 10 and values[0] == 0. and values[10] in (7., 8.):
            return
        try:
            a, c, fitness, threshold = map(float, values[:4])
            accepted = (all(math.isfinite(v) for v in (a, c, fitness, threshold))
                        and a == 1. and c == 1. and 0. <= fitness <= threshold)
        except (ValueError, TypeError):
            accepted = False
        self.quality_until = self.now()+self.p['input_timeout_sec'] if accepted else -math.inf
        if not accepted:
            self.reason = 'map_registration_not_accepted'

    def ready(self, graph_ok=True):
        now = self.now()
        if not graph_ok:
            self.reason = 'map_publisher_identity_mismatch'
        elif now > self.identity_until:
            self.reason = 'map_identity_unconfirmed'
        elif now > self.quality_until:
            self.reason = 'map_localization_unavailable'
        else:
            self.reason = 'tracking'
            return True
        return False
