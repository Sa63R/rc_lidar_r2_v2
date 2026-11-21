"""Task-3 configuration, map identity and world-frame composition (no ROS)."""
from copy import deepcopy
import hashlib
import math
from pathlib import Path
import re
import numpy as np
import yaml
from .corner import CornerProjector, corner_extrinsic, rigid_matrix, vector3, xyz_rpy
from .math_utils import matrix_state, transform


DEFAULTS = {
    'input_topic': '/map_lidar/odom', 'input_parent_frame': 'map',
    'input_child_frame': 'body', 'output_frame': '',
    'output_child_frame': 'task3_corner', 'output_prefix': '/task3/corner',
    'source_to_body_xyz': [0., 0., 0.], 'source_to_body_rpy': [0., 0., 0.],
    'body_to_lidar_xyz': [-.011, -.02329, .04412], 'body_to_lidar_rpy': [0., 0., 0.],
    # User-confirmed LiDAR +X = diagram -Y; R_L_D is inverse mounting yaw.
    'lidar_to_diagram_rpy': [0., 0., math.pi/2],
    # CAD lengths; user 2026-10-25 confirms behind-left when facing away
    # from cable, with left=+X and forward=-Y. Signed in diagram axes.
    'lidar_to_corner_diagram_xyz': [.175, .340, -.3745],
    'output_from_input_xyz': [0., 0., 0.], 'output_from_input_rpy': [0., 0., 0.],
    'input_timeout_sec': .5, 'future_tolerance_sec': .05,
    'calibration_confirmed': False, 'publish_tf': False,
    'xy_smoothing_enabled': False, 'xy_smoothing_min_cutoff_hz': 1.,
    'xy_smoothing_beta': 40., 'xy_smoothing_derivative_cutoff_hz': 1.,
    'xy_smoothing_max_correction_m': .003,
    'map_file': '', 'expected_map_sha256': '',
}


def sha256(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as file:
        for chunk in iter(lambda: file.read(1024*1024), b''):
            h.update(chunk)
    return h.hexdigest()


def verify_map(path, expected):
    if not path and not expected:
        return None
    if not path or not re.fullmatch('[0-9a-fA-F]{64}', expected):
        raise ValueError('map_file and a 64-digit expected_map_sha256 must be supplied together')
    if not Path(path).is_absolute():
        raise ValueError('map_file must be an absolute path')
    actual = sha256(path)
    if actual != expected.lower():
        raise ValueError(f'map SHA256 mismatch: expected {expected}, got {actual}')
    return actual


def load_config(path):
    with Path(path).open(encoding='utf-8') as file:
        config = yaml.safe_load(file)
    if not isinstance(config, dict) or not isinstance(config.get('task3_corner'), dict):
        raise ValueError('configuration must contain task3_corner.ros__parameters')
    values = config['task3_corner'].get('ros__parameters')
    if not isinstance(values, dict):
        raise ValueError('task3_corner.ros__parameters must be a mapping')
    unknown = set(values)-set(DEFAULTS)-{'use_sim_time'}
    if unknown:
        raise ValueError(f'unknown corner parameters: {sorted(unknown)}')
    return config


def prepare(values):
    p = deepcopy(DEFAULTS)
    p.update(values)
    for name in ('input_timeout_sec', 'future_tolerance_sec'):
        if not math.isfinite(p[name]) or p[name] < 0:
            raise ValueError(f'{name} must be finite and nonnegative')
    if p['input_timeout_sec'] == 0:
        raise ValueError('input_timeout_sec must be positive')
    for name in ('calibration_confirmed', 'publish_tf'):
        if type(p[name]) is not bool:
            raise ValueError(f'{name} must be a YAML boolean')
    from .task3_quality import xy_filter
    xy_filter(p)  # Validate filtering options even when disabled.
    parent = p['output_frame'] or p['input_parent_frame']
    if not parent or not p['input_parent_frame'] or not p['input_child_frame']:
        raise ValueError('input/output frame names must be nonempty')
    if not p['output_child_frame'] or p['output_child_frame'] in (parent, p['input_child_frame']):
        raise ValueError('corner child frame must differ from parent and input child')
    prefix = p['output_prefix'].rstrip('/')
    if not prefix or p['input_topic'] in (prefix+'/pose', prefix+'/point', prefix+'/odom'):
        raise ValueError('output prefix and input topic must be distinct')
    world = xyz_rpy(p['output_from_input_xyz'], p['output_from_input_rpy'])
    if parent == p['input_parent_frame'] and not np.allclose(world, np.eye(4), atol=1e-10):
        raise ValueError('a nonidentity world transform requires a different output_frame')
    extrinsic = corner_extrinsic(**{k: p[k] for k in (
        'source_to_body_xyz', 'source_to_body_rpy', 'body_to_lidar_xyz',
        'body_to_lidar_rpy', 'lidar_to_diagram_rpy', 'lidar_to_corner_diagram_xyz')})
    verify_map(p['map_file'], p['expected_map_sha256'])
    return p, CornerProjector(extrinsic, world)


def xyz_rpy_parameters(matrix, prefix):
    state, rp = matrix_state(matrix)
    return {prefix+'_xyz': state[:3].tolist(), prefix+'_rpy': [*rp, float(state[3])]}


def relative_start_config(base_config, source_xyz, source_quaternion):
    """Fix XY origin at the initial corner, with initial diagram +X as world +X.

    Z stays in the input world's height convention. The new frame remains fixed
    after this initialization; recalculating it on every sample would erase motion.
    """
    if 'task3_fixed_map' in base_config:
        raise ValueError('fixed-map configuration must not be re-zeroed; use the fixed-map entry')
    p, projector = prepare(base_config['task3_corner']['ros__parameters'])
    source = xyz_rpy(source_xyz, [0., 0., 0.])
    q = np.asarray(source_quaternion, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-8:
        raise ValueError('initial quaternion must be finite nonzero xyzw')
    from .math_utils import quat_matrix
    source[:3, :3] = quat_matrix(q)
    corner = source @ projector.source_T_corner
    if np.linalg.norm(corner[:2, 0]) < 1e-6:
        raise ValueError('initial diagram +X is vertical; cannot establish horizontal heading')
    yaw = math.atan2(corner[1, 0], corner[0, 0])
    rotation = xyz_rpy([0., 0., 0.], [0., 0., -yaw])
    translation = -rotation[:3, :3] @ corner[:3, 3]
    translation[2] = 0.  # Deferred Z: do not imply zero height above ground.
    p.update(output_frame='task3_start', output_from_input_xyz=translation.tolist(),
             output_from_input_rpy=[0., 0., -yaw])
    result = deepcopy(base_config)
    result['task3_corner']['ros__parameters'] = p
    prepare(p)
    return result


def fused_overrides(corner, fusion):
    """Preserve a fixed output world while moving from map/body to origin/base."""
    result = {
        'input_topic': '/task1/fused/odom',
        'input_parent_frame': fusion.get('output_frame', 'robot_origin'),
        'input_child_frame': fusion.get('output_child_frame', 'robot_base_fused'),
    }
    body_T_base = xyz_rpy(fusion['body_to_base_xyz'], fusion['body_to_base_rpy'])
    result.update(xyz_rpy_parameters(np.linalg.inv(body_T_base), 'source_to_body'))
    if corner.get('output_frame'):
        if corner.get('input_parent_frame', 'map') != fusion.get('lidar_parent_frame', 'map'):
            raise ValueError('field calibration and fusion must refer to the same map frame')
        output_T_map = xyz_rpy(corner.get('output_from_input_xyz', [0.]*3),
                               corner.get('output_from_input_rpy', [0.]*3))
        map_T_origin = xyz_rpy(fusion['map_to_origin_xyz'], fusion['map_to_origin_rpy'])
        result.update(xyz_rpy_parameters(output_T_map @ map_T_origin, 'output_from_input'))
    return result


def import_task2_axis(payload, base_config, map_file, output_frame='task3_field'):
    """Import only map_localization; never route an offline candidate as live odom."""
    expected = payload['map_sha256']
    verify_map(str(Path(map_file).resolve()), expected)
    selected = payload['calibration']['map_localization']
    if selected.get('accepted') is not True:
        raise ValueError('map_localization axis calibration was not accepted')
    pair, axis = selected['pair_calibration'], selected['axis']
    origin = vector3(pair['origin_source_m'], 'origin_source_m')
    pair_R = rigid_matrix(transform([0.]*3, np.asarray(pair['rotation_field_from_source'], float)))[:3, :3]
    axis_R = np.asarray(axis['rotation_field_from_pair_xy'], float)
    if axis_R.shape != (2, 2):
        raise ValueError('axis rotation must be 2x2')
    full_axis = np.eye(3)
    full_axis[:2, :2] = axis_R
    rigid_matrix(transform([0.]*3, full_axis))
    R = full_axis @ pair_R
    result = deepcopy(base_config)
    p, _ = prepare(result['task3_corner']['ros__parameters'])
    if p['input_parent_frame'] != 'map' or p['input_child_frame'] != 'body':
        raise ValueError('import into a map/body LiDAR-entry configuration')
    if not np.allclose(xyz_rpy(p['output_from_input_xyz'], p['output_from_input_rpy']), np.eye(4)):
        raise ValueError('base already has a world transform; avoid applying calibration twice')
    if not output_frame or output_frame in ('map', p['output_child_frame']):
        raise ValueError('use a distinct field output frame')
    p.update(xyz_rpy_parameters(transform(-R @ origin, R), 'output_from_input'))
    p.update({'output_frame': output_frame, 'calibration_confirmed': False,
              'map_file': str(Path(map_file).resolve()), 'expected_map_sha256': expected.lower()})
    result['task3_corner']['ros__parameters'] = p
    prepare(p)
    return result
