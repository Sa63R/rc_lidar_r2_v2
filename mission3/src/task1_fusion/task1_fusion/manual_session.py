"""Operator-paced motion-test stages and measured-reference comparisons (no ROS)."""
import math
import re
import numpy as np


PHASES = (
    ('O', '起点：标记当前目标角点的地面投影O，保持不动'),
    ('A', '保持初始朝向，把目标角点摆到提前标好的A点，停稳'),
    ('O1', '把同一个目标角点摆回提前标好的O点，保持初始朝向'),
    ('C', '保持初始朝向，沿固定初始+Y把目标角点摆到提前标好的C点'),
    ('OY', '把同一个目标角点从C摆回O点，保持初始朝向'),
    ('B', '从O基准姿态绕车心原地逆时针转约90°，停稳；使用理论参照，不量B坐标'),
    ('O2', '恢复初始朝向，把同一个目标角点摆回提前标好的O点'),
)


def selected_phases(point_plan, rotation_only=False):
    """An explicit turn-only workflow has three real stages, never fake skips."""
    if not rotation_only:
        return PHASES
    turn = point_plan.get('B')
    if not isinstance(turn, dict) or turn.get('kind') != 'fixed_corner_at_origin':
        raise ValueError('仅旋转模式需要B为fixed_corner_at_origin：转后同一角点实际对回O')
    for phase in ('O', 'B', 'O2'):
        reference = reference_for_phase({'command': 'done'}, phase, point_plan)
        if reference.get('reference_xy_mm') != [0., 0.]:
            raise ValueError('仅旋转模式需要O/B/O2均有同一O标记的XY=(0,0)参照')
    return tuple(row for row in PHASES if row[0] in ('O', 'B', 'O2'))


def theoretical_turn_xy(center_to_corner_xy_mm, turn_deg):
    """Displacement of a corner about a fixed center; not LiDAR lever geometry."""
    r = np.asarray(center_to_corner_xy_mm, float)
    if r.shape != (2,) or not np.isfinite(r).all() or not math.isfinite(turn_deg):
        raise ValueError('理论旋转需要有限的车心→角点XY向量和角度')
    angle = math.radians(turn_deg)
    R = np.array([[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]])
    return R @ r-r


def initial_origin_problem(xy_m, yaw_deg, max_distance_m=.05, max_heading_deg=5.):
    """Ordering guard, not a proof of physical alignment or an accuracy limit."""
    xy = np.asarray(xy_m,float)
    if xy.shape != (2,) or not np.isfinite(xy).all() or not math.isfinite(yaw_deg):
        return '起点坐标或朝向无效'
    if np.linalg.norm(xy) > max_distance_m:
        return '当前估计位置明显偏离起点，请先把角点摆回O；不会把这里自动改成零点'
    wrapped = (yaw_deg+180.) % 360.-180.
    if abs(wrapped) > max_heading_deg:
        return '当前估计朝向明显偏离初始方向，请先恢复初始朝向'
    return ''


def parse_command(line):
    value = line.strip()
    aliases = {'': 'done', '完成': 'done', '结束': 'finish', '退出': 'abort', 'q': 'abort'}
    command = aliases.get(value, value.lower())
    if command in ('done', 'status', 'help', 'finish', 'abort'):
        return {'command': command}
    if value.lower().startswith('note '):
        return {'command': 'note', 'text': value[5:].strip()}
    try:
        numbers = [float(x) for x in re.split(r'[\s,，]+', value)]
    except ValueError as exc:
        raise ValueError('请输入 实测X 实测Y [实测yaw]，位置mm、角度deg；或done/status/note/finish/abort') from exc
    if len(numbers) not in (2, 3) or not all(math.isfinite(x) for x in numbers):
        raise ValueError('需要2或3个有限数值；位置单位mm，可选角度单位deg')
    return {'command': 'done', 'reference_xy_mm': numbers[:2],
            'reference_yaw_deg': numbers[2] if len(numbers) == 3 else None}


def reference_for_phase(command, phase, point_plan):
    """A preset is used only when explicitly supplied; never infer ideal points."""
    result = dict(command)
    if 'reference_xy_mm' in result:
        result['reference_source'] = 'inline_operator_measurement'
        return result
    point = point_plan.get(phase)
    if point is None:
        result['reference_source'] = 'not_provided'
        return result
    if not isinstance(point, dict):
        raise ValueError('预设点位须为映射或null：'+phase)
    if point.get('kind') == 'theoretical_rotation':
        angle = point.get('turn_deg')
        if isinstance(angle, bool) or not isinstance(angle, (int,float)):
            raise ValueError('理论转角turn_deg须为有限数值')
        xy = theoretical_turn_xy(point.get('center_to_corner_xy_mm'), angle)
        result.update(reference_source='unverified_rigid_rotation_model',
                      theoretical_xy_mm=xy.tolist(), theoretical_yaw_deg=angle,
                      model_assumptions=point.get('assumptions', []),
                      model_parameters={'center_to_corner_xy_mm':point['center_to_corner_xy_mm'],
                                        'turn_deg':angle}, theory_assumptions_verified=False)
        return result
    xy = point.get('xy_mm')
    if (not isinstance(xy, list) or len(xy) != 2 or
            any(isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) for x in xy)):
        raise ValueError('预设点位需要两个有限数值xy_mm：'+phase)
    yaw = point.get('yaw_deg')
    if yaw is not None and (isinstance(yaw, bool) or not isinstance(yaw, (int,float)) or not math.isfinite(yaw)):
        raise ValueError('预设yaw_deg须为有限数值或null：'+phase)
    if point.get('kind') == 'fixed_corner_at_origin':
        if phase != 'B' or xy != [0.,0.] or yaw is not None:
            raise ValueError('固定角点转向参照仅用于B且XY为O、yaw未独立测量')
        result.update(reference_source='fixed_corner_mark_operator_confirmed',
                      reference_xy_mm=[0.,0.],reference_yaw_deg=None,
                      action='rotate_then_realign_same_corner_to_O',
                      reference_scope='操作员在转后朝向将同一角点投影对回O；参照用于比较，不修正输出')
        return result
    result.update(reference_source='premeasured_point_operator_confirmed',
                  reference_xy_mm=list(xy), reference_yaw_deg=yaw)
    return result


def summarize_samples(samples, reference=None, minimum_samples=15,
                      minimum_span_sec=2., max_xy_span_m=.015, max_yaw_span_deg=.5):
    """samples: acquisition stamp, XY in m, yaw in rad; no motion truth inferred."""
    unique = {int(stamp): (int(stamp), *map(float, values)) for stamp, *values in samples}
    rows = np.array([unique[key] for key in sorted(unique)], float)
    if len(rows) < minimum_samples:
        raise ValueError(f'有效定位样本不足：{len(rows)}/{minimum_samples}')
    if not np.isfinite(rows[:, 1:]).all() or min(unique) <= 0:
        raise ValueError('定位样本含无效坐标、角度或时间戳，不能生成本阶段结果')
    # Subtract Python integers first so epoch timestamps retain precision.
    stamps = sorted(unique)
    duration = (stamps[-1]-stamps[0])*1e-9
    if duration < minimum_span_sec:
        raise ValueError('有效静止段跨度不足；请保持车不动后重试本阶段')
    xy = rows[:, 1:3]
    yaw = np.unwrap(rows[:, 3])
    span = np.ptp(xy, axis=0)
    yaw_span = math.degrees(float(np.ptp(yaw)))
    if np.max(span) > max_xy_span_m or yaw_span > max_yaw_span_deg:
        raise ValueError('采样段变化过大；保持车不动后对当前阶段再按回车，无需重做前面阶段')
    mean = np.mean(xy, axis=0)
    angle = math.atan2(math.sin(float(np.mean(yaw))), math.cos(float(np.mean(yaw))))
    report = {'samples': len(rows), 'first_source_stamp_ns': stamps[0],
              'last_source_stamp_ns': stamps[-1], 'duration_sec': duration,
              'mean_xy_m': mean.tolist(), 'mean_yaw_deg': math.degrees(angle),
              'std_xy_mm': (np.std(xy, axis=0)*1000).tolist(),
              'peak_to_peak_xy_mm': (span*1000).tolist(), 'yaw_span_deg': yaw_span,
              'low_variation_check_passed': True,
              'zero_motion_independently_verified': False,
              'operator_reference': reference, 'physical_accuracy_verified': False}
    if reference and 'reference_xy_mm' in reference:
        delta = mean*1000-np.array(reference['reference_xy_mm'])
        report['observed_minus_operator_xy_mm'] = delta.tolist()
        report['observed_minus_operator_distance_mm'] = float(np.linalg.norm(delta))
        if reference.get('reference_yaw_deg') is not None:
            difference = math.degrees(angle)-reference['reference_yaw_deg']
            report['observed_minus_operator_yaw_deg'] = (difference+180.) % 360.-180.
    if reference and 'theoretical_xy_mm' in reference:
        delta = mean*1000-np.array(reference['theoretical_xy_mm'])
        report['observed_minus_theory_xy_mm'] = delta.tolist()
        report['observed_minus_theory_distance_mm'] = float(np.linalg.norm(delta))
        report['theory_assumptions_verified'] = False
        report['theory_comparison_scope'] = '条件几何模型差值，不是独立实测定位误差；转轴/轴向/角度/中心平移均影响结果。'
    return report
