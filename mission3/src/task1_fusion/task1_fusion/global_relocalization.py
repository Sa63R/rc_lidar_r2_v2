"""Candidate confidence checks for automatic map initialization (no ROS)."""
import math
import numpy as np


def matrix(candidate):
    T = np.asarray(candidate['map_T_camera_init'], dtype=float)
    if (T.shape != (4, 4) or not np.isfinite(T).all()
            or not np.allclose(T[3], [0., 0., 0., 1.], atol=1e-5)
            or not np.allclose(T[:3, :3].T @ T[:3, :3], np.eye(3), atol=1e-4)
            or not np.isclose(np.linalg.det(T[:3, :3]), 1., atol=1e-4)
            or T[2, 2] < math.cos(math.radians(15))):
        raise ValueError('invalid or gravity-inconsistent map transform')
    return T


def difference(a, b):
    A, B = matrix(a), matrix(b)
    distance = np.linalg.norm(A[:3, 3]-B[:3, 3])
    angle = math.acos(float(np.clip((np.trace(A[:3, :3].T @ B[:3, :3])-1.)/2., -1., 1.)))
    return float(distance), float(angle)


def select_candidate(report):
    """Conservative engineering gate, not proof of unique physical location.

    Require repeated consistent solutions and reject a similarly good distinct
    location. Thresholds are overlap/geometry quality limits, not cm accuracy.
    """
    usable = []
    for candidate in report.get('candidates', []):
        try:
            matrix(candidate)
            rms = float(candidate['inlier_rms_m'])
            overlap = float(candidate['overlap'])
            e = np.asarray(candidate['support_eigenvalues'], dtype=float)
            if (math.isfinite(rms) and 0. <= rms <= .12 and math.isfinite(overlap)
                    and .70 <= overlap <= 1. and e.shape == (3,) and np.isfinite(e).all()
                    and e[0] >= .02 and e[2] > 0. and e[0]/e[2] >= .001
                    and int(candidate['inliers']) >= 200):
                usable.append(candidate)
        except (KeyError, TypeError, ValueError, OverflowError):
            continue
    if not usable:
        return {'accepted': False, 'reason': 'no_geometrically_supported_candidate'}
    best = min(usable, key=lambda c: (-float(c['overlap']), float(c['inlier_rms_m'])))
    consistent_trials = set()
    for candidate in usable:
        distance, angle = difference(best, candidate)
        if distance <= .30 and angle <= math.radians(5):
            consistent_trials.add(int(candidate['trial']))
        elif (distance >= .75 or angle >= math.radians(15)):
            if (candidate['overlap'] >= best['overlap']-.05
                    and candidate['inlier_rms_m'] <= best['inlier_rms_m']+.04):
                return {'accepted': False, 'reason': 'ambiguous_distinct_locations',
                        'best': best, 'competing': candidate}
    # Trial 3 is an exclusion search, not an independent full-map confirmation.
    if len(consistent_trials & {0, 1, 2}) < 2:
        return {'accepted': False, 'reason': 'insufficient_independent_agreement', 'best': best}
    return {'accepted': True, 'reason': 'consistent_global_candidates', 'best': best,
            'agreement_trials': sorted(consistent_trials), 'global_uniqueness_proven': False,
            'physical_accuracy_verified': False}
