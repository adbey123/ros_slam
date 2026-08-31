"""Offline checks for the scan matcher, with no ROS graph involved."""

import math

import numpy as np
import pytest
from scipy.spatial import cKDTree

from laser_scan_odometry.icp_odom_node import (
    IcpOdometry, se2_apply, se2_compose, se2_inverse, wrap,
)

HX, HY = 3.0, 2.0        # a 6 x 4 m rectangular room
BEAMS = 400


def cast(sx, sy, angle):
    """Range from (sx, sy) along `angle` to the nearest wall."""
    dx, dy = math.cos(angle), math.sin(angle)
    best = math.inf
    walls = ((HX, dx, sx, sy, HY, dy), (-HX, dx, sx, sy, HY, dy),
             (HY, dy, sy, sx, HX, dx), (-HY, dy, sy, sx, HX, dx))
    for lim, d, origin, other_origin, other_lim, other_d in walls:
        if abs(d) < 1e-9:
            continue
        t = (lim - origin) / d
        if t <= 0 or t >= best:
            continue
        if abs(other_origin + t * other_d) <= other_lim + 1e-6:
            best = t
    return best


def scan_at(sx, sy, yaw):
    """Points as the sensor at (sx, sy, yaw) would report them, in its own frame."""
    inc = 2 * math.pi / BEAMS
    pts = []
    for i in range(BEAMS):
        a = -math.pi + i * inc
        r = cast(sx, sy, yaw + a)
        if math.isfinite(r) and r > 0.15:
            pts.append((r * math.cos(a), r * math.sin(a)))
    return np.array(pts)


class BareMatcher:
    """IcpOdometry.match without the rclpy Node machinery."""

    max_iterations = 40
    max_corr = 0.5
    huber = 0.10
    damping = 1e-9
    max_step = 0.5
    min_points = 40
    match = IcpOdometry.match

    def __init__(self, keyframe):
        self.kf_points = keyframe
        self.kf_tree = cKDTree(keyframe)


# --------------------------------------------------------------- SE(2) algebra

def test_wrap_folds_into_pi_range():
    for a in (-7.0, -math.pi, 0.0, math.pi - 1e-9, 7.0):
        assert -math.pi <= wrap(a) < math.pi
    assert wrap(3 * math.pi) == pytest.approx(-math.pi, abs=1e-12)


def test_compose_matches_homogeneous_matrices():
    def hom(p):
        c, s = math.cos(p[2]), math.sin(p[2])
        return np.array([[c, -s, p[0]], [s, c, p[1]], [0, 0, 1]])

    a = np.array([1.0, -2.0, 0.3])
    b = np.array([0.5, 0.25, -1.1])
    assert np.allclose(hom(se2_compose(a, b)), hom(a) @ hom(b), atol=1e-12)


def test_inverse_undoes_compose():
    a = np.array([1.0, -2.0, 0.3])
    assert np.allclose(se2_compose(a, se2_inverse(a)), np.zeros(3), atol=1e-12)
    assert np.allclose(se2_compose(se2_inverse(a), a), np.zeros(3), atol=1e-12)


def test_apply_agrees_with_compose_on_the_origin():
    a = np.array([1.0, -2.0, 0.3])
    b = np.array([0.4, 0.7, 0.0])
    assert np.allclose(se2_apply(a, b[:2].reshape(1, 2))[0],
                       se2_compose(a, b)[:2], atol=1e-12)


def test_repeated_inverse_compose_does_not_drift():
    """The bug this representation exists to prevent.

    Storing rotations as 2x2 matrices and inverting them by transposition
    squares any loss of orthonormality on every round trip, so the pose decays
    away from SO(2) doubly exponentially. A scalar angle cannot leave the
    manifold at all.
    """
    pose = np.zeros(3)
    step = np.array([0.02, 0.0, 0.01])
    for _ in range(20000):
        nxt = se2_compose(pose, step)
        delta = se2_compose(se2_inverse(pose), nxt)
        pose = se2_compose(pose, delta)
    # 20000 steps of 0.01 rad, folded back into [-pi, pi).
    assert pose[2] == pytest.approx(wrap(20000 * 0.01), abs=1e-6)
    assert math.hypot(pose[0], pose[1]) < 1.0e3      # bounded, not exploded
    assert np.all(np.isfinite(pose))


# ----------------------------------------------------------------- the matcher

@pytest.mark.parametrize('dx,dy,dyaw', [
    (0.00, 0.00, 0.00),
    (0.10, 0.00, 0.00),
    (0.00, 0.08, 0.00),
    (0.05, -0.05, 0.10),
    (0.20, 0.15, -0.25),
    (0.00, 0.00, 0.30),
])
def test_match_recovers_known_motion(dx, dy, dyaw):
    kf = scan_at(0.0, 0.0, 0.0)
    src = scan_at(dx, dy, dyaw)
    pose, err, inliers = BareMatcher(kf).match(src, np.zeros(3))

    assert err < 0.03, f'poor fit: {err}'
    assert inliers > 0.9
    assert pose[0] == pytest.approx(dx, abs=0.02)
    assert pose[1] == pytest.approx(dy, abs=0.02)
    assert pose[2] == pytest.approx(dyaw, abs=0.02)


def test_match_reports_failure_on_an_unmatchable_cloud():
    kf = scan_at(0.0, 0.0, 0.0)
    junk = np.random.default_rng(0).uniform(-30.0, -20.0, size=(300, 2))
    _, err, inliers = BareMatcher(kf).match(junk, np.zeros(3))
    assert not math.isfinite(err) or err > 0.2
    assert inliers < 0.6


def test_rotation_is_recovered_despite_angular_resampling():
    """Point-to-point ICP biases this case; point-to-line should not.

    A pure in-place rotation moves every sample along the wall it sits on, so a
    matcher that insists on pairing specific points fights the resampling.
    """
    kf = scan_at(0.0, 0.0, 0.0)
    src = scan_at(0.0, 0.0, 0.25)
    pose, err, _ = BareMatcher(kf).match(src, np.zeros(3))
    assert pose[2] == pytest.approx(0.25, abs=0.01)
    assert math.hypot(pose[0], pose[1]) < 0.02
    assert err < 0.03


def test_tracks_a_full_trajectory_without_diverging():
    """End-to-end dead reckoning over a 4 m drive with 2 rad of rotation."""
    pose = np.zeros(3)
    kf_pose = np.zeros(3)
    last = np.zeros(3)
    matcher = BareMatcher(scan_at(-2.0, 0.0, 0.0))
    worst_xy = worst_yaw = 0.0
    rejects = 0

    for k in range(1, 201):
        sx, yaw = -2.0 + 0.02 * k, 0.01 * k
        src = scan_at(sx, 0.0, yaw)
        seed = se2_compose(se2_inverse(kf_pose), se2_compose(pose, last))
        relative, err, inliers = matcher.match(src, seed)
        new = se2_compose(kf_pose, relative)

        if not math.isfinite(err) or err > 0.10 or inliers < 0.60:
            rejects += 1
            last = np.zeros(3)
            matcher = BareMatcher(src)
            kf_pose = pose.copy()
            continue

        last = se2_compose(se2_inverse(pose), new)
        pose = new
        rel = se2_compose(se2_inverse(kf_pose), pose)
        if np.linalg.norm(rel[:2]) > 0.15 or abs(rel[2]) > 0.10:
            matcher = BareMatcher(src)
            kf_pose = pose.copy()

        worst_xy = max(worst_xy, math.hypot(pose[0] - (sx + 2.0), pose[1]))
        worst_yaw = max(worst_yaw, abs(wrap(pose[2] - yaw)))

    assert rejects == 0, f'{rejects} frames failed to match'
    assert worst_xy < 0.05, f'position drift {worst_xy:.3f} m'
    assert worst_yaw < 0.02, f'heading drift {worst_yaw:.3f} rad'
