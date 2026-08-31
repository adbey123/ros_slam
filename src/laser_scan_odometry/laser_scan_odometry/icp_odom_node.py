#!/usr/bin/env python3
"""2D scan-matching odometry from a single LaserScan.

The lidar platform has no wheel encoders, but slam_toolbox still requires an
``odom -> base`` transform to exist.  This node manufactures one by matching each
incoming scan against a retained keyframe and integrating the result.

Two design choices matter more than the rest:

* **Poses are (x, y, theta) triples, never 2x2 matrices.**  Carrying a rotation
  matrix as accumulated state invites it to drift out of SO(2), and because
  inverting a rotation is normally done by transposing it, any such drift is
  squared on every inverse-and-compose round trip.  A scalar angle cannot leave
  the manifold, so the problem cannot arise.

* **The residual is point-to-line, not point-to-point.**  A scan samples
  surfaces at fixed angular positions, so the sample points slide along a wall
  as the sensor rotates.  Point-to-point ICP tries to pin each sample to a
  particular neighbour and biases the fit; measuring along the local surface
  normal lets samples slide freely, which is what actually happens.

It is dead reckoning, so it drifts.  slam_toolbox corrects that drift in
``map -> odom``, which is the split the ROS navigation stack expects.
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from scipy.spatial import cKDTree
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

IDENTITY = np.zeros(3)


def wrap(angle):
    """Fold an angle into [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def se2_compose(a, b):
    """Return a * b, both SE(2) poses as (x, y, theta)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return np.array([a[0] + c * b[0] - s * b[1],
                     a[1] + s * b[0] + c * b[1],
                     wrap(a[2] + b[2])])


def se2_inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return np.array([-(c * a[0] + s * a[1]),
                     -(-s * a[0] + c * a[1]),
                     wrap(-a[2])])


def se2_apply(a, pts):
    """Transform an (N, 2) array of points by the pose a."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return pts @ np.array([[c, s], [-s, c]]) + a[:2]


def yaw_to_quat(yaw):
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class IcpOdometry(Node):

    def __init__(self):
        super().__init__('icp_odom_node')

        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        # Range gating: drop returns off the robot itself and beyond the range
        # where the sensor is trustworthy.
        self.declare_parameter('min_range', 0.15)
        self.declare_parameter('max_range', 10.0)
        # Matcher tuning.
        self.declare_parameter('max_iterations', 40)
        self.declare_parameter('max_correspondence_distance', 0.5)
        self.declare_parameter('huber_delta', 0.10)
        self.declare_parameter('damping', 1.0e-9)
        self.declare_parameter('max_step', 0.5)
        self.declare_parameter('min_points', 40)
        # Match acceptance.
        self.declare_parameter('max_mean_error', 0.10)
        self.declare_parameter('min_inlier_ratio', 0.60)
        # A scan-to-scan step beyond these is a bad match, not motion.
        self.declare_parameter('max_translation_per_scan', 0.5)
        self.declare_parameter('max_rotation_per_scan', 0.6)
        self.declare_parameter('max_consecutive_failures', 5)
        # Keyframe replacement.
        self.declare_parameter('keyframe_distance', 0.15)
        self.declare_parameter('keyframe_angle', 0.10)

        gp = self.get_parameter
        self.odom_frame = gp('odom_frame').value
        self.base_frame = gp('base_frame').value
        self.publish_tf = gp('publish_tf').value
        self.min_range = gp('min_range').value
        self.max_range = gp('max_range').value
        self.max_iterations = gp('max_iterations').value
        self.max_corr = gp('max_correspondence_distance').value
        self.huber = gp('huber_delta').value
        self.damping = gp('damping').value
        self.max_step = gp('max_step').value
        self.min_points = gp('min_points').value
        self.max_mean_error = gp('max_mean_error').value
        self.min_inlier_ratio = gp('min_inlier_ratio').value
        self.max_step_t = gp('max_translation_per_scan').value
        self.max_step_r = gp('max_rotation_per_scan').value
        self.max_failures = gp('max_consecutive_failures').value
        self.kf_dist = gp('keyframe_distance').value
        self.kf_angle = gp('keyframe_angle').value

        # Pose of the laser in the odom frame, and the keyframe's pose.
        self.pose = IDENTITY.copy()
        self.kf_pose = IDENTITY.copy()
        self.kf_points = None
        self.kf_tree = None
        # Constant-velocity seed for the next match.
        self.last_delta = IDENTITY.copy()
        self.last_stamp = None
        self.laser_to_base = None
        self.consecutive_failures = 0
        self.total_failures = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Scans arrive over WiFi from the lidar laptop, so accept BEST_EFFORT.
        scan_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.sub = self.create_subscription(
            LaserScan, gp('scan_topic').value, self.on_scan, scan_qos)
        self.pub = self.create_publisher(Odometry, gp('odom_topic').value, 10)

        self.get_logger().info(
            f"ICP odometry up: '{gp('scan_topic').value}' -> "
            f"'{gp('odom_topic').value}' ({self.odom_frame} -> {self.base_frame})")

    # ---------------------------------------------------------------- helpers

    def scan_to_points(self, scan):
        ranges = np.asarray(scan.ranges, dtype=np.float64)
        angles = scan.angle_min + np.arange(ranges.size) * scan.angle_increment
        lo = max(self.min_range, scan.range_min)
        hi = min(self.max_range, scan.range_max)
        keep = np.isfinite(ranges) & (ranges > lo) & (ranges < hi)
        r, a = ranges[keep], angles[keep]
        return np.column_stack((r * np.cos(a), r * np.sin(a)))

    def lookup_laser_to_base(self, laser_frame):
        """Static offset from the laser frame to the base frame, as SE(2)."""
        if self.laser_to_base is not None:
            return self.laser_to_base
        try:
            tf = self.tf_buffer.lookup_transform(
                laser_frame, self.base_frame, rclpy.time.Time())
        except Exception as exc:  # noqa: BLE001 - tf2 raises several types
            self.get_logger().warn(
                f'no {laser_frame} -> {self.base_frame} transform yet ({exc}); '
                'assuming they coincide', throttle_duration_sec=5.0)
            return None
        self.laser_to_base = np.array([
            tf.transform.translation.x,
            tf.transform.translation.y,
            quat_to_yaw(tf.transform.rotation),
        ])
        self.get_logger().info(
            'laser->base offset: x={:.3f} y={:.3f} yaw={:.3f}'.format(
                *self.laser_to_base))
        return self.laser_to_base

    def match(self, src, seed):
        """Align src onto the keyframe. Returns (pose, mean_error, inlier_ratio)."""
        pose = np.array(seed, dtype=float)
        for _ in range(self.max_iterations):
            moved = se2_apply(pose, src)
            dist, idx = self.kf_tree.query(
                moved, k=2, distance_upper_bound=self.max_corr)
            valid = np.isfinite(dist[:, 0]) & np.isfinite(dist[:, 1])
            if int(valid.sum()) < self.min_points:
                return pose, float('inf'), 0.0

            p = moved[valid]
            q1, q2 = self.kf_points[idx[valid, 0]], self.kf_points[idx[valid, 1]]
            seg = q2 - q1
            length = np.linalg.norm(seg, axis=1)
            keep = length > 1.0e-6
            p, q1, seg, length = p[keep], q1[keep], seg[keep], length[keep]
            if p.shape[0] < self.min_points:
                return pose, float('inf'), 0.0

            normal = np.column_stack((-seg[:, 1], seg[:, 0])) / length[:, None]
            resid = np.einsum('ij,ij->i', normal, p - q1)

            # Huber weights rather than discarding a fixed worst-case quantile:
            # outliers get damped, but sparse high-residual features such as
            # corners keep their vote, and corners are what constrain rotation.
            weight = np.ones_like(resid)
            large = np.abs(resid) > self.huber
            weight[large] = self.huber / np.abs(resid[large])

            perp = np.column_stack((-p[:, 1], p[:, 0]))
            jac = np.column_stack((
                normal[:, 0],
                normal[:, 1],
                np.einsum('ij,ij->i', normal, perp),
            ))
            weighted = jac * weight[:, None]
            hessian = jac.T @ weighted + self.damping * np.eye(3)
            try:
                delta = np.linalg.solve(hessian, -weighted.T @ resid)
            except np.linalg.LinAlgError:
                return pose, float('inf'), 0.0
            if not np.all(np.isfinite(delta)):
                return pose, float('inf'), 0.0

            norm = float(np.linalg.norm(delta[:2]))
            if norm > self.max_step:
                delta[:2] *= self.max_step / norm
            delta[2] = float(np.clip(delta[2], -self.max_step, self.max_step))

            # The increment was linearised about points already in the keyframe
            # frame, so it composes on the left.
            pose = se2_compose(delta, pose)
            if abs(delta[2]) < 1.0e-6 and norm < 1.0e-6:
                break

        moved = se2_apply(pose, src)
        dist, _ = self.kf_tree.query(
            moved, k=1, distance_upper_bound=self.max_corr)
        inliers = np.isfinite(dist)
        err = float(np.mean(dist[inliers])) if inliers.any() else float('inf')
        return pose, err, float(inliers.mean())

    def set_keyframe(self, points, pose):
        self.kf_points = points
        self.kf_tree = cKDTree(points)
        self.kf_pose = np.array(pose, dtype=float)

    # --------------------------------------------------------------- callback

    def on_scan(self, scan):
        points = self.scan_to_points(scan)
        if points.shape[0] < self.min_points:
            self.get_logger().warn(
                f'only {points.shape[0]} usable returns in this scan; skipping',
                throttle_duration_sec=5.0)
            return

        if self.kf_points is None:
            self.set_keyframe(points, self.pose)
            self.last_stamp = scan.header.stamp
            self.publish(scan)
            return

        # Seed from the constant-velocity model, expressed relative to the
        # keyframe.
        predicted = se2_compose(self.pose, self.last_delta)
        seed = se2_compose(se2_inverse(self.kf_pose), predicted)

        relative, err, inliers = self.match(points, seed)
        new_pose = se2_compose(self.kf_pose, relative)
        step = se2_compose(se2_inverse(self.pose), new_pose)
        step_t = float(np.linalg.norm(step[:2]))
        step_r = abs(step[2])

        rejected = None
        if not math.isfinite(err) or err > self.max_mean_error:
            rejected = f'mean error {err:.3f} m'
        elif inliers < self.min_inlier_ratio:
            # A confident-looking fit can still be wrong; a low inlier ratio
            # catches the case where the residual is small only because the
            # matcher settled on a subset of the scan.
            rejected = f'only {inliers:.0%} of points matched'
        elif step_t > self.max_step_t or step_r > self.max_step_r:
            rejected = f'implausible step {step_t:.2f} m / {step_r:.2f} rad'

        if rejected is not None:
            self.consecutive_failures += 1
            self.total_failures += 1
            # Hold the last good pose and stop extrapolating. Coasting on a
            # stale velocity compounds: each bad frame seeds the next one worse,
            # so the estimate diverges instead of merely stalling.
            self.last_delta = IDENTITY.copy()
            self.get_logger().warn(
                f'scan match rejected ({rejected}); holding pose '
                f'[{self.consecutive_failures} in a row, '
                f'{self.total_failures} total]',
                throttle_duration_sec=2.0)
            if self.consecutive_failures >= self.max_failures:
                # Re-acquire against the current scan. The held pose is wrong by
                # however far we actually moved and only slam_toolbox can fix
                # that, but a stale keyframe never will.
                self.get_logger().error(
                    f'{self.consecutive_failures} consecutive failures; '
                    'resetting the keyframe. Odometry has lost track -- expect '
                    'slam_toolbox to correct a jump.')
                self.set_keyframe(points, self.pose)
                self.consecutive_failures = 0
            self.publish(scan)
            return

        self.consecutive_failures = 0
        self.last_delta = step
        self.pose = new_pose

        # Replace the keyframe once overlap starts to fall away.
        rel = se2_compose(se2_inverse(self.kf_pose), self.pose)
        if float(np.linalg.norm(rel[:2])) > self.kf_dist or abs(rel[2]) > self.kf_angle:
            self.set_keyframe(points, self.pose)

        self.publish(scan)

    def publish(self, scan):
        laser_to_base = self.lookup_laser_to_base(scan.header.frame_id)
        base = self.pose if laser_to_base is None else se2_compose(
            self.pose, laser_to_base)

        dt = 0.0
        if self.last_stamp is not None:
            dt = ((scan.header.stamp.sec - self.last_stamp.sec)
                  + (scan.header.stamp.nanosec - self.last_stamp.nanosec) * 1e-9)
        self.last_stamp = scan.header.stamp

        odom = Odometry()
        odom.header.stamp = scan.header.stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = float(base[0])
        odom.pose.pose.position.y = float(base[1])
        odom.pose.pose.orientation = yaw_to_quat(float(base[2]))
        if dt > 1.0e-6:
            # last_delta is already expressed in the body frame.
            odom.twist.twist.linear.x = float(self.last_delta[0] / dt)
            odom.twist.twist.linear.y = float(self.last_delta[1] / dt)
            odom.twist.twist.angular.z = float(self.last_delta[2] / dt)
        # Dead reckoning, so advertise loose covariances; slam_toolbox is what
        # actually pins the pose down.
        odom.pose.covariance[0] = 0.05
        odom.pose.covariance[7] = 0.05
        odom.pose.covariance[35] = 0.10
        self.pub.publish(odom)

        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp = scan.header.stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = float(base[0])
            tf.transform.translation.y = float(base[1])
            tf.transform.rotation = yaw_to_quat(float(base[2]))
            self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = IcpOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
