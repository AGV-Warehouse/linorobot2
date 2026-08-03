#!/usr/bin/env python3
# Copyright (c) 2026 AGV Warehouse
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Detects a warehouse cart by its four legs in the raw lidar scan and
# publishes the cart's center pose on `detected_dock_pose`, the topic the
# Nav2 docking server consumes when `use_external_detection_pose: true`
# (see linorobot2_navigation/config/navigation.yaml, docking_server).
#
# Subscribes to /scan_raw (the FULL 360 deg scan, before the angle filter
# crops it to the front 180 deg) so the legs beside/behind the robot stay
# visible while it is driving underneath the cart.
#
# Pipeline per scan:
#   points (odom frame) -> gap clustering -> leg-sized clusters ->
#   rectangle hypotheses from leg pairs (side or diagonal) ->
#   score by how many legs sit on the rectangle's corners ->
#   gate against the previous/expected pose -> EMA filter -> publish.
#
# The cart's leg rectangle dimensions come from the Phase 0 measurement
# checklist (see UNDER_CART_DOCKING.md) via cart_leg_detector.yaml.

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros


def normalize_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class RectangleHypothesis:
    __slots__ = ('cx', 'cy', 'yaw', 'score', 'corner_legs')

    def __init__(self, cx, cy, yaw):
        self.cx = cx
        self.cy = cy
        self.yaw = yaw
        self.score = 0
        self.corner_legs = []


class CartLegDetector(Node):

    def __init__(self):
        super().__init__('cart_leg_detector')

        # Cart geometry (measure the real cart -- Phase 0 checklist).
        # Length is the longer leg-to-leg spacing, width the shorter, both
        # measured center-of-leg to center-of-leg.
        self.declare_parameter('rect_length', 0.90)
        self.declare_parameter('rect_width', 0.60)
        self.declare_parameter('leg_diameter', 0.04)
        # Matching tolerances.
        self.declare_parameter('dim_tolerance', 0.03)
        self.declare_parameter('cluster_gap', 0.06)
        self.declare_parameter('min_points_per_leg', 3)
        self.declare_parameter('min_matched_legs', 2)
        self.declare_parameter('max_range', 3.0)
        # Gating: reject detections farther than gate_radius from the last
        # accepted (or expected) pose. 0 disables gating -- bag testing only;
        # gating is the guard against look-alike leg patterns.
        self.declare_parameter('gate_radius', 0.5)
        # Expected dock pose used to seed the gate before the first
        # detection. Frame '' disables the seed.
        self.declare_parameter('expected_dock_frame', '')
        self.declare_parameter('expected_dock_x', 0.0)
        self.declare_parameter('expected_dock_y', 0.0)
        self.declare_parameter('expected_dock_yaw', 0.0)
        # Output smoothing / bookkeeping.
        self.declare_parameter('filter_alpha', 0.3)
        self.declare_parameter('detection_timeout', 1.0)
        self.declare_parameter('output_frame', 'odom')
        self.declare_parameter('publish_markers', True)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pose_pub = self.create_publisher(
            PoseStamped, 'detected_dock_pose', 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, 'cart_leg_detector/markers', 5)

        self.scan_sub = self.create_subscription(
            LaserScan, 'scan_raw', self.scan_callback,
            qos_profile_sensor_data)

        # Filtered pose state (in output_frame).
        self.filtered = None            # (x, y, yaw)
        self.last_accept_time = None    # rclpy Time of last accepted detection

        self.get_logger().info(
            'cart_leg_detector up: rect %.2fx%.2f m, listening on scan_raw'
            % (self.p('rect_length'), self.p('rect_width')))

    def p(self, name):
        return self.get_parameter(name).value

    # ------------------------------------------------------------------
    # Frame helpers
    # ------------------------------------------------------------------

    def lookup_2d(self, target, source, stamp):
        """TF lookup reduced to 2D (tx, ty, yaw); falls back to latest."""
        try:
            t = self.tf_buffer.lookup_transform(
                target, source, stamp, timeout=Duration(seconds=0.05))
        except tf2_ros.TransformException:
            try:
                t = self.tf_buffer.lookup_transform(target, source, Time())
            except tf2_ros.TransformException as e:
                self.get_logger().warn(
                    'TF %s->%s unavailable: %s' % (target, source, e),
                    throttle_duration_sec=5.0)
                return None
        tr = t.transform.translation
        return (tr.x, tr.y, yaw_from_quaternion(t.transform.rotation))

    @staticmethod
    def apply_2d(tf2d, x, y):
        tx, ty, yaw = tf2d
        c, s = math.cos(yaw), math.sin(yaw)
        return (tx + c * x - s * y, ty + s * x + c * y)

    def gate_prior(self, stamp):
        """Pose the gate compares against: fresh filtered pose, else the
        expected dock pose (transformed into output_frame), else None."""
        timeout = Duration(seconds=self.p('detection_timeout'))
        if (self.filtered is not None and self.last_accept_time is not None
                and (Time.from_msg(stamp) - self.last_accept_time) < timeout):
            return self.filtered
        frame = self.p('expected_dock_frame')
        if not frame:
            return self.filtered  # stale prior still better than nothing
        x, y = self.p('expected_dock_x'), self.p('expected_dock_y')
        yaw = self.p('expected_dock_yaw')
        if frame != self.p('output_frame'):
            tf2d = self.lookup_2d(self.p('output_frame'), frame, stamp)
            if tf2d is None:
                return self.filtered
            x, y = self.apply_2d(tf2d, x, y)
            yaw = normalize_angle(yaw + tf2d[2])
        return (x, y, yaw)

    # ------------------------------------------------------------------
    # Detection pipeline
    # ------------------------------------------------------------------

    def scan_callback(self, scan):
        tf2d = self.lookup_2d(
            self.p('output_frame'), scan.header.frame_id, scan.header.stamp)
        if tf2d is None:
            return

        points = self.scan_to_points(scan, tf2d)
        legs = self.find_legs(points)
        if len(legs) < 2:
            self.publish_markers(scan.header.stamp, legs, None)
            return

        prior = self.gate_prior(scan.header.stamp)
        best = self.fit_rectangle(legs, prior)
        if best is not None:
            self.accept(best, prior, scan.header.stamp)
        self.publish_markers(scan.header.stamp, legs, best)

    def scan_to_points(self, scan, tf2d):
        points = []
        max_range = min(self.p('max_range'), scan.range_max)
        angle = scan.angle_min
        for r in scan.ranges:
            if math.isfinite(r) and scan.range_min <= r <= max_range:
                points.append(self.apply_2d(
                    tf2d, r * math.cos(angle), r * math.sin(angle)))
            else:
                points.append(None)  # keep index alignment for gap logic
            angle += scan.angle_increment
        return points

    def find_legs(self, points):
        """Gap-cluster consecutive points, keep leg-sized clusters, return
        centroids. Merges across the +/-180 deg wraparound of a 360 scan."""
        gap = self.p('cluster_gap')
        clusters = []
        current = []
        for pt in points:
            if pt is None:
                if current:
                    clusters.append(current)
                    current = []
                continue
            if current and math.dist(current[-1], pt) > gap:
                clusters.append(current)
                current = []
            current.append(pt)
        if current:
            clusters.append(current)
        # Wraparound: first and last cluster may be the same physical object.
        if (len(clusters) >= 2 and clusters[0] is not clusters[-1]
                and math.dist(clusters[-1][-1], clusters[0][0]) <= gap):
            clusters[0] = clusters.pop() + clusters[0]

        max_span = self.p('leg_diameter') + 2.0 * self.p('dim_tolerance')
        min_pts = self.p('min_points_per_leg')
        legs = []
        for c in clusters:
            if len(c) < min_pts:
                continue
            xs = [q[0] for q in c]
            ys = [q[1] for q in c]
            if math.hypot(max(xs) - min(xs), max(ys) - min(ys)) > max_span:
                continue
            legs.append((sum(xs) / len(xs), sum(ys) / len(ys)))
        # Noise guard: an absurd number of leg-sized clusters means clutter;
        # keep the closest ones to the prior/robot rather than pairing all.
        if len(legs) > 20:
            legs.sort(key=lambda q: math.hypot(q[0], q[1]))
            legs = legs[:20]
        return legs

    def fit_rectangle(self, legs, prior):
        length = self.p('rect_length')
        width = self.p('rect_width')
        tol = self.p('dim_tolerance')
        diag = math.hypot(length, width)
        diag_angle = math.atan2(width, length)
        square = abs(length - width) <= tol

        hypotheses = []
        for i in range(len(legs)):
            for j in range(i + 1, len(legs)):
                ax, ay = legs[i]
                bx, by = legs[j]
                d = math.dist(legs[i], legs[j])
                if d < 1e-6:
                    continue
                mx, my = (ax + bx) / 2.0, (ay + by) / 2.0
                theta = math.atan2(by - ay, bx - ax)
                px, py = -math.sin(theta), math.cos(theta)  # unit perpendicular
                # Pair spans a WIDTH edge: center offset L/2 along the
                # perpendicular, which is the length axis.
                if abs(d - width) <= tol:
                    for sign in (1.0, -1.0):
                        hypotheses.append(RectangleHypothesis(
                            mx + sign * px * length / 2.0,
                            my + sign * py * length / 2.0,
                            math.atan2(sign * py, sign * px)))
                # Pair spans a LENGTH edge: pair direction IS the length axis.
                if abs(d - length) <= tol:
                    for sign in (1.0, -1.0):
                        hypotheses.append(RectangleHypothesis(
                            mx + sign * px * width / 2.0,
                            my + sign * py * width / 2.0,
                            theta))
                # Pair spans a DIAGONAL: center is the midpoint; the length
                # axis sits +/- diag_angle off the pair direction.
                if abs(d - diag) <= 1.5 * tol:
                    hypotheses.append(
                        RectangleHypothesis(mx, my, theta - diag_angle))
                    hypotheses.append(
                        RectangleHypothesis(mx, my, theta + diag_angle))

        if not hypotheses:
            return None

        gate = self.p('gate_radius')
        if prior is not None and gate > 0.0:
            hypotheses = [h for h in hypotheses
                          if math.hypot(h.cx - prior[0], h.cy - prior[1])
                          <= gate]
            if not hypotheses:
                return None

        corner_tol = 1.5 * tol
        min_legs = self.p('min_matched_legs')
        for h in hypotheses:
            self.score_hypothesis(h, legs, length, width, corner_tol)
        hypotheses = [h for h in hypotheses if h.score >= min_legs]
        if not hypotheses:
            return None

        if prior is not None:
            key = (lambda h: (-h.score,
                              math.hypot(h.cx - prior[0], h.cy - prior[1])))
        else:
            key = (lambda h: (-h.score, math.hypot(h.cx, h.cy)))
        return min(hypotheses, key=key)

    def score_hypothesis(self, h, legs, length, width, corner_tol):
        c, s = math.cos(h.yaw), math.sin(h.yaw)
        used = set()
        for lx, wx in ((length / 2.0, width / 2.0),
                       (length / 2.0, -width / 2.0),
                       (-length / 2.0, width / 2.0),
                       (-length / 2.0, -width / 2.0)):
            cx = h.cx + c * lx - s * wx
            cy = h.cy + s * lx + c * wx
            best_k, best_d = None, corner_tol
            for k, leg in enumerate(legs):
                if k in used:
                    continue
                d = math.hypot(leg[0] - cx, leg[1] - cy)
                if d <= best_d:
                    best_k, best_d = k, d
            if best_k is not None:
                used.add(best_k)
                h.corner_legs.append(legs[best_k])
        h.score = len(used)

    # ------------------------------------------------------------------
    # Output
    # ------------------------------------------------------------------

    def accept(self, h, prior, stamp):
        yaw = h.yaw
        # The rectangle is symmetric: yaw, yaw+pi (and +/-pi/2 when square)
        # describe the same legs. Pick the candidate nearest the prior so the
        # published pose does not flip frame-to-frame.
        ref = prior[2] if prior is not None else None
        if ref is not None:
            candidates = [yaw, normalize_angle(yaw + math.pi)]
            if abs(self.p('rect_length') - self.p('rect_width')) \
                    <= self.p('dim_tolerance'):
                candidates += [normalize_angle(yaw + math.pi / 2.0),
                               normalize_angle(yaw - math.pi / 2.0)]
            yaw = min(candidates,
                      key=lambda a: abs(normalize_angle(a - ref)))

        alpha = self.p('filter_alpha')
        now = Time.from_msg(stamp)
        stale = (self.last_accept_time is None or
                 (now - self.last_accept_time) >
                 Duration(seconds=self.p('detection_timeout')))
        if self.filtered is None or stale:
            self.filtered = (h.cx, h.cy, yaw)
        else:
            fx, fy, fyaw = self.filtered
            self.filtered = (
                fx + alpha * (h.cx - fx),
                fy + alpha * (h.cy - fy),
                normalize_angle(fyaw + alpha * normalize_angle(yaw - fyaw)))
        self.last_accept_time = now

        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.p('output_frame')
        msg.pose.position.x = self.filtered[0]
        msg.pose.position.y = self.filtered[1]
        msg.pose.orientation.z = math.sin(self.filtered[2] / 2.0)
        msg.pose.orientation.w = math.cos(self.filtered[2] / 2.0)
        self.pose_pub.publish(msg)

    def publish_markers(self, stamp, legs, best):
        if not self.p('publish_markers'):
            return
        frame = self.p('output_frame')
        arr = MarkerArray()

        m = Marker()
        m.header.stamp = stamp
        m.header.frame_id = frame
        m.ns = 'legs'
        m.id = 0
        m.type = Marker.SPHERE_LIST
        m.action = Marker.ADD if legs else Marker.DELETE
        m.scale.x = m.scale.y = m.scale.z = 0.06
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.6, 0.0, 0.9
        m.points = [Point(x=q[0], y=q[1], z=0.05) for q in legs]
        arr.markers.append(m)

        r = Marker()
        r.header.stamp = stamp
        r.header.frame_id = frame
        r.ns = 'rectangle'
        r.id = 1
        r.type = Marker.LINE_STRIP
        if best is None:
            r.action = Marker.DELETE
        else:
            r.action = Marker.ADD
            r.scale.x = 0.02
            r.color.g, r.color.a = 1.0, 0.9
            c, s = math.cos(best.yaw), math.sin(best.yaw)
            hl = self.p('rect_length') / 2.0
            hw = self.p('rect_width') / 2.0
            corners = [(hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw), (hl, hw)]
            r.points = [Point(x=best.cx + c * lx - s * wx,
                              y=best.cy + s * lx + c * wx,
                              z=0.05) for lx, wx in corners]
        arr.markers.append(r)

        a = Marker()
        a.header.stamp = stamp
        a.header.frame_id = frame
        a.ns = 'pose'
        a.id = 2
        a.type = Marker.ARROW
        if self.filtered is None:
            a.action = Marker.DELETE
        else:
            a.action = Marker.ADD
            a.scale.x, a.scale.y, a.scale.z = 0.4, 0.04, 0.04
            a.color.b, a.color.a = 1.0, 0.9
            a.pose.position.x = self.filtered[0]
            a.pose.position.y = self.filtered[1]
            a.pose.position.z = 0.05
            a.pose.orientation.z = math.sin(self.filtered[2] / 2.0)
            a.pose.orientation.w = math.cos(self.filtered[2] / 2.0)
        arr.markers.append(a)

        self.marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = CartLegDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
