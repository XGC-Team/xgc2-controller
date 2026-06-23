#!/usr/bin/env python3

import math
from dataclasses import dataclass
from typing import Tuple

import rospy
from geometry_msgs.msg import Point, PoseStamped, Vector3
from reference_trajectory.msg import (
    UavBsplineTrajectory,
    UavFlatTrajectory,
    UavFlatTrajectoryPoint,
)

@dataclass
class CurveParams:
    radius: float
    omega: float
    height: float
    line_speed: float
    helix_scl: float
    z_amplitude: float
    z_omega_multiplier: float
    entry_duration: float
    start_x: float
    start_y: float
    start_yaw: float
    center_x: float
    center_y: float


def finite(value: float) -> float:
    return value if math.isfinite(value) else 0.0


def wrap_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def interpolate_angle(start: float, end: float, ratio: float) -> float:
    ratio = max(0.0, min(1.0, ratio))
    delta = wrap_angle(end - start)
    return wrap_angle(start + ratio * delta)


def unwrap_angle_near(value: float, reference: float) -> float:
    return reference + wrap_angle(value - reference)


def smoothstep(value: float) -> float:
    value = max(0.0, min(1.0, value))
    return value * value * (3.0 - 2.0 * value)


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return finite(math.atan2(siny_cosp, cosy_cosp))


def quintic_boundary(t: float, duration: float, p0: float, v0: float, a0: float,
                     p1: float, v1: float, a1: float):
    if duration <= 1e-6:
        return p1, v1, a1, 0.0, 0.0
    tau = max(0.0, min(duration, t))
    T = duration
    c0 = p0
    c1 = v0
    c2 = 0.5 * a0
    T2, T3, T4, T5 = T**2, T**3, T**4, T**5
    c3 = (
        20.0 * (p1 - p0)
        - (8.0 * v1 + 12.0 * v0) * T
        - (3.0 * a0 - a1) * T2
    ) / (2.0 * T3)
    c4 = (
        30.0 * (p0 - p1)
        + (14.0 * v1 + 16.0 * v0) * T
        + (3.0 * a0 - 2.0 * a1) * T2
    ) / (2.0 * T4)
    c5 = (
        12.0 * (p1 - p0)
        - (6.0 * v1 + 6.0 * v0) * T
        - (a0 - a1) * T2
    ) / (2.0 * T5)
    p = c0 + c1 * tau + c2 * tau**2 + c3 * tau**3 + c4 * tau**4 + c5 * tau**5
    v = c1 + 2.0 * c2 * tau + 3.0 * c3 * tau**2 + 4.0 * c4 * tau**3 + 5.0 * c5 * tau**4
    a = 2.0 * c2 + 6.0 * c3 * tau + 12.0 * c4 * tau**2 + 20.0 * c5 * tau**3
    j = 6.0 * c3 + 24.0 * c4 * tau + 60.0 * c5 * tau**2
    s = 24.0 * c4 + 120.0 * c5 * tau
    return p, v, a, j, s


def sample_curve(
    name: str, t: float, params: CurveParams
) -> Tuple[Point, Vector3, Vector3, Vector3, Vector3, float]:
    p = Point()
    v = Vector3()
    a = Vector3()
    jerk = Vector3()
    snap = Vector3()
    yaw = 0.0
    radius = params.radius
    omega = params.omega
    height = params.height
    line_speed = params.line_speed
    helix_scl = params.helix_scl
    z_amplitude = max(0.0, finite(params.z_amplitude))
    z_omega = omega * finite(params.z_omega_multiplier)

    if name == "line":
        p.x = line_speed * t
        p.z = height
        v.x = line_speed
    elif name == "hold":
        p.x = params.start_x
        p.y = params.start_y
        p.z = height
        yaw = params.start_yaw
    elif name == "circle_entry":
        entry_duration = max(params.entry_duration, 0.1)
        if t < entry_duration:
            x = quintic_boundary(
                t, entry_duration, params.start_x, 0.0, 0.0,
                params.center_x + radius, 0.0, -radius * omega * omega)
            y = quintic_boundary(
                t, entry_duration, params.start_y, 0.0, 0.0,
                params.center_y, radius * omega, 0.0)
            p.x, v.x, a.x, jerk.x, snap.x = x
            p.y, v.y, a.y, jerk.y, snap.y = y
            z = quintic_boundary(
                t, entry_duration, height, 0.0, 0.0,
                height, z_amplitude * z_omega, 0.0)
            p.z, v.z, a.z, jerk.z, snap.z = z
            if abs(v.x) > 1e-6 or abs(v.y) > 1e-6:
                tangent_yaw = math.atan2(v.y, v.x)
            else:
                tangent_yaw = params.start_yaw
            yaw = interpolate_angle(
                params.start_yaw,
                tangent_yaw,
                smoothstep(t / entry_duration),
            )
        else:
            wt = omega * (t - entry_duration)
            zt = z_omega * (t - entry_duration)
            p.x = params.center_x + radius * math.cos(wt)
            p.y = params.center_y + radius * math.sin(wt)
            p.z = height + z_amplitude * math.sin(zt)
            v.x = -radius * omega * math.sin(wt)
            v.y = radius * omega * math.cos(wt)
            v.z = z_amplitude * z_omega * math.cos(zt)
            a.x = -radius * omega * omega * math.cos(wt)
            a.y = -radius * omega * omega * math.sin(wt)
            a.z = -z_amplitude * z_omega * z_omega * math.sin(zt)
            jerk.x = radius * omega**3 * math.sin(wt)
            jerk.y = -radius * omega**3 * math.cos(wt)
            jerk.z = -z_amplitude * z_omega**3 * math.cos(zt)
            snap.x = radius * omega**4 * math.cos(wt)
            snap.y = radius * omega**4 * math.sin(wt)
            snap.z = z_amplitude * z_omega**4 * math.sin(zt)
        if t >= entry_duration and (abs(v.x) > 1e-6 or abs(v.y) > 1e-6):
            yaw = math.atan2(v.y, v.x)
    elif name == "circle":
        wt = omega * t
        p.x = params.center_x + radius * math.cos(wt)
        p.y = params.center_y + radius * math.sin(wt)
        p.z = height
        v.x = -radius * omega * math.sin(wt)
        v.y = radius * omega * math.cos(wt)
        a.x = -radius * omega * omega * math.cos(wt)
        a.y = -radius * omega * omega * math.sin(wt)
        jerk.x = radius * omega**3 * math.sin(wt)
        jerk.y = -radius * omega**3 * math.cos(wt)
        snap.x = radius * omega**4 * math.cos(wt)
        snap.y = radius * omega**4 * math.sin(wt)
        yaw = math.atan2(v.y, v.x)
    elif name == "lemniscate":
        wt = omega * t
        p.x = radius * math.sin(wt)
        p.y = 0.5 * radius * math.sin(2.0 * wt)
        p.z = height
        v.x = radius * omega * math.cos(wt)
        v.y = radius * omega * math.cos(2.0 * wt)
        a.x = -radius * omega * omega * math.sin(wt)
        a.y = -2.0 * radius * omega * omega * math.sin(2.0 * wt)
        jerk.x = -radius * omega**3 * math.cos(wt)
        jerk.y = -4.0 * radius * omega**3 * math.cos(2.0 * wt)
        snap.x = radius * omega**4 * math.sin(wt)
        snap.y = 8.0 * radius * omega**4 * math.sin(2.0 * wt)
        yaw = math.atan2(v.y, v.x)
    elif name == "helix_xy":
        wt = omega * t
        p.x = radius * math.cos(wt)
        p.y = radius * math.sin(wt)
        p.z = height + helix_scl * t
        v.x = -radius * omega * math.sin(wt)
        v.y = radius * omega * math.cos(wt)
        v.z = helix_scl
        a.x = -radius * omega * omega * math.cos(wt)
        a.y = -radius * omega * omega * math.sin(wt)
        jerk.x = radius * omega**3 * math.sin(wt)
        jerk.y = -radius * omega**3 * math.cos(wt)
        snap.x = radius * omega**4 * math.cos(wt)
        snap.y = radius * omega**4 * math.sin(wt)
        yaw = math.atan2(v.y, v.x)
    elif name == "helix_yz":
        wt = omega * t
        p.x = line_speed * t
        p.y = radius * math.cos(wt)
        p.z = height + radius * math.sin(wt)
        v.x = line_speed
        v.y = -radius * omega * math.sin(wt)
        v.z = radius * omega * math.cos(wt)
        a.y = -radius * omega * omega * math.cos(wt)
        a.z = -radius * omega * omega * math.sin(wt)
        jerk.y = radius * omega**3 * math.sin(wt)
        jerk.z = -radius * omega**3 * math.cos(wt)
        snap.y = radius * omega**4 * math.cos(wt)
        snap.z = radius * omega**4 * math.sin(wt)
        yaw = math.atan2(v.y, v.x)
    elif name == "torus_knot":
        wt = omega * t
        r = radius * (1.0 + 0.35 * math.cos(3.0 * wt))
        p.x = r * math.cos(2.0 * wt)
        p.y = r * math.sin(2.0 * wt)
        p.z = height + 0.35 * radius * math.sin(3.0 * wt)
    else:
        p.z = height

    p.x = finite(p.x)
    p.y = finite(p.y)
    p.z = finite(p.z)
    v.x = finite(v.x)
    v.y = finite(v.y)
    v.z = finite(v.z)
    a.x = finite(a.x)
    a.y = finite(a.y)
    a.z = finite(a.z)
    jerk.x = finite(jerk.x)
    jerk.y = finite(jerk.y)
    jerk.z = finite(jerk.z)
    snap.x = finite(snap.x)
    snap.y = finite(snap.y)
    snap.z = finite(snap.z)
    return p, v, a, jerk, snap, finite(yaw)


def make_open_uniform_knots(control_count: int, degree: int, duration: float):
    interior = control_count - degree - 1
    knots = [0.0] * (degree + 1)
    if interior > 0:
        for i in range(1, interior + 1):
            knots.append(duration * float(i) / float(interior + 1))
    knots.extend([duration] * (degree + 1))
    return knots


def finite_difference(values, times, index):
    if len(values) < 2:
        return 0.0
    if index <= 0:
        dt = max(1e-6, times[1] - times[0])
        return (values[1] - values[0]) / dt
    if index + 1 >= len(values):
        dt = max(1e-6, times[index] - times[index - 1])
        return (values[index] - values[index - 1]) / dt
    dt = max(1e-6, times[index + 1] - times[index - 1])
    return (values[index + 1] - values[index - 1]) / dt


def main():
    rospy.init_node("uav_reference_trajectory_publisher")
    curve = rospy.get_param("~curve", "lemniscate")
    rate_hz = float(rospy.get_param("~rate", 10.0))
    horizon = float(rospy.get_param("~horizon", 6.0))
    sample_dt = float(rospy.get_param("~sample_dt", 0.05))
    degree = int(rospy.get_param("~bspline_order", 3))
    frame_id = rospy.get_param("~frame_id", "map")
    publish_bspline = bool(rospy.get_param("~publish_bspline", True))
    publish_flat = bool(rospy.get_param("~publish_flat", True))
    time_origin = float(rospy.get_param("~time_origin", 0.0))
    activation_mode = str(rospy.get_param("~activation_mode", "immediate")).lower()
    activation_topic = rospy.get_param(
        "~activation_topic", "alg/reference_trajectory/activate"
    )
    use_activation_pose = bool(rospy.get_param("~use_activation_pose", True))
    curve_params = CurveParams(
        radius=float(rospy.get_param("~radius", 1.0)),
        omega=float(rospy.get_param("~omega", 0.35)),
        height=float(rospy.get_param("~height", 1.5)),
        line_speed=float(rospy.get_param("~line_speed", 0.3)),
        helix_scl=float(rospy.get_param("~helix_scl", 0.15)),
        z_amplitude=float(rospy.get_param("~z_amplitude", 0.0)),
        z_omega_multiplier=float(rospy.get_param("~z_omega_multiplier", 1.0)),
        entry_duration=float(rospy.get_param("~entry_duration", 5.0)),
        start_x=float(rospy.get_param("~start_x", 0.0)),
        start_y=float(rospy.get_param("~start_y", 0.0)),
        start_yaw=float(rospy.get_param("~start_yaw", 0.0)),
        center_x=float(rospy.get_param("~center_x", 0.0)),
        center_y=float(rospy.get_param("~center_y", 0.0)),
    )

    activation_time = None
    trajectory_id = 0 if activation_mode == "controller" else 1
    sequence = 0

    def activate_from_pose(msg: PoseStamped):
        nonlocal activation_time, curve_params, trajectory_id, sequence
        stamp = msg.header.stamp
        activation_time = (
            stamp.to_sec() if not stamp.is_zero() else rospy.Time.now().to_sec()
        )
        if use_activation_pose:
            curve_params.start_x = finite(msg.pose.position.x)
            curve_params.start_y = finite(msg.pose.position.y)
            curve_params.start_yaw = yaw_from_quaternion(msg.pose.orientation)
            if math.isfinite(msg.pose.position.z) and msg.pose.position.z > 0.0:
                curve_params.height = msg.pose.position.z
        trajectory_id += 1
        sequence = 0
        rospy.loginfo(
            "[UavReferencePublisher] Activated %s at t=%.3f start=[%.3f %.3f %.3f] yaw=%.3f",
            curve,
            activation_time,
            curve_params.start_x,
            curve_params.start_y,
            curve_params.height,
            curve_params.start_yaw,
        )

    if activation_mode == "controller":
        rospy.Subscriber(
            activation_topic, PoseStamped, activate_from_pose, queue_size=1
        )
        rospy.loginfo(
            "[UavReferencePublisher] Waiting for controller activation on %s",
            activation_topic,
        )
    elif activation_mode == "immediate":
        activation_time = (
            time_origin if time_origin > 0.0 else rospy.Time.now().to_sec()
        )
        rospy.loginfo(
            "[UavReferencePublisher] Immediate activation at t=%.3f",
            activation_time,
        )
    else:
        rospy.logwarn(
            "[UavReferencePublisher] Unknown activation_mode=%s, using immediate",
            activation_mode,
        )
        activation_time = (
            time_origin if time_origin > 0.0 else rospy.Time.now().to_sec()
        )

    bspline_pub = rospy.Publisher(
        "alg/reference_trajectory/bspline", UavBsplineTrajectory, queue_size=5
    )
    flat_pub = rospy.Publisher(
        "alg/reference_trajectory/flat", UavFlatTrajectory, queue_size=5
    )

    rate = rospy.Rate(rate_hz)
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        if activation_time is None:
            rospy.loginfo_throttle(
                1.0,
                "[UavReferencePublisher] Reference inactive; waiting for activation",
            )
            rate.sleep()
            continue

        elapsed = max(0.0, now.to_sec() - activation_time)
        sequence += 1
        count = max(degree + 2, int(math.ceil(horizon / sample_dt)) + 1)

        flat = UavFlatTrajectory()
        flat.header.stamp = now
        flat.header.frame_id = frame_id
        flat.trajectory_id = trajectory_id
        flat.sequence = sequence
        flat.start_time = now
        flat.frame_id = frame_id
        flat.valid = True

        bspline = UavBsplineTrajectory()
        bspline.header = flat.header
        bspline.trajectory_id = trajectory_id
        bspline.sequence = sequence
        bspline.start_time = now
        bspline.order = degree
        bspline.valid = True
        bspline.yaw_dt = sample_dt

        points = []
        times = []
        yaws = []
        previous_yaw = None
        for i in range(count):
            t = i * sample_dt
            p, v, a, jerk, snap, yaw = sample_curve(curve, elapsed + t, curve_params)
            if previous_yaw is not None:
                yaw = unwrap_angle_near(yaw, previous_yaw)
            previous_yaw = yaw
            point = UavFlatTrajectoryPoint()
            point.t_from_start = t
            point.position = p
            point.velocity = v
            point.acceleration = a
            point.jerk = jerk
            point.snap = snap
            point.yaw = yaw
            points.append(point)
            times.append(t)
            yaws.append(yaw)

        yaw_rates = [finite_difference(yaws, times, i) for i in range(len(points))]
        yaw_accels = [
            finite_difference(yaw_rates, times, i) for i in range(len(points))
        ]
        for point, yaw_rate, yaw_accel in zip(points, yaw_rates, yaw_accels):
            point.yaw_rate = finite(yaw_rate)
            point.yaw_accel = finite(yaw_accel)
            flat.points.append(point)
            bspline.position_control_points.append(point.position)
            bspline.yaw_control_points.append(point.yaw)

        bspline.knots = make_open_uniform_knots(
            len(bspline.position_control_points), degree, horizon
        )
        if publish_bspline:
            bspline_pub.publish(bspline)
        if publish_flat:
            flat_pub.publish(flat)
        rate.sleep()


if __name__ == "__main__":
    main()
