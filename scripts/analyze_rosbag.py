#!/usr/bin/env python3
"""Analyze RoboMaster ROS 2 odometry and command tracking from an MCAP bag."""

from __future__ import annotations

import argparse
import bisect
import math
import statistics
from pathlib import Path

import matplotlib.pyplot as plt
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def wrap_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def open_reader(uri: Path) -> rosbag2_py.SequentialReader:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(uri), storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )
    return reader


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path, help="ROS 2 bag directory")
    parser.add_argument("--output-dir", type=Path, default=Path("results"))
    args = parser.parse_args()

    reader = open_reader(args.bag.expanduser().resolve())
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}

    required = {"/odom", "/cmd_vel", "/joint_states"}
    missing = required - topic_types.keys()
    if missing:
        raise RuntimeError(f"Bag is missing required topics: {sorted(missing)}")

    odom: list[tuple[float, float, float, float, float, float, float]] = []
    cmd: list[tuple[float, float, float, float]] = []
    counts = {name: 0 for name in required}

    while reader.has_next():
        topic, raw, timestamp_ns = reader.read_next()
        if topic not in required:
            continue
        counts[topic] += 1
        message = deserialize_message(raw, get_message(topic_types[topic]))
        time_s = timestamp_ns * 1e-9
        if topic == "/odom":
            pose = message.pose.pose
            twist = message.twist.twist
            odom.append(
                (
                    time_s,
                    pose.position.x,
                    pose.position.y,
                    yaw_from_quaternion(pose.orientation),
                    twist.linear.x,
                    twist.linear.y,
                    twist.angular.z,
                )
            )
        elif topic == "/cmd_vel":
            cmd.append((time_s, message.linear.x, message.linear.y, message.angular.z))

    if len(odom) < 2 or not cmd:
        raise RuntimeError(f"Insufficient data: counts={counts}")

    odom.sort()
    cmd.sort()
    start_t = odom[0][0]
    times = [row[0] - start_t for row in odom]

    x0, y0, yaw0 = odom[0][1], odom[0][2], odom[0][3]
    c0, s0 = math.cos(yaw0), math.sin(yaw0)
    measured_x: list[float] = []
    measured_y: list[float] = []
    measured_yaw: list[float] = []
    measured_vx: list[float] = []
    measured_vy: list[float] = []
    measured_wz: list[float] = []
    for _, x, y, yaw, vx, vy, wz in odom:
        dx, dy = x - x0, y - y0
        measured_x.append(c0 * dx + s0 * dy)
        measured_y.append(-s0 * dx + c0 * dy)
        measured_yaw.append(wrap_angle(yaw - yaw0))
        measured_vx.append(vx)
        measured_vy.append(vy)
        measured_wz.append(wz)

    cmd_times_abs = [row[0] for row in cmd]

    def command_at(time_abs: float) -> tuple[float, float, float]:
        index = bisect.bisect_right(cmd_times_abs, time_abs) - 1
        if index < 0:
            return 0.0, 0.0, 0.0
        return cmd[index][1], cmd[index][2], cmd[index][3]

    commanded_x = [0.0]
    commanded_y = [0.0]
    commanded_yaw = [0.0]
    command_vx: list[float] = []
    command_vy: list[float] = []
    command_wz: list[float] = []

    for i, row in enumerate(odom):
        vx, vy, wz = command_at(row[0])
        command_vx.append(vx)
        command_vy.append(vy)
        command_wz.append(wz)
        if i == 0:
            continue
        dt = row[0] - odom[i - 1][0]
        previous_yaw = commanded_yaw[-1]
        commanded_x.append(
            commanded_x[-1]
            + (vx * math.cos(previous_yaw) - vy * math.sin(previous_yaw)) * dt
        )
        commanded_y.append(
            commanded_y[-1]
            + (vx * math.sin(previous_yaw) + vy * math.cos(previous_yaw)) * dt
        )
        commanded_yaw.append(previous_yaw + wz * dt)

    position_error = [
        math.hypot(mx - cx, my - cy)
        for mx, my, cx, cy in zip(measured_x, measured_y, commanded_x, commanded_y)
    ]
    yaw_error = [
        abs(wrap_angle(myaw - cyaw))
        for myaw, cyaw in zip(measured_yaw, commanded_yaw)
    ]

    intervals = [1000.0 * (b - a) for a, b in zip(times, times[1:])]
    duration = times[-1]
    output = args.output_dir.expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)

    fig, axis = plt.subplots(figsize=(8, 7), constrained_layout=True)
    axis.plot(commanded_x, commanded_y, color="#ef6c00", label="Integrated command")
    axis.plot(measured_x, measured_y, color="#1565c0", linewidth=2, label="Measured odometry")
    axis.scatter([0.0], [0.0], color="#2e7d32", s=70, marker="o", label="Start")
    axis.scatter([measured_x[-1]], [measured_y[-1]], color="#c62828", s=80, marker="x", label="Measured end")
    axis.set(title="RoboMaster Commanded and Measured Trajectory", xlabel="x from start (m)", ylabel="y from start (m)")
    axis.axis("equal")
    axis.grid(alpha=0.3)
    axis.legend()
    fig.savefig(output / "ros2_trajectory.png", dpi=200)
    plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(10, 8), constrained_layout=True)
    axes[0].plot(times, position_error, color="#6a1b9a")
    axes[0].set(title="Command-to-Odometry Position Drift", ylabel="Position error (m)")
    axes[0].grid(alpha=0.3)
    axes[1].plot(times, [math.degrees(v) for v in yaw_error], color="#ad1457")
    axes[1].set(xlabel="Elapsed time (s)", ylabel="Absolute yaw error (deg)")
    axes[1].grid(alpha=0.3)
    fig.savefig(output / "ros2_tracking_drift.png", dpi=200)
    plt.close(fig)

    summary = (
        "RoboMaster ROS 2 Bag Analysis\n"
        "==============================\n"
        f"Bag: {args.bag.expanduser().resolve()}\n"
        f"Duration: {duration:.3f} s\n"
        f"Message counts: {counts}\n"
        f"Mean odometry interval: {statistics.fmean(intervals):.3f} ms\n"
        f"Mean odometry rate: {1000.0 / statistics.fmean(intervals):.3f} Hz\n"
        f"Final position tracking error: {position_error[-1]:.4f} m\n"
        f"Maximum position tracking error: {max(position_error):.4f} m\n"
        f"Final absolute yaw error: {math.degrees(yaw_error[-1]):.3f} deg\n"
        f"Maximum absolute yaw error: {math.degrees(max(yaw_error)):.3f} deg\n"
    )
    (output / "ros2_analysis_summary.txt").write_text(summary, encoding="utf-8")
    print(summary)
    print(f"Saved: {output / 'ros2_trajectory.png'}")
    print(f"Saved: {output / 'ros2_tracking_drift.png'}")
    print(f"Saved: {output / 'ros2_analysis_summary.txt'}")


if __name__ == "__main__":
    main()
