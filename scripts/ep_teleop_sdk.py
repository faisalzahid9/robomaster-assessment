#!/usr/bin/env python3
"""RoboMaster EP keyboard teleoperation and 10 Hz telemetry logger.

Controls
--------
W / S : forward / backward
A / D : strafe left / right
Q / E : rotate counterclockwise / clockwise
Space : immediate stop (program keeps running)
Esc   : stop and exit

The RoboMaster SDK expects chassis yaw speed ``z`` in degrees/second.  This
program accepts the yaw-speed option in radians/second and converts it before
sending commands.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import pygame
from robomaster import robot


@dataclass
class Telemetry:
    """Latest values received by the asynchronous SDK callbacks."""

    battery_pct: Optional[float] = None
    yaw_deg: Optional[float] = None
    pitch_deg: Optional[float] = None
    roll_deg: Optional[float] = None
    wheel_rpm: list[Optional[float]] = field(
        default_factory=lambda: [None, None, None, None]
    )
    lock: threading.Lock = field(default_factory=threading.Lock, repr=False)

    def battery_callback(self, percent) -> None:
        with self.lock:
            self.battery_pct = percent

    def attitude_callback(self, attitude) -> None:
        yaw, pitch, roll = attitude
        with self.lock:
            self.yaw_deg = yaw
            self.pitch_deg = pitch
            self.roll_deg = roll

    def esc_callback(self, esc_data) -> None:
        speed, _angle, _sdk_timestamp, _state = esc_data
        with self.lock:
            self.wheel_rpm = list(speed[:4])

    def snapshot(self):
        with self.lock:
            return (
                self.battery_pct,
                self.yaw_deg,
                self.pitch_deg,
                self.roll_deg,
                *self.wheel_rpm,
            )


def keyboard_command(linear_speed: float, yaw_speed: float):
    """Read held keys from the focused Pygame window."""

    pressed = pygame.key.get_pressed()
    if pressed[pygame.K_SPACE]:
        return 0.0, 0.0, 0.0

    vx = linear_speed * (pressed[pygame.K_w] - pressed[pygame.K_s])
    vy = linear_speed * (pressed[pygame.K_a] - pressed[pygame.K_d])
    wz = yaw_speed * (pressed[pygame.K_q] - pressed[pygame.K_e])
    return float(vx), float(vy), float(wz)


def draw_status(screen, font, small_font, telemetry, command, output, remaining):
    """Draw the focused teleoperation window."""

    screen.fill((18, 34, 56))
    white = (245, 248, 252)
    aqua = (94, 234, 212)
    gold = (245, 187, 72)
    muted = (178, 197, 218)

    lines = [
        ("RoboMaster EP Teleoperation", font, aqua),
        ("Keep this window focused", small_font, gold),
        ("W/S  forward/backward     A/D  strafe left/right", small_font, white),
        ("Q/E  rotate left/right    SPACE  stop", small_font, white),
        ("ESC  stop, save CSV, and exit", small_font, white),
        (
            f"Command SI: vx={command[0]:+.2f}  vy={command[1]:+.2f}  "
            f"wz={command[2]:+.2f}",
            small_font,
            aqua,
        ),
    ]

    data = telemetry.snapshot()
    lines.append(
        (
            f"Battery: {data[0]}%     Yaw: {data[1]} deg",
            small_font,
            muted,
        )
    )
    if remaining is not None:
        lines.append((f"Time remaining: {max(remaining, 0):.1f} s", small_font, gold))
    lines.append((f"CSV: {output}", small_font, muted))

    y = 28
    for text_value, text_font, color in lines:
        surface = text_font.render(text_value, True, color)
        screen.blit(surface, (28, y))
        y += surface.get_height() + 16

    pygame.display.flip()


def telemetry_logger(
    output: Path,
    telemetry: Telemetry,
    start_time: float,
    shutdown: threading.Event,
    rate_hz: float = 10.0,
) -> None:
    """Write one synchronized snapshot of the latest telemetry every 0.1 s."""

    period = 1.0 / rate_hz
    next_sample = time.monotonic()

    with output.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "host_unix_time_s",
                "elapsed_s",
                "battery_pct",
                "yaw_deg",
                "pitch_deg",
                "roll_deg",
                "wheel_1_rpm",
                "wheel_2_rpm",
                "wheel_3_rpm",
                "wheel_4_rpm",
            ]
        )

        while not shutdown.is_set():
            now = time.monotonic()
            remaining = next_sample - now
            if remaining > 0:
                shutdown.wait(remaining)
                continue

            writer.writerow(
                [
                    f"{time.time():.6f}",
                    f"{now - start_time:.6f}",
                    *telemetry.snapshot(),
                ]
            )
            stream.flush()

            next_sample += period
            if next_sample < now - period:
                # If the process was delayed, resume from the current time rather
                # than emitting a burst of old samples.
                next_sample = now + period


def parse_args():
    parser = argparse.ArgumentParser(
        description="RoboMaster EP keyboard teleoperation with CSV telemetry"
    )
    parser.add_argument(
        "--conn-type",
        choices=("rndis", "ap"),
        default="rndis",
        help="Robot connection type (default: rndis)",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("telemetry.csv"),
        help="CSV output path (default: telemetry.csv)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Automatically stop after this many seconds; 0 means no limit",
    )
    parser.add_argument(
        "--linear-speed",
        type=float,
        default=0.30,
        help="Forward/lateral speed in m/s (default: 0.30)",
    )
    parser.add_argument(
        "--yaw-speed",
        type=float,
        default=0.60,
        help="Yaw speed in rad/s (default: 0.60)",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.duration < 0:
        raise SystemExit("--duration cannot be negative")
    if not 0 < args.linear_speed <= 1.0:
        raise SystemExit("For safety, --linear-speed must be in (0, 1.0] m/s")
    if not 0 < args.yaw_speed <= 2.0:
        raise SystemExit("For safety, --yaw-speed must be in (0, 2.0] rad/s")

    args.output = args.output.expanduser().resolve()
    args.output.parent.mkdir(parents=True, exist_ok=True)

    ep_robot = robot.Robot()
    telemetry = Telemetry()
    shutdown = threading.Event()
    logger_thread = None
    subscribed_battery = False
    subscribed_attitude = False
    subscribed_esc = False
    return_code = 0
    pygame_started = False

    try:
        print(f"Connecting with conn_type={args.conn_type!r} ...")
        ep_robot.initialize(conn_type=args.conn_type)
        chassis = ep_robot.chassis
        battery = ep_robot.battery

        print("Connected.")
        print(f"Serial number: {ep_robot.get_sn()}")
        print(f"Firmware: {ep_robot.get_version()}")

        subscribed_battery = battery.sub_battery_info(
            freq=10, callback=telemetry.battery_callback
        )
        subscribed_attitude = chassis.sub_attitude(
            freq=10, callback=telemetry.attitude_callback
        )
        subscribed_esc = chassis.sub_esc(freq=10, callback=telemetry.esc_callback)

        if not all((subscribed_battery, subscribed_attitude, subscribed_esc)):
            raise RuntimeError("One or more telemetry subscriptions failed")

        start_time = time.monotonic()
        logger_thread = threading.Thread(
            target=telemetry_logger,
            args=(args.output, telemetry, start_time, shutdown),
            daemon=True,
        )
        logger_thread.start()

        pygame.init()
        pygame_started = True
        screen = pygame.display.set_mode((760, 390))
        pygame.display.set_caption("RoboMaster EP Teleoperation")
        font = pygame.font.Font(None, 38)
        small_font = pygame.font.Font(None, 27)
        clock = pygame.time.Clock()

        print()
        print("A control window has opened. Keep that window focused.")
        print("W/S: forward/backward   A/D: strafe left/right")
        print("Q/E: rotate left/right  Space: STOP  Esc: stop and exit")
        print(f"Logging telemetry at 10 Hz to: {args.output}")
        if args.duration:
            print(f"Automatic stop after {args.duration:.1f} seconds")

        running = True

        while running:
            now = time.monotonic()
            if args.duration and now - start_time >= args.duration:
                print("Duration complete.")
                break

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                    running = False

            vx, vy, wz = keyboard_command(args.linear_speed, args.yaw_speed)

            # The SDK uses degrees/s for z.
            chassis.drive_speed(
                x=vx,
                y=vy,
                z=math.degrees(wz),
                timeout=0.15,
            )

            remaining = None
            if args.duration:
                remaining = args.duration - (now - start_time)
            draw_status(
                screen,
                font,
                small_font,
                telemetry,
                (vx, vy, wz),
                args.output,
                remaining,
            )
            clock.tick(50)

    except KeyboardInterrupt:
        print("\nCtrl-C received.")
        return_code = 0
    except Exception as exc:
        print(f"\nERROR: {exc}", file=sys.stderr)
        return_code = 1
    else:
        return_code = 0
    finally:
        print("\nStopping chassis...")
        shutdown.set()

        try:
            ep_robot.chassis.drive_speed(x=0.0, y=0.0, z=0.0)
            time.sleep(0.2)
        except Exception:
            pass

        if pygame_started:
            pygame.quit()
        if logger_thread is not None:
            logger_thread.join(timeout=2.0)

        try:
            if subscribed_battery:
                ep_robot.battery.unsub_battery_info()
            if subscribed_attitude:
                ep_robot.chassis.unsub_attitude()
            if subscribed_esc:
                ep_robot.chassis.unsub_esc()
        except Exception:
            pass

        try:
            ep_robot.close()
        except Exception:
            pass

        print(f"Telemetry saved to {args.output}")

    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
