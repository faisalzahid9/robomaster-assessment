#!/usr/bin/env python3
"""Compare RoboMaster RNDIS and AP telemetry timing and ping latency."""

from __future__ import annotations

import argparse
import csv
import re
import statistics
from pathlib import Path

import matplotlib.pyplot as plt


PING_RE = re.compile(r"icmp_seq=(\d+).*?time([=<])([0-9.]+)\s*ms")


def telemetry_intervals(path: Path) -> list[float]:
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    elapsed = [float(row["elapsed_s"]) for row in rows if row.get("elapsed_s")]
    if len(elapsed) < 2:
        raise ValueError(f"Not enough telemetry rows in {path}")
    return [1000.0 * (b - a) for a, b in zip(elapsed, elapsed[1:])]


def ping_samples(path: Path) -> tuple[list[int], list[float]]:
    sequence: list[int] = []
    latency: list[float] = []
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        match = PING_RE.search(line)
        if match:
            sequence.append(int(match.group(1)))
            value = float(match.group(3))
            latency.append(value if match.group(2) == "=" else value / 2.0)
    if not latency:
        raise ValueError(f"No ping samples found in {path}")
    return sequence, latency


def describe(label: str, values: list[float], unit: str) -> None:
    print(
        f"{label}: n={len(values)}, mean={statistics.fmean(values):.4f} {unit}, "
        f"min={min(values):.4f}, max={max(values):.4f}, "
        f"stdev={statistics.stdev(values):.4f}"
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--logs", type=Path, default=Path("logs"))
    parser.add_argument("--output", type=Path, default=Path("logs/network_comparison.png"))
    args = parser.parse_args()

    rndis_dt = telemetry_intervals(args.logs / "rndis_telemetry.csv")
    ap_dt = telemetry_intervals(args.logs / "ap_telemetry.csv")
    rndis_seq, rndis_ping = ping_samples(args.logs / "ping_rndis.txt")
    ap_seq, ap_ping = ping_samples(args.logs / "ping_ap.txt")

    describe("RNDIS telemetry interval", rndis_dt, "ms")
    describe("AP telemetry interval", ap_dt, "ms")
    describe("RNDIS ping", rndis_ping, "ms")
    describe("AP ping", ap_ping, "ms")
    print(f"AP/RNDIS mean-latency ratio: {statistics.fmean(ap_ping) / statistics.fmean(rndis_ping):.2f}x")

    fig, axes = plt.subplots(2, 1, figsize=(11, 8), constrained_layout=True)

    axes[0].plot(range(1, len(rndis_dt) + 1), rndis_dt, label="USB/RNDIS", color="#1565c0")
    axes[0].plot(range(1, len(ap_dt) + 1), ap_dt, label="Wi-Fi/AP", color="#ef6c00", alpha=0.85)
    axes[0].axhline(100.0, color="#2e7d32", linestyle="--", linewidth=1.2, label="10 Hz target (100 ms)")
    axes[0].set(title="Telemetry Sampling Interval", xlabel="Sample interval", ylabel="Delta t (ms)")
    axes[0].grid(alpha=0.25)
    axes[0].legend()

    axes[1].plot(rndis_seq, rndis_ping, label="USB/RNDIS", color="#1565c0", marker=".", markersize=3)
    axes[1].plot(ap_seq, ap_ping, label="Wi-Fi/AP", color="#ef6c00", marker=".", markersize=3)
    axes[1].set(title="ICMP Round-Trip Latency", xlabel="ICMP sequence number", ylabel="Latency (ms)")
    axes[1].grid(alpha=0.25)
    axes[1].legend()

    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=200)
    print(f"Saved plot: {args.output.resolve()}")


if __name__ == "__main__":
    main()
