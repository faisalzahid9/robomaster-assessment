# RoboMaster Assessment

This repository contains my RoboMaster EP assessment work using the
RoboMaster Python SDK and ROS 2 Jazzy.


Faisal Zahid

## System

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic
- DJI RoboMaster EP
- Python 3.12
- RoboMaster Python SDK

## Repository Contents

- `scripts/ep_teleop_sdk.py`: keyboard control and telemetry logger
- `scripts/analyze_rosbag.py`: ROS 2 bag analysis
- `scripts/compare_network_logs.py`: RNDIS and Wi-Fi AP comparison
- `logs/`: telemetry CSV files and ping measurements
- `results/`: generated plots, TF tree, and analysis summary
- `bags/`: ROS 2 bag metadata
- `report/`: final assessment report

## Main Results

- RNDIS mean ping: 0.442 ms
- Wi-Fi AP mean ping: 14.069 ms
- Packet loss: 0% for both connections
- RNDIS and AP telemetry rate: approximately 10 Hz
- ROS 2 odometry rate: 9.723 Hz
- Final position discrepancy: 0.058 m
- Final yaw discrepancy: 13.44 degrees

## Running the SDK Controller

Activate the virtual environment:

```bash
source ~/rm_venv/bin/activate
