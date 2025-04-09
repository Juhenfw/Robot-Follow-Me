# Robot Follower System

A robust system for a robot to follow a target using UWB tracking and LIDAR obstacle avoidance.

## Overview

This system enables a robot to autonomously follow a target (like a person) while safely avoiding obstacles. It uses Ultra-Wideband (UWB) for precise target positioning and LIDAR for obstacle detection.

## Features

- **Target Tracking**: Uses UWB technology to accurately track the target's position
- **Obstacle Avoidance**: Uses LIDAR to detect and avoid obstacles
- **Safety Monitoring**: Comprehensive safety checks to prevent collisions
- **Visualization**: Optional real-time data visualization
- **Simulation Mode**: Fully functional simulation mode for testing
- **Configurable**: Easily configurable parameters via config files

## System Requirements

- Python 3.6+
- Required Python packages:
  - numpy
  - psutil
  - matplotlib (optional, for visualization)

## Installation

1. Clone this repository:
      git clone https://github.com/yourusername/robot-follower.git
   cd robot-follower
