# WIT-JY901b ROS2 Integration

This repository contains ROS2 nodes and example scripts for interfacing with the WIT-Motion JY901b IMU sensor. The sensor data is read via a serial port and published to ROS2 topics for further processing.

## Table of Contents

- [Installation](#installation)
- [Usage](#usage)
- [Nodes](#nodes)
  - [Serial to IMU Node](#serial-to-imu-node)
  - [IMU Listener Node](#imu-listener-node)
- [Examples](#examples)
  - [Basic Reading](#basic-reading)
  - [3D Box Visualization](#3d-box-visualization)
- [Configuration](#configuration)
- [License](#license)

https://category.yahboom.net/products/imu?_pos=1&_sid=f9fcdebd5&_ss=r

## Installation

1. Clone the repository:
    ```bash
    git clone https://github.com/yourusername/WIT-JY901b.git
    cd WIT-JY901b
    ```

2. Install dependencies:
    ```bash
    pip install pyserial pygame PyOpenGL
    ```

3. Build the ROS2 package:
    ```bash
    colcon build
    source install/setup.bash
    ```

## Usage

### Launch the ROS2 Node

To launch the ROS2 node that reads data from the JY901b sensor and publishes it to the `/imu` topic, use the following command:

```bash
ros2 launch witmotion_ros wt901.launch.py