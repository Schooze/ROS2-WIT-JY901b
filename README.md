# ROS2 IMU WIT-JY901b Integration

This repository provides ROS 2 nodes and example scripts for integrating the WIT-Motion JY901b IMU sensor. The sensor data is read via a serial port and published to ROS 2 topics, enabling real-time orientation tracking, motion analysis, and 3D visualization.

📌 Key Features:  
✅ Serial Communication – Reads IMU data via UART/USB  
✅ ROS 2 Node Integration – Publishes IMU data in real-time  
✅ 3D Visualization – Interactive display using PyOpenGL  
✅ Example Scripts – Ready-to-run Python scripts for quick testing  

## 📦 Product Info
  
[📍 **Product Link**](https://category.yahboom.net/products/imu?_pos=1&_sid=f9fcdebd5&_ss=r)  
[📜 **Datasheet**](https://github.com/Schooze/ROS2-WIT-JY901b/blob/main/Witmotion%20JY901%20Datasheet.pdf)



## 📖 Table of Contents

- [Nodes](#nodes)
  - [Serial to IMU Node](#serial-to-imu-node)
  - [IMU Listener Node](#imu-listener-node)
- [Program Examples](#examples) 🛠️
  - [Basic Reading](https://github.com/Schooze/ROS2-WIT-JY901b/blob/main/example/ex1_basic_reading.py) 📊
  - [3D Box Visualization](https://github.com/Schooze/ROS2-WIT-JY901b/blob/main/example/ex2_box.py) 📦
  - [ROS2 Basic Reading](https://github.com/Schooze/ROS2-WIT-JY901b/blob/main/example/ex3_ros_basic_reading.py) 🤖
- [Configuration](#configuration) ⚙️
- [How to Get Started](#how-to-get-started) 🚀
- [License](#license) 📜


## 🚀 Getting Started

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

## License

This project is licensed under the **MIT License** – you're free to use, modify, and distribute it. See the [LICENSE](https://github.com/Schooze/ROS2-WIT-JY901b/blob/main/LICENSE) file for details.

--- 
💡 Need Help? Open an issue or reach out! 🚀

Let me know if you'd like any modifications! 🚀🔥