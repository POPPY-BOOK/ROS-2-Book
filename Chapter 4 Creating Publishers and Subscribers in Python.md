The chapter provides a comprehensive guide to creating publishers and subscribers in ROS 2 using Python, which are essential components for building flexible and modular robotic systems. Here's a summary of the key points and concepts discussed:

### Importance of Publishers and Subscribers

Publishers and subscribers are fundamental to ROS 2 communication. They enable data flow between different parts of a robotic system without direct dependencies, allowing for flexible and modular system design.

### Creating a Publisher

Here's an example of a minimal publisher in Python:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Understanding Publisher Parameters

- **Message Type**: Determines the kind of data you can send. Common types include `String`, `Int32`, `Float64`, `Bool`, `Twist`, `Point`, `Pose`, `Image`, `LaserScan`, and `PointCloud2`.
- **Topic Name**: Descriptive names that indicate what the data represents, such as `cmd_vel`, `robot_status`, and `battery_level`.
- **Queue Size**: Determines how many messages ROS 2 will buffer if subscribers can't keep up. Start with a queue size of 10 and adjust based on your needs.

### Real-World Publisher Example

A practical example of a system monitor publisher that publishes different types of data at varying rates:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
import psutil
import json

class SystemMonitor(Node):
    def __init__(self):
        super().__init__('system_monitor')
        self.cpu_pub = self.create_publisher(Float64, 'cpu_usage', 10)
        self.memory_pub = self.create_publisher(Float64, 'memory_usage', 10)
        self.status_pub = self.create_publisher(String, 'system_status', 10)
        self.cpu_timer = self.create_timer(1.0, self.publish_cpu)
        self.memory_timer = self.create_timer(2.0, self.publish_memory)
        self.status_timer = self.create_timer(5.0, self.publish_status)

    def publish_cpu(self):
        cpu_percent = psutil.cpu_percent(interval=None)
        msg = Float64()
        msg.data = cpu_percent
        self.cpu_pub.publish(msg)

    def publish_memory(self):
        memory_percent = psutil.virtual_memory().percent
        msg = Float64()
        msg.data = memory_percent
        self.memory_pub.publish(msg)

    def publish_status(self):
        status = {
            'node_name': self.get_name(),
            'uptime': self.get_clock().now().seconds_nanoseconds()[0],
            'cpu_count': psutil.cpu_count(),
            'memory_total': psutil.virtual_memory().total
        }
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SystemMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down system monitor')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Creating a Subscriber

Here's an example of a basic subscriber that receives and processes messages:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Understanding Callback Functions

Callback functions are where subscribers process incoming messages. Keep callbacks short and fast to avoid missing messages. If heavy processing is required, consider using separate threads or queues.

### Real-World Subscriber Example

A subscriber that processes robot velocity commands and adds safety checks:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class SafetyController(Node):
    def __init__(self):
        super().__init__('safety_controller')
        self.cmd_sub = self.create_subscription(
            Twist,
            'cmd_vel_raw',
            self.cmd_callback,
            10)
        self.safe_cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.declare_parameter('max_linear_speed', 2.0)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('emergency_stop', False)
        self.commands_received = 0
        self.commands_modified = 0

    def cmd_callback(self, msg):
        self.commands_received += 1
        if self.get_parameter('emergency_stop').value:
            self.get_logger().warn('Emergency stop active - blocking all commands')
            return
        safe_cmd = Twist()
        max_linear = self.get_parameter('max_linear_speed').value
        linear_speed = math.sqrt(msg.linear.x**2 + msg.linear.y**2 + msg.linear.z**2)
        if linear_speed > max_linear:
            scale = max_linear / linear_speed
            safe_cmd.linear.x = msg.linear.x * scale
            safe_cmd.linear.y = msg.linear.y * scale
            safe_cmd.linear.z = msg.linear.z * scale
            self.commands_modified += 1
            self.get_logger().warn(f'Linear velocity limited: {linear_speed:.2f} -> {max_linear:.2f}')
        else:
            safe_cmd.linear = msg.linear
        max_angular = self.get_parameter('max_angular_speed').value
        angular_speed = math.sqrt(msg.angular.x**2 + msg.angular.y**2 + msg.angular.z**2)
        if angular_speed > max_angular:
            scale = max_angular / angular_speed
            safe_cmd.angular.x = msg.angular.x * scale
            safe_cmd.angular.y = msg.angular.y * scale
            safe_cmd.angular.z = msg.angular.z * scale
            self.commands_modified += 1
            self.get_logger().warn(f'Angular velocity limited: {angular_speed:.2f} -> {max_angular:.2f}')
        else:
            safe_cmd.angular = msg.angular
        self.safe_cmd_pub.publish(safe_cmd)
        if self.commands_received % 100 == 0:
            modification_rate = (self.commands_modified / self.commands_received) * 100
            self.get_logger().info(f'Commands processed: {self.commands_received}, modified: {modification_rate:.1f}%')

def main(args=None):
    rclpy.init(args=args)
    node = SafetyController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Safety controller shutting down')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Practice Project: Temperature Monitor

A practical project that simulates real sensor data, publishes it across the ROS 2 network, and has multiple subscribers that react to the data in different ways.

#### Temperature Sensor Node

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature
from std_msgs.msg import Header
import random
import math
import time

class TemperatureSensor(Node):
    def __init__(self):
        super().__init__('temperature_sensor')
        self.publisher_ = self.create_publisher(Temperature, 'temperature/ambient', 10)
        self.timer = self.create_timer(1.0, self.publish_temperature)
        self.base_temp = 293.15
        self.time_start = time.time()

    def publish_temperature(self):
        msg = Temperature()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'sensor_frame'
        elapsed_time = time.time() - self.time_start
        daily_variation = 5.0 * math.sin(elapsed_time * 2 * math.pi / 86400)
        noise = random.uniform(-2.0, 2.0)
        msg.temperature = self.base_temp + daily_variation + noise
        msg.variance = 0.1
        self.publisher_.publish(msg)
        temp_celsius = msg.temperature - 273.15
        self.get_logger().info(f'Published temperature: {temp_celsius:.1f}°C')

def main(args=None):
    rclpy.init(args=args)
    node = TemperatureSensor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Alert Monitor Node

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature

class AlertMonitor(Node):
    def __init__(self):
        super().__init__('alert_monitor')
        self.subscription = self.create_subscription(
            Temperature,
            'temperature/ambient',
            self.temperature_callback,
            10)
        self.hot_threshold = 298.15
        self.cold_threshold = 288.15
        self.last_alert_state = None

    def temperature_callback(self, msg):
        temp_celsius = msg.temperature - 273.15
        if msg.temperature > self.hot_threshold:
            if self.last_alert_state != 'hot':
                self.get_logger().warn(f'HIGH TEMPERATURE ALERT: {temp_celsius:.1f}°C')
                self.last_alert_state = 'hot'
        elif msg.temperature < self.cold_threshold:
            if self.last_alert_state != 'cold':
                self.get_logger().warn(f'LOW TEMPERATURE ALERT: {temp_celsius:.1f}°C')
                self.last_alert_state = 'cold'
        else:
            if self.last_alert_state != 'normal':
                self.get_logger().info(f'Temperature normal: {temp_celsius:.1f}°C')
                self.last_alert_state = 'normal'

def main(args=None):
    rclpy.init(args=args)
    node = AlertMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Data Logger Node

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature
import csv
import os
from datetime import datetime

class DataLogger(Node):
    def __init__(self):
        super().__init__('data_logger')
        self.subscription = self.create_subscription(
            Temperature,
            'temperature/ambient',
            self.temperature_callback,
            10)
        self.log_file = f'temperature_log_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv'
        self.log_path = os.path.join(os.path.expanduser('~'), self.log_file)
        with open(self.log_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'temperature_celsius', 'variance'])

    def temperature_callback(self, msg):
        temp_celsius = msg.temperature - 273.15
        timestamp = datetime.fromtimestamp(
            msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        ).strftime('%Y-%m-%d %H:%M:%S')
        with open(self.log_path, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([timestamp, f'{temp_celsius:.2f}', msg.variance])

def main(args=None):
    rclpy.init(args=args)
    node = DataLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Package Configuration

Update your `setup.py` to include the new nodes:

```python
from setuptools import setup

package_name = 'temp_monitor'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Temperature monitoring system',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'temp_sensor = temp_monitor.temp_sensor:main',
            'alert_monitor = temp_monitor.alert_monitor:main',
            'data_logger = temp_monitor.data_logger:main',
        ],
    },
)
```

### Building and Running

Build the package and run each node in a separate terminal:

```bash
cd ~/ros2_ws
colcon build --packages-select temp_monitor
source install/setup.bash

# Terminal 1 (Temperature Sensor)
ros2 run temp_monitor temp_sensor

# Terminal 2 (Alert Monitor)
ros2 run temp_monitor alert_monitor

# Terminal 3 (Data Logger)
ros2 run temp_monitor data_logger
```

### Testing the System

Use the following commands to test and debug your system:

```bash
ros2 node list
ros2 topic list
ros2 topic echo /temperature/ambient
ros2 topic hz /temperature/ambient
ros2 run rqt_graph rqt_graph
```

By following these guidelines and utilizing the provided code snippets, you can effectively create, manage, and debug publishers and subscribers in ROS 2 for your robotic applications.

### Adding the C++ Version

To demonstrate how the same system can use mixed languages, let's create a C++ version of the temperature sensor.

#### Temperature Sensor in C++

Create the file `~/ros2_ws/src/temp_monitor/src/temp_sensor_cpp.cpp`:

```cpp
#include <chrono>
#include <memory>
#include <random>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/temperature.hpp"

using namespace std::chrono_literals;

class TemperatureSensorCpp : public rclcpp::Node
{
public:
    TemperatureSensorCpp() : Node("temperature_sensor_cpp"), gen_(rd_())
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::Temperature>("temperature/ambient", 10);
        timer_ = this->create_wall_timer(1s, std::bind(&TemperatureSensorCpp::publish_temperature, this));

        base_temp_ = 293.15;  // 20°C in Kelvin
        start_time_ = std::chrono::steady_clock::now();

        RCLCPP_INFO(this->get_logger(), "C++ Temperature sensor started");
    }

private:
    void publish_temperature()
    {
        auto msg = sensor_msgs::msg::Temperature();

        // Set header
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "sensor_frame";

        // Simulate temperature with daily cycle and noise
        auto elapsed = std::chrono::steady_clock::now() - start_time_;
        double elapsed_seconds = std::chrono::duration<double>(elapsed).count();

        double daily_variation = 5.0 * std::sin(elapsed_seconds * 2 * M_PI / 86400);
        std::uniform_real_distribution<> noise_dist(-2.0, 2.0);
        double noise = noise_dist(gen_);

        msg.temperature = base_temp_ + daily_variation + noise;
        msg.variance = 0.1;

        publisher_->publish(msg);

        double temp_celsius = msg.temperature - 273.15;
        RCLCPP_INFO(this->get_logger(), "Published temperature: %.1f°C", temp_celsius);
    }

    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    double base_temp_;
    std::chrono::steady_clock::time_point start_time_;
    std::random_device rd_;
    std::mt19937 gen_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TemperatureSensorCpp>());
    rclcpp::shutdown();
    return 0;
}
```

### CMakeLists.txt Configuration

Update your `CMakeLists.txt` to include the C++ executable:

```cmake
cmake_minimum_required(VERSION 3.8)
project(temp_monitor)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# C++ executable
add_executable(temp_sensor_cpp src/temp_sensor_cpp.cpp)
ament_target_dependencies(temp_sensor_cpp rclcpp sensor_msgs)

# Install executables
install(TARGETS
  temp_sensor_cpp
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
```

### Testing the System

To test the system, you can use the following commands:

```bash
# Build the package
cd ~/ros2_ws
colcon build --packages-select temp_monitor
source install/setup.bash

# Run the C++ temperature sensor
ros2 run temp_monitor temp_sensor_cpp

# Run the Python alert monitor and data logger in separate terminals
ros2 run temp_monitor alert_monitor
ros2 run temp_monitor data_logger
```

### Debugging Tips

When debugging your ROS 2 system, consider the following tips:

- **Check Node List**: Ensure all nodes are running with `ros2 node list`.
- **Echo Topics**: Use `ros2 topic echo /topic_name` to verify messages are being published.
- **Topic Info**: Use `ros2 topic info /topic_name` to check publishers and subscribers.
- **Visualize with rqt_graph**: Run `ros2 run rqt_graph rqt_graph` to visualize the node graph.

By following these guidelines and utilizing the provided code snippets, you can effectively create, manage, and debug publishers and subscribers in ROS 2 for your robotic applications, whether in Python, C++, or a mix of both.