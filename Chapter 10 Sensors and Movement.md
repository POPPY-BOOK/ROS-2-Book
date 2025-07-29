Sure, here are the coding lines extracted from the text:

Python Version for Laser Scanner:
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import math

class LaserScanProcessor(Node):
    def __init__(self):
        super().__init__('laser_scan_processor')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.get_logger().info('Laser scan processor started')

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        valid_ranges = ranges[np.isfinite(ranges)]

        if len(valid_ranges) == 0:
            self.get_logger().warn('No valid laser readings!')
            return

        min_distance = np.min(valid_ranges)
        min_index = np.argmin(ranges)
        angle = msg.angle_min + min_index * msg.angle_increment

        self.get_logger().info(
            f'Closest obstacle: {min_distance:.2f}m at {math.degrees(angle):.1f}°')

        front_angle_range = math.radians(30)
        front_indices = self.get_front_indices(msg, front_angle_range)

        if len(front_indices) > 0:
            front_ranges = ranges[front_indices]
            front_valid = front_ranges[np.isfinite(front_ranges)]

            if len(front_valid) > 0 and np.min(front_valid) < 1.0:
                self.get_logger().warn('Obstacle detected in front!')

    def get_front_indices(self, msg, angle_range):
        total_readings = len(msg.ranges)
        readings_per_radian = total_readings / (msg.angle_max - msg.angle_min)
        center_angle = 0.0
        center_index = int((center_angle - msg.angle_min) * readings_per_radian)
        range_indices = int(angle_range * readings_per_radian)

        start_idx = max(0, center_index - range_indices)
        end_idx = min(total_readings, center_index + range_indices)

        return list(range(start_idx, end_idx))

def main(args=None):
    rclpy.init(args=args)
    laser_processor = LaserScanProcessor()
    try:
        rclpy.spin(laser_processor)
    except KeyboardInterrupt:
        pass
    finally:
        laser_processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

C++ Version for Laser Scanner:
```cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cmath>
#include <algorithm>
#include <numeric>

class LaserScanProcessor : public rclcpp::Node
{
public:
    LaserScanProcessor() : Node("laser_scan_processor")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&LaserScanProcessor::scan_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Laser scan processor started");
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        std::vector<float> valid_ranges;
        for (const auto& range : msg->ranges) {
            if (std::isfinite(range)) {
                valid_ranges.push_back(range);
            }
        }

        if (valid_ranges.empty()) {
            RCLCPP_WARN(this->get_logger(), "No valid laser readings!");
            return;
        }

        auto min_it = std::min_element(msg->ranges.begin(), msg->ranges.end());
        float min_distance = *min_it;
        int min_index = std::distance(msg->ranges.begin(), min_it);

        float angle = msg->angle_min + min_index * msg->angle_increment;

        RCLCPP_INFO(this->get_logger(),
            "Closest obstacle: %.2fm at %.1f°",
            min_distance, angle * 180.0 / M_PI);

        check_front_obstacles(msg);
    }

    void check_front_obstacles(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        float front_angle_range = M_PI / 6;
        int total_readings = msg->ranges.size();
        float readings_per_radian = total_readings / (msg->angle_max - msg->angle_min);

        int center_index = (0.0 - msg->angle_min) * readings_per_radian;
        int range_indices = front_angle_range * readings_per_radian;

        int start_idx = std::max(0, center_index - range_indices);
        int end_idx = std::min(total_readings, center_index + range_indices);

        for (int i = start_idx; i < end_idx; ++i) {
            if (std::isfinite(msg->ranges[i]) && msg->ranges[i] < 1.0) {
                RCLCPP_WARN(this->get_logger(), "Obstacle detected in front!");
                break;
            }
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserScanProcessor>());
    rclcpp::shutdown();
    return 0;
}
```

Python Version for Camera:
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10)
        self.publisher = self.create_publisher(
            Image,
            '/processed_image',
            10)
        self.get_logger().info('Image processor started')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            return

        processed_image = self.process_image(cv_image)

        try:
            processed_msg = self.bridge.cv2_to_imgmsg(processed_image, encoding='bgr8')
            processed_msg.header = msg.header
            self.publisher.publish(processed_msg)
        except Exception as e:
            self.get_logger().error(f'CV Bridge error: {e}')

    def process_image(self, cv_image):
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)
        edges_bgr = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
        return edges_bgr

def main(args=None):
    rclpy.init(args=args)
    image_processor = ImageProcessor()
    try:
        rclpy.spin(image_processor)
    except KeyboardInterrupt:
        pass
    finally:
        image_processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

C++ Version for Camera:
```cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ImageProcessor : public rclcpp::Node
{
public:
    ImageProcessor() : Node("image_processor")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", 10,
            std::bind(&ImageProcessor::image_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/processed_image", 10);

        RCLCPP_INFO(this->get_logger(), "Image processor started");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImagePtr cv_ptr;

        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "CV Bridge error: %s", e.what());
            return;
        }

        cv::Mat processed_image = process_image(cv_ptr->image);

        sensor_msgs::msg::Image::SharedPtr output_msg =
            cv_bridge::CvImage(msg->header, "bgr8", processed_image).toImageMsg();

        publisher_->publish(*output_msg);
    }

    cv::Mat process_image(const cv::Mat& input)
    {
        cv::Mat gray, blurred, edges, edges_bgr;

        cv::cvtColor(input, gray, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 0);
        cv::Canny(blurred, edges, 50, 150);
        cv::cvtColor(edges, edges_bgr, cv::COLOR_GRAY2BGR);

        return edges_bgr;
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageProcessor>());
    rclcpp::shutdown();
    return 0;
}
```

Python Version for Basic Movement Control:
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class BasicMovementController(Node):
    def __init__(self):
        super().__init__('basic_movement_controller')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.movement_loop)
        self.linear_speed = 0.5
        self.angular_speed = 1.0
        self.start_time = time.time()
        self.phase = 'forward'
        self.get_logger().info('Movement controller started')

    def movement_loop(self):
        elapsed_time = time.time() - self.start_time
        cmd = Twist()

        if self.phase == 'forward':
            cmd.linear.x = self.linear_speed
            cmd.angular.z = 0.0

            if elapsed_time > 3.0:
                self.phase = 'turn'
                self.start_time = time.time()

        elif self.phase == 'turn':
            cmd.linear.x = 0.0
            cmd.angular.z = self.angular_speed

            if elapsed_time > 1.57:
                self.phase = 'forward'
                self.start_time = time.time()

        self.cmd_pub.publish(cmd)
        self.get_logger().info(f'Phase: {self.phase}, Linear: {cmd.linear.x:.2f}, Angular: {cmd.angular.z:.2f}')

def main(args=None):
    rclpy.init(args=args)
    controller = BasicMovementController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        stop_cmd = Twist()
        controller.cmd_pub.publish(stop_cmd)
        controller.get_logger().info('Stopping robot...')
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Python Version for Keyboard Teleoperation:
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.linear_speed = 0.5
        self.angular_speed = 1.0
        self.speed_increment = 0.1
        self.old_settings = termios.tcgetattr(sys.stdin)
        self.print_instructions()

    def print_instructions(self):
        print("""
        Keyboard Teleoperation
        ----------------------
        w/x : increase/decrease linear speed
        a/d : increase/decrease angular speed
        i/k : move forward/backward
        j/l : turn left/right
        space : stop
        q : quit

        Current speeds:
        Linear: {:.2f} m/s
        Angular: {:.2f} rad/s
        """.format(self.linear_speed, self.angular_speed))

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        return key

    def run(self):
        try:
            while True:
                key = self.get_key()
                cmd = Twist()

                if key == 'w':
                    self.linear_speed += self.speed_increment
                elif key == 'x':
                    self.linear_speed -= self.speed_increment
                elif key == 'a':
                    self.angular_speed += self.speed_increment
                elif key == 'd':
                    self.angular_speed -= self.speed_increment
                elif key == 'i':
                    cmd.linear.x = self.linear_speed
                elif key == 'k':
                    cmd.linear.x = -self.linear_speed
                elif key == 'j':
                    cmd.angular.z = self.angular_speed
                elif key == 'l':
                    cmd.angular.z = -self.angular_speed
                elif key == ' ':
                    pass
                elif key == 'q':
                    break
                else:
                    print(f"Unknown key: {key}")
                    continue

                self.cmd_pub.publish(cmd)

                if key in ['w', 'x', 'a', 'd']:
                    print(f"Linear: {self.linear_speed:.2f}, Angular: {self.angular_speed:.2f}")
                else:
                    print(f"Command - Linear: {cmd.linear.x:.2f}, Angular: {cmd.angular.z:.2f}")

        except KeyboardInterrupt:
            pass
        finally:
            stop_cmd = Twist()
            self.cmd_pub.publish(stop_cmd)
            print("Stopping robot...")

def main(args=None):
    rclpy.init(args=args)
    teleop = KeyboardTeleop()
    teleop.run()
    teleop.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Python Version for Odometry Tracking:
```python
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math

class OdometryTracker(Node):
    def __init__(self):
        super().__init__('odometry_tracker')
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.initial_position = None
        self.target_distance = 2.0
        self.get_logger().info('Odometry tracker started')

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        yaw = self.quaternion_to_euler(orientation.x, orientation.y, orientation.z, orientation.w)

        if self.initial_position is None:
            self.initial_position = (x, y)
            self.get_logger().info(f'Initial position: ({x:.2f}, {y:.2f})')
            return

        dx = x - self.initial_position[0]
        dy = y - self.initial_position[1]
        distance = math.sqrt(dx*dx + dy*dy)

        self.get_logger().info(f'Position: ({x:.2f}, {y:.2f}), Distance: {distance:.2f}m, Yaw: {math.degrees(yaw):.1f}°')

        if distance < self.target_distance:
            cmd = Twist()
            cmd.linear.x = 0.3
            self.cmd_pub.publish(cmd)
        else:
            cmd = Twist()
            self.cmd_pub.publish(cmd)
            self.get_logger().info('Target distance reached!')

    def quaternion_to_euler(self, x, y, z, w):
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw
```

Python Version for Safety Mechanisms:
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
from example_interfaces.srv import SetBool

class SafeMovementController(Node):
    def __init__(self):
        super().__init__('safe_movement_controller')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.safety_timer = self.create_timer(0.5, self.safety_check)
        self.last_command_time = time.time()
        self.command_timeout = 1.0

    def safety_check(self):
        time_since_command = time.time() - self.last_command_time
        if time_since_command > self.command_timeout:
            stop_cmd = Twist()
            self.cmd_pub.publish(stop_cmd)
            self.get_logger().warn('Safety timeout - stopping robot')

class EmergencyStopController(Node):
    def __init__(self):
        super().__init__('emergency_stop_controller')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.estop_service = self.create_service(
            SetBool,
            '/emergency_stop',
            self.estop_callback)
        self.estop_active = False

    def estop_callback(self, request, response):
        self.estop_active = request.data
        if self.estop_active:
            stop_cmd = Twist()
            self.cmd_pub.publish(stop_cmd)
            self.get_logger().error('EMERGENCY STOP ACTIVATED')
            response.message = 'Emergency stop activated'
        else:
            self.get_logger().info('Emergency stop deactivated')
            response.message = 'Emergency stop deactivated'
        response.success = True
        return response
```

Python Version for Smooth Movement Control:
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class SmoothMovementController(Node):
    def __init__(self):
        super().__init__('smooth_movement_controller')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.max_linear_accel = 0.5
        self.max_angular_accel = 1.0
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.control_timer = self.create_timer(0.1, self.control_loop)
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel_raw',
            self.cmd_callback,
            10)

    def cmd_callback(self, msg):
        self.target_linear = msg.linear.x
        self.target_angular = msg.angular.z

    def control_loop(self):
        dt = 0.1
        linear_error = self.target_linear - self.current_linear
        angular_error = self.target_angular - self.current_angular
        max_linear_change = self.max_linear_accel * dt
        max_angular_change = self.max_angular_accel * dt

        if abs(linear_error) > max_linear_change:
            linear_change = max_linear_change if linear_error > 0 else -max_linear_change
        else:
            linear_change = linear_error

        if abs(angular_error) > max_angular_change:
            angular_change = max_angular_change if angular_error > 0 else -max_angular_change
        else:
            angular_change = angular_error

        self.current_linear += linear_change
        self.current_angular += angular_change

        cmd = Twist()
        cmd.linear.x = self.current_linear
        cmd.angular.z = self.current_angular
        self.cmd_pub.publish(cmd)
```

Python Version for Path Following:
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
import math

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_sub = self.create_subscription(
            Path,
            '/planned_path',
            self.path_callback,
            10)
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/current_pose',
            self.pose_callback,
            10)
        self.path = None
        self.current_pose = None
        self.target_index = 0
        self.goal_tolerance = 0.2
        self.max_linear_speed = 0.5
        self.max_angular_speed = 1.0
        self.control_timer = self.create_timer(0.1, self.follow_path)

    def path_callback(self, msg):
        self.path = msg
        self.target_index = 0
        self.get_logger().info(f'Received path with {len(msg.poses)} waypoints')

    def pose_callback(self, msg):
        self.current_pose = msg

    def follow_path(self):
        if self.path is None or self.current_pose is None:
            return

        if self.target_index >= len(self.path.poses):
            self.stop_robot()
            return

        target_pose = self.path.poses[self.target_index]
        distance = self.calculate_distance(self.current_pose.pose, target_pose.pose)

        if distance < self.goal_tolerance:
            self.target_index += 1
            if self.target_index >= len(self.path.poses):
                self.get_logger().info('Path completed!')
                self.stop_robot()
                return
            else:
                self.get_logger().info(f'Reached waypoint {self.target_index-1}')
                target_pose = self.path.poses[self.target_index]

        cmd = self.calculate_control_command(target_pose)
        self.cmd_pub.publish(cmd)

    def calculate_distance(self, pose1, pose2):
        dx = pose2.position.x - pose1.position.x
        dy = pose2.position.y - pose1.position.y
        return math.sqrt(dx*dx + dy*dy)

    def calculate_control_command(self, target_pose):
        current_x = self.current_pose.pose.position.x
        current_y = self.current_pose.pose.position.y
        current_yaw = self.get_yaw_from_pose(self.current_pose.pose)
        target_x = target_pose.pose.position.x
        target_y = target_pose.pose.position.y
        angle_to_target = math.atan2(target_y - current_y, target_x - current_x)
        angle_error = angle_to_target - current_yaw

        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi

        cmd = Twist()
        cmd.angular.z = max(-self.max_angular_speed, min(self.max_angular_speed, 2.0 * angle_error))

        if abs(angle_error) < 0.5:
            distance = self.calculate_distance(self.current_pose.pose, target_pose.pose)
            cmd.linear.x = max(0.1, min(self.max_linear_speed, distance))
        else:
            cmd.linear.x = 0.1

        return cmd

    def get_yaw_from_pose(self, pose):
        q = pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def stop_robot(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)
```
