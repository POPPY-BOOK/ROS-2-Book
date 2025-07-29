
```python
Figure 13.2: Mission Controller Node Structure
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from enum import Enum

class RobotState(Enum):
    IDLE = 1
    PATROLLING = 2
    INVESTIGATING = 3
    RETURNING = 4
    EMERGENCY = 5

class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')
        
        # Action clients
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(String, 'robot_status', 10)
        
        # Subscribers
        self.camera_sub = self.create_subscription(
            Image, 'camera/image_raw', self.camera_callback, 10)
        self.lidar_sub = self.create_subscription(
            LaserScan, 'scan', self.lidar_callback, 10)
        
        # State management
        self.state = RobotState.IDLE
        self.patrol_points = self.load_patrol_points()
        self.current_target = 0
        self.bridge = CvBridge()
        
        # Safety
        self.emergency_stop = False
        self.min_obstacle_distance = 0.5  # meters
        
        # Timers
        self.state_timer = self.create_timer(0.1, self.state_machine)
        self.safety_timer = self.create_timer(0.05, self.safety_check)
        
        self.get_logger().info('Mission controller started')
    
    def load_patrol_points(self):
        # In a real system, load these from a config file
        return [
            {'x': 2.0, 'y': 0.0, 'theta': 0.0},
            {'x': 2.0, 'y': 2.0, 'theta': 1.57},
            {'x': 0.0, 'y': 2.0, 'theta': 3.14},
            {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        ]

```

```python
def state_machine(self):
    if self.emergency_stop:
        if self.state != RobotState.EMERGENCY:
            self.enter_emergency_state()
        return
    
    if self.state == RobotState.IDLE:
        self.handle_idle_state()
    elif self.state == RobotState.PATROLLING:
        self.handle_patrolling_state()
    elif self.state == RobotState.INVESTIGATING:
        self.handle_investigating_state()
    elif self.state == RobotState.RETURNING:
        self.handle_returning_state()
    elif self.state == RobotState.EMERGENCY:
        self.handle_emergency_state()
    
    # Publish current status
    status_msg = String()
    status_msg.data = f"State: {self.state.name}, Target: {self.current_target}"
    self.status_pub.publish(status_msg)

def handle_idle_state(self):
    # Wait for a command or automatically start patrolling
    self.get_logger().info('Robot is idle. Starting patrol...', once=True)
    self.state = RobotState.PATROLLING
    self.current_target = 0

def handle_patrolling_state(self):
    # Navigate to the next patrol point
    if not self.nav_client.server_is_ready():
        self.get_logger().warn('Navigation server not ready')
        return
    
    target = self.patrol_points[self.current_target]
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = 'map'
    goal.pose.header.stamp = self.get_clock().now().to_msg()
    goal.pose.pose.position.x = target['x']
    goal.pose.pose.position.y = target['y']
    goal.pose.pose.orientation.z = np.sin(target['theta'] / 2.0)
    goal.pose.pose.orientation.w = np.cos(target['theta'] / 2.0)
    
    future = self.nav_client.send_goal_async(goal)
    future.add_done_callback(self.nav_goal_callback)
    
    # Move to next patrol point
    self.current_target = (self.current_target + 1) % len(self.patrol_points)
    
```
```python
def camera_callback(self, msg):
    try:
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Simple object detection (replace with your preferred method)
        objects = self.detect_objects(cv_image)
        
        if objects and self.state == RobotState.PATROLLING:
            self.get_logger().info(f'Detected {len(objects)} objects')
            self.state = RobotState.INVESTIGATING
            
    except Exception as e:
        self.get_logger().error(f'Camera callback error: {e}')

def lidar_callback(self, msg):
    try:
        # Check for obstacles in the front 60 degrees
        front_ranges = []
        angle_increment = msg.angle_increment
        
        for i, range_val in enumerate(msg.ranges):
            angle = msg.angle_min + i * angle_increment
            if -0.5 < angle < 0.5:  # Front 60 degrees
                if msg.range_min < range_val < msg.range_max:
                    front_ranges.append(range_val)
        
        if front_ranges:
            min_distance = min(front_ranges)
            if min_distance < self.min_obstacle_distance:
                self.get_logger().warn(f'Obstacle detected at {min_distance:.2f}m')
                # Don't immediately emergency stop - let navigation handle it
                
    except Exception as e:
        self.get_logger().error(f'Lidar callback error: {e}')

def detect_objects(self, image):
    # Simple color-based detection - replace with YOLO, etc.
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # Detect red objects (adjust ranges as needed)
    lower_red = np.array([0, 100, 100])
    upper_red = np.array([10, 255, 255])
    mask = cv2.inRange(hsv, lower_red, upper_red)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    objects = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 500:  # Minimum size threshold
            x, y, w, h = cv2.boundingRect(contour)
            objects.append({'x': x, 'y': y, 'w': w, 'h': h, 'area': area})
    
    return objects
```


```python
def safety_check(self):
    # Check for emergency conditions
    emergency_conditions = [
        self.check_hardware_status(),
        self.check_communication(),
        self.check_localization(),
        self.check_battery_level()
    ]
    
    if any(emergency_conditions):
        self.emergency_stop = True
        self.get_logger().error('Emergency condition detected!')

def check_hardware_status(self):
    # Check if critical sensors are publishing
    current_time = self.get_clock().now()
    
    # In a real system, track last message times
    # Return True if any sensor is stale
    return False

def check_communication(self):
    # Check if we can reach navigation server
    if not self.nav_client.server_is_ready():
        self.get_logger().warn('Navigation server not available')
        return True
    return False

def check_localization(self):
    # Check if AMCL is providing good localization
    # This would involve checking covariance matrices
    return False

def check_battery_level(self):
    # Monitor battery voltage/percentage
    # Return True if critically low
    return False

def enter_emergency_state(self):
    self.get_logger().error('ENTERING EMERGENCY STATE')
    self.state = RobotState.EMERGENCY
    
    # Stop all motion
    stop_msg = Twist()
    self.cmd_vel_pub.publish(stop_msg)
    
    # Cancel any active navigation goals
    self.nav_client.cancel_all_goals()

def handle_emergency_state(self):
    # Stay stopped until emergency is cleared
    if not self.emergency_stop:
        self.get_logger().info('Emergency cleared, returning to idle')
        self.state = RobotState.IDLE

```

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    world_file = LaunchConfiguration('world_file', default='warehouse.world')
    
    # Include robot launch
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('my_robot_description'),
                'launch',
                'robot.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world_file': world_file
        }.items()
    )
    
    # Navigation launch
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': PathJoinSubstitution([
                FindPackageShare('my_robot_nav'),
                'config',
                'nav2_params.yaml'
            ])
        }.items()
    )
    
    # Mission controller
    mission_controller = Node(
        package='my_robot_mission',
        executable='mission_controller',
        name='mission_controller',
        parameters=[{
            'use_sim_time': use_sim_time,
            'patrol_points_file': PathJoinSubstitution([
                FindPackageShare('my_robot_mission'),
                'config',
                'patrol_points.yaml'
            ])
        }],
        output='screen'
    )
    
    # Web dashboard
    web_dashboard = Node(
        package='my_robot_web',
        executable='dashboard_server',
        name='web_dashboard',
        parameters=[{'port': 8080}],
        output='screen'
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('world_file', default_value='warehouse.world'),
        
        robot_launch,
        nav_launch,
        mission_controller,
        web_dashboard
    ])

```

```python
# config/mission_params.yaml
mission_controller:
  ros__parameters:
    # Patrol configuration
    patrol_points_file: "patrol_points.yaml"
    patrol_speed: 0.5
    pause_time: 2.0
    
    # Detection parameters
    min_object_area: 500
    detection_confidence: 0.7
    investigation_time: 10.0
    
    # Safety parameters
    min_obstacle_distance: 0.5
    emergency_stop_distance: 0.3
    max_linear_velocity: 1.0
    max_angular_velocity: 1.0
    
    # Communication timeouts
    nav_server_timeout: 5.0
    sensor_timeout: 1.0
    
    # Logging
    log_level: "INFO"
    log_to_file: true
    log_file_path: "/tmp/robot_mission.log"
yaml
# config/patrol_points.yaml
patrol_points:
  - name: "entrance"
    x: 0.0
    y: 0.0
    theta: 0.0
    pause_time: 3.0
  - name: "corridor_1"
    x: 5.0
    y: 0.0
    theta: 1.57
    pause_time: 2.0
  - name: "office_area"
    x: 5.0
    y: 3.0
    theta: 3.14
    pause_time: 5.0
  - name: "return_point"
    x: 0.0
    y: 3.0
    theta: 4.71
    pause_time: 1.0

```

```python
# test/test_mission_controller.py
import unittest
from unittest.mock import Mock, patch
import rclpy
from my_robot_mission.mission_controller import MissionController, RobotState

class TestMissionController(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = MissionController()
    
    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()
    
    def test_initial_state(self):
        self.assertEqual(self.node.state, RobotState.IDLE)
        self.assertEqual(self.node.current_target, 0)
        self.assertFalse(self.node.emergency_stop)
    
    def test_patrol_point_loading(self):
        points = self.node.load_patrol_points()
        self.assertIsInstance(points, list)
        self.assertGreater(len(points), 0)
        
        for point in points:
            self.assertIn('x', point)
            self.assertIn('y', point)
            self.assertIn('theta', point)
    
    def test_emergency_stop(self):
        self.node.emergency_stop = True
        self.node.state_machine()
        self.assertEqual(self.node.state, RobotState.EMERGENCY)
    
    @patch('my_robot_mission.mission_controller.cv2')
    def test_object_detection(self, mock_cv2):
        # Mock OpenCV functions
        mock_cv2.cvtColor.return_value = Mock()
        mock_cv2.inRange.return_value = Mock()
        mock_cv2.findContours.return_value = ([], Mock())
        
        # Test with no objects
        image = Mock()
        objects = self.node.detect_objects(image)
        self.assertEqual(len(objects), 0)

if __name__ == '__main__':
    unittest.main()

```

```python
# test/test_integration.py
import unittest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time

class IntegrationTest(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.test_node = Node('integration_test')
        
        # Subscribe to key topics
        self.status_msgs = []
        self.cmd_vel_msgs = []
        
        self.status_sub = self.test_node.create_subscription(
            String, 'robot_status', self.status_callback, 10)
        self.cmd_vel_sub = self.test_node.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
    
    def tearDown(self):
        self.test_node.destroy_node()
        rclpy.shutdown()
    
    def status_callback(self, msg):
        self.status_msgs.append(msg.data)
    
    def cmd_vel_callback(self, msg):
        self.cmd_vel_msgs.append(msg)
    
    def test_system_startup(self):
        # Wait for system to initialize
        start_time = time.time()
        while time.time() - start_time < 10.0:
            rclpy.spin_once(self.test_node, timeout_sec=0.1)
            if self.status_msgs:
                break
        
        self.assertGreater(len(self.status_msgs), 0)
        self.assertIn('IDLE', self.status_msgs[0])
    
    def test_emergency_stop_response(self):
        # Simulate emergency condition
        # This would involve publishing emergency messages
        # and verifying the robot stops
        pass

```

```python
# web/dashboard_server.py
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, LaserScan
import json
import asyncio
import websockets
import threading
from http.server import HTTPServer, SimpleHTTPRequestHandler
import os

class DashboardServer(Node):
    def __init__(self):
        super().__init__('dashboard_server')
        
        # Store latest data
        self.latest_data = {
            'status': 'Unknown',
            'cmd_vel': {'linear': 0.0, 'angular': 0.0},
            'lidar_min_distance': 0.0,
            'timestamp': 0
        }
        
        # Subscribers
        self.status_sub = self.create_subscription(
            String, 'robot_status', self.status_callback, 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.lidar_sub = self.create_subscription(
            LaserScan, 'scan', self.lidar_callback, 10)
        
        # WebSocket clients
        self.clients = set()
        
        # Start servers
        self.start_servers()
    
    def start_servers(self):
        # HTTP server for static files
        os.chdir(os.path.dirname(__file__))
        httpd = HTTPServer(('localhost', 8080), SimpleHTTPRequestHandler)
        http_thread = threading.Thread(target=httpd.serve_forever)
        http_thread.daemon = True
        http_thread.start()
        
        # WebSocket server
        ws_thread = threading.Thread(target=self.start_websocket_server)
        ws_thread.daemon = True
        ws_thread.start()
        
        self.get_logger().info('Dashboard available at http://localhost:8080')
    
    def start_websocket_server(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        
        async def handle_client(websocket, path):
            self.clients.add(websocket)
            try:
                await websocket.wait_closed()
            finally:
                self.clients.discard(websocket)
        
        start_server = websockets.serve(handle_client, 'localhost', 8081)
        loop.run_until_complete(start_server)
        loop.run_forever()
    
    def status_callback(self, msg):
        self.latest_data['status'] = msg.data
        self.latest_data['timestamp'] = self.get_clock().now().nanoseconds
        self.broadcast_data()
    
    def cmd_vel_callback(self, msg):
        self.latest_data['cmd_vel'] = {
            'linear': msg.linear.x,
            'angular': msg.angular.z
        }
        self.broadcast_data()
    
    def lidar_callback(self, msg):
        valid_ranges = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        if valid_ranges:
            self.latest_data['lidar_min_distance'] = min(valid_ranges)
        self.broadcast_data()
    
    def broadcast_data(self):
        if not self.clients:
            return
        
        data_json = json.dumps(self.latest_data)
        
        # Remove disconnected clients
        disconnected = set()
        for client in self.clients:
            try:
                asyncio.run_coroutine_threadsafe(
                    client.send(data_json), 
                    client.loop if hasattr(client, 'loop') else asyncio.get_event_loop()
                )
            except:
                disconnected.add(client)
        
        self.clients -= disconnected
The corresponding HTML dashboard:
<!DOCTYPE html>
<html>
<head>
    <title>Robot Dashboard</title>
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; }
        .status-box { 
            border: 1px solid #ccc; 
            padding: 15px; 
            margin: 10px 0; 
            border-radius: 5px; 
        }
        .status-ok { background-color: #d4edda; }
        .status-warning { background-color: #fff3cd; }
        .status-error { background-color: #f8d7da; }
        .data-row { display: flex; justify-content: space-between; margin: 5px 0; }
    </style>
</head>
<body>
    <h1>Robot Control Dashboard</h1>
    
    <div class="status-box" id="main-status">
        <h2>System Status</h2>
        <div class="data-row">
            <span>State:</span>
            <span id="robot-state">Unknown</span>
        </div>
        <div class="data-row">
            <span>Last Update:</span>
            <span id="last-update">Never</span>
        </div>
    </div>
    
    <div class="status-box">
        <h2>Motion</h2>
        <div class="data-row">
            <span>Linear Velocity:</span>
            <span id="linear-vel">0.0 m/s</span>
        </div>
        <div class="data-row">
            <span>Angular Velocity:</span>
            <span id="angular-vel">0.0 rad/s</span>
        </div>
    </div>
    
    <div class="status-box">
        <h2>Sensors</h2>
        <div class="data-row">
            <span>Closest Obstacle:</span>
            <span id="min-distance">Unknown</span>
        </div>
    </div>
    
    <script>
        const ws = new WebSocket('ws://localhost:8081');
        
        ws.onmessage = function(event) {
            const data = JSON.parse(event.data);
            
            // Update status
            document.getElementById('robot-state').textContent = data.status;
            document.getElementById('last-update').textContent = new Date().toLocaleTimeString();
            
            // Update motion data
            document.getElementById('linear-vel').textContent = data.cmd_vel.linear.toFixed(2) + ' m/s';
            document.getElementById('angular-vel').textContent = data.cmd_vel.angular.toFixed(2) + ' rad/s';
            
            // Update sensor data
            document.getElementById('min-distance').textContent = data.lidar_min_distance.toFixed(2) + ' m';
            
            // Update status box color
            const statusBox = document.getElementById('main-status');
            if (data.status.includes('EMERGENCY')) {
                statusBox.className = 'status-box status-error';
            } else if (data.status.includes('IDLE')) {
                statusBox.className = 'status-box status-warning';
            } else {
                statusBox.className = 'status-box status-ok';
            }
        };
        
        ws.onopen = function(event) {
            console.log('Connected to robot dashboard');
        };
        
        ws.onclose = function(event) {
            console.log('Disconnected from robot dashboard');
        };
    </script>
</body>
</html>

```

```python
# test/test_mission_controller.py
import unittest
from unittest.mock import Mock, patch, MagicMock
import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

from my_robot_mission.mission_controller import MissionController, RobotState

class TestMissionController(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = MissionController()
        # Mock the action client to avoid network calls
        self.node.nav_client = Mock()
        
    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()
    
    def test_emergency_stop_overrides_everything(self):
        # This is the most critical behavior to test
        self.node.state = RobotState.PATROLLING
        self.node.emergency_stop = True
        
        self.node.state_machine()
        
        self.assertEqual(self.node.state, RobotState.EMERGENCY)
        # Verify stop command was sent
        self.node.cmd_vel_pub.publish.assert_called()
        
    def test_object_detection_with_real_data(self):
        # Don't mock OpenCV - test with actual image data
        # Create a simple test image with a red rectangle
        test_image = np.zeros((480, 640, 3), dtype=np.uint8)
        test_image[100:200, 200:300] = [0, 0, 255]  # Red rectangle
        
        objects = self.node.detect_objects(test_image)
        
        self.assertEqual(len(objects), 1)
        self.assertGreater(objects[0]['area'], 500)
    
    def test_lidar_obstacle_detection(self):
        # Create a mock laser scan with an obstacle
        scan_msg = LaserScan()
        scan_msg.angle_min = -3.14
        scan_msg.angle_max = 3.14
        scan_msg.angle_increment = 0.1
        scan_msg.range_min = 0.1
        scan_msg.range_max = 10.0
        
        # Most ranges are far, but one is close
        scan_msg.ranges = [5.0] * 62  # ~6.2 radians / 0.1 increment
        scan_msg.ranges[31] = 0.3  # Obstacle directly in front
        
        # This should trigger obstacle detection
        self.node.lidar_callback(scan_msg)
        
        # Check if emergency stop was triggered
        # (depends on your specific implementation)
        
    def test_patrol_point_cycling(self):
        # Test that patrol points cycle correctly
        initial_target = self.node.current_target
        num_points = len(self.node.patrol_points)
        
        # Simulate completing several patrol points
        for i in range(num_points + 2):
            self.node.handle_patrolling_state()
            
        # Should have cycled back to start
        expected_target = (initial_target + num_points + 2) % num_points
        self.assertEqual(self.node.current_target, expected_target)

```

```python
# test/test_hardware_integration.py
import unittest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image
import time
import threading

class HardwareIntegrationTest(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.test_node = Node('integration_test')
        
        # Track messages we receive
        self.received_messages = {
            'status': [],
            'cmd_vel': [],
            'scan': [],
            'camera': []
        }
        
        # Subscribe to critical topics
        self.test_node.create_subscription(
            String, 'robot_status', 
            lambda msg: self.received_messages['status'].append(msg), 10)
        
        self.test_node.create_subscription(
            Twist, 'cmd_vel',
            lambda msg: self.received_messages['cmd_vel'].append(msg), 10)
            
        self.test_node.create_subscription(
            LaserScan, 'scan',
            lambda msg: self.received_messages['scan'].append(msg), 10)
        
        # Start spinning in background
        self.spin_thread = threading.Thread(target=self.spin_node)
        self.spin_thread.daemon = True
        self.spin_thread.start()
        
    def spin_node(self):
        while rclpy.ok():
            rclpy.spin_once(self.test_node, timeout_sec=0.1)
    
    def tearDown(self):
        self.test_node.destroy_node()
        rclpy.shutdown()
    
    def test_all_sensors_publishing(self):
        """Verify all critical sensors are publishing data"""
        # Wait for messages from all sensors
        timeout = 10.0
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            if (len(self.received_messages['scan']) > 0 and
                len(self.received_messages['status']) > 0):
                break
            time.sleep(0.1)
        
        self.assertGreater(len(self.received_messages['scan']), 0, 
                          "No lidar data received")
        self.assertGreater(len(self.received_messages['status']), 0,
                          "No status messages received")
    
    def test_emergency_stop_latency(self):
        """Test how quickly the robot responds to emergency stop"""
        # This would involve triggering an emergency condition
        # and measuring response time
        start_time = time.time()
        
        # Trigger emergency (implementation specific)
        # ...
        
        # Wait for robot to stop
        while time.time() - start_time < 1.0:
            if self.received_messages['cmd_vel']:
                latest_cmd = self.received_messages['cmd_vel'][-1]
                if (latest_cmd.linear.x == 0.0 and 
                    latest_cmd.angular.z == 0.0):
                    break
            time.sleep(0.01)
        
        response_time = time.time() - start_time
        self.assertLess(response_time, 0.5, 
                       "Emergency stop took too long")
    
    def test_sensor_data_validity(self):
        """Check that sensor data makes sense"""
        # Wait for some lidar data
        timeout = 5.0
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            if self.received_messages['scan']:
                break
            time.sleep(0.1)
        
        self.assertGreater(len(self.received_messages['scan']), 0)
        
        # Check the latest scan
        latest_scan = self.received_messages['scan'][-1]
        
        # Basic sanity checks
        self.assertGreater(len(latest_scan.ranges), 0)
        self.assertGreater(latest_scan.range_max, latest_scan.range_min)
        
        # Check for reasonable values
        valid_ranges = [r for r in latest_scan.ranges 
                       if latest_scan.range_min <= r <= latest_scan.range_max]
        self.assertGreater(len(valid_ranges), 0, 
                          "No valid lidar readings")

```
```python
# See all active nodes
ros2 node list

# Check if a specific node is publishing
ros2 topic echo /robot_status --once

# See the communication graph
ros2 run rqt_graph rqt_graph

# Monitor topic frequencies
ros2 topic hz /scan
ros2 topic hz /cmd_vel

# Check for dropped messages
ros2 topic echo /rosout | grep ERROR

# See parameter values
ros2 param list /mission_controller
ros2 param get /mission_controller use_sim_time


def setup_debug_publisher(self):
    """Add detailed debug information publisher"""
    self.debug_pub = self.create_publisher(String, 'debug_info', 10)
    self.debug_timer = self.create_timer(1.0, self.publish_debug_info)

def publish_debug_info(self):
    """Publish detailed state information for debugging"""
    debug_info = {
        'timestamp': self.get_clock().now().nanoseconds,
        'node_name': self.get_name(),
        'state': self.state.name,
        'current_target': self.current_target,
        'emergency_stop': self.emergency_stop,
        'last_nav_result': getattr(self, 'last_nav_result', 'none'),
        'sensor_health': {
            'lidar_ok': hasattr(self, 'last_lidar_time') and 
                       (self.get_clock().now().nanoseconds - self.last_lidar_time) < 1e9,
            'camera_ok': hasattr(self, 'last_camera_time') and
                        (self.get_clock().now().nanoseconds - self.last_camera_time) < 1e9,
        },
        'performance': {
            'loop_frequency': getattr(self, 'loop_frequency', 0.0),
            'cpu_usage': self.get_cpu_usage(),
            'memory_usage': self.get_memory_usage()
        }
    }
    
    msg = String()
    msg.data = json.dumps(debug_info, indent=2)
    self.debug_pub.publish(msg)

def get_cpu_usage(self):
    """Get current CPU usage percentage"""
    try:
        import psutil
        return psutil.cpu_percent(interval=None)
    except ImportError:
        return 0.0

def get_memory_usage(self):
    """Get current memory usage in MB"""
    try:
        import psutil
        process = psutil.Process()
        return process.memory_info().rss / 1024 / 1024
    except ImportError:
        return 0.0


# In your node constructor
self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

# Use different log levels appropriately
self.get_logger().debug(f'Processing {len(objects)} detected objects')
self.get_logger().info('Starting patrol sequence')
self.get_logger().warn(f'Navigation goal failed: {result.error_code}')
self.get_logger().error(f'Critical sensor failure: {sensor_name}')
self.get_logger().fatal('Emergency stop activated - manual intervention required')
Configure logging in your launch file:
python
# In your launch file
mission_controller = Node(
    package='my_robot_mission',
    executable='mission_controller',
    name='mission_controller',
    parameters=[{
        'use_sim_time': use_sim_time,
    }],
    arguments=['--ros-args', '--log-level', 'DEBUG'],
    output='screen'
)

```
