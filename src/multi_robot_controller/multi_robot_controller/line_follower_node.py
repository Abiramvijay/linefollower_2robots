#!/usr/bin/env python3

import math
import time
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

from .pid_controller import PIDController


class LineFollowerNode(Node):
    def __init__(self):
        super().__init__('line_follower_node')

        self.robot_name = self.declare_parameter('robot_name', 'robot1').value
        self.line_color = str(self.declare_parameter('line_color', 'red').value).lower()
        self.camera_topic = self.declare_parameter('camera_topic', f'/{self.robot_name}/camera/image').value
        self.cmd_vel_topic = self.declare_parameter('cmd_vel_topic', f'/{self.robot_name}/cmd_vel').value
        self.status_topic = self.declare_parameter('status_topic', f'/{self.robot_name}/status').value
        self.odom_topic = self.declare_parameter('odom_topic', f'/{self.robot_name}/odom').value
        self.start_topic = self.declare_parameter('start_topic', '/start_race').value
        self.base_linear_velocity = float(self.declare_parameter('base_linear_velocity', 0.18).value)
        self.max_angular_velocity = float(self.declare_parameter('max_angular_velocity', 1.5).value)
        self.kp = float(self.declare_parameter('kp', 0.0065).value)
        self.ki = float(self.declare_parameter('ki', 0.0002).value)
        self.kd = float(self.declare_parameter('kd', 0.0035).value)
        self.dt = float(self.declare_parameter('controller_dt', 0.05).value)
        self.line_missing_timeout = float(self.declare_parameter('line_missing_timeout', 1.2).value)
        self.path_length = float(self.declare_parameter('path_length', 7.8).value)
        self.track_min_area = int(self.declare_parameter('track_min_area', 80).value)
        self.max_linear_speed = float(self.declare_parameter('max_linear_speed', 0.22).value)
        self.return_speed = float(self.declare_parameter('return_speed', 0.14).value)

        self.started = False
        self.completed = False
        self.returning_home = False
        self.recovery_frames = 0
        self.last_line_error = 0.0
        self.last_odom_position = None
        self.travelled_distance = 0.0
        self.path_history = []
        self.return_waypoint_index = -1
        self.bridge = CvBridge()
        self.pid = PIDController(
            kp=self.kp,
            ki=self.ki,
            kd=self.kd,
            dt=self.dt,
            max_value=self.max_angular_velocity,
            integral_limit=3.5,
        )
        self.last_status_time = time.monotonic()
        self.image_width = 640
        self.image_height = 480

        self.start_sub = self.create_subscription(Bool, self.start_topic, self.start_callback, 10)
        self.return_sub = self.create_subscription(Bool, '/return_home', self.return_callback, 10)
        self.image_sub = self.create_subscription(Image, self.camera_topic, self.image_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)

        self.publish_status('waiting_for_start')
        self.stop_robot(reason='initialised')

    def publish_status(self, status: str) -> None:
        self.status_pub.publish(String(data=f'{self.robot_name}:{status}'))

    def publish_cmd(self, linear_x: float, angular_z: float) -> None:
        cmd = Twist(
            linear=Vector3(x=float(linear_x), y=0.0, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=float(angular_z)),
        )
        self.cmd_pub.publish(cmd)

    def stop_robot(self, reason: str = 'stop') -> None:
        self.publish_cmd(0.0, 0.0)
        if reason and not self.completed:
            self.publish_status(reason)

    def start_callback(self, msg: Bool) -> None:
        if not msg.data:
            self.started = False
            self.completed = False
            self.returning_home = False
            self.recovery_frames = 0
            self.last_line_error = 0.0
            self.last_odom_position = None
            self.travelled_distance = 0.0
            self.path_history.clear()
            self.return_waypoint_index = -1
            self.pid.reset()
            self.stop_robot('waiting_for_start')
            return

        if not self.started and not self.returning_home:
            self.started = True
            self.pid.reset()
            self.recovery_frames = 0
            self.last_line_error = 0.0
            self.last_odom_position = None
            self.travelled_distance = 0.0
            self.path_history.clear()
            self.return_waypoint_index = -1
            self.publish_status('tracking')
            self.get_logger().info(f'{self.robot_name} start signal received. Tracking enabled.')

    def return_callback(self, msg: Bool) -> None:
        if not msg.data:
            return
        if self.returning_home:
            return
        if len(self.path_history) < 2:
            self.get_logger().warning(
                f'{self.robot_name} cannot return home: no traveled path has been recorded.'
            )
            self.publish_status('home_unavailable')
            return

        self.completed = False
        self.returning_home = True
        self.return_waypoint_index = len(self.path_history) - 2
        self.publish_status('returning_home')
        self.get_logger().info(f'{self.robot_name} returning along its recorded path.')

    def odom_callback(self, msg: Odometry) -> None:
        if not self.started or (self.completed and not self.returning_home):
            return
        position = msg.pose.pose.position
        current_position = (position.x, position.y)

        if self.returning_home:
            self.follow_return_path(msg, current_position)
            return

        if self.path_history:
            last_recorded = self.path_history[-1]
            spacing = math.hypot(
                current_position[0] - last_recorded[0],
                current_position[1] - last_recorded[1],
            )
            if spacing >= 0.06:
                self.path_history.append(current_position)
        else:
            self.path_history.append(current_position)

        if self.last_odom_position is not None:
            dx = current_position[0] - self.last_odom_position[0]
            dy = current_position[1] - self.last_odom_position[1]
            self.travelled_distance += float(np.hypot(dx, dy))
            if self.travelled_distance >= self.path_length:
                self.completed = True
                self.publish_cmd(0.0, 0.0)
                self.publish_status('completed')
                self.get_logger().info(f'{self.robot_name} reached the end of its path.')
                return
        self.last_odom_position = current_position

    def follow_return_path(self, odom: Odometry, current_position: Tuple[float, float]) -> None:
        while self.return_waypoint_index >= 0:
            target = self.path_history[self.return_waypoint_index]
            distance = math.hypot(
                target[0] - current_position[0],
                target[1] - current_position[1],
            )
            if distance > 0.10:
                break
            self.return_waypoint_index -= 1

        if self.return_waypoint_index < 0:
            self.returning_home = False
            self.started = False
            self.completed = False
            self.path_history.clear()
            self.publish_cmd(0.0, 0.0)
            self.publish_status('home')
            self.get_logger().info(f'{self.robot_name} returned home.')
            return

        target = self.path_history[self.return_waypoint_index]
        dx = target[0] - current_position[0]
        dy = target[1] - current_position[1]
        target_heading = math.atan2(dy, dx)
        orientation = odom.pose.pose.orientation
        heading = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        heading_error = math.atan2(
            math.sin(target_heading - heading),
            math.cos(target_heading - heading),
        )
        angular = max(-1.2, min(1.2, 2.0 * heading_error))
        linear = self.return_speed if abs(heading_error) < 0.7 else 0.0
        self.publish_cmd(linear, angular)

    def make_mask(self, hsv_image: np.ndarray) -> np.ndarray:
        if self.line_color == 'red':
            lower_a = np.array([0, 80, 50], dtype=np.uint8)
            upper_a = np.array([12, 255, 255], dtype=np.uint8)
            lower_b = np.array([170, 80, 50], dtype=np.uint8)
            upper_b = np.array([180, 255, 255], dtype=np.uint8)
            mask1 = cv2.inRange(hsv_image, lower_a, upper_a)
            mask2 = cv2.inRange(hsv_image, lower_b, upper_b)
            mask = cv2.bitwise_or(mask1, mask2)
        elif self.line_color == 'blue':
            lower = np.array([95, 80, 50], dtype=np.uint8)
            upper = np.array([135, 255, 255], dtype=np.uint8)
            mask = cv2.inRange(hsv_image, lower, upper)
        else:
            raise ValueError(f'Unsupported line color: {self.line_color}')

        kernel = np.ones((5, 5), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask

    def find_line_centroid(self, mask: np.ndarray) -> Optional[Tuple[float, float, float]]:
        height, width = mask.shape[:2]
        roi_start = int(height * 0.40)
        roi = mask[roi_start:, :]
        contours, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        contour = max(
            contours,
            key=lambda item: cv2.contourArea(item) * (
                1.0 + cv2.boundingRect(item)[1] / max(roi.shape[0], 1)
            ),
        )
        area = cv2.contourArea(contour)
        if area < self.track_min_area:
            return None

        moments = cv2.moments(contour)
        if moments['m00'] <= 0:
            return None

        cx = moments['m10'] / moments['m00']
        cy = moments['m01'] / moments['m00'] + roi_start
        return float(cx), float(cy), float(area)

    def image_callback(self, msg: Image) -> None:
        if not self.started or self.completed or self.returning_home:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # pragma: no cover - runtime only
            self.get_logger().warning(f'{self.robot_name} failed to decode image: {exc}')
            return

        if cv_image.size == 0:
            return

        self.image_height, self.image_width = cv_image.shape[:2]
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = self.make_mask(hsv)
        centroid = self.find_line_centroid(mask)

        if centroid is None:
            self.recovery_frames += 1
            if self.recovery_frames > max(1, int(self.line_missing_timeout / self.dt)):
                self.completed = True
                self.publish_cmd(0.0, 0.0)
                self.publish_status('lost_line')
                self.get_logger().warn(f'{self.robot_name} lost the line and stopped safely.')
            else:
                turn_direction = float(np.sign(self.last_line_error))
                self.publish_cmd(
                    self.base_linear_velocity * 0.15,
                    turn_direction * min(0.6, self.max_angular_velocity),
                )
            return

        self.recovery_frames = 0
        cx, _, _ = centroid
        error = ((self.image_width / 2.0) - cx) / (self.image_width / 2.0)
        self.last_line_error = error
        angular_velocity = self.pid.update(error)

        adaptive_gain = max(0.35, 1.0 - min(abs(error), 0.8))
        linear_speed = min(self.max_linear_speed, self.base_linear_velocity * adaptive_gain)

        self.publish_cmd(linear_speed, angular_velocity)


def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot('shutdown')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
