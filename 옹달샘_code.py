import numpy as np
import math
from geopy.distance import geodesic
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from std_msgs.msg import Bool, Float32
import cv2
import os
from tf_transformations import euler_from_quaternion
from sensor_msgs.msg import NavSatFix, NavSatStatus
from mechaship_interfaces.msg import RgbwLedColor
from cv_bridge import CvBridge
from ultralytics import YOLO
from sensor_msgs.msg import CompressedImage
from statistics import median
from math import degrees
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)


def vincenty_distance(lat1, lon1, lat2, lon2):
    point1 = (lat1, lon1)
    point2 = (lat2, lon2)
    distance = geodesic(point1, point2).meters
    return distance


def set_risk_zone(array, center, spread):
    array[center] = 1
    for i in range(1, spread + 1):
        if center - i >= 0:
            array[center - i] = 1
        if center + i <= 180:
            array[center + i] = 1
    return array


class NavigationNode(Node):
    def __init__(self):
        super().__init__("Auto_sailing")
        self.imu_heading = 90.0
        self.max_risk_threshold = 60.0
        self.key_target_degree = 90.0
        self.target_imu_angle = 90.0

        self.median_latitude = 35.0  # start
        self.median_longitude = 126.0  # start

        self.goal_x = [35.0, 35.1]  # turn, end
        self.goal_y = [126.0, 126.1]  # turn, end

        self.latitude_buffer = []
        self.longitude_buffer = []
        self.sailing_section = None
        self.yolo = YOLO("best.pt", task="detect")
        self.br = CvBridge()
        self.camera_subscription = None
        self.color = RgbwLedColor()
        self.color.white = 20
        self.color.green = 0
        self.color.red = 0
        self.color.blue = 0
        lidar_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        gps_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        imu_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.lidar_subscription = self.create_subscription(
            LaserScan, "/scan", self.lidar_callback, lidar_qos_profile
        )
        self.create_subscription(Imu, "/imu", self.imu_callback, imu_qos_profile)
        self.create_subscription(
            NavSatFix, "/gps/fix", self.gps_callback, gps_qos_profile
        )
        self.create_subscription(
            Bool, "/sensor/emo/status", self.emo_switch_callback, 10
        )
        self.rgbw_led_publisher = self.create_publisher(
            RgbwLedColor, "/actuator/rgbwled/color", 10
        )
        self.key_publisher = self.create_publisher(Float32, "/actuator/key/degree", 10)
        self.thruster_publisher = self.create_publisher(
            Float32, "/actuator/thruster/percentage", 10
        )

        self.create_timer(1.0, self.median_gps)
        self.create_timer(0.1, self.timer_callback)

    def emo_switch_callback(self, data: Bool):
        switch_status = data.data
        if switch_status is True:
            self.all_stop()

    def all_stop(self):
        key_msg = Float32(data=90.0)
        self.key_publisher.publish(key_msg)
        thruster_msg = Float32(data=0.0)
        self.thruster_publisher.publish(thruster_msg)
        os.system("killall -SIGINT ros2")

    def timer_callback(self):
        if self.sailing_section == "end":
            self.thruster_msg = Float32(data=0.0)
            self.key_target_degree = Float32(data=90.0)
            self.color.white = 0
            self.color.green = 0
            self.color.red = 0
            self.color.blue = 0

        else:
            self.thruster_msg = Float32(data=20.0)
        self.key_publisher.publish(Float32(data=self.key_target_degree))
        self.thruster_publisher.publish(Float32(data=self.thruster_msg))
        self.rgbw_led_publisher.publish(self.color)

    def gps_callback(self, msg: NavSatFix):
        self.latitude_buffer.append(msg.latitude)
        self.longitude_buffer.append(msg.longitude)

    def median_gps(self):
        if self.latitude_buffer and self.longitude_buffer:
            self.median_latitude = median(self.latitude_buffer)
            self.median_longitude = median(self.longitude_buffer)
            median_msg = NavSatFix()
            median_msg.latitude = self.median_latitude
            median_msg.longitude = self.median_longitude
            median_msg.status.status = NavSatStatus.STATUS_FIX
            self.get_logger().info(
                f"GPS: latitude={self.median_latitude}, longitude={self.median_longitude}"
            )
            self.latitude_buffer.clear()
            self.longitude_buffer.clear()

        if self.sailing_section == "first":
            dist = vincenty_distance(
                self.median_latitude,
                self.median_longitude,
                self.goal_x[0],
                self.goal_y[0],
            )
            if dist < 5.0:
                self.lidar_subscription.destroy()
                self.activate_camera_callback()
                self.first_turn_sequence()
                self.lidar_subscription = self.create_subscription(
                    LaserScan,
                    "/scan",
                    self.lidar_callback,
                    QoSProfile(
                        reliability=QoSReliabilityPolicy.BEST_EFFORT,
                        durability=QoSDurabilityPolicy.VOLATILE,
                        history=QoSHistoryPolicy.KEEP_LAST,
                        depth=5,
                    ),
                )
                self.sailing_section = "second"

        elif self.sailing_section == "second":
            dist_end = vincenty_distance(
                self.median_latitude,
                self.median_longitude,
                self.goal_x[0],
                self.goal_y[0],
            )
            if dist_end < 3.0:
                self.sailing_section = "end"

    def imu_callback(self, msg: Imu):
        quaternion = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )
        roll_rad, pitch_rad, yaw_rad = euler_from_quaternion(quaternion)
        yaw = degrees(yaw_rad)
        if yaw <= 90.0:
            yaw_degree = 90.0 - yaw
        elif yaw > 90.0:
            yaw_degree = 450.0 - yaw

        if 0 <= yaw_degree <= 180:
            self.target_imu_angle = 90.0
        elif 180 < yaw_degree < 360:
            self.target_imu_angle = 270.0
        else:
            self.target_imu_angle = 90.0
        self.imu_heading = yaw_degree

    def lidar_callback(self, data):
        ranges = np.array(data.ranges)
        relevant_data = ranges[500:1500]
        relevant_data = relevant_data[
            (relevant_data != 0) & (relevant_data != float("inf"))
        ]
        cumulative_distance = np.zeros(181)
        sample_count = np.zeros(181)
        average_distance = np.zeros(181)
        risk_values = np.zeros(181)
        risk_map = np.zeros(181)

        for i in range(len(relevant_data)):
            length = relevant_data[i]
            angle_index = round((len(relevant_data) - 1 - i) * 180 / len(relevant_data))
            cumulative_distance[angle_index] += length
            sample_count[angle_index] += 1

        for j in range(181):
            if sample_count[j] != 0:
                average_distance[j] = cumulative_distance[j] / sample_count[j]

        for k in range(181):
            if average_distance[k] != 0:
                risk_values[k] = 135.72 * math.exp(-0.6109 * average_distance[k])

        for k in range(181):
            if risk_values[k] >= self.max_risk_threshold:
                set_risk_zone(risk_map, k, 23)

        safe_angles = np.where(risk_map == 0)[0].tolist()
        heading_diff = float(self.target_imu_angle - self.imu_heading)
        step_factor = 1.0
        if self.target_imu_angle == 90.0:
            desired_heading = self.target_imu_angle + heading_diff * step_factor
        else:
            desired_heading = self.target_imu_angle + heading_diff * step_factor - 180.0

        if len(safe_angles) > 0:
            heading = float(min(safe_angles, key=lambda x: abs(x - desired_heading)))
        else:
            heading = 45.0

        if heading > 135.0:
            heading = 135.0
        if heading < 45.0:
            heading = 45.0

        self.key_target_degree = heading

        self.get_logger().info(
            f"key: {self.key_target_degree:5.1f}, IMU: {self.imu_heading:5.1f}"
        )

    def first_turn_sequence(self):
        self.thruster_msg = Float32(data=20.0)

        while True:
            ranges = np.array(self.lidar_subscription.ranges)
            relevant_data = ranges[500:1500]
            relevant_data = relevant_data[
                (relevant_data != 0) & (relevant_data != float("inf"))
            ]
            cumulative_distance = np.zeros(181)
            sample_count = np.zeros(181)
            average_distance = np.zeros(181)

            for i in range(len(relevant_data)):
                length = relevant_data[i]
                angle_index = round(
                    (len(relevant_data) - 1 - i) * 180 / len(relevant_data)
                )
                cumulative_distance[angle_index] += length
                sample_count[angle_index] += 1

            for j in range(181):
                if sample_count[j] != 0:
                    average_distance[j] = cumulative_distance[j] / sample_count[j]

            if average_distance[90] <= 1.0:
                break

        self.key_target_degree = Float32(data=45.0)
        self.key_publisher.publish(self.key_target_degree)
        time.sleep(1.6)

        self.key_target_degree = Float32(data=90.0)
        self.key_publisher.publish(self.key_target_degree)
        time.sleep(3)

    def camera_callback(self, msg: CompressedImage):
        cv_image = self.br.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        results = self.yolo.predict(source=cv_image, stream=False, conf=0.27, iou=0.6)
        for result in results[0].boxes:
            if int(result.cls) == 0:
                self.key_target_degree = Float32(data=65.0)
                self.deactivate_camera_callback()
                break
            if int(result.cls) == 1:
                self.key_target_degree = Float32(data=90.0)
                self.deactivate_camera_callback()
                break

            if int(result.cls) == 2:
                self.key_target_degree = Float32(data=115.0)
                self.deactivate_camera_callback()
                break
        self.color.white = 0
        self.color.green = 20
        self.color.red = 0
        self.color.blue = 0
        time.sleep(4)

    def activate_camera_callback(self):
        if self.camera_subscription is None:
            self.camera_subscription = self.create_subscription(
                CompressedImage, "/camera/image/compressed", self.camera_callback, 10
            )

    def deactivate_camera_callback(self):
        if self.camera_subscription is not None:
            self.destroy_subscription(self.camera_subscription)
            self.camera_subscription = None


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
