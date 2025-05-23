
import rclpy  # ROS 2 Python 클라이언트 라이브러리
from rclpy.node import Node  # ROS 2 Node 클래스 가져오기
from mechaship_interfaces.msg import RgbwLedColor  # RGBW LED 제어용 사용자 정의 메시지
from rclpy.qos import qos_profile_sensor_data  # 센서 데이터를 위한 QoS 설정
from sensor_msgs.msg import Imu, NavSatFix, LaserScan  # IMU, GPS, LiDAR 센서 메시지 임포트
from std_msgs.msg import Float32, String, ColorRGBA  # 표준 메시지 타입(Float32, String, ColorRGBA) 임포트
from math import radians, degrees, sin, cos, atan2, sqrt  # 수학 관련 함수들(radians, atan2 등) 사용
from tf_transformations import euler_from_quaternion  # 쿼터니언을 오일러각으로 변환하는 함수
import numpy as np  # 수치 계산을 위한 numpy 사용


class KalmanFilter:  # GPS 위치 추정을 위한 간단한 칼만 필터 클래스
    def __init__(self):
        self.x = np.zeros(4)
        self.P = np.eye(4)
        self.F = np.eye(4)
        self.H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])
        self.Q = np.eye(4) * 0.01
        self.R = np.eye(2) * 0.1

    def predict(self, dt):  # 시스템 상태 예측 (시간 간격에 따라 위치 예측)
        self.F[0, 2] = dt
        self.F[1, 3] = dt
        self.x = np.dot(self.F, self.x)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q

    def update(self, z):  # GPS 측정값을 바탕으로 상태 보정
        y = z - np.dot(self.H, self.x)
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        self.x += np.dot(K, y)
        self.P = np.dot(np.eye(4) - np.dot(K, self.H), self.P)

    def get_state(self):  # 필터가 추정한 현재 위치 반환
        return self.x[0], self.x[1]


class GPSNavigation(Node):  # GPS 기반 자율 항법을 수행하는 ROS 2 노드 클래스
    def __init__(self):
        super().__init__('gps_navigation')
        self.declare_parameter("start_direction", "center")
        self.start_direction = self.get_parameter("start_direction").get_parameter_value().string_value

        self.kalman_filter = KalmanFilter()
        self.last_time = None
        self.current_orientation_yaw = None

        self.target_points = [
            (35.19169678, 129.10035789),
            (35.19169676, 129.10065614),
            (35.19173032, 129.10061323),
            (35.19185261, 129.10041128),
        ]
        self.current_target_index = 0
        self.distance_threshold = 5.0

        self.key_angles = {"left": 60.0, "center": 90.0, "right": -120.0}
        self.actuator_percentages = {"left": 30.0, "center": 50.0, "right": 60.0}
        self.thruster_percentage = 0.0

        self.thruster_publisher = self.create_publisher(Float32, "/actuator/thruster/percentage", 10)
        self.key_publisher = self.create_publisher(Float32, "/actuator/key/degree", 10)
        self.command_publisher = self.create_publisher(String, "/avoidance_command", 10)
        self.led_publisher = self.create_publisher(ColorRGBA, "/led/color", 10)

        self.gps_subscriber = self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.imu_subscriber = self.create_subscription(Imu, '/imu', self.imu_callback, qos_profile_sensor_data)  # 센서 데이터를 위한 QoS 설정
        self.lidar_subscriber = self.create_subscription(LaserScan, "/scan", self.lidar_callback, qos_profile_sensor_data)  # 센서 데이터를 위한 QoS 설정

        self.create_timer(0.5, self.navigate_to_target)

        self.set_led_color(1.0, 1.0, 0.0, 1.0)
        self.get_logger().info("통합 네비게이션 노드 초기화 완료")

    def set_led_color(self, r, g, b, a):  # LED 색상을 설정하고 표시
        color = ColorRGBA(r=r, g=g, b=b, a=a)
        self.led_publisher.publish(color)
        self.get_logger().info(f"LED 색상 변경: R={r}, G={g}, B={b}, A={a}")

    def gps_callback(self, msg: NavSatFix):  # GPS 데이터를 수신하여 칼만 필터에 전달
        current_time = self.get_clock().now().nanoseconds / 1e9
        if self.last_time is None:
            self.last_time = current_time
            return
        dt = current_time - self.last_time
        self.last_time = current_time

        gps_measurement = np.array([msg.latitude, msg.longitude])
        if np.any(np.isnan(gps_measurement)):
            self.get_logger().warn("GPS 측정값에 NaN 포함됨 → 칼만 필터 업데이트 생략")
            return

        self.kalman_filter.predict(dt)
        self.kalman_filter.update(gps_measurement)

    def imu_callback(self, msg: Imu):  # IMU 데이터에서 Yaw(방향) 각도 계산
        quaternion = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        _, _, yaw_rad = euler_from_quaternion(quaternion)  # 쿼터니언을 오일러각으로 변환하는 함수
        self.current_orientation_yaw = (degrees(yaw_rad) + 360) % 360

    def lidar_callback(self, data: LaserScan):  # LiDAR 데이터를 이용해 장애물 감지 및 회피 명령 전송
        min_distance = np.nanmin(data.ranges)
        if min_distance < 0.5:
            self.get_logger().info(f"[장애물] 최소 거리: {min_distance:.2f}m")
            self.command_publisher.publish(String(data="회피 명령"))

    def navigate_to_target(self):  # 목표 지점까지 이동하며 현재 방향과 목표 방향 비교 및 제어
        if self.current_orientation_yaw is None:
            self.get_logger().warn("[네비게이션] IMU 데이터 없음")
            return

        current_lat, current_lon = self.kalman_filter.get_state()
        target_lat, target_lon = self.target_points[self.current_target_index]
        distance_to_target = self.haversine_distance(current_lat, current_lon, target_lat, target_lon)

        if distance_to_target < self.distance_threshold:
            self.get_logger().info(f"[네비게이션] 목표 {self.current_target_index + 1} 도달!")
            if self.current_target_index == 2:
                self.turn_left()
                self.schedule_stop_and_resume()
            self.current_target_index += 1
            if self.current_target_index >= len(self.target_points):
                self.get_logger().info("[네비게이션] 모든 목표 도달 완료. 시스템 종료.")
                self.set_led_color(0.0, 0.0, 0.0, 0.0)
                self.thruster_publisher.publish(Float32(data=0.0))
                rclpy.shutdown()
                return

        desired_yaw = self.calculate_bearing(current_lat, current_lon, target_lat, target_lon)
        angle_diff = (desired_yaw - self.current_orientation_yaw + 180) % 360 - 180
        self.key_publisher.publish(Float32(data=angle_diff))
        self.get_logger().info(f"[거리] 목표 {self.current_target_index + 1}까지 {distance_to_target:.2f}m")

    def turn_left(self):  # 현재 방향 기준 왼쪽으로 회전 명령 전송
        new_yaw = (self.current_orientation_yaw - 30) % 360
        self.key_publisher.publish(Float32(data=new_yaw))

    def schedule_stop_and_resume(self):  # 정지 후 3초 뒤에 다시 이동하도록 타이머 설정
        self.thruster_publisher.publish(Float32(data=0.0))
        self.get_logger().info("3초간 정지 후 재개 예정")
        self.stop_timer = self.create_timer(3.0, self.resume_after_stop_once)

    def resume_after_stop_once(self):  # 정지 후 스로틀을 재개하여 다시 움직이도록 설정
        percentage = self.actuator_percentages.get(self.start_direction, 50.0)
        self.thruster_publisher.publish(Float32(data=percentage))
        self.get_logger().info(f"[네비게이션] {self.start_direction} 방향으로 스로틀 재개: {percentage}%")
        self.stop_timer.cancel()

    def haversine_distance(self, lat1, lon1, lat2, lon2):  # 두 GPS 좌표 간 거리 계산 (지구 곡률 반영)
        R = 6371000
        phi1, phi2 = radians(lat1), radians(lat2)
        delta_phi = radians(lat2 - lat1)
        delta_lambda = radians(lon2 - lon1)
        a = sin(delta_phi / 2) ** 2 + cos(phi1) * cos(phi2) * sin(delta_lambda / 2) ** 2
        c = 2 * atan2(sqrt(a), sqrt(1 - a))
        return R * c

    def calculate_bearing(self, lat1, lon1, lat2, lon2):  # 현재 위치에서 목표 위치까지 방향(방위각) 계산
        phi1 = radians(lat1)
        phi2 = radians(lat2)
        delta_lambda = radians(lon2 - lon1)
        x = sin(delta_lambda) * cos(phi2)
        y = cos(phi1) * sin(phi2) - sin(phi1) * cos(phi2) * cos(delta_lambda)
        return (degrees(atan2(x, y)) + 360) % 360


def main(args=None):  # ROS 2 노드를 실행하고 종료 시 정리
    rclpy.init(args=args)
    node = GPSNavigation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':  # 메인 함수 실행 조건
    main()
