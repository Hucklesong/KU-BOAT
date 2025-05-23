import rclpy
import rclpy
from rclpy.node import Node
from mechaship_interfaces.msg import RgbwLedColor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32, String, ColorRGBA
from math import radians, degrees, sin, cos, atan2, sqrt
from tf_transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
import numpy as np
import time

# 칼만 필터
class KalmanFilter:
    # 상태 벡터 및 초기화
    def __init__(self):
        self.x = np.zeros(4)
        self.P = np.eye(4)
        self.F = np.eye(4)
        self.H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])
        self.Q = np.eye(4) * 0.01
        self.R = np.eye(2) * 0.1

    # 상태 전이
    def predict(self, dt):
        self.F[0, 2] = dt
        self.F[1, 3] = dt
        self.x = np.dot(self.F, self.x)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q

    # 측정을 기반으로 상태를 업데이트
    def update(self, z):
        y = z - np.dot(self.H, self.x)
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        self.x += np.dot(K, y)
        self.P = np.dot(np.eye(4) - np.dot(K, self.H), self.P)

    # 현재 상태 반환 (위도, 경도)
    def get_state(self):
        return self.x[0], self.x[1]

# 통합 네비게이션
class GPSNavigation(Node):
    def __init__(self):
        super().__init__('gps_navigation')

        # GPS 네비게이션 초기화
        self.kalman_filter = KalmanFilter()
        self.last_time = None
        self.current_orientation_yaw = None

        # 웨이포인트
        self.target_points = [ # 출발선 기준
            (35.19169678, 129.10035789), # 1번 (왼쪽 모서리 아래쪽)
            (35.19169676, 129.10065614), # 2번 (왼쪽 모서리 위쪽)
            (35.19173032, 129.10061323), # 3번 (오른쪽 모서리 위쪽)
            (35.19185261, 129.10041128), # 4번 (오른쪽 모서리 아래쪽)
        ]
        self.current_target_index = 0
        self.distance_threshold = 5.0 # 거리 임계값 (미터터)
        
        # 방향제어 
        self.key_angles = { # 각도
        "left": 60.0,   # 왼쪽 방향
        "center": 90.0, # 중앙 방향
        "right": -120.0  # 오른쪽 방향
        }
        self.actuator_percentages = { # %
        "left": 30.0,   # 왼쪽
        "center": 50.0, # 중앙
        "right": 60.0   # 오른쪽
        }

        # 센서
        self.start_direction = self.wait_and_choose_direction()

        # 스러스터의 초기 퍼센티지를 0.0%로 설정
        self.thruster_percentage = 0.0

        # "/actuator/thruster/percentage" 토픽에 Float32 메시지를 발행할 퍼블리셔 생성
        self.thruster_publisher = self.create_publisher(
            Float32, "/actuator/thruster/percentage", 10
        )

        self.key_publisher = self.create_publisher(Float32, "/actuator/key/degree", 10)
        self.command_publisher = self.create_publisher(String, "/avoidance_command", 10)
        self.led_publisher = self.create_publisher(ColorRGBA, "/led/color", 10)


        # GPS 데이터
        self.gps_subscriber = self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.imu_subscriber = self.create_subscription(Imu, '/imu', self.imu_callback, qos_profile_sensor_data)
        self.create_timer(0.5, self.navigate_to_target)
        self.get_logger().info("GPS Navigation Node with Kalman Filter Initialized")

        # 장애물 회피
        self.lidar_subscriber = self.create_subscription(
            LaserScan, "/scan", self.lidar_callback, qos_profile_sensor_data
        )

        # 주기적 타이머 생성 (0.5초 간격으로 네비게이션 호출)
        self.create_timer(0.5, self.navigate_to_target)
        
        # LED를 노란색으로 점등
        self.set_led_color(1.0, 1.0, 0.0, 1.0)  # R=1, G=1, B=0 (노란색), Alpha=1
        self.get_logger().info("통합 네비게이션 노드 초기화 완료")

    # LED 색상을 설정하는 메서드
    def set_led_color(self, r, g, b, a):
        color = ColorRGBA()
        color.r = r
        color.g = g
        color.b = b
        color.a = a
        self.get_logger().info(f"LED 색상 설정: R={r}, G={g}, B={b}, A={a}")
        self.led_publisher.publish(color)

    # GPS 데이터    
    def gps_callback(self, msg: NavSatFix):
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        if self.last_time is None:
            self.last_time = current_time
            return

        dt = current_time - self.last_time
        self.last_time = current_time

        gps_measurement = np.array([msg.latitude, msg.longitude])
        self.kalman_filter.predict(dt)
        self.kalman_filter.update(gps_measurement)

    # IMU 데이터
    def imu_callback(self, msg: Imu):
        quaternion = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        _, _, yaw_rad = euler_from_quaternion(quaternion)
        self.current_orientation_yaw = degrees(yaw_rad)
    
    # LiDAR 데이터를 처리하는 콜백
    def lidar_callback(self, data: LaserScan):
        msg = LaserScan()

        msg.ranges = data.ranges[500:1500]
        msg.angle_min = radians(-90)  # 시작 각도
        msg.angle_max = radians(90)   # 종료 각도

        msg.range_min = data.range_min              # LiDAR의 최소 측정 가능 거리
        msg.range_max = data.range_max 

        if data.range_min < 0.5:  # 장애물이 임계 거리보다 가까운 경우
            self.get_logger().info(f"장애물 발견: {data.range_min:.2f}m")
            self.command_publisher.publish(String(data="회피 명령"))

    # 목표 지점으로 이동하는 메서드
    def navigate_to_target(self):
        if self.current_orientation_yaw is None:
            self.get_logger().warn("IMU 데이터 없음")
            return

        current_lat, current_lon = self.kalman_filter.get_state()
        target_lat, target_lon = self.target_points[self.current_target_index]

        distance_to_target = self.haversine_distance(current_lat, current_lon, target_lat, target_lon)
        if distance_to_target < self.distance_threshold:
            self.get_logger().info(f"목표 {self.current_target_index + 1} 도달!")
        
            # 3번 목표에 도달했을 때 왼쪽으로 회전하고 3초간 정지
            if self.current_target_index == 2:  # 3번 목표에 도달하면
                self.turn_left()
                self.stop_for_3_seconds()

            self.current_target_index += 1
            if self.current_target_index >= len(self.target_points):
                self.get_logger().info("모든 목표 지점 도달. LED를 끕니다.")
                self.set_led_color(0.0, 0.0, 0.0, 0.0)  # LED 끔
                self.thruster_publisher.publish(Float32(data=0.0))
                return

        self.get_logger().info(f"목표까지 거리: {distance_to_target:.2f}m")
        self.key_publisher.publish(Float32(data=90.0))

    # 왼쪽으로 회전하는 메서드
    def turn_left(self):
        new_yaw = self.current_orientation_yaw -30  # 왼쪽으로 30도 회전
        new_yaw = new_yaw % 360  # 360도를 넘지 않도록 보정
        self.key_publisher.publish(Float32(data=new_yaw))  # 회전 각도 설정

    # 3초간 정지하는 메서드
    def stop_for_3_seconds(self):
        self.get_logger().info("3초간 정지합니다.")
        self.thruster_publisher.publish(Float32(data=0.0))  # 추진기 멈춤
        time.sleep(3)  # 3초 대기
        self.thruster_publisher.publish(Float32(data=self.actuator_percentages[self.start_direction]))  # 원래 속도로 복귀

    # 두 지점 간 거리를 계산하는 헬퍼 메서드
    def haversine_distance(self, lat1, lon1, lat2, lon2):
        R = 6371000
        phi1, phi2 = radians(lat1), radians(lat2)
        delta_phi = radians(lat2 - lat1)
        delta_lambda = radians(lon2 - lon1)
        a = sin(delta_phi / 2) ** 2 + cos(phi1) * cos(phi2) * sin(delta_lambda / 2) ** 2
        c = 2 * atan2(sqrt(a), sqrt(1 - a))
        return R * c

    def calculate_bearing(self, lat1, lon1, lat2, lon2):
        phi1 = radians(lat1)
        phi2 = radians(lat2)
        delta_lambda = radians(lon2 - lon1)
        x = sin(delta_lambda) * cos(phi2)
        y = cos(phi1) * sin(phi2) - sin(phi1) * cos(phi2) * cos(delta_lambda)
        return (degrees(atan2(x, y)) + 360) % 360

    def wait_and_choose_direction(self):
        try:
            direction = input("왼쪽(left), 중앙(center), 오른쪽(right) 중 하나를 입력하세요: ").strip().lower()
            return direction
        except KeyboardInterrupt:
            print("\n프로그램이 사용자에 의해 중단되었습니다.")
            exit(0)  # 프로그램을 종료합니다.


def main(args=None):
    rclpy.init(args=args)
    node = GPSNavigation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
