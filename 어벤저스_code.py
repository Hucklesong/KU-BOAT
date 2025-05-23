# from math import radians, degrees
# import cv2
# import numpy as np
# import rclpy
# from rclpy.node import Node
# from rclpy.qos import qos_profile_sensor_data
# from std_msgs.msg import Float32


# from sensor_msgs.msg import LaserScan, Imu
# from tf_transformations import euler_from_quaternion


# class LidarPublisher(Node):
#     def __init__(self):
#         super().__init__("lidar_publisher")

#         self.create_subscription(LaserScan,
#                                  "/scan", 
#                                  self.lidar_callback, 
#                                  qos_profile_sensor_data
#                                  )
#         self.create_subscription(Imu, 
#                                  "/imu", 
#                                  self.imu_callback, 
#                                  qos_profile_sensor_data)
#         self.thruster_publisher = self.create_publisher(
#             Float32, "/actuator/thruster/percentage", 10
#         )
#         self.key_publisher = self.create_publisher(Float32, "/actuator/key/degree", 10)
#         self.create_timer(0.1, self.timer_callback)
#         # "/actuator/key/degree" 토픽에 Float32 메시지를 발행할 퍼블리셔 생성

#         # 변수 초기화
#         self.sliced_ranges = np.array([])
#         self.range_max = 1.0
#         self.roll_degree = 0.0
#         self.pitch_degree = 0.0
#         self.yaw_degree = 0.0
#         self.img_size = 1000
#         self.key_degree = 90.0
#         self.thruster_percentage = 50.0 

#     def lidar_callback(self, data: LaserScan):
#         # 데이터 슬라이싱
#         self.sliced_ranges = np.array(data.ranges[500:1500])
#         self.range_max = data.range_max

#         if len(self.sliced_ranges) > 0:
#             # 이동 평균 적용 후 덮어쓰기
#             self.sliced_ranges = self.moving_average(self.sliced_ranges)

#     def moving_average(self, data, window_size=10):
#         """이동 평균 필터를 적용"""
#         kernel = np.ones(window_size) / window_size
#         smoothed_data = np.convolve(data, kernel, mode='same')
#         return smoothed_data
    
#     def imu_callback(self, msg: Imu):
#         quaternion = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
#         roll_rad, pitch_rad, yaw_rad = euler_from_quaternion(quaternion)
#         self.roll_degree = degrees(roll_rad) + 180
#         self.pitch_degree = degrees(pitch_rad) + 180
#         self.yaw_degree = degrees(yaw_rad) + 180

#     def timer_callback(self):

#         if self.sliced_ranges.size==0:
#             self.get_logger().info("No LiDAR data available yet!")
#             return

#         ranges_array = np.array(self.sliced_ranges)
#         angles = np.linspace(-90, 90, len(ranges_array))

#         origin = (self.img_size // 2, self.img_size // 2)
#         scale = self.img_size / (2 * self.range_max)

#         img = np.zeros((self.img_size, self.img_size, 3), dtype=np.uint8)

#         previous_point = None
#         max_distance, min_distance = 0, float('inf')
#         max_point, min_point = None, None
#         max_angle, min_angle = 0, 0

#         for angle, distance in zip(angles, ranges_array):
#             if 0 < distance < self.range_max:
#                 radian = np.deg2rad(angle)
#                 x = int(origin[0] - distance * np.sin(radian) * scale)
#                 y = int(origin[1] - distance * np.cos(radian) * scale)

#                 if previous_point is not None:
#                     cv2.line(img, previous_point, (x, y), (0, 255, 0), 2)
#                 previous_point = (x, y)

#                 if distance > max_distance:
#                     max_distance, max_point, max_angle = distance, (x, y), angle
#                 if distance < min_distance:
#                     min_distance, min_point, min_angle = distance, (x, y), angle

#         cv2.circle(img, origin, 8, (0, 0, 255), -1)
#         if max_point:
#             cv2.circle(img, max_point, 10, (255, 0, 0), -1)
#             cv2.putText(img, f"Max: {max_distance:.2f} m, Angle: {max_angle:.2f}°",
#                         (max_point[0] + 10, max_point[1] - 10),
#                         cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
#         if min_point:
#             cv2.circle(img, min_point, 10, (0, 0, 255), -1)
#             cv2.putText(img, f"Min: {min_distance:.2f} m, Angle: {min_angle:.2f}°",
#                         (min_point[0] + 10, min_point[1] + 20),
#                         cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

#         text_x, text_y_start, line_spacing = 50, 100, 40
#         cv2.putText(img, f"Roll:  {self.roll_degree:.5f}°", (text_x, text_y_start),
#                     cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
#         cv2.putText(img, f"Pitch: {self.pitch_degree:.5f}°", (text_x, text_y_start + line_spacing),
#                     cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
#         cv2.putText(img, f"Yaw:   {self.yaw_degree:.5f}°", (text_x, text_y_start + 2 * line_spacing),
#                     cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
#         cv2.putText(img, f"key:   {self.key_degree:.5f}°", (text_x, text_y_start + 3 * line_spacing),
#                     cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
#         cv2.putText(img, f"key:   {self.thruster_percentage:.5f}°", (text_x, text_y_start + 4 * line_spacing),
#                     cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
#         cv2.imshow("LiDAR Visualization", img)
#         cv2.waitKey(1)
        
        
#         key_msg = Float32()
#         thr_msg = Float32()
#         # 메시지 데이터에 현재 키 각도 할당
#         key_msg.data = self.key_degree
#         thr_msg.data = self.thruster_percentage
#         # 메시지를 "/actuator/thruster/percentage" 토픽에 발행
#         self.thruster_publisher.publish(thr_msg)
#         self.key_degree= 90-max_angle
#         # 메시지를 "/actuator/key/degree" 토픽에 발행
#         self.key_publisher.publish(key_msg)
        

# def main(args=None):
#     rclpy.init(args=args)
#     node = LidarPublisher()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == "__main__":
#     main()



from collections import deque, Counter
from math import radians, degrees
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan, Imu
from tf_transformations import euler_from_quaternion
from mechaship_interfaces.msg import RgbwLedColor
from sensor_msgs.msg import CompressedImage, Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from sensor_msgs.msg import NavSatFix, NavSatStatus
from math import radians, cos, sin, sqrt

CLASSES = [
    'blue_circle', 'blue_cross', 'blue_triangle',
    'green_circle', 'green_cross', 'green_triangle',
    'red_circle', 'red_cross', 'red_triangle'
]
class LidarPublisher(Node):
    FONT = cv2.FONT_HERSHEY_SIMPLEX
    COLOR = (0, 255, 0)
    THICKNESS = 2

    def __init__(self):
        super().__init__("lidar_publisher",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
                         
                         )
        # # 파라미터 가져오기
        # self.image_topic = (
        #     self.get_parameter_or("image_topic", "/image_raw/compressed")
        #     .get_parameter_value()
        #     .string_value
        # )

        # self.detections_topic = (
        #     self.get_parameter_or("detections_topic", "/detections")
        #     .get_parameter_value()
        #     .string_value
        # )

        # self.preview = (
        #     self.get_parameter_or("preview", True).get_parameter_value().bool_value
        # )
        # self.get_logger().info(f"image_topic : {self.image_topic}")
        # self.get_logger().info(f"detections_topic : {self.detections_topic}")

        #노드 만들기

        self.create_subscription(LaserScan,
                                 "/scan", 
                                 self.lidar_callback, 
                                 qos_profile_sensor_data
                                 )
        self.create_subscription(Imu, 
                                 "/imu", 
                                 self.imu_callback, 
                                 qos_profile_sensor_data)
        self.thruster_publisher = self.create_publisher(
                                Float32, 
                                "/actuator/thruster/percentage",
                                10
                                )
        self.key_publisher = self.create_publisher(
                                Float32, 
                                "/actuator/key/degree", 
                                10
                                )
        self.create_timer(
                                0.1, 
                                self.timer_callback
                                )
        self.rgbw_led_publisher = self.create_publisher(
                                RgbwLedColor, 
                                "/actuator/rgbwled/color", 
                                10
                                )   
        # self.create_subscription(
        #                         Bool, 
        #                         "/sensor/emo/status", 
        #                         self.emo_switch_callback, 
        #                         10
        #                         )
        # self.create_subscription(NavSatFix, 
        #                          "/gps/fix", 
        #                          self.gps_callback, 
        #                          10)
        
        # self.processed_image_publisher = self.create_publisher(
        #                         Image, 
        #                         "processed_image",
        #                         qos_profile_sensor_data
        #                         )
        # self.processed_image_compressed_publisher = self.create_publisher(
        #                         CompressedImage,
        #                         "processed_image/compressed",
        #                         qos_profile_sensor_data
        #                         )
        #         # CvBridge 불러오기
        # self.br = CvBridge()

        # # 객체 인식 결과
        # self.detections = Detection2DArray()

        # "/actuator/key/degree" 토픽에 Float32 메시지를 발행할 퍼블리셔 생성

        # 변수 초기화
        self.sliced_ranges = np.array([])
        self.range_max = 1.0
        self.roll_degree = 0.0
        self.pitch_degree = 0.0
        self.yaw_degree = 0.0
        self.img_size = 1000
        self.key_degree = 90.0
        self.thruster_percentage = 50.0 
        self.mode=6
        self.switch_status=False
# WGS84 지구 반경 및 편평률 설정

        # WGS84 지구 반경 및 편평률 설정
        self.a = 6378137.0
        self.f = 1 / 298.257223563
        self.e2 = 2 * self.f - self.f ** 2

        # 초기 위치 설정
        self.origin_lat = None
        self.origin_lon = None
        self.origin_alt = None
        self.origin_ecef = None
        self.latitude = 0.0
        self.longitude = 0.0
        self.altitude = None
        self.x = 0.0
        self.y = 0.0
        self.initialized = False
        self.mean_x = 0  # 초기값
        self.mean_y = 0  # 초기값
        self.recent_x = deque(maxlen=30)  # 최근 30개의 mean_x 값 저장
        self.recent_y = deque(maxlen=30)  # 최근 30개의 mean_y 값 저장
        self.modified_x=0.0
        self.modified_y=0.0
        self.visang_mode=False
        self.key_queue= deque([0] * 10,maxlen=10)
        self.key_queue_weighted = []
        # # 이미지 Subscription 생성
        # self.image_subscription = self.create_subscription(
        #     CompressedImage,
        #     self.image_topic,
        #     self.image_callback,
        #     qos_profile_sensor_data,
        # )

        # # 객체 인식 결과 Subscription 생성
        # self.detection_subscription = self.create_subscription(
        #     Detection2DArray,
        #     self.detections_topic,
        #     self.detection_callback,
        #     qos_profile_sensor_data,
        # )
        # 파라미터 가져오기
        # self.image_topic = (
        #     self.get_parameter_or("image_topic", "/image_raw/compressed"))

        # self.detections_topic = (
        #     self.get_parameter_or("detections_topic", "/detections")
        #     )

        # self.preview = (
        #     self.get_parameter_or("preview", True))

        # self.get_logger().info(f"image_topic : {self.image_topic}")
        # self.get_logger().info(f"detections_topic : {self.detections_topic}")

        # # 객체 인식 결과 이미지 Publisher 생성
        # self.processed_image_publisher = self.create_publisher(
        #     Image, "processed_image", qos_profile_sensor_data
        # )
        # self.processed_image_compressed_publisher = self.create_publisher(
        #     CompressedImage, "processed_image/compressed", qos_profile_sensor_data
        # )

        # # 이미지 Subscription 생성
        # self.image_subscription = self.create_subscription(
        #     CompressedImage,
        #     self.image_topic,
        #     self.image_callback,
        #     qos_profile_sensor_data,
        # )

        # # 객체 인식 결과 Subscription 생성
        # self.detection_subscription = self.create_subscription(
        #     Detection2DArray,
        #     self.detections_topic,
        #     self.detection_callback,
        #     qos_profile_sensor_data,
        # )

        # # CvBridge 불러오기
        # self.br = CvBridge()

        # # 객체 인식 결과
        # self.detections = Detection2DArray()

    
    
    
    
    def lidar_callback(self, data: LaserScan):
        # 데이터 슬라이싱
        self.sliced_ranges = np.array(data.ranges[600:1400])
        self.range_max = data.range_max

        if len(self.sliced_ranges) > 0:
             self.sliced_ranges = self.moving_average(self.sliced_ranges,250)


    def moving_average(self, data, window_size):
        """이동 평균 필터를 적용"""
        kernel = np.ones(window_size) / window_size
        smoothed_data = np.convolve(data, kernel, mode='same')
        return smoothed_data
    
    def imu_callback(self, msg: Imu):
        quaternion = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        roll_rad, pitch_rad, yaw_rad = euler_from_quaternion(quaternion)
        self.roll_degree = degrees(roll_rad) + 180
        self.pitch_degree = degrees(pitch_rad) + 180
        self.yaw_degree = degrees(yaw_rad) + 180
    

    def timer_callback(self):
        if self.sliced_ranges.size == 0:
            self.get_logger().info("No LiDAR data available yet!")
            return

        # LiDAR 데이터 확인 및 주행에 필요한 값 계산
        ranges_array = np.array(self.sliced_ranges)
        angles = np.linspace(-90,90, len(ranges_array))
        max_distance, min_distance = 0, float('inf')
        max_angle, min_angle = 0, 0

        for angle, distance in zip(angles, ranges_array):
            if 0 < distance < self.range_max:
                if distance > max_distance:
                    max_distance, max_angle = distance, angle
                if distance < min_distance:
                    min_distance, min_angle = distance, angle
        # self.sliced_ranges_input = self.sliced_ranges[350:450]
        # self.sliced_ranges_output = self.sliced_ranges[325:475]

#         # 빈 배열 체크
#         if self.sliced_ranges_input.size == 0 or self.sliced_ranges_output.size == 0:
#             self.get_logger().warn("Empty sliced_ranges_input or sliced_ranges_output. Skipping this iteration.")
#             return
#         if self.sliced_ranges_input.size == 0 or self.sliced_ranges_output.size == 0:
#             self.get_logger().info("Empty sliced_ranges detected. Skipping this iteration.")
#             return
#         # input_range에 대해 최소값과 각도 계산
#         if self.sliced_ranges_input.size > 0:
#             min_index_input = np.argmax(self.sliced_ranges_input)
#             min_distance_input = self.sliced_ranges_input[min_index_input]
#             angles_input=np.linspace(-9,9,100)
#             min_angle_input = angles_input[min_index_input]
#         else:
#             self.get_logger().warn("No valid data in sliced_ranges_input.")
#             return
#         self.get_logger().info(f"min_distance_input: {min_distance_input}")

#         if self.sliced_ranges_output.size > 0:
#             min_index_output = np.argmax(self.sliced_ranges_output)
#             min_distance_output = self.sliced_ranges_output[min_index_output]
#         else:
#             self.get_logger().warn("No valid data in sliced_ranges_output.")
#             return  
#         # 주행에 필요한 값 업데이트
#         interval = 10
# # 리스트 생성
#         weights = [5] * 4+ [4] *3 + [3] *2  + [2] *1 +[1]*10
#         sungan_key_degree = (90.0 - 1.0 * max_angle) // interval

#         # 큐에 값 추가
#         self.key_queue.append(sungan_key_degree)
#         self.get_logger().info(f"Current key_queue: {list(self.key_queue)}")
#         # 기존 key_queue 복사본 생성
#         key_queue_copy = list(self.key_queue)  # deque를 list로 변환하여 복사
#         self.key_queue_mod= deque([0] * 40,maxlen=40)
#         # 가중치만큼 값을 큐에 추가
#         for value, weight in zip(key_queue_copy, weights):  # 복사본 사용
#             self.key_queue_mod.extend([value] * weight)

#         # 가장 많이 나온 값 계산
#         self.counter = Counter(self.key_queue_mod).most_common(1)[0][0]
#         self.key_degree = 90-0.7*(90-interval * self.counter)
#         # self.thruster_percentage = 100-self.key_degree  # 필요하면 다른 로직으로 조정
#         self.thruster_percentage = 61
#         threshold = 1.5
#         if  self.sliced_ranges_output.all()<1.0:
#             self.thruster_percentage=35.0
#             self.key_degree=90.0
        
        # if not self.visang_mode and min_distance_input <= threshold:  # 임계값: 1.0
        #     self.visang_mode = True  # 비상 모드 활성화
        #     self.get_logger().info("비상 모드 활성화!")
        # if self.visang_mode:
        #     self.key_degree=90+40*np.sign(min_angle_input)
        # if self.visang_mode and min_distance_output >= threshold: 
        #     self.visang_mode = False  # 비상 모드 비활성화
        #     self.get_logger().info("비상 모드 비활성화! 비상 모드 실행 중단.")
        #     return  # 알고리즘 반복 방지
        # # 메시지 발행
        self.key_degree=90-0.8*max_angle
        self.thruster_percentage=61
        
        self.publish_control_values()

        self.led_control()
        # 시각화 및 로그 출력
        self.visualize_and_log(ranges_array, angles, max_distance, max_angle, min_distance, min_angle)
    
    
    

    def visualize_and_log(self, ranges_array, angles, max_distance, max_angle, min_distance, min_angle):
        """LiDAR 데이터를 시각화하고 로그를 출력"""
        origin = (self.img_size // 2, self.img_size // 2)
        scale = self.img_size / (2 * self.range_max)
        img = np.zeros((self.img_size, self.img_size, 3), dtype=np.uint8)

        previous_point = None
        for angle, distance in zip(angles, ranges_array):
            if 0 < distance < self.range_max:
                radian = np.deg2rad(angle)
                x = int(origin[0] - distance * np.sin(radian) * scale)
                y = int(origin[1] - distance * np.cos(radian) * scale)

                if previous_point is not None:
                    cv2.line(img, previous_point, (x, y), (0, 255, 0), 2)
                previous_point = (x, y)

        max_point = (
            int(origin[0] - max_distance * np.sin(np.deg2rad(max_angle)) * scale),
            int(origin[1] - max_distance * np.cos(np.deg2rad(max_angle)) * scale),
        )
        min_point = (
            int(origin[0] - min_distance * np.sin(np.deg2rad(min_angle)) * scale),
            int(origin[1] - min_distance * np.cos(np.deg2rad(min_angle)) * scale),
        )

        cv2.circle(img, origin, 8, (0, 0, 255), -1)  # 원점
        cv2.circle(img, max_point, 10, (255, 0, 0), -1)
        cv2.putText(img, f"Max: {max_distance:.2f} m, Angle: {max_angle:.2f}°",
                    (max_point[0] + 10, max_point[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.circle(img, min_point, 10, (0, 0, 255), -1)
        cv2.putText(img, f"Min: {min_distance:.2f} m, Angle: {min_angle:.2f}°",
                    (min_point[0] + 10, min_point[1] + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

        # IMU 및 키 관련 정보 추가
        text_x, text_y_start, line_spacing = 50, 100, 40
        cv2.putText(img, f"Roll:  {self.roll_degree:.5f}°", (text_x, text_y_start),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(img, f"Pitch: {self.pitch_degree:.5f}°", (text_x, text_y_start + line_spacing),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(img, f"Yaw:   {self.yaw_degree:.5f}°", (text_x, text_y_start + 2 * line_spacing),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(img, f"Key:   {self.key_degree:.5f}°", (text_x, text_y_start + 3 * line_spacing),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(img, f"Thruster: {self.thruster_percentage:.5f}°", (text_x, text_y_start + 4 * line_spacing),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(img, f"localx: {self.modified_x:.5f}°", (text_x, text_y_start + 5 * line_spacing),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(img, f"localy: {self.modified_y:.5f}°", (text_x, text_y_start + 6 * line_spacing),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(img, f"lat: {self.latitude:.5f}°", (text_x, text_y_start + 7 * line_spacing),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(img, f"long: {self.longitude:.5f}°", (text_x, text_y_start + 8 * line_spacing),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        visang_mode_text = f"Visang Mode: {'ON' if self.visang_mode else 'OFF'}"
        cv2.putText(
            img,                                  # 표시할 이미지
            visang_mode_text,                     # 텍스트 내용
            (text_x, text_y_start + 9 * line_spacing),  # 텍스트 위치
            cv2.FONT_HERSHEY_SIMPLEX,             # 폰트 종류
            1,                                    # 폰트 크기
            (255, 255, 255),                      # 텍스트 색상 (흰색)
            2                                     # 텍스트 두께
        )   

        cv2.imshow("LiDAR Visualization", img)
        cv2.waitKey(1)

        # 로그 출력
        self.get_logger().info(f"Max Distance: {max_distance:.2f} m, Angle: {max_angle:.2f}°")
        self.get_logger().info(f"Min Distance: {min_distance:.2f} m, Angle: {min_angle:.2f}°")

    def publish_control_values(self):
        """키 각도와 쓰러스터 값을 퍼블리시"""
        key_msg = Float32()
        thr_msg = Float32()
        key_msg.data = float(self.key_degree)
        thr_msg.data = float(self.thruster_percentage)

        self.key_publisher.publish(key_msg)
        self.thruster_publisher.publish(thr_msg)
    def led_control(self):
        """RGBW LED 제어 함수"""
        msg = RgbwLedColor()
        msg.red = 0
        msg.green = 0
        msg.blue = 0
        msg.white = 0

        if self.mode == 1:
            # 빨간색 켜기
            self.get_logger().info("Red")
            msg.red = 20
            self.rgbw_led_status = "R"

        elif self.mode == 2:
            # 초록색 켜기
            self.get_logger().info("Green")
            msg.green = 20
            self.rgbw_led_status = "G"

        elif self.mode == 3:
            # 파란색 켜기
            self.get_logger().info("Blue")
            msg.blue = 20
            self.rgbw_led_status = "B"

        elif self.mode == 4:
            # 흰색 켜기
            self.get_logger().info("White")
            msg.white = 20
            self.rgbw_led_status = "W"

        elif self.mode == 5:
            # 모든 LED 끄기
            self.get_logger().info("OFF")
            self.rgbw_led_status = "N"
        elif self.mode == 6:
            # 모든 LED 끄기
            self.get_logger().info("Y")
            msg.red=20
            msg.green=20
            self.rgbw_led_status = "Y"


        # 설정된 색상을 퍼블리시
        self.rgbw_led_publisher.publish(msg)
    def emo_switch_callback(self, data: Bool):
        # 수신한 EMO 스위치 상태 데이터 추출
        self.switch_status = data.data

    def detection_callback(self, detections_msg: Detection2DArray) -> None:
        # self.detections 변수 업데이트
        self.detections = detections_msg

    # 이미지 Subscription callback
    def image_callback(self, compressed_image: CompressedImage) -> None:
        # compressed image를 OpenCV 이미지로 변환
        origin_image = self.br.compressed_imgmsg_to_cv2(compressed_image, "bgr8")

        for detection in self.detections.detections:
            detection: Detection2D  # type hint
            cx = detection.bbox.center.position.x
            cy = detection.bbox.center.position.y
            sx = detection.bbox.size_x
            sy = detection.bbox.size_y

            min_pt = (round(cx - sx / 2.0), round(cy - sy / 2.0))
            max_pt = (round(cx + sx / 2.0), round(cy + sy / 2.0))

            cv2.rectangle(origin_image, min_pt, max_pt, self.COLOR, self.THICKNESS)

            hypo: ObjectHypothesisWithPose  # type hint
            hypo = detection.results[0]
            label = "{} {:.3f}".format(
                CLASSES[int(hypo.hypothesis.class_id)], hypo.hypothesis.score
            )
            pos = (min_pt[0] + self.THICKNESS, max_pt[1] - self.THICKNESS - 1)
            cv2.putText(
                origin_image, label, pos, self.FONT, 0.75, self.COLOR, 1, cv2.LINE_AA
            )

        # 미리보기 화면 표시
        if self.preview:
            cv2.imshow("Processed Image", origin_image)
            cv2.waitKey(1)

        processed_image_msg = self.br.cv2_to_imgmsg(origin_image, "bgr8")
        processed_image_msg.header = compressed_image.header

        processed_image_compressed_msg = self.br.cv2_to_compressed_imgmsg(
            origin_image, "jpeg"
        )
        processed_image_compressed_msg.header = compressed_image.header

        # processed_image Publish
        self.processed_image_publisher.publish(processed_image_msg)
        self.processed_image_compressed_publisher.publish(
            processed_image_compressed_msg
        )
    def gps_callback(self, msg: NavSatFix):
        """
        GPS 데이터를 받아 위도, 경도를 기준으로 x, y 좌표를 계산.
        초기 위치는 위도와 경도가 모두 0이 아니면 설정됨.
        """
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.altitude = msg.altitude

        if self.latitude == 0.0 or self.longitude == 0.0:
            self.get_logger().info("Invalid GPS signal received. Waiting for valid data.")
            return

        if not self.initialized and self.latitude != 0.0 and self.longitude != 0.0:
            self.origin_lat = self.latitude
            self.origin_lon = self.longitude
            self.origin_alt = self.altitude
            self.origin_ecef = self.latlon_to_ecef(self.origin_lat, self.origin_lon, self.origin_alt)
            self.initialized = True
            self.get_logger().info("Initial position set as origin (0, 0)")

        try:
            ecef_coords = self.latlon_to_ecef(self.latitude, self.longitude, self.altitude)
            self.x, self.y = self.ecef_to_local(ecef_coords, self.origin_ecef, self.origin_lat, self.origin_lon)
            self.get_logger().info(f"GPS Fix: x = {self.x:.3f} m, y = {self.y:.3f} m")

        except Exception as e:
            self.get_logger().error(f"Error during GPS conversion: {e}")         
        # 최근 데이터 저장
        self.recent_x.append(self.x)
        self.recent_y.append(self.y)
        
        # 최근 데이터의 합산 출력
        self.mean_x = sum(self.recent_x)/30
        self.mean_y = sum(self.recent_y)/30
        angle_deg = -23.0  # 시계 방향 30도 회전
        angle_rad = radians(angle_deg)
        cos_theta = cos(angle_rad)
        sin_theta = sin(angle_rad)
        
        self.modified_x = cos_theta * self.mean_x + sin_theta * self.mean_y
        self.modified_y = -sin_theta * self.mean_x + cos_theta * self.mean_y 
        
           
    def latlon_to_ecef(self, lat, lon, alt):
        """
        위도(lat), 경도(lon), 고도(alt)를 ECEF 좌표로 변환.
        """
        lat_rad = radians(lat)
        lon_rad = radians(lon)

        N = self.a / sqrt(1 - self.e2 * sin(lat_rad) ** 2)

        x = (N + alt) * cos(lat_rad) * cos(lon_rad)
        y = (N + alt) * cos(lat_rad) * sin(lon_rad)
        z = (N * (1 - self.e2) + alt) * sin(lat_rad)

        return x, y, z

    def ecef_to_local(self, ecef_coords, origin_ecef, origin_lat, origin_lon):
        """
        ECEF 좌표를 기준점(Local Tangent Plane)으로 변환.
        """
        origin_lat_rad = radians(origin_lat)
        origin_lon_rad = radians(origin_lon)

        dx = ecef_coords[0] - origin_ecef[0]
        dy = ecef_coords[1] - origin_ecef[1]
        dz = ecef_coords[2] - origin_ecef[2]

        sin_lat = sin(origin_lat_rad)
        cos_lat = cos(origin_lat_rad)
        sin_lon = sin(origin_lon_rad)
        cos_lon = cos(origin_lon_rad)

        t_matrix = [
            [-sin_lon, cos_lon, 0],
            [-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat],
            [cos_lat * cos_lon, cos_lat * sin_lon, sin_lat]
        ]

        x_local = t_matrix[0][0] * dx + t_matrix[0][1] * dy + t_matrix[0][2] * dz
        y_local = t_matrix[1][0] * dx + t_matrix[1][1] * dy + t_matrix[1][2] * dz

        return x_local, y_local
    def detection_callback(self, detections_msg: Detection2DArray) -> None:
        # self.detections 변수 업데이트
        self.detections = detections_msg

    # 이미지 Subscription callback
    def image_callback(self, compressed_image: CompressedImage) -> None:
        # compressed image를 OpenCV 이미지로 변환
        origin_image = self.br.compressed_imgmsg_to_cv2(compressed_image, "bgr8")

        for detection in self.detections.detections:
            detection: Detection2D  # type hint
            cx = detection.bbox.center.position.x
            cy = detection.bbox.center.position.y
            sx = detection.bbox.size_x
            sy = detection.bbox.size_y

            min_pt = (round(cx - sx / 2.0), round(cy - sy / 2.0))
            max_pt = (round(cx + sx / 2.0), round(cy + sy / 2.0))

            cv2.rectangle(origin_image, min_pt, max_pt, self.COLOR, self.THICKNESS)

            hypo: ObjectHypothesisWithPose  # type hint
            hypo = detection.results[0]
            label = "{} {:.3f}".format(
                CLASSES[int(hypo.hypothesis.class_id)], hypo.hypothesis.score
            )
            pos = (min_pt[0] + self.THICKNESS, max_pt[1] - self.THICKNESS - 1)
            cv2.putText(
                origin_image, label, pos, self.FONT, 0.75, self.COLOR, 1, cv2.LINE_AA
            )

        # 미리보기 화면 표시
        if self.preview:
            cv2.imshow("Processed Image", origin_image)
            cv2.waitKey(1)

        processed_image_msg = self.br.cv2_to_imgmsg(origin_image, "bgr8")
        processed_image_msg.header = compressed_image.header

        processed_image_compressed_msg = self.br.cv2_to_compressed_imgmsg(
            origin_image, "jpeg"
        )
        processed_image_compressed_msg.header = compressed_image.header

        # processed_image Publish
        self.processed_image_publisher.publish(processed_image_msg)
        self.processed_image_compressed_publisher.publish(
            processed_image_compressed_msg
        )

def main(args=None):
    rclpy.init(args=args)
    node = LidarPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
