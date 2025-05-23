import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import numpy as np

class GPSMeasurement(Node):
    def __init__(self):
        super().__init__('gps_measurement')
        self.lat_values = []
        self.lon_values = []
        self.measurement_count = 60  # 60초 동안 측정
        
        self.gps_subscriber = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.gps_callback,
            10
        )
        
    def gps_callback(self, msg):
        if len(self.lat_values) < self.measurement_count:
            self.lat_values.append(msg.latitude)
            self.lon_values.append(msg.longitude)
        else:
            avg_lat = np.mean(self.lat_values)
            avg_lon = np.mean(self.lon_values)
            print(f"\n평균 GPS 좌표:\n위도: {avg_lat:.8f}\n경도: {avg_lon:.8f}")
            self.destroy_node()
            rclpy.shutdown()

def main():
    rclpy.init()
    node = GPSMeasurement()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

# 4. 평균 GPS 좌표:
#위도: 35.19185261
#경도: 129.10041128

# 1. 평균 GPS 좌표:
#위도: 35.19169678
#경도: 129.10035789

# 2. 평균 GPS 좌표:
#위도: 35.19169676
#경도: 129.10065614

#3 . 평균 GPS 좌표:
#위도: 35.19173032
#경도: 129.10061323

# 3p l
# 3p m
# 3p r