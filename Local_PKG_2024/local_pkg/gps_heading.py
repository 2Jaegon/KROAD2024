###2023년 8월 15일 수정본###
# get heading using gps

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from sensor_msgs.msg import NavSatFix
from utm import from_latlon
import matplotlib.pyplot as plt
from std_msgs.msg import Float64
from std_msgs.msg import Float32
import math


class GpsImuHeading(Node):

    def __init__(self):
        super().__init__('gps_heading')
        qos_profile = QoSProfile(depth=10)

        self.gps_subscriber = self.create_subscription(NavSatFix, '/fix', self.subscribe_gps_message, qos_profile_sensor_data)
        
        self.gps_heading_publisher = self.create_publisher(Float64, 'Local/heading', qos_profile_sensor_data)
        self.make_heading = self.create_timer(1/20, self.make_gps_heading)
        self.gps_distance = 0.03 # gps 이전 점과 현재 점 사이 간격을 어느정도 할것인지. (m단위)
        self.gps_array_index = 20 # gps 점을 몇개까지 저장할건지
        self.make_heading_index = 2 # gps heading을 구할 때, 이전 몇 번째의 좌표를 활용할건지
        self.first_gps = True

        self.current_x = -1.0
        self.current_y = -1.0
        self.gps_heading = -1.0
        self.gps_heading_ = -1.0
        self.delta_heading = -1.0

        self.x_array = []
        self.y_array = []
        

    def subscribe_gps_message(self, msg): ##gps data를 subscribe받아 x_array, y_array에 append
        lat = msg.latitude
        lon = msg.longitude
        alt = msg.altitude
        utm_coords = from_latlon(lat, lon)
        easting = utm_coords[0]
        northing = utm_coords[1]
        distance = math.sqrt(math.pow(northing-self.current_y,2)+math.pow(easting-self.current_x,2))

        if self.first_gps == True:
            self.current_x = easting
            self.current_y = northing
            self.first_gps = False

        elif (self.current_x != easting and self.current_y != northing) and (distance >= self.gps_distance):
            
            if(len(self.x_array) >= self.gps_array_index):
                self.x_array.pop(0)
                self.y_array.pop(0)

            self.x_array.append(easting)
            self.y_array.append(northing)

        # elif(distance < self.gps_distance):
        #     print("ERP is at rest")

        self.current_x = easting
        self.current_y = northing


    def make_gps_heading(self):
        if(len(self.x_array) >= self.make_heading_index):
            heading = math.atan2(self.y_array[-1]-self.y_array[-self.make_heading_index],self.x_array[-1]-self.x_array[-self.make_heading_index])
            gps_heading_msg = Float64()
            gps_heading_msg.data = heading
            self.gps_heading_publisher.publish(gps_heading_msg)


def main(args=None):

    rclpy.init(args=args)
    heading_node = GpsImuHeading()
    rclpy.spin(heading_node)
    heading_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
