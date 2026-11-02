###2023.09.07###
###ShinHyeonhak###
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64, Float32, Float32MultiArray
import numpy as np
import pyproj

class RealHeadingPublisher(Node):
    def __init__(self):
        super().__init__('utm')
        self.publisher = self.create_publisher(Float64, '/Local/realheading', qos_profile_sensor_data)
        self.gps1subscription = self.create_subscription(NavSatFix, '/gps1/fix', self.gps1_callback, qos_profile_sensor_data)
        self.gps2subscription = self.create_subscription(NavSatFix, '/gps2/fix', self.gps2_callback, qos_profile_sensor_data)
        self.gps1_lat = np.nan
        self.gps1_lon = np.nan
        self.gps2_lat = np.nan
        self.gps2_lon = np.nan
        self.transformer = pyproj.Transformer.from_crs("EPSG:4326", "EPSG:32652", always_xy=True)
        
    def gps1_callback(self, msg):
        # NavSatFix 데이터를 읽어옴
        if msg.latitude != np.nan and msg.longitude !=np.nan:
            self.gps1_lat = msg.latitude
            self.gps1_lon = msg.longitude
            
        else :
            self.get_logger().warn('GPS1 is Not Working')
        
        if self.gps2_lat != np.nan and self.gps2_lon !=np.nan:
            x0, y0 = self.transformer.transform(self.gps1_lon,self.gps1_lat)
            x, y = self.transformer.transform(self.gps2_lon,self.gps2_lat)
            # x0,y0,z0,t0=pyproj.transform(pyproj.Proj(init='epsg:4326'), pyproj.Proj(init='epsg:32652'), self.gps1_lon,self.gps1_lat)
        #"EPSG:4326"과 "EPSG:32652"는 각각 WGS84(세계 지리 정보 시스템 기준)와 UTM Zone 52N 좌표계를 의미합니다.
            # x,y,z,t=pyproj.transform(pyproj.Proj(init='epsg:4326'), pyproj.Proj(init='epsg:32652'), self.gps2_lon, self.gps2_lat )
            heading_rad = np.arctan2(y-y0, x-x0)
            msg_heading=Float64()
            msg_heading.data=heading_rad
            
            # 헤딩값(라디안 단위) 게시.
            self.publisher.publish(msg_heading)
            self.get_logger().info(f'👍👍 Real_heading = {msg_heading}👍👍')

    def gps2_callback(self, msg):
        # NavSatFix 데이터를 읽어옴
        if msg.latitude != np.nan and msg.longitude !=np.nan:
            self.gps2_lat = msg.latitude
            self.gps2_lon = msg.longitude
        else :
            self.get_logger().warn('GPS2 is Not Working')

    # def calculate_heading(lat_0, long_0, lat_1, long_1) :
    #     x0,y0,z0,t0=pyproj.transform(pyproj.Proj(init='epsg:4326'), pyproj.Proj(init='epsg:32652'), long_0,lat_0 )
    #     #"EPSG:4326"과 "EPSG:32652"는 각각 WGS84(세계 지리 정보 시스템 기준)와 UTM Zone 52N 좌표계를 의미합니다.
    #     x,y,z,t=pyproj.transform(pyproj.Proj(init='epsg:4326'), pyproj.Proj(init='epsg:32652'), long_1,lat_1 )
    #     heading = np.arctan2(y-y0, x-x0)
    #     return heading
            

def main(args=None):
    rclpy.init(args=args)
    RealHeading_publisher_node = RealHeadingPublisher()
    rclpy.spin(RealHeading_publisher_node)
    RealHeading_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

