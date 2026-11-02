###2023년 8월 15일 수정본###
# get heading using ebimu
# ros2 foxy

import time
import struct
from serial import Serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from math import sin, cos
import math
from rclpy.qos import qos_profile_sensor_data, QoSProfile


class ImuNode(Node):

    def __init__(self):
        super().__init__('imu_driver')

        self.imu_data_publisher = self.create_publisher(Imu, 'imu', qos_profile_sensor_data) # declare publisher
        self.imu_heading_publisher = self.create_publisher(Float32, 'imu_heading', qos_profile_sensor_data)

        # Get parameters
        self.declare_parameter('port', '/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_0001-if00-port0')
        self.declare_parameter('IMU_ID', '106-1')

        # Get parameter values
        self.port = self.get_parameter('port').value
        self.IMU_ID = self.get_parameter('IMU_ID').value

        self.ser = Serial(self.port, 921600)


        pub_rate = 100  #100hz

        self.lg = self.get_logger() # logger
        
        self.okay = 1
        self.accel_factor = 9.80665 #중력 가속도
        self.imu_msg = Imu()
        self.imu_heading_msg = Float32()
        # self.imu_parameter()
        self.timer = self.create_timer(1/pub_rate, self.publish_imu_data)  # 50Hz로 타이머 생성


    def publish_imu_data(self):

        if self.okay == 1:
            # Read data from the serial port
            data = self.ser.readline().decode().strip().split(',')
            
            if len(data) == 11 and data[0] == self.IMU_ID:

                print(data)
                self.imu_msg.header.stamp = self.get_clock().now().to_msg()
                self.imu_msg.header.frame_id = 'imu_frame'
                
                quaternion = self.euler_to_quaternion(float(data[1]), float(data[2]), float(data[3]))

                self.imu_msg.orientation.x = quaternion[0] * math.pi/180
                self.imu_msg.orientation.y = quaternion[1] * math.pi/180
                self.imu_msg.orientation.z = quaternion[2] * math.pi/180
                self.imu_msg.orientation.w = quaternion[3] * math.pi/180
                self.imu_msg.angular_velocity.x = float(data[4])
                self.imu_msg.angular_velocity.y = float(data[5])
                self.imu_msg.angular_velocity.z = float(data[6])
                self.imu_msg.linear_acceleration.x = float(data[7]) * self.accel_factor
                self.imu_msg.linear_acceleration.y = float(data[8]) * self.accel_factor
                self.imu_msg.linear_acceleration.z = float(data[9]) * self.accel_factor
                
                self.imu_heading_msg.data = float(data[3])
                self.imu_heading_publisher.publish(self.imu_heading_msg)
                self.imu_data_publisher.publish(self.imu_msg)
                # print("yaw : ", data[3])


    def imu_parameter(self):
        self.lg.info('trying to connect IMU')
        # imu_parameter = ['<??sem2>', '<??ssg5>','<??lpfg3>','<??avcg_e1>','<??avca_e1>','<??cg>','<??cas>']
        imu_parameter = ['<??cg>','<??cas>','<??cmo>']

        # '<??sem2>' 능동적인 자기장 왜곡방지 기능 활성화 ( 0: off, 1: 0n)
        # '<??ssg5>' 자이로센서의 감도 설정 1~5 (5: 2000dps, default)
        # '<??lpfg3>' 자이로 센서의 Digital Low Pass Filter 0~9 ->진동 환경에서 정밀한 데이터를 출력 가능 (3 : 97Hz default)
        # '<??avcg_e1>' AVC 자이로센서 진동 제거 능력, 진동이 심한 환경에서 활성화, ( 0:default, 1:on )
        # '<??avca_e1>' AVC 가속도센서 진동 제거 능력, 진동이 심한 환경에서 활성화 ( 0:default, 1:on )
        # '<??cg>' 자이로센서 xyz축 캘리브레이션. 
        # '<??cas>' 가속도센서 xyz축 캘리브레이션. 지표면과 수평이 되도록 유지한 상태에서 실행.
        # '<??cmoxy>' roll, pitch축 offset설정. 현재 roll, pitch값을 원점으로 0으로 설정
        # '<??cmo>' roll, pitch, yaw축 offset설정. 현재 roll, pitch, yaw값을 원점으로 0으로 설정 (가끔 오류가 발생해서 초기 yaw가 160, -150 이런식으로 발생하는 경우가 있으니 그럴 때에 다시 실행해주기)

        okay_count = 0
        for i in range(len(imu_parameter)):
            self.ser.write(imu_parameter[i].encode())
            self.okay = self.ser.readline().decode().strip()[0:4]
            
            while self.okay != '<ok>':
                time.sleep(0.1)
                self.okay = self.ser.readline().decode().strip()[0:4]
                print(okay_count, " / ", len(imu_parameter))
            

            okay_count += 1
            print(okay_count, " / ", len(imu_parameter))
            self.okay = 0
        
        
        if okay_count == len(imu_parameter):
            print(okay_count, " / ", len(imu_parameter))
            self.lg.info('Parameter setting success')
            self.okay = 1
        else :
            self.lg.info('Parameter setting faild')
            rclpy.shutdown()


    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        오일러 각도를 쿼터니언으로 변환하는 함수
        :param roll: Roll 각도 (radians)
        :param pitch: Pitch 각도 (radians)
        :param yaw: Yaw 각도 (radians)
        :return: 변환된 쿼터니언 (x, y, z, w)
        """
        
        cy = cos(yaw * 0.5)
        sy = sin(yaw * 0.5)
        cp = cos(pitch * 0.5)
        sp = sin(pitch * 0.5)
        cr = cos(roll * 0.5)
        sr = sin(roll * 0.5)

        qw = cy * cp * cr + sy * sp * sr
        qx = cy * cp * sr - sy * sp * cr
        qy = sy * cp * sr + cy * sp * cr
        qz = sy * cp * cr - cy * sp * sr

        return qx, qy, qz, qw


def main(args=None):
    rclpy.init(args=args)
    imu_node = ImuNode()
    rclpy.spin(imu_node)
    imu_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
