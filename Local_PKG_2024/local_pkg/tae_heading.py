###2023년 8월 21일 수정본###
# control serial bridge 코드 수정으로 인한 수정.
# get heading using gps and imu

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from sensor_msgs.msg import NavSatFix, Imu
from utm import from_latlon
from std_msgs.msg import Float64, Float32, Float32MultiArray
import math
import numpy as np
import time
from scipy.signal import butter, lfilter_zi, lfilter

class GpsImuHeading(Node):

    def __init__(self):
        super().__init__('tae_heading')

        #--------------------------------------[ 설정 파라미터 ]------------------------------------------
        self.declare_parameter('imu_version', 1) #[0: 원래 있던 IMU], [1: 새로 산 IMU]
        self.imu_version = self.get_parameter('imu_version').get_parameter_value().integer_value
        self.get_logger().info(f"IMU Version is {self.imu_version}")

        self.effective_velocity = 5 # gps헤딩 유효값 속도[km/h] (해당 속도부터 GPS로 imu값 보정)
        self.gps_heading_max_velocity = 25 # gps heading가 매우 유효한 속도[km/h] (해당 속도부터 GPS헤딩만 사용)
        self.gps_hz = 10.0 # GPS subscribe 속도[HZ]
        self.imu_hz = 100.0 # IMU subscribe 속도[HZ]
        self.odom_hz = 40.0 # erp serial odometry 계산 텀
        self.odom_gps_error = 0.05 # erp odometry와 gps의 거리 에러값으로 gps 신뢰성 판단
        self.cutoff_freq = 5  # low pass filter 사용시 저역 통과 필터의 절단 주파수 설정 (Hz) [클 수록 높은 주파수도 통과]

        #--------------------------------------[ Pub and Sub ]-----------------------------------------
        self.gps_subscriber = self.create_subscription(NavSatFix, '/fix', self.gps_callback, qos_profile_sensor_data)
        self.erp_serial_subscriber = self.create_subscription(Float32MultiArray, '/ERP/serial_data', self.erp_serial_callback, qos_profile_sensor_data)

        if self.imu_version == 0:
            self.imu_subscriber = self.create_subscription(Float32, '/imu_heading', self.imu_callback, qos_profile_sensor_data)
        elif self.imu_version == 1:
            self.subscription = self.create_subscription(Imu,'/imu',self.imu_callback, qos_profile_sensor_data)

        self.heading_publisher = self.create_publisher(Float64, 'Local/heading', qos_profile_sensor_data)        

        #--------------------------------- [ Get serial Odometry ] ---------------------------------------
        self.timer1 = self.create_timer(1/self.odom_hz, self.get_odometry)

        #--------------------------------- [ Check input Data  ] ---------------------------------------
        self.timer2 = self.create_timer(3.0, self.check_input_data)

        #--------------------------------------[ 멤버 변수 선언 ] ---------------------------------------
        self.first_run = True
        self.gps_on = False
        self.imu_on = False
        self.erp_serial_on = False

        self.corrections_imu_with_gps = False
        self.first_corrections = True

        self.first_odometry = True
        self.odometry_x = None
        self.odometry_y = None
        
        self.first_gps = False
        self.gps_x_ = None
        self.gps_y_ = None
        self.prev_time = None

        self.gps_heading = None
        self.imu_heading = None
        self.imu_data_ = None
        self.imu_input_error_value = 10.0 # imu 이전값과 현재값 차이가 10도 이상 난다면 에러로 파악
        self.low_pass_filter_heading = None
        self.final_heading = None
        self.gps_effectiveness = 0.0


        self.gear = 0.0
        self.serial_velocity = 0.0
        self.car_steering = 0.0
        self.wheel_base = 1.04
        self.track_width = 0.985

        self.imu_input_heading = None
        self.imu_input_heading_ = None
        self.imu_input_error_data = 0.0

        self.correction_value = 0.0 # gps으로 imu보정값
        self.imu_up_and_down = 0
        self.correction_count = 0
        
        nyquist_freq = 0.5 * self.imu_hz
        normalized_cutoff = self.cutoff_freq / nyquist_freq
        self.b, self.a = butter(1, normalized_cutoff, btype='low', analog=False)
        self.zi = None

    def check_input_data(self):
        if self.erp_serial_on == False:
            self.get_logger().warning('ERP Serial data not entered')
        if self.gps_on == False:
            self.get_logger().warning('Gps data not entered')
        if self.imu_on == False:
            self.get_logger().warning('Imu data not entered')
        if self.erp_serial_on and self.gps_on and self.imu_on and self.first_run:
            permissible_error = math.atan2(0.02,(self.effective_velocity/3.6/self.gps_hz))*180/math.pi
            self.get_logger().info(f"🚨 permissible error for {self.effective_velocity}km/h : {permissible_error}° 🚨")
            self.first_run = False

        self.erp_serial_on = False
        self.gps_on = False
        self.imu_on = False


    def erp_serial_callback(self, msg):
        self.erp_serial_on = True
        self.gear = msg.data[2]
        left_velocity = msg.data[3]
        left_steering = msg.data[4]
        self.serial_velocity = self.get_vehicle_center_velocity(left_velocity, left_steering)


    def imu_callback(self, msg):
        self.imu_on = True

        if self.gps_heading != None:

            input_msg = msg
            imu_resize_data = self.imu_input_data_resize(input_msg)

            self.correction_value = self.get_correction_value(imu_resize_data)
            imu_correction_data = imu_resize_data + self.correction_value
            
            self.imu_heading = imu_correction_data

            if self.zi is None:
                self.zi = lfilter_zi(self.b, self.a) * self.imu_heading

            # 필터링 적용
            self.low_pass_filter_heading, self.zi = lfilter(self.b, self.a, [self.imu_heading], zi=self.zi)


        if(self.low_pass_filter_heading != None):

            heading_resize = self.low_pass_filter_heading[0]

            while (heading_resize < -180 or heading_resize > 180):
                if heading_resize >= 180:
                    heading_resize -= 360
                if heading_resize < -180:
                    heading_resize += 360

            self.final_heading = heading_resize * math.pi / 180

            heading_msg = Float64()
            heading_msg.data =  self.final_heading
            self.heading_publisher.publish(heading_msg)


    def gps_callback(self, msg):
        lat = msg.latitude
        lon = msg.longitude
        alt = msg.altitude

        if np.isnan(lat) or np.isnan(lon) or np.isnan(alt):
            return
        
        self.gps_on = True
        utm_coords = from_latlon(lat, lon)
        self.gps_x = utm_coords[0]
        self.gps_y = utm_coords[1]

        distance_error = self.get_gps_odom_distance()

        self.odometry_x = self.gps_x
        self.odometry_y = self.gps_y

        if self.first_gps == False: # 처음 gps data처리
            self.first_gps = True
        elif(abs(self.serial_velocity) >= self.effective_velocity/3.6 and distance_error < self.odom_gps_error):
            self.correction()
            self.corrections_imu_with_gps = True
        else:
            self.corrections_imu_with_gps = False

        self.gps_x_ = self.gps_x
        self.gps_y_ = self.gps_y


    def get_odometry(self):

        if self.serial_velocity != 0.0 and self.final_heading is not None:
            current_time = time.time()
            if self.first_odometry == True:
                self.prev_time = current_time
                self.first_odometry = False

            elapsed_time = current_time - self.prev_time
            self.prev_time = current_time
            
            delta_x = self.serial_velocity * math.cos(self.final_heading) * elapsed_time
            delta_y = self.serial_velocity * math.sin(self.final_heading) * elapsed_time

            self.odometry_x += delta_x
            self.odometry_y += delta_y


    def get_vehicle_center_velocity(self, left_velocity, left_steering):

        if abs(left_steering) < 1e-5:  # steering angle이 0인 경우
            # print("직진")
            Vc = left_velocity
            self.car_steering = left_steering
        else:

            if left_steering < 0: # 우회전
                alpha = abs(left_steering)
                if (self.wheel_base/math.tan(alpha)-self.track_width) > 1e-4:
                    beta = abs(math.atan(self.wheel_base/(self.wheel_base/math.tan(alpha)-self.track_width)))
                else:
                    return
                # print("우회전")
                R_out = self.wheel_base/math.sin(alpha)
                R_in = self.wheel_base/math.sin(beta)
                
                v_out = left_velocity
                v_in = R_in/R_out*v_out

                Vc = (v_out + v_in)/2.0
            else:
                beta = abs(left_steering)
                if (self.wheel_base/math.tan(beta)+self.track_width) > 1e-4:
                    alpha = abs(math.atan(self.wheel_base/(self.wheel_base/math.tan(beta)+self.track_width)))
                else:
                    return
                # print("좌회전")
                R_out = self.wheel_base/math.sin(alpha)
                R_in = self.wheel_base/math.sin(beta)
                
                v_in = left_velocity
                v_out = R_out/R_in*v_in

                Vc = (v_out + v_in)/2.0
            self.car_steering = (alpha+beta)/2.0
        
        return Vc        


    def get_gps_odom_distance(self):

        if self.odometry_x is not None and self.first_odometry == False:
            odom_distance = math.sqrt(math.pow(self.gps_x_-self.odometry_x,2)+(math.pow(self.gps_y_-self.odometry_y,2)))
            gps_distance = math.sqrt(math.pow(self.gps_x_-self.gps_x,2)+(math.pow(self.gps_y_-self.gps_y,2)))

            distance_error = abs(odom_distance - gps_distance)
        else:
            distance_error = 0.0

        return distance_error


    def find_imu_input_error(self, input_heading, input_heading_):
        
        if input_heading != None and input_heading_ != None:
            error = input_heading-input_heading_
            if abs(error) > self.imu_input_error_value and abs(error) < 360 - self.imu_input_error_value:
                self.imu_input_error_data = self.imu_input_error_data + error
                self.get_logger().error(f"imput imu data ERROR {error}")
            real_heading = input_heading - self.imu_input_error_data

            if real_heading <= -180:
                real_heading += 360
            elif real_heading >= 180:
                real_heading -= 360
        else:
            real_heading = input_heading

        return real_heading


    def imu_input_data_resize(self, input_msg):
        if self.imu_version == 0:
            self.imu_input_heading = input_msg.data
            imu_data = self.find_imu_input_error(self.imu_input_heading, self.imu_input_heading_) # input된 imu data error 파악
            self.imu_input_heading_ = self.imu_input_heading
            imu_data = -imu_data #gps와 같은 좌표계 사용

        elif self.imu_version == 1:
            q_x, q_y, q_z, q_w = input_msg.orientation.x, input_msg.orientation.y, input_msg.orientation.z, input_msg.orientation.w
            self.imu_input_heading = self.quaternion_to_euler(q_x, q_y, q_z, q_w)*180/math.pi
            imu_data = self.find_imu_input_error(self.imu_input_heading, self.imu_input_heading_) # input된 imu data error 파악
            self.imu_input_heading_ = self.imu_input_heading

        if imu_data < 0:
            imu_data = imu_data + 360 # imu 값 범위를 0~360으로 설정
        elif imu_data > 360:
            imu_data = imu_data - 360

        if self.imu_data_ != None:
            heading_difference = imu_data - self.imu_data_
            
            if heading_difference < -350:
                self.imu_up_and_down += 1
            elif heading_difference > 350:
                self.imu_up_and_down -= 1
        
        self.imu_data_ = imu_data
        imu_data = imu_data + 360 * self.imu_up_and_down

        return imu_data


    def get_correction_value(self, imu_data):
        if self.corrections_imu_with_gps == True:
            self.correction_count +=1
            # self.get_logger().info(f'corrections imu with gps {self.correction_count}: ')
            if self.first_corrections == True:
                self.get_logger().info('👍👍 corrections imu with gps 👍👍')
                self.first_corrections = False
                self.correction_value = self.gps_heading - imu_data
                self.corrections_imu_with_gps = False

            else:
                self.correction_value = self.gps_heading - imu_data - (self.gps_heading - self.imu_heading)*(1-self.gps_effectiveness)
                self.corrections_imu_with_gps = False

        return self.correction_value


    def correction(self):
        effectiveness = self.serial_velocity/self.gps_heading_max_velocity
        if effectiveness >= 1.0:
            effectiveness = 1.0
        self.gps_effectiveness = effectiveness

        heading = math.atan2(self.gps_y-self.gps_y_, self.gps_x-self.gps_x_)*180/math.pi
        
        if self.gear == 2.0:
            # self.get_logger().info('back')
            if heading > 0:
                heading = heading - 180
            else:
                heading = heading + 180
        else:
            heading = heading

        if heading < 0:
            heading = heading + 360

        if self.first_corrections == True:
            self.gps_heading = heading
        else:
            
            updown = 0
            imu_resize = self.imu_heading

            while (imu_resize < 0 or imu_resize >= 360):
                if imu_resize >= 360:
                    imu_resize -= 360
                    updown += 1
                if imu_resize < 0:
                    imu_resize += 360
                    updown -= 1
            
            heading = heading + 360*updown
            
            if self.imu_heading - heading > 180:
                heading += 360
                
            elif heading - self.imu_heading > 180:
                heading -= 360

            self.gps_heading = heading
            # print("correct")


    def quaternion_to_euler(self, qx, qy, qz, qw):

        t3 = 2.0 * (qw * qz + qx * qy)
        t4 = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(t3, t4)

        return yaw



def main(args=None):

    rclpy.init(args=args)
    heading_node = GpsImuHeading()
    rclpy.spin(heading_node)
    heading_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
