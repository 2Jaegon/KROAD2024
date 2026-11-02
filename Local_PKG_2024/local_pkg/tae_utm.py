import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, Transform
import tf2_ros
import math
from math import *
import matplotlib.pyplot as plt
import threading
from std_msgs.msg import Float32, Float64, Float32MultiArray
from utm import from_latlon
import time
from geometry_msgs.msg import PointStamped
import numpy as np


class ImuOdometryNode(Node):

    def __init__(self):
        super().__init__('tae_utm')

        control_qos_profile = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST,
                            depth=1,
                            reliability=QoSReliabilityPolicy.BEST_EFFORT)
        
        #--------------------------------------[ 설정 파라미터 ]------------------------------------------
        self.plot_on = True # plot을 띄울것인가?
        self.imu_hz = 100.0 # IMU subscribe 속도[HZ]
        self.publish_hz = 100.0 # publish 속도[HZ]
        self.array_index = 300 #그래프에 표시할 odom점 갯수
        self.gps_array_index = 10 #그래프에 표시할 fix utm점 갯수

        self.steering_filter_k = 0.5 # steering low pass filter 계수
        self.ay_filter_k = 0.5 # Ay low pass filter 계수
        self.ax_filter_k = 0.5 # Ax low pass filter 계수
        self.beta_filter_k = 0.7 # beta low pass filter 계수

        self.car_w = 0.985 # 차의 윤거
        self.car_l = 1.04 # 차의 축거

        self.imu_calibration_count = 300
        
        #--------------------------------------[ pub and sub ]------------------------------------------        
        self.subscription = self.create_subscription(Imu, 'imu', self.imu_callback, qos_profile_sensor_data)
        self.gps_subscriber = self.create_subscription(NavSatFix, '/fix', self.gps_callback, qos_profile_sensor_data)
        self.erp_serial_subscriber = self.create_subscription(Float32MultiArray, '/ERP/serial_data', self.erp_serial_callback, control_qos_profile)
        self.utm_publisher = self.create_publisher(PointStamped, '/Local/utm', qos_profile_sensor_data)
        self.beta_publisher = self.create_publisher(Float64,'/beta', qos_profile_sensor_data)

        self.array_timer = self.create_timer(1/20, self.array_timer_callback)
        self.publish_timer = self.create_timer(1/self.publish_hz, self.timer_callback)


        #--------------------------------------[ 멤버 변수 선언 ] ---------------------------------------
        self.steering = None
        self.steering_ = None

        self.current_x = None
        self.current_y = None
        self.current_x_ = None
        self.current_y_ = None
        self.gps_x = None
        self.gps_y = None

        self.gps_x_array = []
        self.gps_y_array = []
        self.x_array = []
        self.y_array = []

        self.left_velocity = None
        self.serial_velocity = None
        self.yaw_rate_velocity = None

        self.left_velocity_array = []
        self.yaw_rate_velocity_array = []
        self.gps_error_array = []
        self.tae_error_array = []

        self.yaw_rate = None
        self.ax = None
        self.ay = None
        self.az = None

        self.yaw_rate_array = []
        self.ax_array = []
        self.ay_array = []
        self.az_array = []

        self.beta = None
        self.beta_ = None
        self.yaw_dot = None

        self.gps_error_distance = None
        self.tae_error_distance = None

        self.imu_callback_count = 0
        self.ax_sum = 0.0
        self.ay_sum = 0.0
        self.az_sum = 0.0

        self.roll_offset_deg = 0.0
        self.pitch_offset_deg = 0.0
        self.yaw_offset_deg = 0.0

    def calculate_offsets(self):
        # Calculate average acceleration values
        ax_avg = self.ax_sum / self.imu_calibration_count
        ay_avg = self.ay_sum / self.imu_calibration_count
        az_avg = self.az_sum / self.imu_calibration_count

        # Calculate roll and pitch offsets based on average acceleration values
        self.roll_offset_deg = -np.rad2deg(np.arctan2(ay_avg, -az_avg))
        self.pitch_offset_deg = -np.rad2deg(np.arctan2(-ax_avg, np.sqrt(ay_avg**2 + (-az_avg)**2)))

        self.get_logger().info("👍👍 IMU calibration is complete. You may move the ERP now 👍👍")


    def erp_serial_callback(self, msg):
        self.left_velocity = msg.data[3]
        steering = msg.data[4]

        if self.steering_ is None:
            self.steering_ = steering

        self.steering = steering*self.steering_filter_k + self.steering_*(1-self.steering_filter_k)
        self.steering_ = self.steering
        self.serial_velocity = self.get_vehicle_center_velocity(self.left_velocity, self.steering)

    def get_vehicle_center_velocity(self, left_velocity, steer):

        if abs(steer) < 1e-5:  # steering angle이 0인 경우
            Vc = left_velocity
        else:
            out_steer = atan2(self.car_l, self.car_l/tan(steer)+self.car_w/2)
            in_steer = atan2(self.car_l, self.car_l/tan(steer)-self.car_w/2)
            
            R_out = self.car_l/sin(out_steer)
            R_in = self.car_l/sin(in_steer)
            
            v_in = left_velocity
            v_out = R_out/R_in*v_in

            Vc = (v_out + v_in)/2.0
        
        return Vc


    def gps_callback(self,msg):

        lat = msg.latitude
        lon = msg.longitude
        utm_coords = from_latlon(lat, lon)

        if self.current_x_ is not None:
            self.tae_error_distance = sqrt(pow(self.current_x - utm_coords[0],2)+pow(self.current_y - utm_coords[1],2))

        self.current_x = utm_coords[0]
        self.current_y = utm_coords[1]
        self.gps_x = utm_coords[0]
        self.gps_y = utm_coords[1]

        if self.current_x_ is None:
            self.current_x_ = self.current_x
            self.current_y_ = self.current_y
        else:
            if self.beta is None:
                self.beta = atan2(self.current_y - self.current_y_, self.current_x - self.current_x_)
            else:
                input_beta = atan2(self.current_y - self.current_y_, self.current_x - self.current_x_)
                self.beta = input_beta

        self.gps_error_distance = sqrt(pow(self.gps_x - self.current_x_,2)+pow(self.gps_y - self.current_y_,2))

        self.current_x_ = self.current_x
        self.current_y_ = self.current_y

        if(len(self.gps_x_array) >= self.gps_array_index):
            self.gps_x_array.pop(0)
            self.gps_y_array.pop(0)
        self.gps_x_array.append(self.current_x)
        self.gps_y_array.append(self.current_y)


    def imu_callback(self, msg):

        dt = 1/self.imu_hz

        imu_data = msg
        self.yaw_rate = imu_data.angular_velocity.z
        
        ax = -msg.linear_acceleration.x
        ay = -msg.linear_acceleration.y
        az = msg.linear_acceleration.z


        self.imu_callback_count += 1
        self.ax_sum += ax
        self.ay_sum += ay
        self.az_sum += az
        if self.imu_callback_count == self.imu_calibration_count:
            self.calculate_offsets()

        # Rotation matrix for the offset angle (e.g., roll, pitch, yaw)
        roll_offset = np.deg2rad(self.roll_offset_deg)  # replace with actual value
        pitch_offset = np.deg2rad(self.pitch_offset_deg)  # replace with actual value
        yaw_offset = np.deg2rad(self.yaw_offset_deg)  # replace with actual value

        R_x = np.array([[1,         0,                  0                   ],
                        [0,         np.cos(roll_offset), -np.sin(roll_offset)],
                        [0,         np.sin(roll_offset), np.cos(roll_offset) ]
                        ])

        
        R_y= np.array([[np.cos(pitch_offset),    0,      np.sin(pitch_offset)],
                       [0,                      1,      0                   ], 
                       [-np.sin(pitch_offset),   0,      np.cos(pitch_offset)]
                       ])
                
        
        R_z= [[np.cos(yaw_offset),    -np.sin(yaw_offset),    0],
                [np.sin(yaw_offset),    	np.cos(yaw_offset) ,  	0],
                [    	0 ,               		0 ,               	  	1]
                ]

        # Rotation matrix
        R = R_z @ R_y @ R_x

        # Acceleration vector
        a = np.array([ax, ay, az])

        # Apply rotation to the acceleration vector
        a_rotated = R @ a

        self.az = a_rotated[2]

        if self.ax is not None:
            self.ax = self.ax_filter_k*a_rotated[0] + (1-self.ax_filter_k)*self.ax
        else:
            self.ax = a_rotated[0]
            
        if self.ay is not None:
            self.ay = self.ay_filter_k*a_rotated[1] + (1-self.ay_filter_k)*self.ay
        else:
            self.ay = a_rotated[1]

        if self.left_velocity is not None:
            r_left = self.left_velocity/abs(self.yaw_rate)

            if self.yaw_rate > 0:
                r_right = r_left + self.car_w
            else:
                r_right = r_left - self.car_w

            right_velocity = abs(self.yaw_rate)*r_right

            self.yaw_rate_velocity = (self.left_velocity + right_velocity)/2.0


        if self.yaw_rate_velocity is not None and self.current_x is not None and self.beta is not None:
            if self.yaw_rate_velocity < 0.1:
                pass
            else:
                beta_dot = self.ay/self.yaw_rate_velocity - self.yaw_rate
                predicted = self.beta + beta_dot*dt
                self.beta = self.beta_filter_k*predicted + (1-self.beta_filter_k)*self.beta
                
                self.yaw_dot = self.beta + self.closer_to_zero(abs(self.ay)*self.steering/10.0, self.steering/2.5)
                
                self.current_x = self.current_x + self.yaw_rate_velocity * cos(self.yaw_dot) *dt
                self.current_y = self.current_y + self.yaw_rate_velocity * sin(self.yaw_dot) *dt

        if self.current_x is not None:
            if(len(self.x_array) >= self.array_index):
                self.x_array.pop(0)
                self.y_array.pop(0)
            self.x_array.append(self.current_x)
            self.y_array.append(self.current_y)


    def array_timer_callback(self):
        
        if self.plot_on is True:
            if self.left_velocity is not None:
                if(len(self.left_velocity_array) >= self.array_index):
                    self.left_velocity_array.pop(0)
                    self.left_velocity_array.pop(0)
                self.left_velocity_array.append(self.left_velocity)

            if self.yaw_rate_velocity is not None:
                if(len(self.yaw_rate_velocity_array) >= self.array_index):
                    self.yaw_rate_velocity_array.pop(0)
                    self.yaw_rate_velocity_array.pop(0)
                self.yaw_rate_velocity_array.append(self.yaw_rate_velocity)
            
            if self.yaw_rate is not None:
                if(len(self.yaw_rate_array) >= self.array_index):
                    self.yaw_rate_array.pop(0)
                    self.yaw_rate_array.pop(0)
                self.yaw_rate_array.append(self.yaw_rate)

            if self.ax_array is not None:
                if(len(self.ax_array) >= self.array_index):
                    self.ax_array.pop(0)
                    self.ax_array.pop(0)
                self.ax_array.append(self.ax)

            if self.ay is not None:
                if(len(self.ay_array) >= self.array_index):
                    self.ay_array.pop(0)
                    self.ay_array.pop(0)
                self.ay_array.append(self.ay)
            
            if self.az is not None:
                if(len(self.az_array) >= self.array_index):
                    self.az_array.pop(0)
                    self.az_array.pop(0)
                self.az_array.append(self.az)
            
            if self.gps_error_distance is not None:
                if(len(self.gps_error_array) >= self.array_index):
                    self.gps_error_array.pop(0)
                    self.gps_error_array.pop(0)
                self.gps_error_array.append(self.gps_error_distance)

            if self.tae_error_distance is not None:
                if(len(self.tae_error_array) >= self.array_index):
                    self.tae_error_array.pop(0)
                    self.tae_error_array.pop(0)
                self.tae_error_array.append(self.tae_error_distance)


    def timer_callback(self):
        if self.current_x is not None:
            utm_point = PointStamped()
            utm_point.header.stamp = self.get_clock().now().to_msg()
            utm_point.header.frame_id = 'map'
            utm_point.point.x = self.current_x
            utm_point.point.y = self.current_y

            self.utm_publisher.publish(utm_point)

        if self.yaw_dot is not None:
            beta_msg = Float64()
            beta_msg.data = float(self.yaw_dot)
            self.beta_publisher.publish(beta_msg)

    def closer_to_zero(self, num1, num2):
        if abs(num1) < abs(num2):
            return num1
        else:
            return num2


def spin_node(node):
    rclpy.spin(node)

def main(args=None):


    rclpy.init(args=args)

    node = ImuOdometryNode()

    user_input = input("Do you want plot? (Y/N): ")
    node.plot_on = True if user_input.lower() == 'y' else False

    node.get_logger().info("🚨Please wait for a moment for IMU calibration.🚨")

    try:
        thread = threading.Thread(target=spin_node, args=(node,))
        thread.start()
        rate = node.create_rate(10)

        fig, (ax1,ax2,ax3,ax4) = plt.subplots(4, 1)

        if node.plot_on == True:
            while rclpy.ok():                

                ax1.cla()

                if(len(node.x_array) > 0):
                    ax1.plot(node.x_array, node.y_array, marker='o', linestyle='--', color='blue')
                    ax1.plot(node.gps_x_array, node.gps_y_array, marker='o', linestyle='', color='red')

                    ax1.set_xlabel('UTM X')
                    ax1.set_ylabel('UTM Y')
                    ax1.set_title('Plot odom UTM')


                ax2.cla()

                if len(node.left_velocity_array) > 0:
                    ax2.plot(range(len(node.left_velocity_array)), node.left_velocity_array, linestyle='-', color='blue', label='left_velocity')
                if len(node.yaw_rate_velocity_array) > 0:
                    ax2.plot(range(len(node.yaw_rate_velocity_array)), node.yaw_rate_velocity_array, linestyle='-', color='green', label='yawrate_velocity')
                
                if len(node.left_velocity_array) > 0 or len(node.yaw_rate_velocity_array) > 0:
                    ax2.set_xlabel('time')
                    ax2.set_ylabel('velocity')
                    ax2.set_title('Plot velocity')
                    ax2.legend(loc="upper left")
                    ax2.grid(True)


                ax3.cla()
                if len(node.yaw_rate_array) > 0:
                    ax3.plot(range(len(node.yaw_rate_array)), node.yaw_rate_array, linestyle='-', color='yellow', label='yaw_rate')
                if len(node.ax_array) > 0:
                    ax3.plot(range(len(node.ax_array)), node.ax_array, linestyle='-', color='green', label='ax')
                if len(node.ay_array) > 0:
                    ax3.plot(range(len(node.ay_array)), node.ay_array, linestyle='-', color='blue', label='ay')
                # if len(node.az_array) > 0:
                #     ax3.plot(range(len(node.az_array)), node.az_array, linestyle='-', color='red', label='az')

                if len(node.yaw_rate_array) > 0 or len(node.ax_array) > 0 or len(node.ay_array) > 0:
                    ax3.legend(loc="upper left")
                    ax3.set_title('Ay and yaw_rate')
                    ax3.grid(True)

                ax4.cla()
                if len(node.gps_error_array) > 0:
                    ax4.plot(range(len(node.gps_error_array)), node.gps_error_array, linestyle='-', color='red', label='gps_error')
                if len(node.tae_error_array) > 0:
                    ax4.plot(range(len(node.tae_error_array)), node.tae_error_array, linestyle='-', color='green', label='tae_error')


                if len(node.gps_error_array) > 0 or len(node.tae_error_array) > 0:
                    ax4.legend(loc="upper left")
                    ax4.set_title('gps and tae error')
                    ax4.grid(True)

                plt.tight_layout()
                plt.draw()
                plt.pause(0.05)


        thread.join()  # 스레드가 종료될 때까지 기다림

    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        # Save the figure before destroying the node and shutting down.
        fig.savefig('plot.png')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
