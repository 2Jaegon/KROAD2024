import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from std_msgs.msg import Float64MultiArray, Float64, String, Int16
from geometry_msgs.msg import PointStamped

from matplotlib import pyplot as plt
import matplotlib
matplotlib.use("TkAgg")
import pandas as pd
import numpy as np

class plottingNode(Node):
    def __init__(self):
        super().__init__('plotting_node')
        qosprofile = QoSProfile(depth=1)  # QoS depth 값 조정
        qosprofile.reliability = ReliabilityPolicy.BEST_EFFORT # best_effort

        self.mission_number = 0

        self.local_path = []
        self.local_yaw = []
        self.local_k = []
        self.mission_data = 0
        self.previous_mission_data = None

        self.car_odom = [0.0, 0.0]
        self.car_yaw = 0.0
        self.trajectories = []  # 모든 궤적 데이터를 저장할 리스트

        self.rp_s_static_point = []
        self.pl_small_static_point = []

        self.rp_b_static_point = []
        self.pl_big_static_point = []

        self.rp_prl_parking_point = []
        self.pl_parking_center_point = []

        self.rp_parking_edge_out_point = []
        self.pl_parking_edge_out_point = []

        self.rp_u_uturn_point = []
        self.pl_uturn_center_point = []

        self.rp_traffic_cone = []
        self.pl_traffic_cone = []
        self.pl_dwa_ref_point = []

        self.Lwall = []
        self.Rwall = []
        self.tunnel_small = []

        self.pl_mission_number_sub = self.create_subscription(Int16, '/Planning/mission', self.pl_mission_number_callback, qosprofile)

        # 본선/예선
        self.pl_path_sub = self.create_subscription(Float64MultiArray, '/Planning/local_path', self.lo_path_callback, qosprofile)
        self.pl_yaw_sub = self.create_subscription(Float64MultiArray, '/Planning/path_yaw', self.lo_yaw_callback, qosprofile)
        self.pl_k_sub = self.create_subscription(Float64MultiArray, '/Planning/curvature', self.lo_k_callback, qosprofile)
        self.pl_small_static_utm_sub = self.create_subscription(Float64MultiArray, '/Planning/plot_small_static_utm', self.pl_small_static_callback, qosprofile)
        self.pl_big_static_utm_sub = self.create_subscription(Float64MultiArray, '/Planning/plot_big_static_utm', self.pl_big_static_callback, qosprofile)
        self.pl_parking_center_sub = self.create_subscription(Float64MultiArray, '/Planning/plot_parking_center', self.pl_parking_center_callback, qosprofile)
        self.pl_parking_edge_out_sub = self.create_subscription(Float64MultiArray, '/Planning/plot_parking_edge_out', self.pl_parking_edge_out_callback, qosprofile)
        self.pl_uturn_center_sub = self.create_subscription(Float64MultiArray, '/Planning/plot_u_turn_point_utm', self.pl_uturn_center_callback, qosprofile)

        self.lo_c_utm_sub = self.create_subscription(PointStamped, '/Local/utm', self.lo_c_UTM_callback, qosprofile)
        self.lo_c_yaw_sub = self.create_subscription(Float64, '/Local/heading', self.lo_c_yaw_callback, qosprofile)

        self.rp_small_object_sub = self.create_subscription(Float64MultiArray, '/Convert/small_object_UTM', self.rp_small_cone_UTM_callback, qosprofile)
        self.rp_u_turn_point_sub = self.create_subscription(Float64MultiArray, '/Convert/u_turn_point', self.rp_uturn_point_callback, qosprofile)
        self.rp_big_object_sub = self.create_subscription(Float64MultiArray, '/Convert/big_object_UTM', self.rp_big_cone_UTM_callback, qosprofile)
        self.rp_prl_points_sub = self.create_subscription(Float64MultiArray, '/Convert/prl_points', self.rp_prl_parking_point_callback, qosprofile)
        self.rp_prl_edge_out_sub = self.create_subscription(Float64MultiArray, '/Convert/parking_edge', self.rp_prl_parking_edge_out_callback, qosprofile)
        
        self.file_path_sub = self.create_subscription(String, 'Planning/plot_file_path', self.file_path_callback, qosprofile)
        self.trajectory_sub = self.create_subscription(Float64MultiArray, '/Planning/plot_all_path', self.trajectory_callback, qosprofile)  # 궤적 구독 추가

        # 터널
        self.tunnel_L_wall_sub = self.create_subscription(Float64MultiArray, '/Planning/plot_tunnel_L_wall', self.tunnel_L_wall_callback, qosprofile)
        self.tunnel_R_wall_sub = self.create_subscription(Float64MultiArray, '/Planning/plot_tunnel_R_wall', self.tunnel_R_wall_callback, qosprofile)
        self.tunnel_small_static_sub = self.create_subscription(Float64MultiArray, '/Planning/plot_tunnel_small_static', self.pl_tunnel_small_callback, qosprofile)

        # 협로
        self.pl_traffic_cone_sub = self.create_subscription(Float64MultiArray, '/Planning/plot_traffic_cone_utm', self.pl_traffic_cone_callback, qosprofile)
        self.rp_traffic_cone_sub = self.create_subscription(Float64MultiArray, '/Planning/narrow_object_UTM', self.rp_traffic_cone_callback, qosprofile)
        self.pl_dwa_ref_point_sub = self.create_subscription(Float64MultiArray, '/Planning/plot_dwa_ref_point', self.pl_dwa_ref_point_callback, qosprofile)

        self.global_path = pd.DataFrame()

        self.timer = self.create_timer(0.01, self.draw_plot)  # 타이머 설정

    def read_columns(self, file_path):
        try:
            # 데이터 프레임으로 파일 읽기 (쉼표를 구분자로 사용)
            df = pd.read_csv(file_path, delimiter=',', header=None)
            
            # 첫 번째와 두 번째 열만 선택
            columns = df.iloc[:, [0, 1]]
            
            return columns
        except Exception as e:
            self.get_logger().error(f"파일 읽기 오류: {e}")
            return pd.DataFrame()

    def draw_plot(self):
        self.get_logger().info("그래프 그리는 중...")  
        plt.clf()
        plt.gcf().canvas.mpl_connect('key_release_event', lambda event: [exit(0) if event.key == 'escape' else None])
        plt.axis("equal")

        # 차량의 현재 UTM 좌표를 중심으로 설정
        center_x, center_y = self.car_odom[0], self.car_odom[1]

        if self.mission_number == 15:
            center_x, center_y = 0, 0
        else:
            center_x, center_y = self.car_odom[0], self.car_odom[1]

        padding = 20  # 패딩 값을 설정하여 중심에서 일정 거리 범위를 시각화
        arrow_length = 2.0 
        h_width = 1.0
        h_length = 1.0

        if self.mission_number == 999 or self.mission_number == 998:
            padding = 10
            arrow_length = 2.0 
            h_width = 1.0
            h_length = 1.0
            
        else:
            padding = 30
            arrow_length = 4.0 
            h_width = 2.0
            h_length = 2.0

        tn_padding_m01 = padding - 2 if self.mission_number == 15 else 0 #터널용 패딩(상대좌표)
        tn_padding_m02 = padding - 10 if self.mission_number == 15 else 0 #터널용 패딩(상대좌표)

        plt.xlim(center_x - padding + tn_padding_m01, center_x + padding - tn_padding_m02)
        plt.ylim(center_y - padding + tn_padding_m02, center_y + padding - tn_padding_m02)

        if not self.global_path.empty:
            plt.plot(self.global_path.iloc[:, 0].to_numpy(), self.global_path.iloc[:, 1].to_numpy(), '-k')

        if self.trajectories:
            plt.plot(self.trajectories[0::2], self.trajectories[1::2], 'or', color='orange', markersize=0.2, linewidth=0.1)

        if self.local_path:
            plt.plot(self.local_path[0::2], self.local_path[1::2], 'ob', markersize=1.3, linewidth=0.5)
        plt.plot(self.car_odom[0], self.car_odom[1], 'og')

        if self.rp_s_static_point:
            plt.plot(self.rp_s_static_point[0::2], self.rp_s_static_point[1::2], 'or', color='orange')

        if self.pl_small_static_point:
            plt.plot(self.pl_small_static_point[0::2], self.pl_small_static_point[1::2], 'o', color='red', markersize=5)

        if self.rp_b_static_point:
            plt.plot(self.rp_b_static_point[0::2], self.rp_b_static_point[1::2], 'or', color='orange')

        if self.pl_big_static_point:
            plt.plot(self.pl_big_static_point[0::2], self.pl_big_static_point[1::2], 'o', color='red', markersize=5)

        if self.rp_prl_parking_point:
            plt.plot(self.rp_prl_parking_point[0::2], self.rp_prl_parking_point[1::2], 'or', color='orange')

        if self.pl_parking_center_point:
            plt.plot(self.pl_parking_center_point[0::2], self.pl_parking_center_point[1::2], 'o', color='red', markersize=5)

        if self.pl_parking_edge_out_point:
            plt.plot(self.pl_parking_edge_out_point[0::2], self.pl_parking_edge_out_point[1::2], 'or', color='orange')

        if self.rp_parking_edge_out_point:
            plt.plot(self.rp_parking_edge_out_point[0::2], self.rp_parking_edge_out_point[1::2], 'o', color='red', markersize=5)            

        if self.rp_u_uturn_point:
            plt.plot(self.rp_u_uturn_point[0::2], self.rp_u_uturn_point[1::2], 'or', color='orange')

        if self.pl_uturn_center_point:
            plt.plot(self.pl_uturn_center_point[0::2], self.pl_uturn_center_point[1::2], 'o', color='red', markersize=5)

        # 협로 관련
        if len(self.Lwall) > 0 :
            plt.plot(self.Lwall[0::2], self.Lwall[1::2], color ='black', linewidth = 3)

        if len(self.Rwall) > 0 :
            plt.plot(self.Rwall[0::2], self.Rwall[1::2], color ='black', linewidth = 3)
        
        if len(self.tunnel_small) > 1 :
            for i in range(0, ((len(self.tunnel_small))>>1), 2) :
                plt.plot(self.tunnel_small[i], self.tunnel_small[i+1], 'o', color='red', markersize=5)

        # rp_traffic_cone을 연한 색으로 그리기
        if self.rp_traffic_cone:
            for i in range(0, len(self.rp_traffic_cone), 3):
                x = self.rp_traffic_cone[i]
                y = self.rp_traffic_cone[i+1]
                cls = int(self.rp_traffic_cone[i+2])
                if cls == 1:
                    color = 'lightblue'
                elif cls == 2:
                    color = 'orange'
                else:
                    color = 'gray'  # 클래스가 0이거나 예상치 못한 경우
                plt.plot(x, y, 'or', color=color)

        # pl_traffic_cone을 진한 색으로 그리기
        if self.pl_traffic_cone:
            for i in range(0, len(self.pl_traffic_cone), 3):
                x = self.pl_traffic_cone[i]
                y = self.pl_traffic_cone[i+1]
                cls = int(self.pl_traffic_cone[i+2])
                if cls == 1:
                    color = 'blue'
                elif cls == 2:
                    color = 'yellow'
                else:
                    color = 'black'  # 클래스가 0이거나 예상치 못한 경우
                plt.plot(x, y, 'o', color=color, markersize=5)

        if self.pl_dwa_ref_point:
            plt.plot(self.pl_dwa_ref_point[0::2], self.pl_dwa_ref_point[1::2], 'o', color='purple', markersize=3)

        # 화살표로 yaw 표시
        plt.arrow(self.car_odom[0], self.car_odom[1], arrow_length * np.cos(self.car_yaw), arrow_length * np.sin(self.car_yaw), head_width=h_width, head_length=h_length, fc='g', ec='g')

        plt.grid(True)
        plt.pause(0.001)

    def file_path_callback(self, msg):
        if self.previous_mission_data != msg.data:
            self.mission_data = msg.data
            self.previous_mission_data = self.mission_data
            ################## 맵 경로 수정 필요!!!!!!!!!!!!!!!!!!!! #####################
            path = f'{self.mission_data}.txt'
            self.global_path = self.read_columns(path)
            if self.global_path.empty:
                self.get_logger().error(f"{path}에서 데이터를 불러오는데 실패했습니다.")
            else:
                self.get_logger().info(f"{path}에서 새로운 미션 데이터를 불러왔습니다.")

    def pl_mission_number_callback(self, msg):
        self.mission_number = msg.data
        if self.mission_number == 15:
            self.local_path = []  # 터널에 들어갈 때 기존의 로컬 경로 초기화
            self.tunnel_small = []  # 터널용 작은 장애물도 초기화 필요시

    def lo_path_callback(self, msg):
        self.local_path = msg.data

    def lo_yaw_callback(self, msg):
        self.local_yaw = msg.data

    def lo_k_callback(self, msg):
        self.local_k = msg.data

    def lo_c_UTM_callback(self, msg):
        self.car_odom = [msg.point.x, msg.point.y]

    def lo_c_yaw_callback(self, msg):
        self.car_yaw = msg.data

    def trajectory_callback(self, msg):
        self.trajectories = msg.data

    # Planning
    def pl_small_static_callback(self, msg):
        self.pl_small_static_point = msg.data

    def pl_parking_center_callback(self, msg):
        self.pl_parking_center_point = msg.data

    def pl_parking_edge_out_callback(self, msg):
        self.pl_parking_edge_out_point = msg.data

    def pl_uturn_center_callback(self, msg):
        self.pl_uturn_center_point = msg.data
        
    def pl_big_static_callback(self, msg):
        self.pl_big_static_point = msg.data

    def pl_traffic_cone_callback(self, msg):
        self.pl_traffic_cone = msg.data

    # Convert
    def rp_small_cone_UTM_callback(self, msg):
        self.rp_s_static_point = msg.data

    def rp_big_cone_UTM_callback(self, msg):
        self.rp_b_static_point = msg.data

    def rp_prl_parking_point_callback(self, msg):
        self.rp_prl_parking_point = msg.data

    def rp_prl_parking_edge_out_callback(self, msg):
        self.rp_parking_edge_out_point = msg.data

    def rp_uturn_point_callback(self, msg):
        self.rp_u_uturn_point = msg.data

    def rp_traffic_cone_callback(self, msg):
        self.rp_traffic_cone = msg.data

    # 터널 콜백
    def tunnel_L_wall_callback(self, msg):
        self.Lwall = msg.data

    def tunnel_R_wall_callback(self, msg):
        self.Rwall = msg.data
    
    def pl_tunnel_small_callback(self, msg):
        self.tunnel_small = msg.data

    def pl_dwa_ref_point_callback(self, msg):
        self.pl_dwa_ref_point = msg.data

def main(args=None):
    rclpy.init(args=args)
    node = plottingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
