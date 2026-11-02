/***
 * 작성자: 판단 파트
 * 설명: ros2 구동을 위한 Planning Node 구현
 * Style
 * -클래스: PascalCase
 * -멤버변수: snake_case_   (snake_case 뒤에 밑줄)(파이썬 사용 x)
 * -함수: camelCase
 * -변수: snake_case
 * -상수: kPascalCase   (PascalCase앞에 k 접두어)
 ***/

// ros2 bag play <bag파일 폴더 이름> --loop  : 백파일 실행방법

/* c++ 헤더파일 */
#include <iostream>
#include <vector>
#include <unordered_map>
#include <memory>

/* ros2 헤더파일 */
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/nav_sat_fix.hpp>

// /* matplotcpp 헤더파일 */
// #include <matplot/matplot.h>

/* 사용자 정의 헤더파일 */
#include "planning.hpp"
#include "local.hpp"
#include "vision.hpp"
#include "lidar.hpp"
#include "control.hpp"
#include "cubic_spline.hpp"
#include "evade_path_generator.hpp"
#include "cubic_spline.hpp"
#include "nlohmann/json.hpp"

class PlanningNode : public rclcpp::Node
{
private:
    /* subscriber 객체*/
    // ERP_SERIAL
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr erp_data_sub_;

    // Local
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr local_car_utm_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr local_car_yaw_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr local_car_imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr local_nav_status_sub_; // Tunnel

    // Lidar flag(bool)
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr dynamic_stop_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr parallel_stop_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr tunnel_end_sub_;
    // Lidar RCS(relative coordinate system)
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr tunnel_wall_points_rc_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr tunnel_lane_points_rc_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr tunnel_center_rc_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr tunnel_ceiling_rc_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr tunnel_small_object_rc_sub_;
    // Lidar ACS(absolute coordinate system: UTM coordinate systme)
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr small_object_utm_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr big_object_utm_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr parallel_park_points_utm_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr parallel_parking_edge_out_utm_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr u_turn_point_utm_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr delivery_point_utm_sub_;

    // Vision
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr traffic_light_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr distance_to_stopline_sub_;

    /* publisher 객체 */
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr local_path_pos_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr local_path_yaw_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr local_path_k_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr control_target_velocity_pub_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr mission_number_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr tunnel_dynamic_pub_;

    /* plot용 publisher 객체 */
    // rcs
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr plot_file_path_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr plot_small_static_utm_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr plot_all_path_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr plot_parking_center_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr plot_parking_edge_out_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr plot_big_static_utm_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr plot_u_turn_point_utm_pub_;
    // acs
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr plot_tunnel_L_wall_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr plot_tunnel_R_wall_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr plot_tunnel_small_static_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr plot_gps_mode_pub_;

    /************************ 수정 필요 부분 시작 *************************** */
    /* 초깃값 세팅 */ // 세팅 필수!!!!!
    // map들어있는 폴더 경로 복사해서 붙여넣기('/' 필요 없음!!)
    std::string gp_text_adress_ = "./src/Planning_PKG_2024/map/map_final_final"; // 본선
    // std::string gp_text_adress_ = "./src/Planning_PKG_2024/map/map_tryout_tryout"; // 예선

    // map에 맞춰서 미션 넣기!!
    ////////////*************************************************************************////////////

    std::vector<int> mission_list_ = {4, 7, 2, 11, 2323,
                                      22, 21, 12, 21, 9,
                                      22, 22, 4, 23, 4,
                                      21, 21, 14}; // 본선 미션 리스트

    // std::vector<int> mission_list_ = {4, 25, 16, 15}; // 예선 미션 리스트

    ////////////*************************************************************************////////////
    // std::vector<int> mission_list_ = {7, 21, 11, 9, 12, 0, 14}; // 산학 통합실험 미션 리스트

    /************************ 수정 필요 부분 끝 *************************** */

    std::unordered_map<int, std::string> mission_dic_ =
        {{0, "default"}, {1, "v_4"}, {2, "v_5"}, {3, "v_6"}, {4, "full_velocity"}, {7, "delivery_A"}, {9, "delivery_B"}, {11, "small_static"}, {12, "big_static"}, {14, "parking"}, {15, "tunnel"}, {16, "uturn"}, {17, "dynamic_object"}, {21, "stop_line"}, {22, "stop_left"}, {23, "stop_no_traffic"}, {25, "bump"}, {2323, "double_stop"}};

    /* Planning 클래스 스마트 포인터 */
    std::shared_ptr<Planning> planning_smtptr_;
    Planning *planning_ptr_;

    /* data 클래스들 관련 */
    Local *local_ptr_;
    Vision *vision_ptr_;
    Lidar *lidar_ptr_;
    Control *control_ptr_;

    /* 타이머 동기화 함수(계속 실행되는 함수) 포인터 */
    rclcpp::TimerBase::SharedPtr timer_;

    /* start flag */
    bool node_start_flag_ = false;

    /* 파라미터 json */
    nlohmann::json json_data;

public:
    PlanningNode() : Node("planning_node")
    {
        /* 파라미터 읽어오기 */
        std::ifstream inparam("./src/Planning_PKG_2024/data/param.json");
        if (!inparam.is_open())
        {
            while (1)
                std::cerr << "파라미터 파일 안열림\n";
        }
        else
        {
            inparam >> json_data;
            inparam.close();
        }

        /* 클래스 인스턴스화 */
        planning_smtptr_ = std::make_shared<Planning>(gp_text_adress_, mission_list_, mission_dic_, json_data["basic_data"], json_data["missions"]);
        planning_ptr_ = planning_smtptr_.get();
        local_ptr_ = &(planning_ptr_->getLocal());
        vision_ptr_ = &(planning_ptr_->getVision());
        lidar_ptr_ = &(planning_ptr_->getLidar());
        control_ptr_ = &(planning_ptr_->getControl());

        /* qos 설정 */
        // auto qos = rclcpp::QoS(rclcpp::KeepLast(10)); // reliable
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliability(rclcpp::ReliabilityPolicy::BestEffort); // best_effort

        /* --------------------------subscriber 구현--------------------------- */
        // ERP Data Subscriptions
        erp_data_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/ERP/serial_data", qos, std::bind(&Local::erpDataCallback, local_ptr_, std::placeholders::_1));

        // Local Subscriptions
        local_car_utm_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/Local/utm", qos, std::bind(&Local::localCurUTMCallback, local_ptr_, std::placeholders::_1));
        local_car_yaw_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/Local/heading", qos, std::bind(&Local::localCurYawCallback, local_ptr_, std::placeholders::_1));
        local_car_imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", qos, std::bind(&Local::localImuCallback, local_ptr_, std::placeholders::_1));
        local_nav_status_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/fix", qos, std::bind(&Local::localNavStatusCallback, local_ptr_, std::placeholders::_1));

        // Lidar Flag Subscriptions
        dynamic_stop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/LiDAR/dynamic_stop", qos, std::bind(&Lidar::lidarDynamicCallback, lidar_ptr_, std::placeholders::_1));
        parallel_stop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/LiDAR/park_ok2", qos, std::bind(&Lidar::lidarParkingStopCallback, lidar_ptr_, std::placeholders::_1));
        tunnel_end_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/LiDAR/tunnel_end", qos, std::bind(&Lidar::lidarTunnelEndCallback, lidar_ptr_, std::placeholders::_1));
        // Lidar RCS(relative coordinate system) Subscriptions
        tunnel_ceiling_rc_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/LiDAR/ceiling_end", qos, std::bind(&Lidar::lidarTunnelCeilingCallback, lidar_ptr_, std::placeholders::_1));
        tunnel_small_object_rc_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/LiDAR/object_cen", qos, std::bind(&Lidar::lidarTunnelSmallStaticCallback, lidar_ptr_, std::placeholders::_1));
        tunnel_wall_points_rc_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/LiDAR/wall_dist", qos, std::bind(&Lidar::lidarTunnelWallDistCallback, lidar_ptr_, std::placeholders::_1));
        tunnel_lane_points_rc_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/LiDAR/lane", qos, std::bind(&Lidar::lidarTunnelLaneCallback, lidar_ptr_, std::placeholders::_1));
        tunnel_center_rc_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/LiDAR/center_lane", qos, std::bind(&Lidar::lidarTunnelCenterCallback, lidar_ptr_, std::placeholders::_1));
        // Lidar ACS(absolute coordinate system: UTM coordinate systme) Subscriptions
        small_object_utm_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/Convert/small_object_UTM", qos, std::bind(&Lidar::lidarSmallStaticCallback, lidar_ptr_, std::placeholders::_1));
        big_object_utm_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/Convert/big_object_UTM", qos, std::bind(&Lidar::lidarBigStaticCallback, lidar_ptr_, std::placeholders::_1));
        parallel_park_points_utm_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/Convert/prl_points", qos, std::bind(&Lidar::lidarParkingPointCallback, lidar_ptr_, std::placeholders::_1));
        parallel_parking_edge_out_utm_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/Convert/parking_edge", qos, std::bind(&Lidar::lidarParkingEdgeOutCallback, lidar_ptr_, std::placeholders::_1));
        u_turn_point_utm_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/Convert/u_turn_point", qos, std::bind(&Lidar::lidarUTurnPointCallback, lidar_ptr_, std::placeholders::_1));
        delivery_point_utm_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/Convert/deli_UTM", qos, std::bind(&Lidar::lidarDelibaryFlagCallback, lidar_ptr_, std::placeholders::_1));

        // Vision Subscriptions
        traffic_light_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/Vision/traffic_sign", qos, std::bind(&Vision::visionTrafficLightCallback, vision_ptr_, std::placeholders::_1));
        distance_to_stopline_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/Vision/stopline", qos, std::bind(&Vision::visionStoplineCallback, vision_ptr_, std::placeholders::_1));

        /* --------------------------publisher 구현---------------------------- */
        // 제어에 pub 하는 publisher
        local_path_pos_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/Planning/local_path", qos);
        local_path_yaw_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/Planning/path_yaw", qos);
        local_path_k_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/Planning/curvature", qos);
        control_target_velocity_pub_ = this->create_publisher<std_msgs::msg::Float64>("/Planning/target_velocity", qos);
        mission_number_pub_ = this->create_publisher<std_msgs::msg::Int16>("/Planning/mission", qos);

        tunnel_dynamic_pub_ = this->create_publisher<std_msgs::msg::Bool>("/Planning/is_dynamic", qos);

        // plot용 publisher
        plot_file_path_pub_ = this->create_publisher<std_msgs::msg::String>("/Planning/plot_file_path", qos);
        plot_small_static_utm_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/Planning/plot_small_static_utm", qos);
        plot_all_path_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/Planning/plot_all_path", qos);
        plot_parking_center_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/Planning/plot_parking_center", qos);
        plot_parking_edge_out_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/Planning/plot_parking_edge_out", qos);
        plot_big_static_utm_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/Planning/plot_big_static_utm", qos);
        plot_u_turn_point_utm_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/Planning/plot_u_turn_point_utm", qos);
        // tunnel
        plot_gps_mode_pub_ = this->create_publisher<std_msgs::msg::Bool>("/Planning/plot_gps_mode", qos);
        plot_tunnel_L_wall_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/Planning/plot_tunnel_L_wall", qos);
        plot_tunnel_R_wall_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/Planning/plot_tunnel_R_wall", qos);
        plot_tunnel_small_static_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/Planning/plot_tunnel_small_static", qos);

        /* time_spine 함수 실행 */
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&PlanningNode::timer_callback, this));
    }

    void timer_callback()
    {
        if (!node_start_flag_)
        {
            // local_ptr_->waitAllCallbacks();
            if (local_ptr_->callbacksIsCalled())
            {
                node_start_flag_ = true;
                planning_ptr_->setDefaultStatus();
            }
            else
            {
                std::cout << "utm, heading, serial, imu 안들어옴\n";
            }
        }
        else
        {
            /* 다음 맵으로 변경, 끝났으면 종료 */
            if (!(planning_ptr_->judgeMapChange()))
            {
                timer_->cancel();
                return;
            }

            /* 상태 표시 */
            planning_ptr_->printStatus();

            /* 여기다 조건 분기 구현(어떤 미션에 따라서 어떤 미션 실행할건지) */
            planning_ptr_->chooseFunc();

            /* 시각화 노드 publish 함수 */
            plotPathPubFunc();

            /* 인지, 제어 노드에 publish 함수 */
            publishFunc();
        }
    }

    void publishFunc()
    {
        std_msgs::msg::Int16 mission_flag;
        std_msgs::msg::Float64MultiArray local_pos;
        std_msgs::msg::Float64MultiArray local_k;
        std_msgs::msg::Float64MultiArray local_yaw;
        std_msgs::msg::Float64 target_velocity;

        const double *utm_point = local_ptr_->getAddCurCarUTMPos();

        // local_path pub 준비
        const Path &local_path = planning_ptr_->getRefLocalPath();
        int closest_index = local_path.getClosestIndex(utm_point); // 가장 가까운 위치 찾기

        const std::vector<std::vector<double>> &local_pos_arr = local_path.getRefPosArr();
        const std::vector<double> &local_k_arr = local_path.getRefKArr();
        const std::vector<double> &local_yaw_arr = local_path.getRefYawArr();

        if (local_pos_arr[0][0] != -82.82)
        {
            int end_index = closest_index + 300;
            if (local_pos_arr.size() <= closest_index + 300)
            {
                end_index = local_pos_arr.size();
            }

            for (int i = closest_index; i < end_index; i++) // 내 위치기준 300개의 점을 선택
            {
                if (i < local_pos_arr.size() && i < local_k_arr.size() && i < local_yaw_arr.size())
                {
                    local_pos.data.push_back(local_pos_arr[i][0]);
                    local_pos.data.push_back(local_pos_arr[i][1]);
                    local_k.data.push_back(local_k_arr[i]);
                    local_yaw.data.push_back(local_yaw_arr[i]);
                }
                else
                {
                    std::cerr << "인덱스 범위 초과: " << i << "\n";
                }
            }
        }
        else
        {
            for (int i = 0; i < local_pos_arr.size(); ++i)
            {
                local_pos.data.push_back(local_pos_arr[i][0]);
                local_pos.data.push_back(local_pos_arr[i][1]);
                local_k.data.push_back(local_k_arr[i]);
                local_yaw.data.push_back(local_yaw_arr[i]);
            }
        }

        mission_flag.data = local_ptr_->getCurMissionNumber();
        target_velocity.data = control_ptr_->getTargetVelocity();

        if (mission_flag.data == 15)
        {
            std_msgs::msg::Bool dynamic_mode_msg;
            dynamic_mode_msg.data = lidar_ptr_->getDynamicMode();
            tunnel_dynamic_pub_->publish(dynamic_mode_msg);
        }

        mission_number_pub_->publish(mission_flag);
        local_path_pos_pub_->publish(local_pos);
        local_path_yaw_pub_->publish(local_yaw);
        local_path_k_pub_->publish(local_k);
        control_target_velocity_pub_->publish(target_velocity);
    }

    void plotPathPubFunc()
    {
        /* plot publish 위한 포인터 */
        std_msgs::msg::String file_path;
        std_msgs::msg::Float64MultiArray small_static_points;
        std_msgs::msg::Float64MultiArray all_path_pos;
        std_msgs::msg::Float64MultiArray uturn_point;
        std_msgs::msg::Float64MultiArray parking_center;
        std_msgs::msg::Float64MultiArray parking_edge_out;
        std_msgs::msg::Float64MultiArray big_static_points;

        /* publish 준비 */
        std::string file_p = gp_text_adress_;
        file_p.append("/");
        file_p.append(local_ptr_->getCurFileName());
        file_path.data = file_p;

        const std::vector<std::vector<double>> &ref_small_static_points = lidar_ptr_->getRefSmallStaticPoint();
        for (int i = 0; i < ref_small_static_points.size(); ++i)
        {
            small_static_points.data.push_back(ref_small_static_points[i][0]);
            small_static_points.data.push_back(ref_small_static_points[i][1]);
        }

        /* frenet_frame_trajectory plot test */
        const std::vector<std::vector<std::vector<double>>> &ref_all_path_vec = planning_ptr_->getAllPathVector();
        std::cout << "trajectory size: " << ref_all_path_vec.size() << "\n";
        for (int i = 0; i < ref_all_path_vec.size(); ++i)
        {
            for (int j = 0; j < ref_all_path_vec[i].size(); ++j)
            {
                all_path_pos.data.push_back(ref_all_path_vec[i][j][0]);
                all_path_pos.data.push_back(ref_all_path_vec[i][j][1]);
            }
        }

        const std::vector<double> &ref_parking_center = lidar_ptr_->getRefParkingPoint();
        if (ref_parking_center.size() > 0)
        {
            parking_center.data.push_back(ref_parking_center[0]);
            parking_center.data.push_back(ref_parking_center[1]);
        }

        const std::vector<std::vector<double>> &ref_parking_edge_out = lidar_ptr_->getRefParkingEdgeOut();
        if (ref_parking_edge_out.size() > 0)
        {
            for (int i = 0; i < ref_parking_edge_out.size(); ++i)
            {
                parking_edge_out.data.push_back(ref_parking_edge_out[i][0]);
                parking_edge_out.data.push_back(ref_parking_edge_out[i][1]);
            }
        }

        const std::vector<std::vector<double>> &ref_uturn_point = lidar_ptr_->getRefUTurnPoint();
        if (ref_uturn_point.size() > 0)
        {
            for (int i = 0; i < ref_uturn_point.size(); ++i)
            {
                uturn_point.data.push_back(ref_uturn_point[i][0]);
                uturn_point.data.push_back(ref_uturn_point[i][1]);
            }
        }

        const std::vector<double> &ref_big_static_points = lidar_ptr_->getRefBigStaticPoint();
        for (size_t i; i < ref_big_static_points.size(); ++i)
            big_static_points.data.push_back(ref_big_static_points[i]);

        plot_file_path_pub_->publish(file_path);
        plot_small_static_utm_pub_->publish(small_static_points);
        plot_all_path_pub_->publish(all_path_pos);
        plot_parking_center_pub_->publish(parking_center);
        plot_parking_edge_out_pub_->publish(parking_edge_out);
        plot_big_static_utm_pub_->publish(big_static_points);
        plot_u_turn_point_utm_pub_->publish(uturn_point);

        // 터널일때만 publish
        int mission_num = local_ptr_->getCurMissionNumber();
        if (mission_num == 15)
        {
            std_msgs::msg::Bool is_gps_msg;
            is_gps_msg.data = !(local_ptr_->getRcsMode());
            plot_gps_mode_pub_->publish(is_gps_msg);

            std_msgs::msg::Float64MultiArray L_wall_msg;
            std_msgs::msg::Float64MultiArray R_wall_msg;

            const std::vector<double> &L_wall = lidar_ptr_->getRefTunnelLWall();
            const std::vector<double> &R_wall = lidar_ptr_->getRefTunnelRWall();

            for (size_t i = 0; i < L_wall.size(); i++)
            {
                L_wall_msg.data.emplace_back(L_wall[i]);
            }
            for (size_t i = 0; i < R_wall.size(); i++)
            {
                R_wall_msg.data.emplace_back(R_wall[i]);
            }

            plot_tunnel_L_wall_pub_->publish(L_wall_msg);
            plot_tunnel_L_wall_pub_->publish(R_wall_msg); // 여기까지 벽 퍼블리시

            // 터널 정적 소형
            std_msgs::msg::Float64MultiArray tunnel_small_msg;
            const std::vector<double> &ref_tunnel_small_static = lidar_ptr_->getRefTunnelSmallStaticPoint();
            for (int i = 0; i < ref_tunnel_small_static.size(); i++)
            {
                tunnel_small_msg.data.push_back(ref_tunnel_small_static[i]);
            }
            plot_tunnel_small_static_pub_->publish(tunnel_small_msg);
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv); // 초기화
    rclcpp::spin(std::make_shared<PlanningNode>());
    rclcpp::shutdown();
    return 0;
}