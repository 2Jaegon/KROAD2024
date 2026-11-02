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

// /* matplotcpp 헤더파일 */
// #include <matplot/matplot.h>

/* 사용자 정의 헤더파일 */
#include "amz_planning.hpp"
#include "local.hpp"
#include "vision.hpp"
#include "lidar.hpp"
#include "control.hpp"
#include "cubic_spline.hpp"
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

    // Lidar flag(bool)

    // Lidar ACS(absolute coordinate system: UTM coordinate systme)
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr traffic_cone_utm_sub_;

    /* publisher 객체 */
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr local_path_pos_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr local_path_yaw_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr local_path_k_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr control_target_velocity_pub_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr mission_number_pub_;

    /* plot용 publisher 객체 */
    // rcs
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr plot_file_path_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr plot_traffic_cone_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr plot_dwa_ref_point_pub_;

    /* 초깃값 세팅 */ // 세팅 필수!!!!!
    // map들어있는 폴더 경로 복사해서 붙여넣기('/' 필요 없음!!)
    std::string gp_text_adress_ = "/home/kroad/morai_ws_2024/src/Planning_PKG_2024/map/map_final_1";
    // map에 맞춰서 미션 넣기!!
    std::vector<int> mission_list_ = {999, 998}; // 협로 미션 리스트
    std::unordered_map<int, std::string> mission_dic_ =
        {{0, "default"}, {999, "first_lap"}, {998, "second_lap"}};

    /* Planning 클래스 스마트 포인터 */
    std::shared_ptr<AmzPlanning> planning_smtptr_;
    AmzPlanning *planning_ptr_;

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
        std::ifstream inparam("./src/Amz_Planning_PKG_2024/data/param.json");
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
        planning_smtptr_ = std::make_shared<AmzPlanning>(gp_text_adress_, mission_list_, mission_dic_, json_data["basic_data"], json_data["missions"]);
        planning_ptr_ = planning_smtptr_.get();
        local_ptr_ = &(planning_ptr_->getLocal());
        vision_ptr_ = &(planning_ptr_->getVision());
        lidar_ptr_ = &(planning_ptr_->getLidar());
        control_ptr_ = &(planning_ptr_->getControl());

        /* qos 설정 */
        // auto qos = rclcpp::QoS(rclcpp::KeepLast(10)); // reliable
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliability(rclcpp::ReliabilityPolicy::BestEffort); // best_effort

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

        // Lidar Flag Subscriptions
        // Lidar ACS(absolute coordinate system: UTM coordinate systme) Subscriptions
        traffic_cone_utm_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/Planning/narrow_object_UTM", qos, std::bind(&Lidar::lidarTrafficConeCallback, lidar_ptr_, std::placeholders::_1));

        /* --------------------------publisher 구현---------------------------- */
        // 제어에 pub 하는 publisher
        local_path_pos_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/Planning/local_path", qos);
        local_path_yaw_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/Planning/path_yaw", qos);
        local_path_k_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/Planning/curvature", qos);
        control_target_velocity_pub_ = this->create_publisher<std_msgs::msg::Float64>("/Planning/target_velocity", qos);
        mission_number_pub_ = this->create_publisher<std_msgs::msg::Int16>("/Planning/mission", qos);

        // plot용 publisher
        plot_file_path_pub_ = this->create_publisher<std_msgs::msg::String>("/Planning/plot_file_path", qos);
        plot_traffic_cone_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/Planning/plot_traffic_cone_utm", qos);
        plot_dwa_ref_point_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("Planning/plot_dwa_ref_point", qos);

        /* time_spine 함수 실행 */
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&PlanningNode::timer_callback, this));
    }

    void timer_callback()
    {
        if (!node_start_flag_)
        {
            if (local_ptr_->callbacksIsCalled() && checkCanStart())
            {
                node_start_flag_ = true;
            }
            else
            {
                std::cout << "utm, heading, serial, imu 안들어옴\n";
            }
        }
        else
        {
            /* 상태 표시 */
            planning_ptr_->printStatus();

            /* 여기다 조건 분기 구현(어떤 미션에 따라서 어떤 미션 실행할건지) */
            planning_ptr_->doAmzMission();

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

        mission_flag.data = local_ptr_->getCurMissionNumber();
        target_velocity.data = control_ptr_->getTargetVelocity();

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
        std_msgs::msg::Float64MultiArray traffic_cones;
        std_msgs::msg::Float64MultiArray ref_point;

        /* publish 준비 */
        std::string file_p = gp_text_adress_;
        file_p.append("/");
        // file_p.append(local_ptr_->getCurFileName());
        file_path.data = file_p;

        const std::vector<std::vector<double>> &ref_traffic_cones = lidar_ptr_->getRefTrafficCone();
        for (int i = 0; i < ref_traffic_cones.size(); ++i)
        {
            traffic_cones.data.push_back(ref_traffic_cones[i][0]);
            traffic_cones.data.push_back(ref_traffic_cones[i][1]);
            traffic_cones.data.push_back(ref_traffic_cones[i][2]);
        }

        ref_point.data.push_back(planning_ptr_->getDwaRefPoint()[0]);
        ref_point.data.push_back(planning_ptr_->getDwaRefPoint()[1]);

        plot_file_path_pub_->publish(file_path);
        plot_traffic_cone_pub_->publish(traffic_cones);
        plot_dwa_ref_point_pub_->publish(ref_point);
    }

    bool checkCanStart()
    {
        int blue_count = 0;
        int yellow_count = 0;

        const std::vector<std::vector<double>> &traffic_cones = lidar_ptr_->getRefTrafficCone();
        for (auto &cone : traffic_cones)
        {
            if (cone[2] == 1)
            {
                ++blue_count;
            }
            if (cone[2] == 2)
            {
                ++yellow_count;
            }
        }

        if (yellow_count > 1 && blue_count > 1)
            return true;
        else
        {
            std::cout << "충분한 콘 인지 못함!\n";
        }
        return false;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv); // 초기화
    rclcpp::spin(std::make_shared<PlanningNode>());
    rclcpp::shutdown();
    return 0;
}