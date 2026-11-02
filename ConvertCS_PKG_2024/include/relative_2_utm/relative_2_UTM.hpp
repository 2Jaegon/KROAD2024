#pragma once

#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <queue>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int16.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

using namespace std;

class Relative_2_UTM : public rclcpp::Node
{
public:
    Relative_2_UTM();
    void calc_relative_2_UTM();

private:
    // callback function
    void heading_cb(const std_msgs::msg::Float64::SharedPtr msg);
    void car_UTM_cb(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void big_object_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void obq_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void mission_cb(const std_msgs::msg::Int16::SharedPtr msg);
    void prl_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void prl_edge_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void small_object_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void u_turn_point_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void deli_fusion_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    // calculation relative to UTM
    void big_cal_obj_UTM(float c_heading, double sensor_distanse[2], vector<vector<double>> object_cen, double utm_gps[2]);
    void obq_cal_obj_UTM(float c_heading, double sensor_distanse[2], vector<vector<double>> object_cen, double utm_gps[2]);
    void prl_cal_obj_UTM(float c_heading, double sensor_distanse[2], vector<vector<double>> object_cen, double utm_gps[2]);
    void parking_edge_obj_UTM(float c_heading, double sensor_distanse[2], vector<vector<double>> object_cen, double utm_gps[2]);
    void small_cal_obj_UTM(float c_heading, double sensor_distanse[2], vector<vector<double>> object_cen, double utm_gps[2]);
    void u_turn_point_cal_UTM(float c_heading, double sensor_distanse[2], vector<vector<double>> object_cen, double utm_gps[2]);
    void deli_flag_point_cal_UTM(float c_heading, double sensor_distanse[2], vector<vector<double>> object_cen, double utm_gps[2]);

    // publish funcion
    void big_obstacle_UTM_Pub(std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> big_obstacle_UTM);
    void obq_point_UTM_Pub(std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> big_obstacle_UTM);
    void prl_parking_UTM_Pub(std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> prl_point_UTM);
    void parking_edge_UTM_Pub(std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> parking_edge_UTM);
    void small_obstacle_UTM_Pub(std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> big_obstacle_UTM);
    void u_turn_point_UTM_Pub(std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> big_obstacle_UTM);
    void deli_flag_point_UTM_Pub(std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> big_obstacle_UTM);

    // pub
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr big_obstacle_UTM;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr obq_obstacle_UTM;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr prl_point_UTM;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr parking_edge_UTM;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr small_obstacle_UTM;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr u_turn_point_UTM;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr deli_flag_point_UTM;

    // sub
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr heading_sub;           // heading
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr car_utm_sub; // utm

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr big_obstacle_sub;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr small_obstacle_sub;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr obq_sub;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr mission_sub;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr prl_sub;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr prl_edge_sub;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr u_turn_point_sub;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr deli_flag_point_sub;

    // class member variable
    double inter_sensor_distanse[2]{0.35, 0}; // 차량 내 센서 위치 고려
    vector<vector<double>> obj_cen;           // 물체 중심 상대좌표 저장
    vector<vector<double>> obj_UTM;           // 물체 중심 절대좌표 저장
    double c_UTM[2]{};                        // car_UTM [x, y]
    float c_heading = 0.0;                    // car_heading(radian)
    bool check_cb_sub[2]{};                   // 차량 UTM,heading이 콜백되고 있는지 확인
    double tmp_distance = 0.0;
    int mission = 0;             // 수행중인 미션 mission number
    bool mission_state[30] = {}; // 수행중인 미션의 point가 들어왔는지 여부( index = mission number )
    std::chrono::milliseconds loop_duration;
};