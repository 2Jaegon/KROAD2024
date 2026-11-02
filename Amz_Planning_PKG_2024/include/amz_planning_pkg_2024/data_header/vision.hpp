/***
 * 작성자: 정주훈
 * 설명: Vision 클래스 헤더파일
 * Style
 * -클래스: PascalCase
 * -멤버변수: snake_case_   (snake_case 뒤에 밑줄)(파이썬 사용 x)
 * -함수: camelCase
 * -변수: snake_case
 * -상수: kPascalCase   (PascalCase앞에 k 접두어)
 ***/

#ifndef VISION_H_
#define VISION_H_

#include <iostream>
#include <vector>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

class Vision
{
private:
    std::vector<int> v_light_ = {0, 0, 1, 1}; // 신호등 색깔여부[빨강, 주황, 좌회전, 초록] int32mutiarry로 받음
    std::vector<int> parking_points_;         // 주차할 점 좌표 3개     // 이것도 뭐임...?
    double stop_line_pos_[2];                 // 정지선 좌표

public:
    /* getter 함수 */
    const std::vector<int> &getRefVLight() const
    {
        return v_light_;
    }

    void setRefVLightToGreen()
    {
        v_light_ = {0, 2, 1, 1};
    }

    const std::vector<int> &getRefParkingPoints() const
    {
        return parking_points_;
    }

    const double *getStopLinePos() const
    {
        return stop_line_pos_;
    }

    /* setter 함수 */
    void setParkingPoints(std::vector<int> &parking_points)
    {
        parking_points_ = parking_points;
    }

    /* callback 함수 */
    void visionTrafficLightCallback(std_msgs::msg::Int32MultiArray::SharedPtr msg)
    {
        v_light_ = msg->data;
    }

    void visionStoplineCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        stop_line_pos_[0] = msg->point.x;
        stop_line_pos_[1] = msg->point.y;
    }

    /* test용 함수 */
    void printAllData()
    {
        std::cout << "############## v_light ##############\n";
        for (auto n : v_light_)
            std::cout << n << " ";
        std::cout << "\n";
        std::cout << "\n";
        std::cout << "############## parking_points ##############\n";
        for (auto n : parking_points_)
            std::cout << n << " ";
        std::cout << "\n";
        std::cout << "############## stop_line ##############\n";
        std::cout << stop_line_pos_[0] << " " << stop_line_pos_[1] << "\n";
    }
};

#endif