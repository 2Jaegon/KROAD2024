/***
 * 작성자: 박용현
 * 설명: Lidar 클래스 헤더파일
 * Style
 * -클래스: PascalCase
 * -멤버변수: snake_case_   (snake_case 뒤에 밑줄)(파이썬 사용 x)
 * -함수: camelCase
 * -변수: snake_case
 * -상수: kPascalCase   (PascalCase앞에 k 접두어)
 ***/

#ifndef LIDAR_H_
#define LIDAR_H_

#include <vector>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <algorithm>
#include <climits>
#include "nlohmann/json.hpp"

#include "utility_function.hpp"
#include "static_clusters.hpp"
#include "dynamic_clusters.hpp"

class Lidar
{
private:
    // 정적 소형
    StaticPosCluster small_static_cluster_;

    // 동적
    bool is_dynamic_stop_ = false;
    bool tunnel_dynamic_mode_ = false;

    // 정적 대형
    std::vector<double> big_static_point_; // [x_left, y_left, x_right, x_right]
    StaticPosCluster big_static_cluster_;

    // 주차
    std::vector<double> parking_center_point;
    StaticPosCluster parking_point_cluster_;           // 주차 칸 중심점 하나
    std::vector<std::vector<double>> parking_edge_out; // 주차 꼭짓점 4개
    bool is_parking_stop_ = false;

    // 터널
    std::vector<double> tunnel_small_static_point_ = {};
    std::vector<double> tunnel_wall_ = {};
    std::vector<double> L_wall_ = {};
    std::vector<double> R_wall_ = {};
    double L_len_ = -1; // 길이를 라이다쪽에 저장할 이유가?
    double R_len_ = -1;
    std::vector<double> tunnel_lane_ = {};
    double tunnel_ceiling_ = 0;
    std::vector<double> tunnel_center_ = {};
    bool is_tunnel_end_ = false;
    int end_count_ = 0;

    // 유턴
    StaticPosCluster u_turn_point_cluster_;

    // 배달
    std::vector<double> delivery_flag_a;
    std::vector<double> delivery_flag_b; // TODO: 형식 인지랑 상의
    StaticClassCluster delivery_flag_a_cluster_;
    StaticClassCluster delivery_flag_b_cluster_;

public:
    /* 생성자 */
    // 클러스터링하는 기준 epsilon, min_point_size, max_point_size 조정 필요!!!!
    Lidar(nlohmann::json &param) : small_static_cluster_(param["small_static_eps"], param["small_static_min"], param["small_static_max"]),
                                   big_static_cluster_(param["big_static_eps"], param["big_static_min"], param["big_static_max"]),
                                   parking_point_cluster_(param["parking_point_eps"], param["parking_point_min"], param["parking_point_max"], 0.9999),
                                   delivery_flag_a_cluster_(param["delivery_flag_a_eps"], param["delivery_flag_a_min"], param["delivery_flag_a_max"]),
                                   delivery_flag_b_cluster_(param["delivery_flag_b_eps"], param["delivery_flag_b_min"], param["delivery_flag_b_max"]),
                                   u_turn_point_cluster_(param["u_turn_point_eps"], param["u_turn_point_min"], param["u_turn_point_max"])
    {
    }

    /* getter 함수 */
    const std::vector<std::vector<double>> &getRefSmallStaticPoint() const
    {
        return small_static_cluster_.getRefClusterTrustPoints();
    }

    const std::vector<double> &getRefBigStaticPoint() const
    {
        return big_static_point_;
    }

    const std::vector<double> &getRefParkingPoint() const
    {
        // return parking_point_cluster_.getRefClusterTrustPoints()[0];
        return parking_center_point;
    }

    const std::vector<std::vector<double>> &getRefParkingEdgeOut() const
    {
        return parking_edge_out;
    }

    bool getIsParkingStop() const
    {
        return is_parking_stop_;
    }

    const bool getIsDynamicStop() const
    {
        return is_dynamic_stop_;
    }

    const bool getDynamicMode()
    {
        return tunnel_dynamic_mode_;
    }

    const std::vector<double> &getRefTunnelSmallStaticPoint() const
    {
        return tunnel_small_static_point_;
    }
    const std::vector<double> &getRefTunnelWall()
    {
        return tunnel_wall_;
    }
    const std::vector<double> &getRefTunnelLWall()
    {
        return L_wall_;
    }
    const std::vector<double> &getRefTunnelRWall()
    {
        return R_wall_;
    }
    const double getTunnelLLen()
    {
        return L_len_;
    }
    const double getTunnelRLen()
    {
        return R_len_;
    }
    const std::vector<double> &getRefTunnelLane()
    {
        return tunnel_lane_;
    }
    const double getTunnelCeiling()
    {
        return tunnel_ceiling_;
    }
    const std::vector<double> &getRefTunnelCenter()
    {
        return tunnel_center_;
    }
    const bool getIsTunnelEnd()
    {
        return is_tunnel_end_;
    }
    const std::vector<std::vector<double>> &getRefUTurnPoint()
    {
        return u_turn_point_cluster_.getRefClusterTrustPoints();
    }
    const std::vector<double> &getRefDeliveryFlagA()
    {
        // return delivery_flag_a_cluster_.getRefClusterTrustPoints()[0];
        return delivery_flag_a;
    }
    const std::vector<double> &getRefDeliveryFlagB()
    {
        return delivery_flag_b;
    }
    const std::vector<std::vector<double>> &getRefAllDeliveryFlagB()
    {
        return delivery_flag_b_cluster_.getRefClusterTrustPoints();
    }

    /* setter 함수 */
    void setDynamicMode(bool does_dynamic_start)
    {
        tunnel_dynamic_mode_ = does_dynamic_start;
    }

    /* callback 함수 */
    void lidarSmallStaticCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) // 정적소형
    {
        for (size_t i = 0; i < msg->data.size(); i += 2)
        {
            std::vector<double> point(2, 0);
            point[0] = msg->data[i];
            point[1] = msg->data[i + 1];
            small_static_cluster_.update(point);
        }
    }

    void lidarDynamicCallback(const std_msgs::msg::Bool::SharedPtr msg) // 동적 장애물
    {
        is_dynamic_stop_ = msg->data;
    }

    void lidarBigStaticCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) // 정적 대형
    {
        // for (size_t i = 0; i < msg->data.size(); i += 2)
        // {
        //     std::vector<double> point(2, 0);
        //     point[0] = msg->data[i];
        //     point[1] = msg->data[i + 1];
        //     big_static_cluster_.update(point);
        // }
        // if (big_static_cluster_.getRefClusterTrustPoints().size() > 3)
        // {
        //     big_static_point_.clear();
        //     big_static_point_.push_back(big_static_cluster_.getRefClusterTrustPoints()[2][0]);
        //     big_static_point_.push_back(big_static_cluster_.getRefClusterTrustPoints()[2][1]);
        //     big_static_point_.push_back(big_static_cluster_.getRefClusterTrustPoints()[3][0]);
        //     big_static_point_.push_back(big_static_cluster_.getRefClusterTrustPoints()[3][1]);
        // }
        // else if (big_static_cluster_.getRefClusterTrustPoints().size() > 1)
        // {
        //     big_static_point_.clear();
        //     big_static_point_.push_back(big_static_cluster_.getRefClusterTrustPoints()[0][0]);
        //     big_static_point_.push_back(big_static_cluster_.getRefClusterTrustPoints()[0][1]);
        //     big_static_point_.push_back(big_static_cluster_.getRefClusterTrustPoints()[1][0]);
        //     big_static_point_.push_back(big_static_cluster_.getRefClusterTrustPoints()[1][1]);
        // }
        big_static_point_ = msg->data;
    }

    void lidarDelibaryFlagCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) // 배달 미션
    {
        for (size_t i = 0; i < msg->data.size(); i += 3)
        {
            std::vector<double> point(3, 0);
            point[0] = msg->data[i];
            point[1] = msg->data[i + 1];
            point[2] = msg->data[i + 2];
            if (point[2] > 0 && point[2] < 4) // 1~3: a flag
            {
                delivery_flag_a_cluster_.update(point);
            }
            if (point[2] > 3 && point[2] < 7) // 4~6: b flag
            {
                point[2] -= 3;
                delivery_flag_b_cluster_.update(point);
            }
        }
        if (delivery_flag_a_cluster_.getRefClusterTrustPoints().size() > 0)
            delivery_flag_a = delivery_flag_a_cluster_.getRefClusterTrustPoints()[0];
        if (delivery_flag_b_cluster_.getRefClusterTrustPoints().size() > 0)
            delivery_flag_b = delivery_flag_b_cluster_.getRefClusterTrustPoints()[0]; // TODO : 인지랑 협의 후 수정 필요
    }

    void lidarParkingPointCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) // 평행주차 주차박스 중점 포인트
    {
        if (msg->data.size() > 0)
        {
            std::vector<double> point(2, 0);
            point[0] = msg->data[0];
            point[1] = msg->data[1];
            parking_point_cluster_.update(point);
        }
        if (parking_point_cluster_.getRefClusterTrustPoints().size() > 0)
            parking_center_point = parking_point_cluster_.getRefClusterTrustPoints()[0];
    }

    void lidarParkingEdgeOutCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) // 평행주차 주차박스 중점 포인트
    {
        parking_edge_out.clear();
        for (size_t i = 0; i < msg->data.size(); i += 2)
        {
            std::vector<double> point(2, 0);
            point[0] = msg->data[i];
            point[1] = msg->data[i + 1];
            parking_edge_out.push_back(point);
        }
    }

    void lidarParkingStopCallback(const std_msgs::msg::Bool::SharedPtr msg) // 평주차 정지 메시지
    {
        // is_parking_stop_ = msg->data;
        if (msg->data)
        {
            is_parking_stop_ = true;
        }
    }

    void lidarUTurnPointCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) // U턴 포인트
    {
        for (size_t i = 0; i < msg->data.size(); i += 2)
        {
            std::vector<double> point(2, 0);
            point[0] = msg->data[i];
            point[1] = msg->data[i + 1];
            u_turn_point_cluster_.update(point);
        }
    }

    // 터널
    void lidarTunnelSmallStaticCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if ((msg->data).size() > 0)
        {
            tunnel_small_static_point_.clear(); // 초기화 후 업데이트
            for (size_t i = 0; i < msg->data.size(); i++)
            {
                tunnel_small_static_point_.emplace_back(msg->data[i]);
            }
        }
        else
        {
            std::cout << "터널 내부 정적 소형 감지 X" << std::endl;
        }
    }

    void lidarTunnelWallDistCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if ((msg->data).size() > 7)
        {
            tunnel_wall_.clear(); // 초기화 후 업데이트
            for (size_t i = 0; i < msg->data.size(); i++)
            {
                tunnel_wall_.emplace_back(msg->data[i]);
            }

            std::vector<double> wall01;
            std::vector<double> wall02;
            vectorSlice(wall01, tunnel_wall_, 0, 4);
            vectorSlice(wall02, tunnel_wall_, 4, 8);

            if (wall01[1] > wall02[1])
            {
                L_wall_ = wall01;
                R_wall_ = wall02;
            }
            else
            {
                L_wall_ = wall02;
                R_wall_ = wall01;
            }

            L_len_ = std::hypot(L_wall_[2] - L_wall_[0], L_wall_[3] - L_wall_[1]);
            R_len_ = std::hypot(R_wall_[2] - R_wall_[0], R_wall_[3] - R_wall_[1]);
        }
    }

    void lidarTunnelLaneCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (!(msg->data).empty())
        {
            tunnel_lane_.clear(); // 초기화 후 업데이트
            for (size_t i = 0; i < msg->data.size(); i++)
            {
                tunnel_lane_.emplace_back(msg->data[i]);
            }
        }
    }

    void lidarTunnelCeilingCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (!(msg->data).empty())
        {
            tunnel_ceiling_ = msg->data[0];
        }
        else
        {
            tunnel_ceiling_ = 0;
        }
    }

    void lidarTunnelCenterCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (!(msg->data).empty())
        {
            tunnel_center_.clear(); // 초기화 후 업데이트

            for (size_t i = 0; i < msg->data.size(); i++)
            {
                tunnel_center_.emplace_back(msg->data[i]);
            }
        }
    }

    void lidarTunnelEndCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        // 50번 연속 true(토픽이 10hz로 돌기에 5초 대기)여야 끝으로 간주
        // 실험해보고 30번도 해보기
        end_count_ = std::clamp((msg->data ? end_count_ + 1 : 0), 0, INT_MAX - 1);
        is_tunnel_end_ = ((end_count_ >= 50) ? true : false);
    }

    /* 기능 관련 함수 */
    void clearAllClusters()
    {
        big_static_cluster_.clearCluster();
        small_static_cluster_.clearCluster();
        parking_point_cluster_.clearCluster();
        delivery_flag_a_cluster_.clearCluster();
        delivery_flag_b_cluster_.clearCluster();
        u_turn_point_cluster_.clearCluster();
    }

    /* test용 함수 */
    void printAllData()
    {
        std::cout << "############## small static point ##############\n";
        for (auto n : small_static_cluster_.getRefClusterTrustPoints())
            std::cout << n[0] << " " << n[1];
        std::cout << "\n";
        std::cout << "############## big static point ##############\n";
        for (auto n : big_static_point_)
            std::cout << n << " ";
        std::cout << "\n";
        std::cout << "############## parking point ##############\n";
        for (auto n : parking_center_point)
            std::cout << n << " ";
        std::cout << "\n";
        // std::cout << "############## tunnel small static point ##############\n";
        // for (auto n : tunnel_small_static_point_)
        //     std::cout << n[0] << " " << n[1];
        // std::cout << "\n";
    }
};

#endif
