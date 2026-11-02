/***
 * 작성자: 박용현
 * 설명 : 배달왕 박용현 간다 ㅋㅋ
 * Style
 * -클래스: PascalCase
 * -멤버변수: snake_case_   (snake_case 뒤에 밑줄)(파이썬 사용 x)
 * -함수: camelCase
 * -변수: snake_case
 * -상수: kPascalCase   (PascalCase앞에 k 접두어)
 ***/

#ifndef _DELIVERY_NEW_H_
#define _DELIVERY_NEW_H_

#include <vector>
#include <iostream>
#include <cmath>
#include <fstream>
#include <algorithm>
#include "local.hpp"
#include "lidar.hpp"
#include "control.hpp"
#include "path.hpp"
#include "car_wait_timer.hpp"
#include "nlohmann/json.hpp"

class DeliveryNew
{
private:
    // State Enum
    enum class StateA
    {
        INIT,
        GO_SLOW,
        WAIT_FOR_DISTANCE,
        WAIT_FOR_TIMER,
        DONE
    };

    enum class StateB
    {
        INIT,
        FIND_DELIVERY_POINT,
        WAIT_FOR_CHANGE,
        CHANGE_DELIVERY_LANE,
        WAIT_FOR_DISTANCE,
        WAIT_FOR_TIMER,
        WAIT_FOR_GENERATE_R_PATH,
        GENERATE_REVERSE_PATH,
        REVERSE,
        WAIT_FOR_GENERATE_E_PATH,
        GENERATE_ESCAPE_PATH,
        GEAR_AND_PATH_CHANGE,
        ESCAPE,
        DONE
    };

    Local &local_;
    Lidar &lidar_;
    Path &local_path_;
    Control &control_;

    std::vector<double> delivery_a_pos_ = {302492.38457754906, 4123763.1814164612}; // 배달 정지 지점

    StateA state_a_ = StateA::INIT;
    StateB state_b_ = StateB::INIT;

    std::vector<double> delivery_stop_point_;

    /* 파라미터 상수 */
    const double kBChangeDistance_;
    const int kBChangeOffset_;
    const int kBEscapePathOffset;
    const double kBStopDistance_;
    const double kBReverseDistance_;

    const double kAStartVelocity_;
    const double kAEndVelocity_;
    const double kBStartVelocity_;
    const double kBReverseVelocity_;
    const double kBEndVelocity_;
    const double kBEscapeVelocity_;

public:
    DeliveryNew(Local &local, Lidar &lidar, Control &control, Path &local_path, nlohmann::json &param)
        : local_(local), lidar_(lidar), control_(control), local_path_(local_path),
          kAStartVelocity_(param["a_start_velocity"]), kAEndVelocity_(param["a_end_velocity"]),
          kBStartVelocity_(param["b_start_velocity"]), kBReverseVelocity_(param["b_reverse_velocity"]),
          kBEndVelocity_(param["b_end_velocity"]), kBChangeDistance_(param["b_change_distance"]),
          kBChangeOffset_(param["b_change_offset"]),
          kBEscapePathOffset(param["b_escape_path_offset"]), kBStopDistance_(param["b_stop_distance"]),
          kBReverseDistance_(param["b_reverse_distance"]), kBEscapeVelocity_(param["b_escape_velocity"])
    {
    }

    void deliveryA()
    {
        switch (state_a_)
        {
        case StateA::INIT:
        {
            control_.setTargetVelocity(kAStartVelocity_);
            delivery_stop_point_ = local_.getCurClosestPos(delivery_a_pos_);
            state_a_ = StateA::GO_SLOW;
            break;
        }
        case StateA::GO_SLOW:
        {
            const double *car_utm = local_.getAddCurCarUTMPos();
            std::vector<double> path_car_utm = local_path_.getClosestPos(car_utm);
            double distance_to_flag = std::hypot(delivery_stop_point_[0] - path_car_utm[0], delivery_stop_point_[1] - path_car_utm[1]);

            if (distance_to_flag < 10)
            {
                control_.setTargetVelocity(1.5);
                state_a_ = StateA::WAIT_FOR_DISTANCE;
            }
            break;
        }
        case StateA::WAIT_FOR_DISTANCE:
        {
            const double *car_utm = local_.getAddCurCarUTMPos();
            std::vector<double> path_car_utm = local_path_.getClosestPos(car_utm);
            double distance_to_flag = std::hypot(delivery_stop_point_[0] - path_car_utm[0], delivery_stop_point_[1] - path_car_utm[1]);

            if (distance_to_flag <= 0.64)
            {
                control_.setTargetVelocity(0);
                state_a_ = StateA::WAIT_FOR_TIMER;
            }
            break;
        }
        case StateA::WAIT_FOR_TIMER:
        {
            if (control_.waitAndDepart(5, kAEndVelocity_))
            {
                state_a_ = StateA::DONE;
            }
            break;
        }
        case StateA::DONE:
        {
            std::cout << "A DONE\n";
            break;
        }
        }
    }

    void deliveryB()
    {
        switch (state_b_)
        {
        case StateB::INIT:
        {
            control_.setTargetVelocity(kBStartVelocity_);
            state_b_ = StateB::FIND_DELIVERY_POINT;
            break;
        }
        case StateB::FIND_DELIVERY_POINT:
        {
            const std::vector<double> &b_flag = lidar_.getRefDeliveryFlagB();
            if (b_flag.size() > 0)
            {
                std::vector<double> delivery_point = {b_flag[0], b_flag[1]};
                delivery_stop_point_ = local_.getRefIdxPath(1).getClosestPos(delivery_point);
                state_b_ = StateB::WAIT_FOR_CHANGE;
            }
            else
            {
                std::cout << "이상함\n";
            }
            break;
        }
        case StateB::WAIT_FOR_CHANGE:
        {
            const double *cur_car_pos = local_.getAddCurCarUTMPos();
            std::vector<double> delivery_path_car_pos = local_.getRefIdxPath(1).getClosestPos(cur_car_pos);
            double distance = std::hypot(delivery_stop_point_[0] - delivery_path_car_pos[0], delivery_stop_point_[1] - delivery_path_car_pos[1]);

            std::cout << "차선 변경 대기 거리: " << distance << "\n";

            if (distance < kBChangeDistance_)
            {
                state_b_ = StateB::CHANGE_DELIVERY_LANE;
            }
            break;
        }
        case StateB::CHANGE_DELIVERY_LANE:
        {
            const Path &delivery_lane_global_path = local_.getRefIdxPath(1); // 다른 차선의 글로벌 패스
            const std::vector<std::vector<double>> &local_path_pos_arr = local_path_.getRefPosArr();
            const std::vector<std::vector<double>> &another_lane_pos_arr = delivery_lane_global_path.getRefPosArr();

            int cur_path_closest_index = local_path_.getClosestIndex(local_.getAddCurCarUTMPos());
            int cur_path_start_index = std::clamp(cur_path_closest_index, 0, static_cast<int>(local_path_.getRefPosArr().size()));
            int cur_path_end_index = std::clamp(cur_path_closest_index + 20, 2, static_cast<int>(local_path_.getRefPosArr().size()));

            int another_path_closest_index = delivery_lane_global_path.getClosestIndex(local_path_.getRefPosArr()[cur_path_end_index]);
            int another_path_start_index = another_path_closest_index + kBChangeOffset_;
            int another_path_end_index = another_path_start_index + 10;

            std::vector<std::vector<double>> spline_points;

            spline_points.assign(local_path_pos_arr.begin() + cur_path_start_index, local_path_pos_arr.begin() + cur_path_end_index);
            spline_points.insert(spline_points.end(), another_lane_pos_arr.begin() + another_path_start_index, another_lane_pos_arr.begin() + another_path_end_index);

            auto t = generateSplinePath(spline_points, 0.05);

            Path delivery_path("delivery_path", std::get<0>(t), std::get<1>(t), std::get<2>(t));
            local_path_.updatePath(cur_path_start_index, local_path_.getRefPosArr().size(), delivery_path, 0, std::get<0>(t).size());
            local_path_.updatePath(local_path_.getRefPosArr().size(), local_path_.getRefPosArr().size(), delivery_lane_global_path, another_path_end_index, delivery_lane_global_path.getRefPosArr().size());
            state_b_ = StateB::WAIT_FOR_DISTANCE;
            break;
        }
        case StateB::WAIT_FOR_DISTANCE:
        {
            const double *car_utm = local_.getAddCurCarUTMPos();
            std::vector<double> path_car_utm = local_.getRefIdxPath(1).getClosestPos(car_utm);
            double distance_to_flag = std::hypot(delivery_stop_point_[0] - path_car_utm[0], delivery_stop_point_[1] - path_car_utm[1]);

            if (distance_to_flag <= kBStopDistance_)
            {
                control_.setTargetVelocity(0);
                state_b_ = StateB::WAIT_FOR_GENERATE_R_PATH;
            }
            std::cout << "표지판까지 거리: " << distance_to_flag << "\n";
            break;
        }
        case StateB::WAIT_FOR_GENERATE_R_PATH:
        {
            if (control_.waitAndDepart(1, 0))
            {
                state_b_ = StateB::GENERATE_REVERSE_PATH;
            }
            break;
        }
        case StateB::GENERATE_REVERSE_PATH:
        {
            const Path &delivery_path = local_.getRefIdxPath(1);
            const std::vector<std::vector<double>> &delivery_path_pos_arr = delivery_path.getRefPosArr();
            const double *car_utm = local_.getAddCurCarUTMPos();
            int path_car_index = delivery_path.getClosestIndex(car_utm);
            // const std::vector<std::vector<double>> &delivery_path_pos_arr = delivery_path.getRefPosArr();
            // const std::vector<double> &delivery_path_yaw_arr = delivery_path.getRefYawArr();
            // const std::vector<double> &delivery_path_k_arr = delivery_path.getRefKArr();

            // std::vector<std::vector<double>> new_path_pos_arr(path_car_index + 1);
            // std::vector<double> new_path_yaw_arr(path_car_index + 1);
            // std::vector<double> new_path_k_arr(path_car_index + 1);

            // for (int i = path_car_index; i >= 0; --i)
            // {
            //     new_path_pos_arr[path_car_index - i] = delivery_path_pos_arr[i];
            //     new_path_yaw_arr[path_car_index - i] = delivery_path_yaw_arr[i];
            //     new_path_k_arr[path_car_index - i] = delivery_path_k_arr[i];
            // }

            std::vector<double> reverse_vector = {delivery_path_pos_arr[path_car_index - 1][0] - delivery_path_pos_arr[path_car_index][0], delivery_path_pos_arr[path_car_index - 1][1] - delivery_path_pos_arr[path_car_index][1]};

            std::vector<std::vector<double>> new_path_pos_arr;
            std::vector<double> new_path_yaw_arr;
            std::vector<double> new_path_k_arr;

            for (int i = 1; i <= path_car_index; ++i)
            {
                new_path_pos_arr.push_back({car_utm[0] + i * reverse_vector[0], car_utm[1] + i * reverse_vector[1]});
                new_path_yaw_arr.push_back(delivery_path.getRefYawArr()[path_car_index]);
                new_path_k_arr.push_back(0);
            }

            Path reverse_path("reverse_path", new_path_pos_arr, new_path_yaw_arr, new_path_k_arr);
            local_path_.updatePath(0, local_path_.getRefPosArr().size(), reverse_path, 0, reverse_path.getRefPosArr().size());

            state_b_ = StateB::WAIT_FOR_TIMER;
            break;
        }
        case StateB::WAIT_FOR_TIMER:
        {
            if (control_.waitAndDepart(5, kBReverseVelocity_))
            {
                state_b_ = StateB::REVERSE;
            }
            break;
        }
        case StateB::REVERSE:
        {
            const double *car_utm = local_.getAddCurCarUTMPos();
            std::vector<double> path_car_utm = local_path_.getClosestPos(car_utm);
            double distance_to_flag = std::hypot(delivery_stop_point_[0] - path_car_utm[0], delivery_stop_point_[1] - path_car_utm[1]);

            if (distance_to_flag > kBReverseDistance_)
            {
                control_.setTargetVelocity(0);
                state_b_ = StateB::WAIT_FOR_GENERATE_E_PATH;
            }
            break;
        }
        case StateB::WAIT_FOR_GENERATE_E_PATH:
        {
            if (control_.waitAndDepart(1, 0))
            {
                state_b_ = StateB::GENERATE_ESCAPE_PATH;
            }
            break;
        }
        case StateB::GENERATE_ESCAPE_PATH:
        {
            std::vector<std::vector<double>> spline_points;
            const double *car_utm = local_.getAddCurCarUTMPos();
            int first_path_start_index = local_path_.getClosestIndex(car_utm);
            spline_points.push_back(local_path_.getRefPosArr()[first_path_start_index]);
            int second_path_start_index = local_.getRefIdxPath(0).getClosestIndex(car_utm) + kBEscapePathOffset;
            for (size_t i = 0; i < 10; ++i)
            {
                spline_points.push_back(local_.getRefIdxPath(0).getRefPosArr()[second_path_start_index + i]);
            }
            auto t = generateSplinePath(spline_points, 0.05);
            Path escape_path("escape_path", std::get<0>(t), std::get<1>(t), std::get<2>(t));
            local_path_.updatePath(0, local_path_.getRefPosArr().size(), escape_path, 0, std::get<0>(t).size());
            local_path_.updatePath(local_path_.getRefPosArr().size(), local_path_.getRefPosArr().size(), local_.getRefIdxPath(0), second_path_start_index + 10, local_.getRefIdxPath(0).getRefPosArr().size());

            state_b_ = StateB::GEAR_AND_PATH_CHANGE;

            break;
        }
        case StateB::GEAR_AND_PATH_CHANGE:
        {
            if (control_.waitAndDepart(1, kBEscapeVelocity_))
            {
                state_b_ = StateB::ESCAPE;
            }
            break;
        }
        case StateB::ESCAPE:
        {
            control_.setTargetVelocity(kBEscapeVelocity_);

            const double *car_utm = local_.getAddCurCarUTMPos();
            std::vector<double> path_car_utm = local_path_.getClosestPos(car_utm);
            double distance_to_flag = std::hypot(delivery_stop_point_[0] - path_car_utm[0], delivery_stop_point_[1] - path_car_utm[1]);

            std::cout << "탈출중~~~~\n";

            if (distance_to_flag > 10)
            {
                state_b_ = StateB::DONE;
            }

            break;
        }
        case StateB::DONE:
        {
            control_.setTargetVelocity(kBEndVelocity_);
            std::cout << "B 배달 완료\n";
            break;
        }
        }
    }
};

#endif
