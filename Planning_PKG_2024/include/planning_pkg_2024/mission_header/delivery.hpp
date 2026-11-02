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

#ifndef _DELIVERY_H_
#define _DELIVERY_H_

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

class Delivery
{
private:
    // State Enum
    enum class StateA
    {
        INIT,
        WAIT_FOR_DISTANCE,
        WAIT_FOR_TIMER,
        WAIT_FOR_LIDAR,
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
        GENERATE_REVERSE_PATH,
        REVERSE,
        GEAR_AND_PATH_CHANGE,
        DONE
    };

    Local &local_;
    Lidar &lidar_;
    Path &local_path_;
    Control &control_;

    std::vector<double> delivery_a_pos_ = {346303.9996086832, 4069644.2808275241}; // 배달 정지 지점

    StateA state_a_ = StateA::INIT;
    StateB state_b_ = StateB::INIT;

    std::vector<double> a_flag_;
    std::vector<double> delivery_stop_point_;
    int delivery_stop_index_;

    /* 파라미터 상수 */
    const double kBChangeDistance_;
    const int kBChangeOffset_;
    const int kBEscapePathOffset;
    const double kBReverseDistance_;

    const double kAStartVelocity_;
    const double kAEndVelocity_;
    const double kBStartVelocity_;
    const double kBReverseVelocity_;
    const double kBEndVelocity_;

    // 파일 입출력을 위한 함수
    void saveAFlagToFile(const std::string &filename)
    {
        std::ofstream outfile(filename);
        if (outfile.is_open())
        {
            for (const auto &value : a_flag_)
            {
                outfile << value << " ";
            }
            outfile.close();
        }
        else
        {
            std::cerr << "파일을 열 수 없습니다: " << filename << "\n";
        }
    }

    void loadAFlagFromFile(const std::string &filename)
    {
        std::ifstream infile(filename);
        if (infile.is_open())
        {
            a_flag_.clear();
            double value;
            while (infile >> value)
            {
                a_flag_.push_back(value);
            }
            infile.close();
        }
        else
        {
            std::cerr << "파일을 열 수 없습니다: " << filename << "\n";
        }
    }

public:
    Delivery(Local &local, Lidar &lidar, Control &control, Path &local_path, nlohmann::json &param)
        : local_(local), lidar_(lidar), control_(control), local_path_(local_path),
          kAStartVelocity_(param["a_start_velocity"]), kAEndVelocity_(param["a_end_velocity"]),
          kBStartVelocity_(param["b_start_velocity"]), kBReverseVelocity_(param["b_reverse_velocity"]),
          kBEndVelocity_(param["b_end_velocity"]), kBChangeDistance_(param["b_change_distance"]),
          kBChangeOffset_(param["b_change_offset"]),
          kBEscapePathOffset(param["b_escape_path_offset"]),
          kBReverseDistance_(param["b_reverse_distance"])
    {
        a_flag_.reserve(3);
        a_flag_[0] = 0;
        a_flag_[1] = 0;
        a_flag_[2] = 1;
    }

    void doDeliveryAMission()
    {
        switch (state_a_)
        {
        case StateA::INIT:
        {
            control_.setTargetVelocity(kAStartVelocity_);
            delivery_stop_point_ = local_.getCurClosestPos(delivery_a_pos_);
            state_a_ = StateA::WAIT_FOR_DISTANCE;
            break;
        }
        case StateA::WAIT_FOR_DISTANCE:
        {
            const double *car_utm = local_.getAddCurCarUTMPos();
            std::vector<double> path_car_utm = local_path_.getClosestPos(car_utm);
            double distance_to_flag = std::hypot(delivery_stop_point_[0] - path_car_utm[0], delivery_stop_point_[1] - path_car_utm[1]);

            if (control_.decelByDistanceToStop(distance_to_flag))
            {
                state_a_ = StateA::WAIT_FOR_TIMER;
            }
            break;
        }
        case StateA::WAIT_FOR_TIMER:
        {
            if (control_.waitAndDepart(5, kAEndVelocity_))
            {
                state_a_ = StateA::WAIT_FOR_LIDAR;
            }
            break;
        }
        case StateA::WAIT_FOR_LIDAR:
        {
            if (!lidar_.getRefDeliveryFlagA().empty())
            {
                a_flag_ = lidar_.getRefDeliveryFlagA();
                saveAFlagToFile("./src/Planning_PKG_2024/data/a_flag.txt"); // a_flag를 파일에 저장
                state_a_ = StateA::DONE;
            }
            break;
        }
        case StateA::DONE:
        {
            std::cout << "A Flag: " << a_flag_[2] << "\n";
            state_a_ = StateA::DONE;
            break;
        }
        default:
        {
            std::cout << "default\n";
        }
        }

        // test용 출력
        if (!lidar_.getRefDeliveryFlagA().empty())
            std::cout << "flag: " << lidar_.getRefDeliveryFlagA()[2] << "\n";
        else
            std::cout << "flag 값 없음\n";
    }

    void doDeliveryBMission()
    {
        switch (state_b_)
        {
        case StateB::INIT:
        {
            control_.setTargetVelocity(kBStartVelocity_);
            loadAFlagFromFile("./src/Planning_PKG_2024/data/a_flag.txt");
            state_b_ = StateB::FIND_DELIVERY_POINT;
            break;
        }
        case StateB::FIND_DELIVERY_POINT:
        {
            const std::vector<std::vector<double>> &b_flags = lidar_.getRefAllDeliveryFlagB();
            for (const auto &flag : b_flags)
            {
                if (flag[2] == a_flag_[2])
                {
                    std::vector<double> delivery_point = {flag[0], flag[1]};
                    delivery_stop_index_ = local_.getRefIdxPath(1).getClosestIndex(delivery_point);
                    delivery_stop_point_ = local_.getRefIdxPath(1).getRefPosArr()[delivery_stop_index_];
                    state_b_ = StateB::WAIT_FOR_CHANGE;
                    break;
                }
            }
            break;
        }
        case StateB::WAIT_FOR_CHANGE:
        {
            const double *cur_car_pos = local_.getAddCurCarUTMPos();
            std::vector<double> cur_closest_pos = local_.getRefIdxPath(1).getClosestPos(cur_car_pos);
            double distance = std::hypot(delivery_stop_point_[0] - cur_closest_pos[0], delivery_stop_point_[1] - cur_closest_pos[1]);

            if (distance < kBChangeDistance_)
            {
                state_b_ = StateB::CHANGE_DELIVERY_LANE;
            }
            break;
        }
        case StateB::CHANGE_DELIVERY_LANE:
        {
            const Path &another_lane_global_path = local_.getRefIdxPath(1); // 다른 차선의 글로벌 패스
            const std::vector<std::vector<double>> &local_path_pos_arr = local_path_.getRefPosArr();
            const std::vector<std::vector<double>> &another_lane_pos_arr = another_lane_global_path.getRefPosArr();

            int cur_path_closest_index = local_path_.getClosestIndex(local_.getAddCurCarUTMPos());
            int cur_path_end_index = std::clamp(cur_path_closest_index, 0, static_cast<int>(local_path_.getRefPosArr().size()));
            int cur_path_start_index = std::clamp(cur_path_closest_index + 5, 2, static_cast<int>(local_path_.getRefPosArr().size()));

            int another_path_closest_index = another_lane_global_path.getClosestIndex(local_path_.getRefPosArr()[cur_path_end_index]);
            int another_path_start_index = another_path_closest_index + kBChangeOffset_;
            int another_path_end_index = another_path_start_index + 5;

            std::vector<std::vector<double>> spline_points;

            spline_points.assign(local_path_pos_arr.begin() + cur_path_start_index, local_path_pos_arr.begin() + cur_path_end_index);
            spline_points.insert(spline_points.end(), another_lane_pos_arr.begin() + another_path_start_index, another_lane_pos_arr.begin() + another_path_end_index);

            auto t = generateSplinePath(spline_points, 0.05);

            Path delivery_path("delivery_path", std::get<0>(t), std::get<1>(t), std::get<2>(t));
            local_path_.updatePath(cur_path_start_index, local_path_.getRefPosArr().size(), delivery_path, 0, std::get<0>(t).size());
            local_path_.updatePath(local_path_.getRefPosArr().size(), local_path_.getRefPosArr().size(), another_lane_global_path, another_path_end_index, another_lane_global_path.getRefPosArr().size());
            state_b_ = StateB::WAIT_FOR_DISTANCE;
            break;
        }
        case StateB::WAIT_FOR_DISTANCE:
        {
            const double *car_utm = local_.getAddCurCarUTMPos();
            std::vector<double> path_car_utm = local_path_.getClosestPos(car_utm);
            double distance_to_flag = std::hypot(delivery_stop_point_[0] - path_car_utm[0], delivery_stop_point_[1] - path_car_utm[1]);

            if (control_.decelByDistanceToStop(distance_to_flag))
            {
                state_b_ = StateB::WAIT_FOR_TIMER;
            }
            std::cout << "distance to flag: " << distance_to_flag << "\n";
            break;
        }
        case StateB::WAIT_FOR_TIMER:
        {
            if (control_.waitAndDepart(5, kBReverseVelocity_))
            {
                state_b_ = StateB::GENERATE_REVERSE_PATH;
            }
            break;
        }
        case StateB::GENERATE_REVERSE_PATH:
        {
            const double *car_utm = local_.getAddCurCarUTMPos();
            int path_car_index = local_path_.getClosestIndex(car_utm);
            const std::vector<std::vector<double>> &local_path_pos_arr = local_path_.getRefPosArr();
            const std::vector<double> &local_path_yaw_arr = local_path_.getRefYawArr();
            const std::vector<double> &local_path_k_arr = local_path_.getRefKArr();

            std::vector<std::vector<double>> new_path_pos_arr(path_car_index + 1);
            std::vector<double> new_path_yaw_arr(path_car_index + 1);
            std::vector<double> new_path_k_arr(path_car_index + 1);

            for (int i = path_car_index; i >= 0; --i)
            {
                new_path_pos_arr[path_car_index - i] = local_path_pos_arr[i];
                new_path_yaw_arr[path_car_index - i] = local_path_yaw_arr[i];
                new_path_k_arr[path_car_index - i] = local_path_k_arr[i];
            }

            Path reverse_path("reverse_path", new_path_pos_arr, new_path_yaw_arr, new_path_k_arr);
            local_path_.updatePath(0, local_path_.getRefPosArr().size(), reverse_path, 0, reverse_path.getRefPosArr().size());

            state_b_ = StateB::REVERSE;
            break;
        }
        case StateB::REVERSE:
        {
            const double *car_utm = local_.getAddCurCarUTMPos();
            std::vector<double> path_car_utm = local_path_.getClosestPos(car_utm);
            double distance_to_flag = std::hypot(delivery_stop_point_[0] - path_car_utm[0], delivery_stop_point_[1] - path_car_utm[1]);

            if (distance_to_flag > kBReverseDistance_)
            {
                state_b_ = StateB::GEAR_AND_PATH_CHANGE;
            }
            break;
        }
        case StateB::GEAR_AND_PATH_CHANGE:
        {
            if (control_.waitAndDepart(1, kBEndVelocity_))
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

                state_b_ = StateB::DONE;
            }
            break;
        }
        case StateB::DONE:
        {
            std::cout << "B 배달 완료\n";
            break;
        }
        default:
        {
            std::cout << "default\n";
        }
        }

        // test용 출력
        std::cout << "a_flag: " << a_flag_[2] << "\n";
        if (!lidar_.getRefAllDeliveryFlagB().empty())
        {
            for (const auto &n : lidar_.getRefAllDeliveryFlagB())
            {
                std::cout << "좌표: " << n[0] << " " << n[1] << "  b_flag: " << n[2] << "\n";
            }
        }
        else
        {
            std::cout << "b flag 값 없음\n";
        }
    }
};

#endif
