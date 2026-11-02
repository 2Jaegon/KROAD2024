/***
 * 작성자: 김호진
 * 설명: 정적대형 관련 클래스
 * Style
 * -클래스: PascalCase
 * -멤버변수: snake_case_   (snake_case 뒤에 밑줄)(파이썬 사용 x)
 * -함수: camelCase
 * -변수: snake_case
 * -상수: kPascalCase   (PascalCase앞에 k 접두어)
 ***/

#ifndef BIG_STATIC_H_
#define BIG_STATIC_H_

#include <algorithm>
#include "local.hpp"
#include "lidar.hpp"
#include "control.hpp"
#include "path.hpp"
#include "control.hpp"
#include "nlohmann/json.hpp"

class BigStatic
{
private:
    // Define states for the state machine
    enum class State
    {
        INIT,
        CHECK_OBSTACLE,
        PATH_UPDATE,
        LANE_CHANGE,
        WAIT_FOR_NEXT_CHANGE,
        DONE
    };

    Local &local_;
    Lidar &lidar_;
    Control &control_;
    Path &local_path_;

    // 상태변수
    State current_state_ = State::INIT;
    int cur_lane_idx_ = 0;
    int change_lane_count_ = 0;
    bool lane_change_flag_ = false;

    std::vector<double> before_obs_path_pos_;
    const double obj_dis_thrseshold_ = 2.0;

    /* 파라미터 상수 */
    const int kSplineOffset_;
    const int kLaneChangeOffset_;
    const double kStartVelocity_;
    const double kStraightVelocity_;
    const double kChangeVelocity_;
    const double kEndVelocity_;
    const double kCarLength_;

public:
    BigStatic(Local &local, Lidar &lidar, Control &control, Path &local_path, nlohmann::json &param)
        : local_(local), lidar_(lidar), control_(control), local_path_(local_path),
          kStartVelocity_(param["start_velocity"]), kStraightVelocity_(param["straight_velocity"]),
          kChangeVelocity_(param["lane_change_velocity"]), kEndVelocity_(param["end_velocity"]),
          kLaneChangeOffset_(param["lane_change_offset"]), kCarLength_(param["car_length"]),
          kSplineOffset_(param["spline_offset"])
    {
    }

    void doBigStaticMission()
    {
        std::cout << "현재 상태: " << static_cast<int>(current_state_) << "\n";

        const double *car_pos = local_.getAddCurCarUTMPos();
        const std::vector<double> &big_static_points = lidar_.getRefBigStaticPoint(); // fst : xy xy // snd xy xy

        switch (current_state_)
        {
        case State::INIT:
        {
            control_.setTargetVelocity(kStartVelocity_);
            current_state_ = State::CHECK_OBSTACLE;
            break;
        }

        case State::CHECK_OBSTACLE:
        {
            const Path &cur_lane_global_path = local_.getRefIdxPath(cur_lane_idx_); // 현재 차선의 글로벌 패스
            const std::vector<std::vector<double>> &cur_lane_pos_arr = cur_lane_global_path.getRefPosArr();

            if (change_lane_count_ >= 1)
            {
                control_.setTargetVelocity(kChangeVelocity_);
            }
            else
            {
                control_.setTargetVelocity(kStraightVelocity_);
            }

            if (checkLaneObstacle(cur_lane_global_path, big_static_points))
            {
                lane_change_flag_ = true;
            }

            if (lane_change_flag_)
            {
                lane_change_flag_ = false;
                current_state_ = State::PATH_UPDATE;
            }

            break;
        }

        case State::PATH_UPDATE:
        {
            const Path &another_lane_global_path = local_.getRefIdxPath((cur_lane_idx_ + 1) % 2); // 다른 차선의 글로벌 패스
            const std::vector<std::vector<double>> &another_lane_pos_arr = another_lane_global_path.getRefPosArr();
            const std::vector<std::vector<double>> &local_path_pos_arr = local_path_.getRefPosArr();

            std::vector<double> obj_left = {big_static_points[0], big_static_points[1]};                                                             // 장애물의 왼쪽 끝 좌표
            std::vector<double> obj_right = {big_static_points[2], big_static_points[3]};                                                            // 장애물의 오른쪽 끝 좌표
            std::vector<double> obj_center = {(big_static_points[0] + big_static_points[2]) / 2, (big_static_points[1] + big_static_points[3]) / 2}; // x y

            std::cout << "*** --- 현재 글로벌 패스 상에 장애물 있음 --- ***" << std::endl;
            std::cout << "*** --- 경로 변경 준비 --- ***" << std::endl;

            std::vector<std::vector<double>> spline_points;

            int obj_index = local_path_.getClosestIndex(obj_center);
            int cur_path_index = local_path_.getClosestIndex(local_.getAddCurCarUTMPos());
            int cur_path_start_index = std::clamp(cur_path_index, 0, static_cast<int>(local_path_pos_arr.size()) - 2);
            int cur_path_end_index = std::clamp(cur_path_index + kSplineOffset_, 2, static_cast<int>(local_path_pos_arr.size()));

            int another_path_start_index = std::clamp(another_lane_global_path.getClosestIndex(obj_center) - kLaneChangeOffset_, 0, static_cast<int>(another_lane_pos_arr.size()) - 2);
            int another_path_end_index = std::clamp(another_path_start_index + kSplineOffset_, 2, static_cast<int>(another_lane_pos_arr.size()));

            spline_points.assign(local_path_pos_arr.begin() + cur_path_start_index, local_path_pos_arr.begin() + cur_path_end_index);
            spline_points.insert(spline_points.end(), another_lane_pos_arr.begin() + another_path_start_index, another_lane_pos_arr.begin() + another_path_end_index);

            auto t = generateSplinePath(spline_points, 0.05);
            std::cout << "새로운 경로 생성\n";

            Path evade_path("evadePath", std::get<0>(t), std::get<1>(t), std::get<2>(t));
            local_path_.updatePath(cur_path_start_index, local_path_.getRefPosArr().size(), evade_path, 0, std::get<0>(t).size());
            local_path_.updatePath(local_path_.getRefPosArr().size(), local_path_.getRefPosArr().size(), another_lane_global_path, another_path_end_index, another_lane_global_path.getRefPosArr().size());

            std::cout << "경로 변경 완료" << std::endl;
            cur_lane_idx_ = (cur_lane_idx_ + 1) % 2;

            before_obs_path_pos_ = local_path_.getClosestPos(obj_center);
            current_state_ = State::LANE_CHANGE;

            break;
        }

        case State::LANE_CHANGE:
        {
            control_.setTargetVelocity(kChangeVelocity_);
            const double *car_utm = local_.getAddCurCarUTMPos();
            std::vector<double> path_car_utm = local_path_.getClosestPos(car_utm);
            double distance_to_change_lane = std::hypot(before_obs_path_pos_[0] - path_car_utm[0], before_obs_path_pos_[1] - path_car_utm[1]);
            std::cout << "distance_to_change_lane: " << distance_to_change_lane << "\n";

            if (distance_to_change_lane < 0.2)
            {
                change_lane_count_++;
                if (change_lane_count_ == 2)
                    current_state_ = State::DONE;
                else
                {
                    current_state_ = State::WAIT_FOR_NEXT_CHANGE;
                }
            }

            break;
        }

        case State::WAIT_FOR_NEXT_CHANGE:
        {
            control_.setTargetVelocity(kChangeVelocity_);
            const double *car_utm = local_.getAddCurCarUTMPos();
            std::vector<double> path_car_utm = local_path_.getClosestPos(car_utm);
            const Path &cur_lane_global_path = local_.getRefIdxPath(cur_lane_idx_); // 현재 차선의 글로벌 패스

            double distance_to_car_end = std::hypot(before_obs_path_pos_[0] - path_car_utm[0], before_obs_path_pos_[1] - path_car_utm[1]);
            std::cout << "distance_to_car_end: " << distance_to_car_end << "\n";

            if (checkLaneObstacle(cur_lane_global_path, big_static_points))
            {
                lane_change_flag_ = true;
            }

            if (distance_to_car_end > kCarLength_)
            {
                current_state_ = State::CHECK_OBSTACLE;
            }

            break;
        }

        case State::DONE:
        {
            std::cout << "정적 대형 미션 종료\n";
            control_.setTargetVelocity(kEndVelocity_);
            break;
        }
        }
    }

    bool checkLaneObstacle(const Path &cur_lane_global_path, const std::vector<double> &big_static_points)
    {
        const std::vector<std::vector<double>> &cur_lane_pos_arr = cur_lane_global_path.getRefPosArr();

        if (big_static_points.size() >= 4)
        {
            for (size_t i = 0; i < big_static_points.size(); i += 4)
            {
                std::vector<double> obj_left = {big_static_points[i], big_static_points[i + 1]};                                                                     // 장애물의 왼쪽 끝 좌표
                std::vector<double> obj_right = {big_static_points[i + 2], big_static_points[i + 3]};                                                                // 장애물의 오른쪽 끝 좌표
                std::vector<double> obj_center = {(big_static_points[i] + big_static_points[i + 2]) / 2, (big_static_points[i + 1] + big_static_points[i + 3]) / 2}; // x y

                // 거리 기반
                double obj_to_lane_dis = cur_lane_global_path.getClosestDistance(obj_center);
                std::cout << "글로벌 패스와 장애물 사이 거리" << obj_to_lane_dis << std::endl;
                std::cout << "장애물 거리 " << obj_to_lane_dis << std::endl;

                if (obj_to_lane_dis < obj_dis_thrseshold_)
                {
                    return true;
                }
            }
        }
        return false;
    }
};

#endif
