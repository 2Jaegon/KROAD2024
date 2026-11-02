/***
 * 작성자: 정성윤
 * 설명: 정적 소형 회피 알고리즘 관련 클래스(회피점 생성, 회피경로 생성) 헤더파일(모듈화)
 * Style
 * -클래스: PascalCase
 * -멤버변수: snake_case_   (snake_case 뒤에 밑줄)(파이썬 사용 x)
 * -함수: camelCase
 * -변수: snake_case
 * -상수: kPascalCase   (PascalCase앞에 k 접두어)
 ***/

#ifndef EVADE_CIRCLE_POINT_GENERATOR_H_
#define EVADE_CIRCLE_POINT_GENERATOR_H_

#include <iostream>
#include <vector>
#include <cmath>
#include "lidar.hpp"
#include "local.hpp"
#include "path.hpp"
#include "control.hpp"
#include "utility_function.hpp"
#include "nlohmann/json.hpp"

class EvadeCirclePointGenerator
{
private:
    const Path &original_path_;
    const std::vector<std::vector<double>> &obstacle_points_;
    int first_index_offset_ = 60;
    int last_index_offset_ = 60;

    // 2D 벡터 간 외적을 계산하여 방향을 판단하는 함수, (+) -> 왼쪽(반시계), (-) -> 오른쪽(시계)
    double crossProduct(const std::vector<double> &u, const std::vector<double> &v)
    {
        return u[0] * v[1] - u[1] * v[0];
    }

public:
    EvadeCirclePointGenerator(const Path &original_path, const std::vector<std::vector<double>> &obstacle_points, int first_index_offset, int last_index_offset)
        : original_path_(original_path), obstacle_points_(obstacle_points), first_index_offset_(first_index_offset), last_index_offset_(last_index_offset)
    {
    }

    std::vector<std::vector<double>> getEvadePoint(const double *start_car_pos, double inner_ratio, double radian, double inner_distance, double evade_distance)
    {
        std::vector<std::vector<double>> evade_points;
        std::vector<std::vector<double>> obs_points = obstacle_points_;

        // 장애물 포인트를 자동차 위치와의 거리로 정렬
        std::sort(obs_points.begin(), obs_points.end(), [&start_car_pos](const std::vector<double> &a, const std::vector<double> &b)
                  { double dist_a = std::hypot(start_car_pos[0] - a[0], start_car_pos[1] - a[1]);
                    double dist_b = std::hypot(start_car_pos[0] - b[0], start_car_pos[1] - b[1]);
                    return dist_a < dist_b; });

        int first_closest_index = original_path_.getClosestIndex(obs_points.front());
        int last_closest_index = original_path_.getClosestIndex(obs_points.back());

        if (first_closest_index - first_index_offset_ - 10 < 0)
        {
            evade_points.push_back(original_path_.getRefPosArr()[0]);
            evade_points.push_back(original_path_.getRefPosArr()[1]);
        }
        else
        {
            for (size_t i = 0; i < 10; ++i)
            {
                evade_points.push_back(original_path_.getRefPosArr()[first_closest_index - first_index_offset_ - 10 + i]);
            }
        }

        for (size_t i = 0; i < obs_points.size() - 1; ++i)
        {
            const std::vector<double> &obs_pos = obs_points[i];

            double vec_size = std::hypot(obs_points[i + 1][0] - obs_points[i][0], obs_points[i + 1][1] - obs_points[i][1]);
            std::vector<double> obs_unit_direction_vec = {(obs_points[i + 1][0] - obs_points[i][0]) / vec_size, (obs_points[i + 1][1] - obs_points[i][1]) / vec_size};
            int closest_index = original_path_.getClosestIndex(obs_points[i]);
            vec_size = std::hypot(original_path_.getRefPosArr()[closest_index + 1][0] - original_path_.getRefPosArr()[closest_index][0], original_path_.getRefPosArr()[closest_index + 1][1] - original_path_.getRefPosArr()[closest_index][1]);
            std::vector<double> path_unit_direction_vec = {(original_path_.getRefPosArr()[closest_index + 1][0] - original_path_.getRefPosArr()[closest_index][0]) / vec_size, (original_path_.getRefPosArr()[closest_index + 1][1] - original_path_.getRefPosArr()[closest_index][1]) / vec_size};

            double sin_value = crossProduct(path_unit_direction_vec, obs_unit_direction_vec);

            int sign = 0;

            if (sin_value <= 0) // 오른쪽(시계방향)
                sign = 1;
            else
                sign = -1;

            double inner_cos_theta = std::cos(sign * radian);
            double inner_sin_theta = std::sin(sign * radian);

            std::vector<double> middle_evade_unit_vec = {path_unit_direction_vec[0] * inner_cos_theta - path_unit_direction_vec[1] * inner_sin_theta, path_unit_direction_vec[0] * inner_sin_theta + path_unit_direction_vec[1] * inner_cos_theta};

            std::vector<double> middle_evade_vec = {middle_evade_unit_vec[0] * inner_distance, middle_evade_unit_vec[1] * inner_distance};

            std::vector<double> middle_pos(2, 0);
            middle_pos[0] = obs_pos[0] * inner_ratio + obs_points[i + 1][0] * (1 - inner_ratio);
            middle_pos[1] = obs_pos[1] * inner_ratio + obs_points[i + 1][1] * (1 - inner_ratio);

            std::vector<double> first_middle_point = {middle_pos[0] - middle_evade_vec[0], middle_pos[1] - middle_evade_vec[1]};
            std::vector<double> seconde_middle_point = {middle_pos[0] + middle_evade_vec[0], middle_pos[1] + middle_evade_vec[1]};

            std::vector<double> evade_direction_vec = {sign * path_unit_direction_vec[1], -sign * path_unit_direction_vec[0]};

            std::vector<double> first_evade_point = {obs_pos[0] + evade_direction_vec[0] * evade_distance,
                                                     obs_pos[1] + evade_direction_vec[1] * evade_distance};

            std::vector<double> second_evade_point = {obs_points[i + 1][0] - evade_direction_vec[0] * evade_distance,
                                                      obs_points[i + 1][1] - evade_direction_vec[1] * evade_distance};

            if (i == 0)
            {
                evade_points.push_back(first_evade_point);
            }
            evade_points.push_back(first_middle_point);
            evade_points.push_back(middle_pos);
            evade_points.push_back(seconde_middle_point);
            evade_points.push_back(second_evade_point);
        }

        if (last_closest_index + last_index_offset_ + 9 >= original_path_.getRefPosArr().size())
        {
            evade_points.push_back(original_path_.getRefPosArr()[original_path_.getRefPosArr().size() - 2]);
            evade_points.push_back(original_path_.getRefPosArr()[original_path_.getRefPosArr().size() - 1]);
        }
        else
        {
            for (size_t i = 0; i < 10; ++i)
            {
                evade_points.push_back(original_path_.getRefPosArr()[last_closest_index + last_index_offset_ + i]);
            }
        }

        return evade_points;
    }

    std::vector<std::vector<double>> getVirtualEvadePoint(const double *start_car_pos, double inner_ratio, double radian, double inner_distance, double evade_distance)
    {
        double forward_distance = 3;
        double right_distance = 1.5;

        std::vector<std::vector<double>> evade_points;
        std::vector<std::vector<double>> obs_points = obstacle_points_;

        int closest_index = original_path_.getClosestIndex(obs_points[0]);
        double vec_size = std::hypot(original_path_.getRefPosArr()[closest_index + 1][0] - original_path_.getRefPosArr()[closest_index][0], original_path_.getRefPosArr()[closest_index + 1][1] - original_path_.getRefPosArr()[closest_index][1]);
        std::vector<double> path_unit_direction_vec = {(original_path_.getRefPosArr()[closest_index + 1][0] - original_path_.getRefPosArr()[closest_index][0]) / vec_size, (original_path_.getRefPosArr()[closest_index + 1][1] - original_path_.getRefPosArr()[closest_index][1]) / vec_size};

        // 첫 번째 장애물 위치
        const std::vector<double> &first_obstacle = obs_points.front();
        // Path 방향으로 forward_distance 만큼 이동한 가상의 장애물 위치
        std::vector<double> virtual_obstacle = {
            first_obstacle[0] + path_unit_direction_vec[0] * forward_distance, // 경로 방향으로 forward_distance 만큼 이동
            first_obstacle[1] + path_unit_direction_vec[1] * forward_distance};

        // 오른쪽으로 right_distance 만큼 이동 (Path의 오른쪽 방향은 path_unit_direction의 수직 방향)
        std::vector<double> right_offset = {path_unit_direction_vec[1] * right_distance, -path_unit_direction_vec[0] * right_distance};

        // 가상의 장애물 위치에 오른쪽 이동 적용
        virtual_obstacle[0] += right_offset[0];
        virtual_obstacle[1] += right_offset[1];

        obs_points.push_back(virtual_obstacle);

        int first_closest_index = original_path_.getClosestIndex(obs_points.front());
        int last_closest_index = original_path_.getClosestIndex(obs_points.back());

        if (first_closest_index - first_index_offset_ - 10 < 0)
        {
            evade_points.push_back(original_path_.getRefPosArr()[0]);
            evade_points.push_back(original_path_.getRefPosArr()[1]);
        }
        else
        {
            for (size_t i = 0; i < 10; ++i)
            {
                evade_points.push_back(original_path_.getRefPosArr()[first_closest_index - first_index_offset_ - 10 + i]);
            }
        }

        for (size_t i = 0; i < obs_points.size() - 1; ++i)
        {
            const std::vector<double> &obs_pos = obs_points[i];

            double vec_size = std::hypot(obs_points[i + 1][0] - obs_points[i][0], obs_points[i + 1][1] - obs_points[i][1]);
            std::vector<double> obs_unit_direction_vec = {(obs_points[i + 1][0] - obs_points[i][0]) / vec_size, (obs_points[i + 1][1] - obs_points[i][1]) / vec_size};
            int closest_index = original_path_.getClosestIndex(obs_points[i]);
            vec_size = std::hypot(original_path_.getRefPosArr()[closest_index + 1][0] - original_path_.getRefPosArr()[closest_index][0], original_path_.getRefPosArr()[closest_index + 1][1] - original_path_.getRefPosArr()[closest_index][1]);
            std::vector<double> path_unit_direction_vec = {(original_path_.getRefPosArr()[closest_index + 1][0] - original_path_.getRefPosArr()[closest_index][0]) / vec_size, (original_path_.getRefPosArr()[closest_index + 1][1] - original_path_.getRefPosArr()[closest_index][1]) / vec_size};

            double sin_value = crossProduct(path_unit_direction_vec, obs_unit_direction_vec);

            int sign = 0;

            if (sin_value <= 0) // 오른쪽(시계방향)
                sign = 1;
            else
                sign = -1;

            double inner_cos_theta = std::cos(sign * radian);
            double inner_sin_theta = std::sin(sign * radian);

            std::vector<double> middle_evade_unit_vec = {path_unit_direction_vec[0] * inner_cos_theta - path_unit_direction_vec[1] * inner_sin_theta, path_unit_direction_vec[0] * inner_sin_theta + path_unit_direction_vec[1] * inner_cos_theta};

            std::vector<double> middle_evade_vec = {middle_evade_unit_vec[0] * inner_distance, middle_evade_unit_vec[1] * inner_distance};

            std::vector<double> middle_pos(2, 0);
            middle_pos[0] = obs_pos[0] * inner_ratio + obs_points[i + 1][0] * (1 - inner_ratio);
            middle_pos[1] = obs_pos[1] * inner_ratio + obs_points[i + 1][1] * (1 - inner_ratio);

            std::vector<double> first_middle_point = {middle_pos[0] - middle_evade_vec[0], middle_pos[1] - middle_evade_vec[1]};
            std::vector<double> seconde_middle_point = {middle_pos[0] + middle_evade_vec[0], middle_pos[1] + middle_evade_vec[1]};

            std::vector<double> evade_direction_vec = {sign * path_unit_direction_vec[1], -sign * path_unit_direction_vec[0]};

            std::vector<double> first_evade_point = {obs_pos[0] + evade_direction_vec[0] * evade_distance,
                                                     obs_pos[1] + evade_direction_vec[1] * evade_distance};

            std::vector<double> second_evade_point = {obs_points[i + 1][0] - evade_direction_vec[0] * evade_distance,
                                                      obs_points[i + 1][1] - evade_direction_vec[1] * evade_distance};

            if (i == 0)
            {
                evade_points.push_back(first_evade_point);
            }
            evade_points.push_back(first_middle_point);
            evade_points.push_back(middle_pos);
            evade_points.push_back(seconde_middle_point);
            evade_points.push_back(second_evade_point);
        }

        if (last_closest_index + last_index_offset_ + 9 >= original_path_.getRefPosArr().size())
        {
            evade_points.push_back(original_path_.getRefPosArr()[original_path_.getRefPosArr().size() - 2]);
            evade_points.push_back(original_path_.getRefPosArr()[original_path_.getRefPosArr().size() - 1]);
        }
        else
        {
            for (size_t i = 0; i < 10; ++i)
            {
                evade_points.push_back(original_path_.getRefPosArr()[last_closest_index + last_index_offset_ + i]);
            }
        }

        return evade_points;
    }
};

class EvadeCirclePathGenerator
{
private:
    // Define states for the state machine
    enum class State
    {
        INIT,
        FIRST_EVADE_PATH,
        SECOND_EVADE_PATH,
        APPROACH_EVADE,
        EVADE,
        DONE
    };

    Local &local_;
    Lidar &lidar_;
    Control &control_;
    Path &local_path_;

    // 상태변수
    State current_state_ = State::INIT;

    /* 파라미터 상수 */
    const double kStartVelocity_;
    const double kEndVelocity_;
    const double kInnerRatio_;
    const double kInnerDistance_;
    const double kEvadeRadian_;
    const double kEvadeDistance_;
    const double kLaneWidth_;

    // 차량 시작 지점 기록
    double start_car_pos_[2] = {0, 0};

public:
    EvadeCirclePathGenerator(Local &local, Lidar &lidar, Control &control, Path &local_path, nlohmann::json &param)
        : local_(local), lidar_(lidar), control_(control), local_path_(local_path),
          kStartVelocity_(param["start_velocity"]), kEndVelocity_(param["end_velocity"]),
          kInnerRatio_(param["inner_ratio"]), kEvadeRadian_(param["evade_radian"]),
          kInnerDistance_(param["inner_distance"]), kEvadeDistance_(param["evade_distance"]),
          kLaneWidth_(param["lane_width"])
    {
    }

    void getValidObstacles(const std::vector<std::vector<double>> &obstacles, const Path &original_path, const double lane_width, std::vector<std::vector<double>> &valid_obstacles)
    {
        // 유효하지 않은(lane 밖) 장애물 걸러내기
        for (const std::vector<double> &pos : obstacles)
        {
            if (original_path.getClosestDistance(pos) < lane_width)
                valid_obstacles.push_back(pos);
        }
    }

    void doSmallStaticMission()
    {
        const std::vector<std::vector<double>> &small_static_points = lidar_.getRefSmallStaticPoint();
        std::vector<std::vector<double>> obs_points;
        getValidObstacles(small_static_points, local_.getRefCurGlobalPath(), kLaneWidth_, obs_points);

        switch (current_state_)
        {
        case State::INIT:
        {
            control_.setTargetVelocity(kStartVelocity_);
            current_state_ = State::FIRST_EVADE_PATH;
            break;
        }

        case State::FIRST_EVADE_PATH: // TODO: 사용할지 결정
        {
            if (obs_points.size() >= 2)
            {
                control_.setTargetVelocity(1.5);

                // 장애물 인식 시 차량 위치 기록
                const double *cur_car_pos = local_.getAddCurCarUTMPos();
                start_car_pos_[0] = cur_car_pos[0];
                start_car_pos_[1] = cur_car_pos[1];

                current_state_ = State::SECOND_EVADE_PATH;
            }
            else if (obs_points.size() >= 1)
            {
                control_.setTargetVelocity(1.5);

                // 장애물 인식 시 차량 위치 기록
                const double *cur_car_pos = local_.getAddCurCarUTMPos();
                start_car_pos_[0] = cur_car_pos[0];
                start_car_pos_[1] = cur_car_pos[1];

                EvadeCirclePointGenerator epg(local_.getRefCurGlobalPath(), obs_points, 150, 180);
                std::vector<std::vector<double>> evade_points = epg.getVirtualEvadePoint(start_car_pos_, kInnerRatio_, kEvadeRadian_, kInnerDistance_, kEvadeDistance_);
                int start_index = local_.getRefCurGlobalPath().getClosestIndex(evade_points.front());
                int end_index = local_.getRefCurGlobalPath().getClosestIndex(evade_points.back());
                // 스플라인 경로 생성
                auto t = generateSplinePath(evade_points, 0.05);
                Path evade_path("firstEvadePath", std::get<0>(t), std::get<1>(t), std::get<2>(t));
                local_path_.updatePath(start_index, end_index, evade_path, 0, std::get<0>(t).size());

                current_state_ = State::SECOND_EVADE_PATH;
            }

            std::cout << "FIRST EVADE PATH  1️⃣1️⃣1️⃣1️⃣1️⃣1️⃣1️⃣1️⃣1️⃣1️⃣1️⃣1️⃣\n";

            break;
        }

        case State::SECOND_EVADE_PATH:
        {
            if (obs_points.size() >= 2)
            {
                EvadeCirclePointGenerator epg(local_.getRefCurGlobalPath(), obs_points, 150, 180);
                std::vector<std::vector<double>> evade_points = epg.getEvadePoint(start_car_pos_, kInnerRatio_, kEvadeRadian_, kInnerDistance_, kEvadeDistance_);
                int start_index = local_.getRefCurGlobalPath().getClosestIndex(evade_points.front());
                int end_index = local_.getRefCurGlobalPath().getClosestIndex(evade_points.back());
                // 스플라인 경로 생성
                auto t = generateSplinePath(evade_points, 0.05);
                Path evade_path("secondEvadePath", std::get<0>(t), std::get<1>(t), std::get<2>(t));
                local_path_.updatePath(0, local_path_.getRefPosArr().size(), local_.getRefCurGlobalPath(), 0, local_.getRefCurGlobalPath().getRefPosArr().size());
                local_path_.updatePath(start_index, end_index, evade_path, 0, std::get<0>(t).size());

                current_state_ = State::APPROACH_EVADE;
                std::cout << "두번째 경로 변경 완료" << std::endl;
            }

            std::cout << "FIRST EVADE PATH  2️⃣2️⃣2️⃣2️⃣2️⃣2️⃣2️⃣2️⃣2️⃣2️⃣2️⃣2️⃣\n";

            break;
        }

        case State::APPROACH_EVADE:
        {
            std::cout << "접근 중 ⏫⏫⏫⏫⏫⏫⏫⏫⏫⏫⏫⏫⏫" << std::endl;

            std::sort(obs_points.begin(), obs_points.end(), [this](const std::vector<double> &a, const std::vector<double> &b)
                      { double dist_a = std::hypot(start_car_pos_[0] - a[0], start_car_pos_[1] - a[1]);        
                        double dist_b = std::hypot(start_car_pos_[0] - b[0], start_car_pos_[1] - b[1]);        
                        return dist_a < dist_b; });

            const std::vector<double> &closest_obstacle = obs_points.front();

            const double *cur_car_pos = local_.getAddCurCarUTMPos();
            double distance_to_first_obstacle = std::hypot(cur_car_pos[0] - closest_obstacle[0],
                                                           cur_car_pos[1] - closest_obstacle[1]);

            if (distance_to_first_obstacle < 3.5)
            {
                current_state_ = State::EVADE;
                std::cout << "회피 시작" << std::endl;
            }
            break;
        }

        case State::EVADE:
        {
            std::cout << "회피 중 🔀🔀🔀🔀🔀🔀🔀🔀🔀🔀🔀🔀🔀" << std::endl;
            const double *cur_car_pos = local_.getAddCurCarUTMPos();

            bool done_flag = true;

            for (auto &obs : obs_points)
            {
                double distance_to_obstacle = std::hypot(cur_car_pos[0] - obs[0],
                                                         cur_car_pos[1] - obs[1]);

                if (distance_to_obstacle < 5.5)
                {
                    done_flag = false;
                    break;
                }
            }

            if (done_flag)
            {
                current_state_ = State::DONE;
                std::cout << "회피 종료 🔵🔵🔵🔵🔵🔵🔵🔵" << std::endl;
            }
            break;
        }

        case State::DONE:
        {
            control_.setTargetVelocity(kEndVelocity_);
            std::cout << "미션 종료 🔵🔵🔵🔵🔵🔵🔵🔵🔵🔵🔵🔵🔵🔵\n";
            break;
        }
        }
    }
};

#endif
