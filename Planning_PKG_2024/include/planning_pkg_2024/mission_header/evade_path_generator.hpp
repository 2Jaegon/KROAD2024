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

#ifndef EVADE_PATH_GENERATOR_H_
#define EVADE_PATH_GENERATOR_H_

#include <iostream>
#include <vector>
#include "lidar.hpp"
#include "local.hpp"
#include "path.hpp"
#include "control.hpp"
#include "utility_function.hpp"
#include "nlohmann/json.hpp"

class EvadePointGenerator
{
private:
    const Path &original_path_;
    const std::vector<std::vector<double>> &obstacle_points_;
    int first_index_offset_ = 60;
    int last_index_offset_ = 60;

public:
    EvadePointGenerator(const Path &original_path, const std::vector<std::vector<double>> &obstacle_points, int first_index_offset, int last_index_offset)
        : original_path_(original_path), obstacle_points_(obstacle_points), first_index_offset_(first_index_offset), last_index_offset_(last_index_offset)
    {
    }

    std::vector<std::vector<double>> getEvadePoint(const double *car_pos, double evade_distance, double inner_ratio)
    {
        std::vector<std::vector<double>> evade_points;
        std::vector<std::vector<double>> obs_points = obstacle_points_;

        // 장애물 포인트를 자동차 위치와의 거리로 정렬
        std::sort(obs_points.begin(), obs_points.end(), [&car_pos](const std::vector<double> &a, const std::vector<double> &b)
                  { double dist_a = std::hypot(car_pos[0] - a[0], car_pos[1] - a[1]);
                    double dist_b = std::hypot(car_pos[0] - b[0], car_pos[1] - b[1]);
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

        for (size_t i = 0; i < obs_points.size(); ++i)
        {
            const std::vector<double> &obs_pos = obs_points[i];
            std::vector<double> closest_pos = original_path_.getClosestPos(obs_pos);
            double distance = std::hypot(closest_pos[0] - obs_pos[0], closest_pos[1] - obs_pos[1]);
            std::vector<double> evade_point(2, 0);
            evade_point[0] = obs_pos[0] + ((closest_pos[0] - obs_pos[0]) / distance) * evade_distance;
            evade_point[1] = obs_pos[1] + ((closest_pos[1] - obs_pos[1]) / distance) * evade_distance;
            evade_points.push_back(evade_point);
            if (i + 1 < obs_points.size())
            {
                std::vector<double> middle_pos(2, 0);
                middle_pos[0] = obs_pos[0] * inner_ratio + obs_points[i + 1][0] * (1 - inner_ratio);
                middle_pos[1] = obs_pos[1] * inner_ratio + obs_points[i + 1][1] * (1 - inner_ratio);
                evade_points.push_back(middle_pos);
            }
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

class EvadePathGenerator
{
private:
    // Define states for the state machine
    enum class State
    {
        INIT,
        FIRST_EVADE_PATH,
        SECOND_EVADE_PATH,
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

public:
    EvadePathGenerator(Local &local, Lidar &lidar, Control &control, Path &local_path, nlohmann::json &param)
        : local_(local), lidar_(lidar), control_(control), local_path_(local_path), kStartVelocity_(param["start_velocity"]), kEndVelocity_(param["end_velocity"])
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
        getValidObstacles(small_static_points, local_.getRefCurGlobalPath(), 2, obs_points);

        switch (current_state_)
        {
        case State::INIT:
        {
            control_.setTargetVelocity(kStartVelocity_);
            current_state_ = State::FIRST_EVADE_PATH;
            break;
        }

        case State::FIRST_EVADE_PATH:
        {
            if (obs_points.size() >= 1)
            {
                EvadePointGenerator epg(local_.getRefCurGlobalPath(), obs_points, 180, 60);
                std::vector<std::vector<double>> evade_points = epg.getEvadePoint(local_.getAddCurCarUTMPos(), 2, 0.5);
                int start_index = local_.getRefCurGlobalPath().getClosestIndex(evade_points.front());
                int end_index = local_.getRefCurGlobalPath().getClosestIndex(evade_points.back());
                // 스플라인 경로 생성
                auto t = generateSplinePath(evade_points, 0.05);
                Path evade_path("firstEvadePath", std::get<0>(t), std::get<1>(t), std::get<2>(t));
                local_path_.updatePath(start_index, end_index, evade_path, 0, std::get<0>(t).size());

                current_state_ = State::SECOND_EVADE_PATH;
                std::cout << "첫번째 경로 변경 완료" << std::endl;
            }
            break;
        }

        case State::SECOND_EVADE_PATH:
        {
            if (obs_points.size() >= 2)
            {
                EvadePointGenerator epg(local_.getRefCurGlobalPath(), obs_points, 150, 180);
                std::vector<std::vector<double>> evade_points = epg.getEvadePoint(local_.getAddCurCarUTMPos(), 2, 0.5);
                int start_index = local_.getRefCurGlobalPath().getClosestIndex(evade_points.front());
                int end_index = local_.getRefCurGlobalPath().getClosestIndex(evade_points.back());
                // 스플라인 경로 생성
                auto t = generateSplinePath(evade_points, 0.05);
                Path evade_path("secondEvadePath", std::get<0>(t), std::get<1>(t), std::get<2>(t));
                local_path_.updatePath(start_index, end_index, evade_path, 0, std::get<0>(t).size());

                current_state_ = State::DONE;
                std::cout << "두번째 경로 변경 완료" << std::endl;
            }
            break;
        }

        case State::DONE:
        {
            control_.setTargetVelocity(kEndVelocity_);
            std::cout << "미션 종료\n";
            break;
        }
        }
    }
};

#endif
