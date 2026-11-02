/***
 * 작성자: 정성윤
 * 설명: 판단 작업을 모두 하는 planning 클래스 구현
 * Style
 * -클래스: PascalCase
 * -멤버변수: snake_case_   (snake_case 뒤에 밑줄)(파이썬 사용 x)
 * -함수: camelCase
 * -변수: snake_case
 * -상수: kPascalCase   (PascalCase앞에 k 접두어)
 ***/

#ifndef AMZ_PLANNING_H_
#define AMZ_PLANNING_H_

/* c++ 헤더파일 */
#include <iostream>
#include <vector>
#include <unordered_map>
#include <memory>
#include "nlohmann/json.hpp"

/* 사용자 정의 헤더파일 */
#include "local.hpp"
#include "vision.hpp"
#include "lidar.hpp"
#include "control.hpp"
#include "cubic_spline.hpp"
#include "utility_function.hpp"

#include "local_driving.hpp"
#include "local_dwa_driving.hpp"
#include "global_path_generator.hpp"

/* ros2 헤더파일 */
#include "std_msgs/msg/int16.hpp"

class AmzPlanning
{
private:
    // Define states for the state machine
    enum class State
    {
        INIT,
        WAIT_FOR_DISTANCE,
        LOCAL_PATH_DRIVING,
        GENERATE_GLOBAL_PATH,
        GLOBAL_PATH_DRIVING,
        DONE
    };

    /* 기본 객체 */
    Local local_;     // localization 관련 정보 객체
    Lidar lidar_;     // lidar 관련 정보 객체
    Vision vision_;   // vision 관련 정보 객체
    Control control_; // control 관련 정보 객체
    Path local_path_; // local_path 객체(최종 publish 하는 path임)

    /* 기능 모듈 객체 */
    LocalDriving local_driving_;
    LocalDwaDriving local_dwa_driving_;
    GlobalPathGenerator global_path_generator_;

    /* 상태변수 */
    State current_state_ = State::INIT;

    /* 기록변수 */
    std::vector<double> start_pos_ = {0.0, 0.0};
    int lap_count_ = 0;

    /* 파라미터 상수 */
    const double kFirstLabVelocity_;
    const double kClosingLabVelocity_;
    const int kTotalLaps_ = 2; // TODO: 필요한지 고민

    /* 글로벌 패스 객체 */
    Path global_path_; // global_path 객체(한 바퀴 이후 생성)

public:
    AmzPlanning(std::string &gp_text_adress, std::vector<int> &mission_list, std::unordered_map<int, std::string> &mission_dic, nlohmann::json &basic_data, nlohmann::json &mission_param)
        : local_(gp_text_adress, mission_list, mission_dic),
          local_path_("local_path"), global_path_("global_path"),
          lidar_(basic_data["lidar"]), control_(basic_data["control"]),
          local_driving_(local_, lidar_, control_, local_path_, mission_param["amz_rrt"]),
          local_dwa_driving_(local_, lidar_, control_, local_path_, mission_param["amz_dwa"]),
          global_path_generator_(local_, lidar_, control_, local_path_, mission_param["global_path_generator"]),
          kFirstLabVelocity_(mission_param["first_lab_velocity"]), kClosingLabVelocity_(mission_param["closing_lab_velocity"])
    {
    }

    /* subscriber를 위한 객체 참조 반환 함수 */
    Local &getLocal() { return local_; }
    Lidar &getLidar() { return lidar_; }
    Vision &getVision() { return vision_; }
    Control &getControl() { return control_; }

    /* getter 함수 */
    const Path &getRefLocalPath() const
    {
        return local_path_;
    }

    /* 기능 관련 함수 */
    void printStatus() // 현재 상태 표시 함수
    {
        std::cout << fixed;
        std::cout.precision(10);
        std::cout << "##################################################################\n";
        std::cout << "현재 utm 좌표: ";
        const double *pos = local_.getAddCurCarUTMPos();
        std::cout << pos[0] << " " << pos[1] << "\n";

        std::cout << "yaw 값: ";
        std::cout << local_.getCurCarYaw() << "\n";

        std::cout << "현재 속도: ";
        std::cout << local_.getCurCarVelocity() << "\n";

        const double *accel = local_.getAddCurCarUTMAcceleration();
        std::cout << "X축 가속도: ";
        std::cout << accel[0] << "\n"
                  << "Y축 가속도: " << accel[1] << "\n";
    }

    void doAmzMission() // 협로 미션 함수
    {
        switch (current_state_)
        {
        case State::INIT:
        {
            control_.setTargetVelocity(2);

            // 초기 local_path 생성
            std::vector<std::vector<double>> spline_points;
            const double *car_pos = local_.getAddCurCarUTMPos();
            double car_yaw = local_.getCurCarYaw();
            spline_points.push_back({car_pos[0], car_pos[1]});
            double step_size = 0.2;                               // 0.2 간격
            double new_x = car_pos[0] + step_size * cos(car_yaw); // x 좌표 계산
            double new_y = car_pos[1] + step_size * sin(car_yaw); // y 좌표 계산
            spline_points.push_back({new_x, new_y});
            auto t = generateSplinePath(spline_points, 0.05);
            Path front_local_path("localPath", std::get<0>(t), std::get<1>(t), std::get<2>(t));
            local_path_.updatePath(0, local_path_.getRefPosArr().size(), front_local_path, 0, std::get<0>(t).size());

            const double *cur_car_pos = local_.getAddCurCarUTMPos();
            start_pos_[0] = cur_car_pos[0];
            start_pos_[1] = cur_car_pos[1];

            current_state_ = State::WAIT_FOR_DISTANCE;
            break;
        }

        case State::WAIT_FOR_DISTANCE:
        {
            local_driving_.doLocalDriving();
            // local_dwa_driving_.doLocalDriving();

            control_.setTargetVelocity(kFirstLabVelocity_);

            // setTargetVelocityByPath();

            const double *car_pos = local_.getAddCurCarUTMPos();
            // global_path_generator_.recordPoint(local_.getCurClosestPos(car_pos));

            double distance_to_start = std::hypot(car_pos[0] - start_pos_[0], car_pos[1] - start_pos_[1]);
            if (distance_to_start > 2.5)
            {
                current_state_ = State::LOCAL_PATH_DRIVING;
            }
            break;
        }

        case State::LOCAL_PATH_DRIVING:
        {
            local_driving_.doLocalDriving();
            // local_dwa_driving_.doLocalDriving();

            control_.setTargetVelocity(kFirstLabVelocity_);

            // setTargetVelocityByPath();

            const double *car_pos = local_.getAddCurCarUTMPos();
            // global_path_generator_.recordPoint({car_pos[0], car_pos[1]});
            global_path_generator_.recordPoint(local_path_.getClosestPos(car_pos));

            if (checkLapComplete())
            {
                ++lap_count_;
                current_state_ = State::GENERATE_GLOBAL_PATH;
            }
            break;
        }

        case State::GENERATE_GLOBAL_PATH:
        {
            local_.changeNextMission();
            global_path_generator_.pathClose();
            global_path_.updatePath(0, global_path_.getRefPosArr().size(), global_path_generator_.getGlobalPath(), 0, global_path_generator_.getGlobalPath().getRefPosArr().size());
            local_path_.updatePath(0, local_path_.getRefPosArr().size(), global_path_, 0, global_path_.getRefPosArr().size());

            current_state_ = State::GLOBAL_PATH_DRIVING;
            break;
        }

        case State::GLOBAL_PATH_DRIVING:
        {
            control_.setTargetVelocity(kClosingLabVelocity_);

            if (checkLocalPathComplete())
            {
                local_path_.updatePath(0, local_path_.getRefPosArr().size(), global_path_, 0, global_path_.getRefPosArr().size());
                ++lap_count_;
            }
            break;
        }

        case State::DONE:
        {
            std::cout << "종료!!!!\n";
            break;
        }
        }
    }

    bool checkLapComplete() // 차가 lap을 돌아 다시 시작지점으로 왔는지 검사하는 함수
    {
        const double *car_pos = local_.getAddCurCarUTMPos();
        double distance_to_start = std::hypot(car_pos[0] - start_pos_[0], car_pos[1] - start_pos_[1]);
        if (distance_to_start < 1.5)
        {
            return true;
        }
        return false;
    }

    bool checkLocalPathComplete()
    {
        const std::vector<std::vector<double>> &local_path_pos_arr = local_path_.getRefPosArr();
        int cur_index = local_path_.getClosestIndex(local_.getAddCurCarUTMPos());

        return ((local_path_pos_arr.size() - cur_index) < 340) ? true : false;
    }

    void setTargetVelocityByPath()
    {
        const std::vector<std::vector<double>> &local_path_pos_arr = local_path_.getRefPosArr();
        int cur_index = local_path_.getClosestIndex(local_.getAddCurCarUTMPos());
        int local_length = local_path_pos_arr.size() - cur_index;

        double distance = std::hypot(local_path_pos_arr.back()[0] - local_path_pos_arr[cur_index][0], local_path_pos_arr.back()[1] - local_path_pos_arr[cur_index][1]);
        control_.decelByDistance(distance, 0.5);
        std::cout << "dist: " << distance << "\n";
        std::cout << "b_dist: " << control_.getBrakingDistance() << "\n";
    }

    const std::vector<double> getDwaRefPoint() const
    {
        return local_dwa_driving_.getRefPoint();
    }
};

#endif
