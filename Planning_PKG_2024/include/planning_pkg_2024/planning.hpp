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

#ifndef PLANNING_H_
#define PLANNING_H_

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
#include "evade_path_generator.hpp"
#include "cubic_spline.hpp"
#include "frenet_frame.hpp"
#include "emergency_stop.hpp"
#include "utility_function.hpp"
#include "hard_small_static.hpp"
#include "auto_parking.hpp"
#include "delivery.hpp"
#include "delivery_new.hpp"
#include "evade_circle_path_generator.hpp"
#include "evade_circle_path_generator.hpp"
#include "big_static.hpp"
#include "stop_traffic.hpp"
#include "u_turn.hpp"
#include "big_static_new.hpp"
#include "auto_parking_new.hpp"
#include "stop_traffic_new.hpp"
#include "2023_tunnel.hpp"
#include "2024_new_tunnel.hpp"
#include "2024_old_tunnel.hpp"

/* ros2 헤더파일 */
#include "std_msgs/msg/int16.hpp"

class Planning
{
private:
    /* 기본 객체 */
    Local local_;     // localization 관련 정보 객체
    Lidar lidar_;     // lidar 관련 정보 객체
    Vision vision_;   // vision 관련 정보 객체
    Control control_; // control 관련 정보 객체
    Path local_path_; // local_path 객체(최종 publish 하는 path임)

    /* 기능 모듈 객체 */
    FrenetFrame frenet_frame_;                             // frenet frame 연산 객체
    EvadeCirclePathGenerator evade_circle_path_generator_; // 소형
    DeliveryNew delivery_new_;                             // 배달
    BigStatic big_static_;                                 // 대형
    UturnClass uturn_;                                     // 유턴
    ParkingNewClass parking_new_;                          // 주차
    StopTrafficNew stop_traffic_new_;                      // 신호등, 정지선
    Tunnel2024NEW tunnel_2024_new_;                        // 터널

    int prev_mission_num_ = 0;

public:
    Planning(std::string &gp_text_adress, std::vector<int> &mission_list, std::unordered_map<int, std::string> &mission_dic, nlohmann::json &basic_data, nlohmann::json &mission_param)
        : local_(gp_text_adress, mission_list, mission_dic),
          lidar_(basic_data["lidar"]),
          control_(basic_data["control"]),
          local_path_("local_path"),
          evade_circle_path_generator_(local_, lidar_, control_, local_path_, mission_param["small_static"]),
          big_static_(local_, lidar_, control_, local_path_, mission_param["big_static"]),
          delivery_new_(local_, lidar_, control_, local_path_, mission_param["delivery_new"]),
          parking_new_(local_, lidar_, control_, local_path_, mission_param["parking"]),
          stop_traffic_new_(local_, control_, local_path_, vision_, mission_param["stop_traffic"]),
          uturn_(local_, lidar_, control_, local_path_, mission_param["u_turn"]),
          tunnel_2024_new_(local_, lidar_, control_, local_path_, mission_param["tunnel_KCITY"])
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
    void curGlobalToLocalCopy() // 현재 global path를 local path로 복사하는 함수
    {
        const Path &cur_global_path = local_.getRefCurGlobalPath();
        local_path_.setPosArr(cur_global_path.getRefPosArr());
        local_path_.setYawArr(cur_global_path.getRefYawArr());
        local_path_.setKArr(cur_global_path.getRefKArr());
        local_path_.updateKDTree();
    }

    void setDefaultStatus() // 프로그램 실행 시 기본값 세팅
    {
        local_.setCurMissionIndex();
        curGlobalToLocalCopy();
    }

    bool judgeMapChange() // 미션 바뀌는지 검사 후, 값 세팅하는 함수
    {
        /* 다음 미션으로 넘어가야 하는지 검사 */
        if (local_.checkMissionComplete() && checkLocalPathComplete()) // && !local_.getRcsMode())
        {
            if (local_.changeNextMission())
            {
                std::cout << "다음 미션!\n";
                curGlobalToLocalCopy();
                lidar_.clearAllClusters();
                return true;
            }
            else
            {
                std::cout << "종료!!";
                return false;
            }
        }
        // else if (local_.checkPathComplete()) // TODO: checkPathComplete 수정
        // {
        //     if (local_.changeNextPath())
        //     {
        //         std::cout << "다음 맵!\n";
        //         curGlobalToLocalCopy();
        //         lidar_.clearAllClusters();
        //     }
        //     return true;
        // }
        else
            return true;
    }

    bool checkLocalPathComplete()
    {
        const std::vector<std::vector<double>> &local_path_pos_arr = local_path_.getRefPosArr();
        int cur_index = local_path_.getClosestIndex(local_.getAddCurCarUTMPos());

        return ((local_path_pos_arr.size() - cur_index) < 340) ? true : false;
    }

    bool checkMissionChange()
    {
        if (prev_mission_num_ != local_.getCurMissionIndex())
        {
            vision_.setRefVLightToGreen();
            prev_mission_num_ = local_.getCurMissionIndex();
            return true;
        }
        return false;
    }

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

        std::cout << "가장 가까운 글로벌 패스 좌표: ";
        std::cout << local_.getCurClosestPos(pos)[0] << " " << local_.getCurClosestPos(pos)[1] << "\n";
        std::cout << "글로벌 패스 인덱스: ";
        std::cout << local_.getCurClosestIndex(pos) << "\n";

        std::cout << "현재 미션 번호: " << local_.getCurMissionNumber() << "\n";
        std::cout << "현재 미션: " << local_.getCurMission() << "\n";
        std::cout << "🌐🌐🌐🌐🌐🌐🌐🌐🌐🌐🌐🌐🌐🌐🌐🌐🌐🌐🌐🌐🌐🌐🌐🌐" << "\n";
    }

    void chooseFunc() // 미션에 맞는 함수를 선택하는 함수
    {
        // 여기다 조건 분기 구현(어떤 미션에 따라서 어떤 미션 실행할건지)
        int mission_number = local_.getCurMissionNumber();
        switch (mission_number)
        {
        case 1: // 속도 4
            control_.setTargetVelocity(4);
            break;
        case 2: // 속도 5
            control_.setTargetVelocity(5);
            break;
        case 3: // 속도 6
            control_.setTargetVelocity(6);
            break;
        case 4: // 최대속도
            control_.setTargetVelocity(6.9);
            break;
        case 7: // 배달 A
            delivery_new_.deliveryA();
            break;
        case 9: // 배달 B
            delivery_new_.deliveryB();
            break;
        case 11: // 정적 소형
            evade_circle_path_generator_.doSmallStaticMission();
            break;
        case 12: // 정적 대형
            big_static_.doBigStaticMission();
            break;
        case 14: // 주차
            parking_new_.new_prl_mission_func();
            break;
        case 15: // 터널
            // std::cout << " 현재 NavStatus 확인 -> " << local_.getNavStatus() << "\n";
            // tunnel_2024_old_.driveTunnel();
            tunnel_2024_new_.driveTunnel();
            break;
        case 16: // 유턴
            uturn_.uturnvoid();
            break;
        case 21: // 신호등 직진
            if (checkMissionChange())
                stop_traffic_new_.resetStatus();
            stop_traffic_new_.traffic_straight();
            break;
        case 22: // 신호등 좌회전
            if (checkMissionChange())
                stop_traffic_new_.resetStatus();
            stop_traffic_new_.traffic_left();
            break;
        case 23: // 정지선
            if (checkMissionChange())
                stop_traffic_new_.resetStatus();
            stop_traffic_new_.stopline_no_light();
            break;
        case 25: // 방지턱
            if (checkMissionChange())
                stop_traffic_new_.resetStatus();
            stop_traffic_new_.bump();
            break;
        case 2323: // 우회전 정지선 2번
            if (checkMissionChange())
                stop_traffic_new_.resetStatus();
            stop_traffic_new_.double_stopline_no_light();
            break;
        default: // default: 속도 2
            control_.setTargetVelocity(2);
            break;
        }
    }

    /********************* 테스트 *********************/
    /* 미션 함수 */
    void getOptimalTrajectoryFF() // 정적 소형 frenet_frame // 작성자: 정성윤
    {
        // 시작하면 초기값 설정
        if (!frenet_frame_.getStartFlag())
            frenet_frame_.setDefault(local_.getRefCurGlobalPath(), lidar_.getRefSmallStaticPoint());

        // 최적 경로 생성
        Path optimal_trajectory = frenet_frame_.getOptimalTrajectory(local_.getAddCurCarUTMPos(), local_.getCurCarVelocity(), local_.getAddCurCarUTMAcceleration(), local_.getCurCarYaw(), lidar_.getRefSmallStaticPoint());

        // local_path 업데이트
        // int start_point = local_path_.getClosestIndex(local_.getAddCurUTMPos());
        // local_path_.updatePath(start_point, start_point + optimal_trajectory.getRefPosArr().size(), optimal_trajectory, 0, optimal_trajectory.getRefPosArr().size());
        // if (frenet_frame_.getMakePathCount() == 0)
        local_path_.updatePath(0, local_path_.getRefPosArr().size(), optimal_trajectory, 0, optimal_trajectory.getRefPosArr().size());
    }

    /* frenet_frame_trajectory plot test */
    const std::vector<std::vector<std::vector<double>>> &getAllPathVector() const
    {
        return frenet_frame_.getAllPathVector();
    }
};

#endif
