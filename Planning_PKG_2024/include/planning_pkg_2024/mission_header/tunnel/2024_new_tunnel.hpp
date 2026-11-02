/***
 * 작성자: 조윤서
 * 설명: 터널 내부 주행
 * Style
 * -클래스: PascalCase
 * -멤버변수: snake_case_   (snake_case 뒤에 밑줄)(파이썬 사용 x)
 * -함수: camelCase
 * -변수: snake_case
 * -상수: kPascalCase   (PascalCase앞에 k 접두어)
 ***/

#ifndef TUNNEL_2024_NEW_HPP_
#define TUNNEL_2024_NEW_HPP_

#include <iostream>
#include <numeric>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <vector>
#include <map>
#include <tuple>

#include "local.hpp"
#include "utility_function.hpp"
#include "lidar.hpp" //라이다에 벡터 포함
#include "kdtree.hpp"
// #include "dbscan.hpp"
#include "path.hpp"
#include "control.hpp"
// #include "hard_small_static.hpp"
#include "frenet_frame.hpp"
#include "emergency_stop.hpp"
#include "evade_path_generator.hpp"
#include "virtual_tunnel.hpp"

class Tunnel2024NEW
{
private:
    // 함수 외부에서 가져오는 애들
    Lidar &lidar_;
    Control &control_;
    Local &local_;
    Path &local_path_;

    // 저장되는 path들
    std::vector<std::vector<double>> wall_path_;
    std::vector<std::vector<double>> cen_path_;

    // 모드
    // 1순위 : GPS, 2순위 : 벽모드, 3순위 : 차선중앙주행
    // int kDriveMode = 1;

    // 저장이 필요한 애들...
    std::vector<double> L_wall_;
    std::vector<double> R_wall_;
    std::vector<double> lidar_center_ = {};

    bool dynamic_check_init = false;

    bool stop_trigger_ = false; // 동적 장애물

    std::chrono::system_clock::time_point tunnel_entry_t_;
    std::chrono::system_clock::time_point last_time_for_small_;

    // 터널 체크 부분
    std::chrono::system_clock::time_point tunnel_start_t_;

    double tunnel_ceiling_ = 0.0;
    bool in_tunnel_ = false; // 얘 변수명 좀 바꿔야할 것 같기도...?
    std::chrono::system_clock::time_point no_ceil_t_;
    std::chrono::system_clock::time_point no_tunnel_t_;

    bool change_gps_ = false;
    bool small_ing_ = true;

    // GPS 나가는데 쓰는 변수들
    bool gps_ok_ = false;
    std::chrono::system_clock::time_point gps_ok_time_;

    // std::vector<std::vector<double>> tunnel_lane_2D_;

    // 상수들
    // double safety_distance_R = 0.3;                                            // 목표로 하는 안전 거리 설정
    // double safety_distance_L = 1.2;                                            // 최소 거리 (주행하는 차량의 크기에 맞게 조정 필요)
    std::vector<std::vector<double>> kCarTail = {{-2, 0}, {-1, 0}, {-0.5, 0}}; // 차량 후방 꼬리좌표
    std::vector<std::vector<double>> kTunnelMark = {{-82.82, -82.82}};         // 터널임을 알려주는 좌표
    const double curCarPos[2] = {0.0, 0.0};                                    // 터널임을 알려주는 좌표

    // Parmeter 상수들
    const int kFindNum;                // "head"                head 경로 몇 개 찍을 지
    const double kRatio;               // "ratio"               n : 1 내분점 생성
    const int kMinDist;                // "obstacle_min_dist"   장애물과 최소 거리 원래는 8. 4.0 기준으로 15
    const double kSafetyDist;          // "between_obs_n_wall"  장애물의 왼쪽 오른쪽 판별 기준. 모라이는 4.0이 안전, KCITY : 5.0
    const double kWallOffset1;         // "obs_offset01"        터널 장애물 오프셋 할 거리01 / KCITY :1.5, morai 1.1
    const double kWallOffset2;         // "obs_offset02"        터널 장애물 오프셋 할 거리02 / KCITY :1.9, morai 1.5
    const double kVelocityInit;        // "vel_init"            터널 미션 시작할 때 처음 속도.
    const double kVelocitySmallStatic; // "vel_smallstatic"     정적 장애물 발견 ~ 수행까지 속도
    const double kVelocityDynamic;     // "vel_dynamic"         정적 장애물 끝 ~ 진출까지 속도

    // const double kEps = 3;            // 군집 포인트 간 최대 거리
    // const unsigned int kMSamples = 2; // 군집 내 최소 포인트 개수

    std::vector<double> divPoint(const std::vector<double> R, const std::vector<double> L, double ratio);
    void pathFilter(std::vector<std::vector<double>> &result, const std::vector<std::vector<double>> origin);

    // ---------------------------------- 추가 코드~ ----------------------------------
    enum class DriveStatus
    {
        NOMODE, // 아무 모드 없음
        LANE,   // 차선 CENTER 주행. 저번 코드의 LANE과 같습니다.
        WALL,   // 벽 모드 주행
        GPS,    // GPS 주행
        INIT    // 초기 세팅

    };

    DriveStatus now_status_ = DriveStatus::INIT;

    /*
    small_static_status_는 바로 직전 장애물 감지 개수와 지금 장애물 감지 개수를 저장하는 코드입니다.
    small_static_status_는 다음과 같은 순서로 변화해야 정적 소형이 끝났다고 인식합니다.
    X0 -> 01 -> 12 -> 21 -> 10 (X는 무관항)
    small_static_flag_는 상기의 스테이터스가 얼마나 변화했는지 알려주는 것으로 최대 4의 값을 가집니다. 4이후 변동은 없습니다.
    올바른 순서로 이동한다면 플래그가 1씩 증가하고, 순서가 어긋나게 되면 플래그는 0으로 초기화 됩니다.
    자기 자신으로의 loop나 12~21사이의 22는 플래그 변화에 영향을 끼치지 않습니다.
    */
    int small_static_flag_ = 0; // 초기값 : 0 , 최대값 : 5
    int StoD_ = 0;              // default : 0, 시간 기반 : 1, status 기반 : 2

    // -------------------------------------------------------------------------------

public:
    Tunnel2024NEW(Local &local, Lidar &lidar, Control &control, Path &local_path, nlohmann::json &param)
        : lidar_(lidar), local_(local), local_path_(local_path), control_(control),
          kFindNum(param["head"]), kRatio(param["ratio"]), kMinDist(param["obstacle_min_dist"]),
          kSafetyDist(param["between_obs_n_wall"]), kWallOffset1(param["obs_offset01"]), kWallOffset2(param["obs_offset02"]),
          kVelocityInit(param["vel_init"]), kVelocitySmallStatic(param["vel_smallstatic"]), kVelocityDynamic(param["vel_dynamic"]) {};

    // tunnel_driving.cpp
    void driveTunnel();
    void tunnelJamming(bool start_sign = false);
    void driveGPS();
    void driveWall();
    void driveLaneCenter();
    void laneDrive();

    // tunnel_mode_check.cpp
    void modeCheck();
    bool checkGPS();
    bool checkWall();
    bool checkCenterLane();

    // tunnel_obstacle.cpp
    void checkStaticToDynamic();
    void dynamicObjFunc();
    void checkSSStatus(int now);

    // tunnel_utils.cpp
    std::vector<int> get_sorted_indices(const std::vector<double> &v);
    void sort_coor_by_dist(std::vector<std::vector<double>> &ans, std::vector<std::vector<double>> &coor);
    std::vector<std::vector<double>> curvatureOptimizedInterpolation(const std::vector<std::vector<double>> &points);
};

#endif