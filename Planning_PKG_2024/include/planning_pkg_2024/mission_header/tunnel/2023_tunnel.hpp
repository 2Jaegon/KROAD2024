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

/*
=====================================================================

본 코드는 2023년도(4기) 판단 - 터널 코드를 python에서 c++로 직역해둔 코드입니다.

=====================================================================
*/

#ifndef TUNNEL_2023_HPP_
#define TUNNEL_2023_HPP_

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
#include "emergency_stop.hpp"
#include "frenet_frame.hpp"

#include "evade_path_generator.hpp"

class Tunnel
{
private:
    // 함수 외부에서 가져오는 애들
    Lidar &lidar_;
    Control &control_;
    Local &local_;
    Path &local_path_;
    FrenetFrame frenet_frame_; // frenet frame 연산 객체
    // HardSmallStatic hard_small_static_; // 나중에 이식하는 걸로...
    EmergencyStop e_stop_;

    // 저장되는 path들
    std::vector<std::vector<double>> wall_path_;
    std::vector<std::vector<double>> cen_path_;

    // 함수 내부에서 사용되는 애들
    bool cen_mode_ = false;
    bool wall_mode_ = false;
    bool gps_mode_ = false;

    // 모드
    // 1순위 : GPS, 2순위 : 벽모드, 3순위 : 차선중앙주행
    // int kDriveMode = 1;

    // 저장이 필요한 애들...
    std::vector<double> L_wall_;
    std::vector<double> R_wall_;
    std::vector<double> lidar_center_;

    int flag_num = 0;

    std::chrono::system_clock::time_point last_time_for_small;

    // 터널 체크 부분
    // bool is_tunnel_ = false;
    bool check_tunnel_ = false;
    std::chrono::seconds elapsed_t_; // 벽 감지 상태(sec)
    std::chrono::system_clock::time_point start_t_;

    double tunnel_ceiling_;
    bool in_tunnel_ = false; // 얘 변수명 좀 바꿔야할 것 같기도...?
    std::chrono::system_clock::time_point gps_t_;
    std::chrono::system_clock::time_point ceil_t_;

    bool change_gps_ = false;
    bool small_ing_ = true;

    std::chrono::system_clock::time_point tunnel_small_check_;
    bool change_flag_ = false;

    std::vector<std::vector<double>> tunnel_lane_2D_;
    /* 풀 리퀘스트 안나와서 추가하는 주석*/

    // 상수들
    // double safety_distance_R = 0.3;                                            // 목표로 하는 안전 거리 설정
    // double safety_distance_L = 1.2;                                            // 최소 거리 (주행하는 차량의 크기에 맞게 조정 필요)
    std::vector<std::vector<double>> kCarTail = {{-2, 0}, {-1, 0}, {-0.5, 0}}; // 차량 후방 꼬리좌표
    std::vector<std::vector<double>> kTunnelMark = {{-82.82, -82.82}};         // 터널임을 알려주는 좌표
    const double curCarPos[2] = {0.0, 0.0};                                    // 터널임을 알려주는 좌표

    // Parmeter 상수들
    const int kFindNum;        // "head"                head 경로 몇 개 찍을 지
    const double kRatio;       // "ratio"               n : 1 내분점 생성
    const int kMinDist;        // "obstacle_min_dist"   장애물과 최소 거리 원래는 8. 4.0 기준으로 15
    const double kSafetyDist;  // "between_obs_n_wall"  장애물의 왼쪽 오른쪽 판별 기준. 모라이는 4.0이 안전, KCITY : 5.0
    const double kWallOffset1; // "obs_offset01"        터널 장애물 오프셋 할 거리01 / KCITY :1.5, morai 1.1
    const double kWallOffset2; // "obs_offset02"        터널 장애물 오프셋 할 거리02 / KCITY :1.9, morai 1.5

    const double kEps = 3;            // 군집 포인트 간 최대 거리
    const unsigned int kMSamples = 2; // 군집 내 최소 포인트 개수

    std::vector<double> divPoint(const std::vector<double> R, const std::vector<double> L, double ratio)
    {
        double x_r = R[0];
        double y_r = R[1];
        double x_l = L[0];
        double y_l = L[1];

        double x_internal = (x_l + ratio * x_r) / (1 + ratio);
        double y_internal = (y_l + ratio * y_r) / (1 + ratio);

        std::vector<double> sub = {x_internal, y_internal};
        return sub;
    }

    void pathFilter(std::vector<std::vector<double>> &result, const std::vector<std::vector<double>> origin)
    {
        /* x좌표가 0이상인 애들만 남겨주는 필터용 함수입니다. */
        /* 터널은 상대 좌표계를 쓰므로 무조건 좌표가 0 이상이어야 함!!! */

        result.clear(); // garbage 값 청소...
        for (const auto &point : origin)
        {
            if (point[0] >= 0)
            {
                result.emplace_back(point);
            }
        }
    }

public:
    Tunnel(Local &local, Lidar &lidar, Control &control, Path &local_path, nlohmann::json &param)
        : lidar_(lidar), local_(local), local_path_(local_path), control_(control), e_stop_(control),
          kFindNum(param["head"]), kRatio(param["ratio"]), kMinDist(param["obstacle_min_dist"]),
          kSafetyDist(param["between_obs_n_wall"]), kWallOffset1(param["obs_offset01"]), kWallOffset2(param["obs_offset02"])
    {
        control_.setTargetVelocity(2.0); // 초기 속도 설정
    };
    ~Tunnel() {}; // 기본 소멸자 정의

    void setWallMode(bool check)
    {
        wall_mode_ = check;
    }

    /* Other Functions */
    void tunnelDrive() // 터널인지 감지하는 코드
    {
        // create_mission_path_ = true; //원 코드는 planning node쪽 -> 이거는 진짜 패스 생성하고나서 체크하는걸로...

        /* 차선 센터 체크 */
        if ((lidar_.getRefTunnelCenter()).size() > 1)
        {
            lidar_center_ = lidar_.getRefTunnelCenter();
            cen_mode_ = true;
        } // 센터 모드 true
        else
        {
            cen_mode_ = false;
        } // 센터 모드 false

        tunnelDetection(); // 터널 탐지
        driveMode();       // 각 모드 체크 현황에 따라 주행모드 설정

        return;
    }

    void tunnelDetection()
    {
        /* 벽이 제대로 감지되는 상황인가? */
        double L_len = lidar_.getTunnelLLen();
        double R_len = lidar_.getTunnelRLen();
        if (L_len != -1 && R_len != -1)
        {
            // 디버깅용
            std::cout << "LLLLLLLLLLL : " << L_len << "\n";
            std::cout << "RRRRRRRRRRR : " << R_len << "\n";

            if (R_len >= 8 && L_len >= 12)
            { // 터널이 확실히 감지되는 지점 (R:8, L:12)
                std::cout << "벽 보는 중 : " << elapsed_t_.count() << "\n";
                if (!check_tunnel_)
                { // 벽 처음 보기 시작할 때
                    start_t_ = std::chrono::system_clock::now();
                    check_tunnel_ = true;
                }

                if (control_.checkWaitTime(start_t_, 2))
                {
                    check_tunnel_ = false;
                    modeCheck(L_len, R_len);
                }
            }
            else
            {
                if (check_tunnel_)
                {
                    std::cout << "-----초기화-----" << "\n";
                    check_tunnel_ = false;                              // 터널 아님!
                    start_t_ = std::chrono::system_clock::time_point(); // epoch(1970.01.01)으로 초기화
                } // 바뀐 게 있을 때만(터널 안->터널 밖 탈출) 초기화 진행.
            }
        }
    }

    void modeCheck(double L_len, double R_len)
    {
        tunnel_ceiling_ = lidar_.getTunnelCeiling();

        if (tunnel_ceiling_ >= 0)
        {
            std::cout << "천장 길이 : " << tunnel_ceiling_ << "\n";
            if (!in_tunnel_ && tunnel_ceiling_ > 30 && lidar_.getRefTunnelLWall().size() >= kFindNum && lidar_.getRefTunnelRWall().size() >= kFindNum) // 같다 포함되는지 아닌지가 헷갈리는데 일단 함. 참고.
            {                                                                                                                                          // 터널 첫진입
                L_wall_ = lidar_.getRefTunnelLWall();
                R_wall_ = lidar_.getRefTunnelRWall();
                wall_mode_ = true;
                in_tunnel_ = true;
            }

            else
            {
                if (tunnel_ceiling_ == 0)
                {
                    // 8초 이상 감지 안되면 gps 모드로 전환
                    if (gps_t_ == std::chrono::system_clock::time_point())
                    {
                        gps_t_ = std::chrono::system_clock::now();
                    }
                    else
                    {
                        if (control_.checkWaitTime(gps_t_, 4))
                        {
                            gps_mode_ = true;
                        }
                    }
                }

                else if (tunnel_ceiling_ < 30 || R_len < 8 || L_len < 12 || lidar_.getRefTunnelLWall().size() < kFindNum || lidar_.getRefTunnelRWall().size() < kFindNum)
                {
                    // 벽모드가 실행 불가한 상황.
                    gps_t_ = std::chrono::system_clock::time_point();
                    if (ceil_t_ == gps_t_)
                    {
                        ceil_t_ = std::chrono::system_clock::now();
                    } // gps 초기화 한 김에 또 에폭 불러내는 것보단 초기화한 gps_t와 비교하는 게 낫다고 생각하여 gps_t와 비교하니 참고바람.

                    if (control_.checkWaitTime(ceil_t_, 8))
                        wall_mode_ = false; // 8초이상 안잡히면 월모드해제~
                }

                else
                {
                    gps_t_ = std::chrono::system_clock::time_point();
                    ceil_t_ = gps_t_; // 이미 gps_t가 초기화된 상태이므로...
                    // wall_mode_ = false;
                    // wall_mode_ = true;
                }
            }
        }
        return;
    }

    void driveMode()
    {
        if (gps_mode_)
        {

            if (change_gps_) // 터널 밖으로 나올 때
            {
                std::cout << " ========= GPS 주행중 ========= " << "\n";
                // change_gps_ = false;

                // const double *utm_point = local_.getAddCurCarUTMPos();
                // int start_idx = local_path_.getClosestIndex(utm_point);

                const Path &g_path = local_.getRefCurGlobalPath();
                // int replace_start_idx = g_path.getClosestIndex(local_.getAddCurCarUTMPos());

                local_path_.updatePath(0, local_path_.getRefPosArr().size(), g_path, 0, g_path.getRefPosArr().size());
            }
        }

        else if (wall_mode_)
        {
            std::cout << " ========= 벽 보고 주행중 ========= " << "\n";
            wallDrive();
        }
        else if (cen_mode_)
        {
            std::cout << " ========= 차선 중앙 주행중 ========= " << "\n";
            // laneDrive(); // 기존 코드는 이게 아니다...
            cenDrive(); // <--- 얘임.
        } // 차선 주행!!!

        else
        {
            std::cout << " XXXXXXXX !!! 인식 되는 값 없음 !!! XXXXXXXX " << "\n";
            // laneDrive();
        }

        // 동적 장애물 코드.
        if (last_time_for_small != std::chrono::system_clock::time_point())
        {
            if (small_ing_ && control_.checkWaitTime(last_time_for_small, 4)) // 처음 동적으로 전환될 때.
            {
                small_ing_ = false;
                lidar_.setDynamicMode(true);
            }

            if (lidar_.getDynamicMode())
            { // 다이나믹 모드라면 시작
                dynamicObjFunc();
                if (e_stop_.getStopMoreThanOnce()) // 한 번 멈췄으면 동적 실행 중지
                {
                    lidar_.setDynamicMode(false);
                    gps_mode_ = true;
                    change_gps_ = true;
                }
            }
        }

        return;
    } // 여기까지

    void dynamicObjFunc() // 동적 장애물 코드
    {
        // 밖에서 라이다 트루면 함수 실행
        if (lidar_.getIsDynamicStop() && !e_stop_.getStopTrigger())
        {
            e_stop_.setStopTrigger(true);
        }
        e_stop_.stopMomentFunc();
    }

    /* 아래는 각 주행 별 상세 코드 */
    void wallDrive() // 벽 포인트 kFindNum 이상만큼은 있어야 오류없이 디버깅된다...!!!!
    {
        L_wall_ = lidar_.getRefTunnelLWall(); // 업데이트... 해줘야 됨!!!
        R_wall_ = lidar_.getRefTunnelRWall();
        // 기울기와 절편 구하기
        double slope_l;
        double slope_r;
        try
        {
            // 예외 자체가 발생하면 안되는 부분이긴하지만... 혹시 몰라 처리해둡니다
            if (L_wall_[2] == L_wall_[0])
            {
                throw L_wall_[2];
            }
            if (R_wall_[2] == R_wall_[0])
            {
                throw R_wall_[2];
            }

            slope_l = (L_wall_[3] - L_wall_[1]) / (L_wall_[2] - L_wall_[0]);
            slope_r = (R_wall_[3] - R_wall_[1]) / (R_wall_[2] - R_wall_[0]);
        }
        catch (double e)
        { // division 0 오류 시 바로 cenDrive!!!
            std::cout << "에러 발생!!! 모드 긴급 전환 : " << e << "\n";
            std::cout << "에러 발생!!! 모드 긴급 전환 : " << e << "\n";
            std::cout << "에러 발생!!! 모드 긴급 전환 : " << e << "\n";
            cenDrive();
            return;
        }

        double y_intercept_l = L_wall_[1] - (slope_l * L_wall_[0]); // b = y - ax
        double y_intercept_r = R_wall_[1] - (slope_r * R_wall_[0]);

        std::vector<std::vector<double>> l_points_ = {};
        std::vector<std::vector<double>> r_points_ = {};

        // 벽에 대한 포인트 구하기(세밀하게)
        for (double i = 0; i < 13; i += 0.5)
        { // 13 * 2
            // 각 좌표를 0부터 12.5까지 증가시킴 (터널은 상대좌표!!)
            // index 1의 값은 y = ax+b임.
            // x좌표는 동일한게 맞아요~
            std::vector<double> l_tmp = {i, slope_l * i + y_intercept_l};
            std::vector<double> r_tmp = {i, slope_r * i + y_intercept_r};

            l_points_.emplace_back(l_tmp);
            r_points_.emplace_back(r_tmp);
        }

        // KDTree로 가장 가까운 점들과 거리 찾기
        // l_points는 절대로 비어있지 않습니다. 디버깅시 참고.
        KDTree<std::vector<double>> l_tree(l_points_);
        KDTree<std::vector<double>> r_tree(r_points_); // 고로 KDTREE도 안 비어있어용~

        std::vector<std::vector<double>> wall_cen_point = {};

        for (int i = kFindNum; i > 0; i--)
        // 왜 뒤
        {
            wall_cen_point.emplace_back(divPoint(r_points_[r_points_.size() - i], l_points_[l_points_.size() - i], kRatio));
        }

        std::vector<double> wall_road_vec = {wall_cen_point[1][0] - wall_cen_point[0][0], wall_cen_point[1][1] - wall_cen_point[0][1]};

        // 단위 벡터 계산
        double wall_road_vec_size = std::hypot(wall_road_vec[0], wall_road_vec[1]); // 벡터의 크기
        std::vector<double> wall_road_norm_vec = {wall_road_vec[0] / wall_road_vec_size, wall_road_vec[1] / wall_road_vec_size};

        std::vector<double> wall_car_vec = {1, 0}; // 차량이 현재 주행 중인 방향
        double wall_yawvec = std::inner_product(wall_road_vec.begin(), wall_road_vec.end(), wall_car_vec.begin(), 0);
        double wall_cos_theta = wall_yawvec / wall_road_vec_size;
        double wall_car_yaw = acos(wall_cos_theta);

        double cross_product_wall = wall_road_vec[0] * wall_car_vec[1] - wall_road_vec[1] * wall_car_vec[0];
        if (cross_product_wall < 0)
        {
            local_.setCurYaw(-wall_car_yaw);
        }
        else
        {
            local_.setCurYaw(wall_car_yaw);
        }

        // // 아래에서 정적 소형 진행
        std::vector<std::vector<double>> rotation_matrix = {{0, -1},
                                                            {1, 0}};
        // 수작업으로 행렬 곱셈 수행
        std::vector<double> wall_small_offset_vec(2);
        wall_small_offset_vec[0] = rotation_matrix[0][0] * wall_road_norm_vec[0] + rotation_matrix[0][1] * wall_road_norm_vec[1];
        wall_small_offset_vec[1] = rotation_matrix[1][0] * wall_road_norm_vec[0] + rotation_matrix[1][1] * wall_road_norm_vec[1];

        // // 아래는 옛날 코드
        std::vector<double> mv_point01 = {};
        std::vector<double> mv_point02 = {};
        std::vector<std::vector<double>> go;

        // std::vector<double> &small_obj = lidar_.getRefTunnelSmallStaticPoint();
        if ((!(lidar_.getRefTunnelSmallStaticPoint().empty())) && (small_ing_))
        {
            // control_.setTargetVelocity(2.5);
            std::cout << "장애물 발견" << "\n";
            if (lidar_.getRefTunnelSmallStaticPoint().size() < 2)
            {
                vstackCPP(go, kCarTail, wall_cen_point);
            }

            else if (2 <= lidar_.getRefTunnelSmallStaticPoint().size() && lidar_.getRefTunnelSmallStaticPoint().size() < 4)
            {
                double car_dist = std::hypot(lidar_.getRefTunnelSmallStaticPoint()[0], lidar_.getRefTunnelSmallStaticPoint()[1]);
                std::cout << car_dist << "\n";
                std::cout << "장애물 한 개 발견" << "\n";
                std::cout << "그냥 장애물 거리 : " << car_dist << "\n";

                // 여기 밑에 evadePath 쓸 수 있으면 좋을듯? 지금은 적용 X
                std::vector<double> obj1 = {lidar_.getRefTunnelSmallStaticPoint()[0], lidar_.getRefTunnelSmallStaticPoint()[1]};

                if (car_dist < kMinDist)
                {
                    std::vector<double> obj1_closet_wall = r_tree.getClosestPos(obj1);
                    double dist_obj1 = std::hypot(obj1_closet_wall[0] - obj1[0], obj1_closet_wall[1] - obj1[1]);

                    std::cout << "장애물 1번 벽이랑 거리 : " << dist_obj1 << "\n";

                    std::vector<std::vector<double>> for_mv = {};
                    std::vector<std::vector<double>> tmp = {};
                    if ((2 <= dist_obj1) && (dist_obj1 < kSafetyDist))
                    {
                        mv_point01.emplace_back(obj1[0] + kWallOffset2 * wall_small_offset_vec[0]);
                        mv_point01.emplace_back(obj1[1] + kWallOffset2 * wall_small_offset_vec[1]);
                        for_mv.emplace_back(mv_point01);

                        if (!mv_point01.empty())
                        {
                            vstackCPP(tmp, kCarTail, for_mv);
                            vstackCPP(go, tmp, wall_cen_point);
                        }
                    }
                    else if ((kSafetyDist <= dist_obj1) && (dist_obj1 < 8))
                    {
                        mv_point01.emplace_back(obj1[0] - kWallOffset2 * wall_small_offset_vec[0]);
                        mv_point01.emplace_back(obj1[1] - kWallOffset2 * wall_small_offset_vec[1]);
                        for_mv.emplace_back(mv_point01);
                        if (!mv_point01.empty())
                        {
                            vstackCPP(tmp, kCarTail, for_mv);
                            vstackCPP(go, tmp, wall_cen_point);
                        }
                    }
                    else
                    {
                        vstackCPP(go, kCarTail, wall_cen_point);
                    }
                }

                else
                {
                    vstackCPP(go, kCarTail, wall_cen_point);
                }
            }

            else if (4 <= lidar_.getRefTunnelSmallStaticPoint().size())
            {
                std::cout << "장애물 2개 발견" << "\n";
                std::vector<double> obj1 = {lidar_.getRefTunnelSmallStaticPoint()[0], lidar_.getRefTunnelSmallStaticPoint()[1]};
                std::vector<double> obj2 = {lidar_.getRefTunnelSmallStaticPoint()[2], lidar_.getRefTunnelSmallStaticPoint()[3]};

                std::vector<double> *fst_obj;
                std::vector<double> *scd_obj;

                if (std::hypot(obj1[0], obj1[1]) < std::hypot(obj2[0], obj2[1]))
                {
                    fst_obj = &obj1;
                    scd_obj = &obj2;
                }
                else
                {
                    fst_obj = &obj2;
                    scd_obj = &obj1;
                }

                double car_dist_fst = std::hypot((*fst_obj)[0], (*fst_obj)[1]);
                double car_dist_scd = std::hypot((*scd_obj)[0], (*scd_obj)[1]);

                // std::cout << "첫번째 장애물 거리 : " << car_dist_fst << "\n";
                // std::cout << "두번째 장애물 거리 : " << car_dist_scd << "\n";

                if (car_dist_fst < kMinDist)
                {
                    std::vector<double> obj1_closet_wall = r_tree.getClosestPos(*fst_obj);
                    double dist_fst = std::hypot(obj1_closet_wall[0] - (*fst_obj)[0], obj1_closet_wall[1] - (*fst_obj)[1]);

                    std::cout << "첫 번째 장애물 벽까지와 거리 : " << dist_fst << "\n";

                    if ((2 < dist_fst) && (dist_fst < kSafetyDist))
                    {
                        mv_point01.emplace_back((*fst_obj)[0] + kWallOffset2 * wall_small_offset_vec[0]);
                        mv_point01.emplace_back((*fst_obj)[1] + kWallOffset2 * wall_small_offset_vec[1]);
                    }
                    else if ((kSafetyDist < dist_fst) && (dist_fst < 8))
                    {
                        mv_point01.emplace_back((*fst_obj)[0] - kWallOffset2 * wall_small_offset_vec[0]);
                        mv_point01.emplace_back((*fst_obj)[1] - kWallOffset2 * wall_small_offset_vec[1]);
                    }
                }

                if (car_dist_scd < kMinDist)
                {
                    std::vector<double> obj2_closet_wall = r_tree.getClosestPos(*scd_obj);
                    double dist_scd = std::hypot(obj2_closet_wall[0] - (*scd_obj)[0], obj2_closet_wall[1] - (*scd_obj)[1]);

                    std::cout << "두 번째 장애물 벽까지와 거리 : " << dist_scd << "\n";

                    if ((2 < dist_scd) && (dist_scd < kSafetyDist))
                    {
                        mv_point02.emplace_back((*scd_obj)[0] + kWallOffset1 * wall_small_offset_vec[0]);
                        mv_point02.emplace_back((*scd_obj)[1] + kWallOffset1 * wall_small_offset_vec[1]);
                    }
                    else if ((kSafetyDist < dist_scd) && (dist_scd < 8))
                    {
                        mv_point02.emplace_back((*scd_obj)[0] - kWallOffset1 * wall_small_offset_vec[0]);
                        mv_point02.emplace_back((*scd_obj)[1] - kWallOffset1 * wall_small_offset_vec[1]);
                    }
                }

                // -----
                std::vector<std::vector<double>> for_mv01 = {};
                for_mv01.emplace_back(mv_point01);
                std::vector<std::vector<double>> for_mv02 = {};
                for_mv02.emplace_back(mv_point02);
                std::vector<std::vector<double>> tmp01;
                std::vector<std::vector<double>> tmp02;

                if ((mv_point01.size() > 0) && (mv_point02.size() > 0))
                {
                    vstackCPP(tmp01, kCarTail, for_mv01);
                    vstackCPP(tmp02, for_mv02, wall_cen_point);
                    vstackCPP(go, tmp01, tmp02);
                }
                else if ((mv_point01.size() > 0) && (mv_point02.size() == 0))
                {
                    vstackCPP(tmp01, kCarTail, for_mv01);
                    vstackCPP(go, tmp01, wall_cen_point);
                }
                else if ((mv_point01.size() == 0) && (mv_point02.size() > 0))
                {
                    vstackCPP(tmp02, kCarTail, for_mv02);
                    vstackCPP(go, tmp02, wall_cen_point);
                }
                else
                {
                    vstackCPP(go, kCarTail, wall_cen_point);
                }

                last_time_for_small = std::chrono::system_clock::now();
            }
        }

        else
        {
            if (local_.getCurMissionNumber() != 17)
            {
                std::cout << "장애물 미발견" << "\n";
            }
            vstackCPP(go, kCarTail, wall_cen_point);
        }

        if (go.size() >= 1)
        {
            // 플롯을 위한 publish
            // 보간
            auto csp_result = generateSplinePath(go, 0.05);

            std::vector<std::vector<double>> path01 = std::get<0>(csp_result);
            std::vector<std::vector<double>> path02;
            pathFilter(path02, path01);
            vstackCPP(wall_path_, kTunnelMark, path02);

            std::vector<double> wall_yaw = std::get<1>(csp_result);
            std::vector<double> wall_k = std::get<2>(csp_result);

            // Path update
            Path tunnel_wall_("wallDrive_tunnel", wall_path_, wall_yaw, wall_k);

            // const double *utm_point = local_.getAddCurCarUTMPos();
            // int start_idx = local_path_.getClosestIndex(utm_point);
            // end index : 교체할 부분의 범위가 삭제할 부분의 최대 범위를 넘어가면 삭제
            // int end_idx = local_path_.getRefPosArr().size() - start_idx < (wall_path_.size() - 1) ? local_path_.getRefPosArr().size() : start_idx + (wall_path_.size() - 1); // 오류고 자시고 로직을 좀 바꾸는 게 좋을 것 같습니다.

            local_path_.updatePath(0, local_path_.getRefPosArr().size(), tunnel_wall_, 0, tunnel_wall_.getRefPosArr().size());
        }
        return;
    }

    /* ----------------------------------------------------------- */

    void cenDrive()
    {
        std::vector<std::vector<double>> center_p; // 라이다의 1차원 배열 2차원으로 저장
        reshape1Dto2D(center_p, lidar_center_);    // 1차원 > 2차원 벡터
        // print2Dvec(center_p, "터널 센터포인트");   // 디버깅용 출력

        int cen_p_final_i = center_p.size() - 1;
        std::vector<double> further_vec = {center_p[cen_p_final_i][0] - center_p[cen_p_final_i - 1][0],
                                           center_p[cen_p_final_i][1] - center_p[cen_p_final_i - 1][1]};
        double further_vec_size = std::hypot(further_vec[0], further_vec[1]);
        std::vector<double> further_norm_vec = {further_vec[0] / further_vec_size, further_vec[1] / further_vec_size};
        std::vector<std::vector<double>> further_p = {{center_p[cen_p_final_i][0] + further_norm_vec[0] * 10,
                                                       center_p[cen_p_final_i][1] + further_norm_vec[1] * 10}};

        std::vector<std::vector<double>> path_tmp;
        std::vector<std::vector<double>> tmp_tmp;
        vstackCPP(tmp_tmp, kCarTail, center_p);
        vstackCPP(path_tmp, tmp_tmp, further_p);

        // 큐빅 스플라인 보간
        auto csp_result = generateSplinePath(path_tmp);

        std::vector<std::vector<double>> path01 = std::get<0>(csp_result);
        std::vector<std::vector<double>> path02;
        pathFilter(path02, path01);
        vstackCPP(cen_path_, kTunnelMark, path02);

        std::vector<double> cen_yaw = std::get<1>(csp_result);
        std::vector<double> cen_k = std::get<2>(csp_result);

        // Path update
        Path tunnel_center("centerDrive_tunnel", cen_path_, cen_yaw, cen_k);

        // const double *utm_point = local_.getAddCurCarUTMPos();
        // int start_idx = local_path_.getClosestIndex(utm_point);
        // end index : 교체할 부분의 범위가 삭제할 부분의 최대 범위를 넘어가면 삭제...
        // int end_idx = (local_path_.getRefPosArr().size() - start_idx < (cen_path_.size() - 1))? local_path_.getRefPosArr().size() : start_idx + (cen_path_.size() - 1);

        local_path_.updatePath(0, local_path_.getRefPosArr().size(), tunnel_center, 0, tunnel_center.getRefPosArr().size());
    }

    /* ----------------------------------------------------------- */

    std::vector<int> get_sorted_indices(const std::vector<double> &v)
    {
        // 인덱스를 초기화합니다.
        std::vector<int> sorted_indices(v.size());
        iota(sorted_indices.begin(), sorted_indices.end(), 0);

        // lst의 값을 기준으로 인덱스를 정렬합니다.
        sort(sorted_indices.begin(), sorted_indices.end(), [&v](int a, int b)
             { return v[a] > v[b]; });

        return sorted_indices;
    }

    void sort_coor_by_dist(std::vector<std::vector<double>> &ans, std::vector<std::vector<double>> &coor)
    {
        sort(coor.begin(), coor.end(), [](const std::vector<double> &a, const std::vector<double> &b)
             { return std::hypot(a[0], a[1]) < std::hypot(b[0], b[1]); });
    }

    void laneDrive()
    { // 나아아아중에 --- 이제 가자!!
      // ---여기서부터할거임 아마두???
      // lidar.tunnel_lane을 받아오는데

        // 아래는 일단 무시하세염~

        // std::vector<double> lane = lidar_.getRefTunnelLane();
        // if (!(lane.empty())){
        //     tunnel_lane_2D_.clear();
        //     for(int i=0; i < (lane.size() / 2); i++){
        //         std::vector<double> lane_pair = {lane[2*i], lane[2*i+1]};
        //         tunnel_lane_2D_.emplace_back(lane_pair);
        //     }

        //         // DBscan을 위한 파이썬 모듈 불러내기 --- c++ 끌어와서 하기

        //         //DBSCAN 시작~ ... 그냥 pcl 깡으로 받아서 하면 좋을 것 같은데 지금 pcl이 되나?
        //         Point *p = new Point;
        //         std::vector<Point> p_forDS;

        //         for (size_t i = 0; i < tunnel_lane_2D_.size(); i++) {
        //             p[i].x = tunnel_lane_2D_[i][0];
        //             p[i].y = tunnel_lane_2D_[i][1];
        //             p[i].clusterID = UNCLASSIFIED;

        //             p_forDS.emplace_back(p[i]);
        //         }

        //         DBSCAN ds(m_samples, eps, p_forDS);
        //         ds.run();

        //         std::vector<std::vector<double>> fst;
        //         std::vector<std::vector<double>> scd;
        //         std::vector<std::vector<double>> thd;

        //         for(int i=0;i < ds.m_points.size(); i++){
        //             if (ds.m_points[i].clusterID == 0){fst.emplace_back(tunnel_lane_2D[i]);}
        //             else if (ds.m_points[i].clusterID == 1){scd.emplace_back(tunnel_lane_2D[i]);}
        //             else if (ds.m_points[i].clusterID == 2){thd.emplace_back(tunnel_lane_2D[i]);}
        //         }

        //             std::vector<int> indices = get_sorted_indices({fst[0][1], scd[0][1], thd[0][1]});
        //             std::tuple<int, int, int> result = make_tuple(indices[0], indices[1], indices[2]);

        //             std::map<std::tuple<int, int, int>, std::vector<std::vector<std::vector<double>>>> order_mapping;
        //             order_mapping[make_tuple(0, 1, 2)] = {fst, scd, thd};
        //             order_mapping[make_tuple(0, 2, 1)] = {fst, thd, scd};
        //             order_mapping[make_tuple(1, 0, 2)] = {scd, fst, thd};
        //             order_mapping[make_tuple(1, 2, 0)] = {scd, thd, fst};
        //             order_mapping[make_tuple(2, 0, 1)] = {thd, fst, scd};
        //             order_mapping[make_tuple(2, 1, 0)] = {thd, scd, fst};

        //             std::vector<std::vector<std::vector<double>>> order = order_mapping[result];
        //             std::vector<std::vector<double>> fst_order; std::vector<double> fst_order_flat;
        //             sort_coor_by_dist(fst_order, order[0]);
        //             flatten(fst_order_flat, fst_order);

        //             std::vector<std::vector<double>> scd_order; std::vector<double> scd_order_flat;
        //             sort_coor_by_dist(scd_order, order[1]);
        //             flatten(scd_order_flat, scd_order);

        //             std::vector<std::vector<double>> thd_order; std::vector<double> thd_order_flat;
        //             sort_coor_by_dist(thd_order, order[2]);
        //             flatten(thd_order_flat, thd_order);

        //             // 선형회귀는 python binding하여야함...

        //             // 아직이용...
        //             int ind;
        //             double dist = 0.0; int near_index = -1; //코드에서 사용하진 않지만 디버깅용으로 남김
        //             for(int i = 0; i < kFindNum; i++){
        //                 ind = kFindNum - i;
        //             }
        // }
        // else {
        //     std::cout << "지금 tunnel lane도 없어용 ..." << "\n";
        // }
    }
};

#endif
