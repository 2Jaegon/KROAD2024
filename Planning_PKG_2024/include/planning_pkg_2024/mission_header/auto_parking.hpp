/***
 * 작성자: 정주훈
 * 설명: 구형 주차
 * Style
 * -클래스: PascalCase
 * -멤버변수: snake_case_   (snake_case 뒤에 밑줄)(파이썬 사용 x)
 * -함수: camelCase
 * -변수: snake_case
 * -상수: kPascalCase   (PascalCase앞에 k 접두어)
 ***/

#ifndef AUTO_PARKING_H_
#define AUTO_PARKING_H_

#include <iostream>
#include <vector>
#include <cmath>
#include <tuple>
#include "kdtree.hpp"
#include "local.hpp"
#include "lidar.hpp"
#include "control.hpp"
#include "utility_function.hpp"
#include <chrono>
using namespace std;

class PrlAutoParking
{
private:
    int num_points = 10; // 평행주차 점 몇개찍을지.
    int add_point = 300;
    double ds = 0.5;                   // 15m 추가로 주는거긴함
    double out_path_circle_radius = 2; // 최소 회전반경임
    double small_circle_radius = 2.7;  // 작은원 (주차로 가는 원)
    double big_circle_radius = 3.3;    // 큰원 (먼저 들어오는 원)
    double for_stop_d = 30;            // 실험값
    double offset_gap = 0.3;           // 실험값
    double offset_d = 1.2;             // 실험값
    double offset_short = 0.4;         // 실험값

public:
    double vector_2_radian(const vector<double> &vector)
    { // 벡터를 라디안으로 바꿔주는 함수
        return atan2(vector[1], vector[0]);
    }

    //**********************************************************수정 필요***********************************************************
    vector<vector<double>> points_on_circle_arc(double radius, const vector<double> &center, double start_angle, double end_angle, int num_points)
    { // 원방에서 일정범위만 잘라주는 함수
        vector<vector<double>> circle_points;
        double delta_theta = (end_angle - start_angle) / (num_points - 1); // 각 점 각도차이
        for (int i = 0; i < num_points; ++i)
        {
            double theta = start_angle + i * delta_theta;
            double x = center[0] + radius * cos(theta);
            double y = center[1] + radius * sin(theta);
            // circle_points.emplace_back({x, y});
            circle_points.push_back({x, y});
        }
        return circle_points;
    }

    vector<vector<double>> ready_parking_path(int ready_parking_point_k_index, int ready_parking_point_k_index_10, vector<double> &start_parking_point, vector<double> parking_point, /*vector<vector<double>> set_vec,*/ const vector<vector<double>> &local_pos_arr)
    { // 주차시작점까지 가는 path
        std::vector<std::vector<double>> spline_point;

        // vector<double> normalized_front_vec = set_vec[2];
        double dis = hypot(start_parking_point[0] - parking_point[0], start_parking_point[1] - parking_point[1]);
        vector<double> normalized_ready_vec = {(start_parking_point[0] - parking_point[0]) / dis, (start_parking_point[1] - parking_point[1]) / dis};

        // ready_parking_point_k_index_10 에서 ready_parking_point_k_index까지의 점 넣어주기
        for (int i = ready_parking_point_k_index_10; i < ready_parking_point_k_index; i++)
            spline_point.push_back(local_pos_arr[i]);
        spline_point.push_back(start_parking_point);

        // start_parking_point에서 normalized_front_vec 방향으로 ds 간격으로 add_point만큼의 새로운 점들을 넣어주어 차량의 헤딩을 원하는 방향으로 고정시키기위함
        for (int i = 1; i < add_point; i++)
        {
            spline_point.push_back({start_parking_point[0] + i * ds * normalized_ready_vec[0], start_parking_point[1] + i * ds * normalized_ready_vec[1]});
        }

        return spline_point;
    }

    // ******************************다른거는 안바꿔도 이거는 무조건 바꿔야됨**************************************
    vector<vector<double>> out_parking_path(int start_parking_point_index, vector<double> car_utm, vector<double> parking_point, vector<vector<double>> set_vec, const std::vector<std::vector<double>> &local_pos_arr)
    { // 주차 후 나가는 경로 생성
        vector<double> normalized_center_to_global_vec = set_vec[0];
        vector<double> normalized_global_to_center_vec = {-1 * set_vec[0][0], -1 * set_vec[0][1]};
        vector<double> out_path_circle_cen = {car_utm[0] + out_path_circle_radius * normalized_center_to_global_vec[0], car_utm[1] + out_path_circle_radius * normalized_center_to_global_vec[1]};

        double start_angle = vector_2_radian(normalized_global_to_center_vec);
        double end_angle = start_angle + M_PI / 2;

        std::vector<std::vector<double>> out_parking_path_point;

        out_parking_path_point = points_on_circle_arc(out_path_circle_radius, out_path_circle_cen, start_angle, end_angle, num_points);

        // parking_point와 start_parking_point_index의 점을 넣어주고 추가로 start_parking_point_index + 10까지의 점들 넣어주기
        for (int i = 0; i < 10; i++)
            out_parking_path_point.push_back(local_pos_arr[start_parking_point_index + i]);

        return out_parking_path_point;
    }

    // vector<vector<double>> parking_space(const vector<double> &parking_center_point, vector<vector<double>> set_vec)
    // {   // 주차공간 꼭짓점 계산
    //     double parking_width = 3;
    //     double parking_length = 5;
    //     vector<double> normalized_center_to_global_vec = set_vec[0];
    //     vector<double> normalized_front_vec = set_vec[2];

    //     vector<double> left_front = {parking_center_point[0] + parking_width / 2 * normalized_center_to_global_vec[0] + parking_length / 2 * normalized_front_vec[0], parking_center_point[1] + parking_width / 2 * normalized_center_to_global_vec[1] + parking_length / 2 * normalized_front_vec[1]};
    //     vector<double> left_back = {parking_center_point[0] + parking_width / 2 * normalized_center_to_global_vec[0] - parking_length / 2 * normalized_front_vec[0], parking_center_point[1] + parking_width / 2 * normalized_center_to_global_vec[1] - parking_length / 2 * normalized_front_vec[1]};
    //     vector<double> right_front = {parking_center_point[0] - parking_width / 2 * normalized_center_to_global_vec[0] + parking_length / 2 * normalized_front_vec[0], parking_center_point[1] - parking_width / 2 * normalized_center_to_global_vec[1] + parking_length / 2 * normalized_front_vec[1]};
    //     vector<double> right_back = {parking_center_point[0] - parking_width / 2 * normalized_center_to_global_vec[0] - parking_length / 2 * normalized_front_vec[0], parking_center_point[1] - parking_width / 2 * normalized_center_to_global_vec[1] - parking_length / 2 * normalized_front_vec[1]};

    //     vector<vector<double>> parking_space_point = {left_front, right_front, left_back, right_back};
    //     return parking_space_point;
    // }

    vector<vector<double>> create_vec(int parking_center_point_index, double parking_center_point_distance, const vector<double> &parking_center_point, vector<double> parking_center_point_k)
    { // 방향 단위 벡터 집합
        vector<double> center_to_global_vec = {parking_center_point_k[0] - parking_center_point[0], parking_center_point_k[1] - parking_center_point[1]};
        // ^ 주차 중앙점에서 글로벌패스 방향으로의 벡터
        vector<double> normalized_center_to_global_vec = {center_to_global_vec[0] / parking_center_point_distance, center_to_global_vec[1] / parking_center_point_distance};
        // ^ 주차 중앙점에서 글로벌패스 방향으로의 단위 벡터

        vector<vector<double>> back_matrix = {{0, -1}, {1, 0}};
        vector<double> back_vec = {back_matrix[0][0] * normalized_center_to_global_vec[0] + back_matrix[0][1] * normalized_center_to_global_vec[1], back_matrix[1][0] * normalized_center_to_global_vec[0] + back_matrix[1][1] * normalized_center_to_global_vec[1]};
        // ^ 주차 중앙점에서 뒷방향으로의 단위 벡터

        vector<vector<double>> front_matrix = {{0, 1}, {-1, 0}};
        vector<double> normalized_front_vec = {front_matrix[0][0] * normalized_center_to_global_vec[0] + front_matrix[0][1] * normalized_center_to_global_vec[1], front_matrix[1][0] * normalized_center_to_global_vec[0] + front_matrix[1][1] * normalized_center_to_global_vec[1]};
        // ^ 주차 중앙점에서 앞방향으로의 단위 벡터

        vector<vector<double>> set_vec;

        set_vec.push_back(normalized_center_to_global_vec);
        set_vec.push_back(back_vec);
        set_vec.push_back(normalized_front_vec);

        return set_vec;
    }

    vector<double> create_parking_center_point(vector<vector<double>> parking_edge_out)
    { // parking_point를 생성 하기위한 함수
        vector<double> parking_center_point = {parking_edge_out[3][0] + parking_edge_out[2][0], parking_edge_out[0][1] + parking_edge_out[3][1]};

        return parking_center_point;
    }

    vector<double> create_parking_point(const vector<double> &parking_center_point, vector<vector<double>> set_vec)
    { // parking_point를 생성 하기위한 함수
        vector<double> normalized_center_to_global_vec = set_vec[0];
        vector<double> back_vec = set_vec[1];
        vector<double> normalized_front_vec = set_vec[2];

        vector<double> parking_point = {parking_center_point[0] + offset_d * back_vec[0] - offset_short * normalized_center_to_global_vec[0], parking_center_point[1] + offset_d * back_vec[1] - offset_short * normalized_center_to_global_vec[1]};

        return parking_point;
    }

    vector<double> create_start_parking_point(const vector<double> &parking_center_point, vector<vector<double>> set_vec)
    { // start_parking_point를 생성 하기위한 함수

        vector<double> normalized_center_to_global_vec = set_vec[0];
        vector<double> back_vec = set_vec[1];
        vector<double> normalized_front_vec = set_vec[2];

        vector<double> parking_point = {parking_center_point[0] + offset_d * back_vec[0] + offset_short * normalized_center_to_global_vec[0], parking_center_point[1] + offset_d * back_vec[1] + offset_short * normalized_center_to_global_vec[1]};

        vector<double> small_circle_center_offset = {parking_point[0] + ((small_circle_radius + big_circle_radius) * cos(M_PI / 3)) * normalized_center_to_global_vec[0], parking_point[1] + ((small_circle_radius + big_circle_radius) * cos(M_PI / 3)) * normalized_center_to_global_vec[1]};

        vector<double> theory_start_parking_point = {small_circle_center_offset[0] + (small_circle_radius + big_circle_radius) * sin(M_PI / 3) * normalized_front_vec[0], small_circle_center_offset[1] + (small_circle_radius + big_circle_radius) * sin(M_PI / 3) * normalized_front_vec[1]};

        vector<double> start_parking_point = {theory_start_parking_point[0] + offset_gap * normalized_center_to_global_vec[0], theory_start_parking_point[1] + offset_gap * normalized_center_to_global_vec[1]};

        return start_parking_point;
    }

    vector<vector<double>> parking_path(vector<double> parking_point, const vector<double> &parking_center_point, vector<double> parking_center_point_k, vector<vector<double>> set_vec)
    { // 용승path 생성 함수

        vector<double> normalized_center_to_global_vec = set_vec[0];
        vector<double> back_vec = set_vec[1];
        vector<double> normalized_front_vec = set_vec[2];

        vector<double> small_circle_center = {parking_point[0] + small_circle_radius * normalized_center_to_global_vec[0], parking_point[1] + small_circle_radius * normalized_center_to_global_vec[1]};
        vector<double> small_circle_center_offset = {parking_point[0] + ((small_circle_radius + big_circle_radius) * cos(M_PI / 3)) * normalized_center_to_global_vec[0], parking_point[1] + ((small_circle_radius + big_circle_radius) * cos(M_PI / 3)) * normalized_center_to_global_vec[1]};

        vector<double> theory_start_parking_point = {small_circle_center_offset[0] + (small_circle_radius + big_circle_radius) * sin(M_PI / 3) * normalized_front_vec[0], small_circle_center_offset[1] + (small_circle_radius + big_circle_radius) * sin(M_PI / 3) * normalized_front_vec[1]};
        vector<double> start_parking_point = {theory_start_parking_point[0] + offset_gap * normalized_center_to_global_vec[0], theory_start_parking_point[1] + offset_gap * normalized_center_to_global_vec[1]};

        vector<double> big_circle_center = {theory_start_parking_point[0] - big_circle_radius * normalized_center_to_global_vec[0], theory_start_parking_point[1] - big_circle_radius * normalized_center_to_global_vec[1]};
        vector<double> process_big_circle_center = {start_parking_point[0] - big_circle_radius * normalized_center_to_global_vec[0], start_parking_point[1] - big_circle_radius * normalized_center_to_global_vec[1]};

        vector<double> vec_circle_12 = {big_circle_center[0] - small_circle_center[0], big_circle_center[1] - small_circle_center[1]};
        vector<double> vec_circle_21 = {small_circle_center[0] - big_circle_center[0], small_circle_center[1] - big_circle_center[1]};

        double rad_circle_12 = vector_2_radian(vec_circle_12);
        double rad_circle_21 = vector_2_radian(vec_circle_21);

        // 원의 방정식(일정범위만 잘라주는 함수) 구하기 전에 각도 범위 구하기
        double small_st_angle = rad_circle_12;
        double small_fin_angle = rad_circle_12 - M_PI / 3;

        double big_st_angle = rad_circle_21 - M_PI / 3;
        double big_fin_angle = rad_circle_21;

        // 원방 만들기
        vector<vector<double>> prcar_utm_1 = points_on_circle_arc(small_circle_radius, small_circle_center_offset, small_st_angle, small_fin_angle, num_points);
        vector<vector<double>> prcar_utm_2 = points_on_circle_arc(big_circle_radius, process_big_circle_center, big_st_angle, big_fin_angle, num_points);

        // 정지 이후 예비 경로 만들기
        vector<double> future_point = {parking_point[0] + for_stop_d * normalized_center_to_global_vec[0], parking_point[1] + for_stop_d * normalized_center_to_global_vec[1]};
        // vector<double> out_future_point = {start_parking_point[0] + 17 * normalized_front_vec[0], start_parking_point[1] + 17 * normalized_front_vec[1]};

        // 최종 경로 생성
        vector<vector<double>> parking_path_point = prcar_utm_2;

        for (size_t i = 0; i < prcar_utm_1.size(); i++)
        {
            parking_path_point.push_back(prcar_utm_1[i]);
        }
        // parking_point[0] * 0.5 * i * back_vec[0] , parking_point[1] * 0.5 * i * back_vec[1]
        // parking_path_point.push_back({parking_point[0] + 15 * back_vec[0], parking_point[1] + 15 * back_vec[1]});

        parking_path_point.push_back(future_point);

        return parking_path_point;
    }
};

// 주차 미션수행클래스
class ParkingClass
{

private:
    enum class State
    {
        INIT,
        MAKE_READY_PATH,
        GO_STARTING_PARKING_POINT,
        MAKE_PARKING_PATH,
        GO_PARKING_POINT,
        MAKE_OUT_PATH,
        DONE
    };

    int ready_parking_point_k_index; // 주차공간을 확인받은 차량의 kd_tree 한 점에 index

    int ready_parking_point_k_index_10; // 주차공간을 확인받은 차량의 kd_tree 한 점 10 index 전의 점

    double parking_center_point_distance; // parking_center_point와 글로벌 패스의 거리

    vector<double> parking_center_point_k; // 인지한테 받은 주차 공간의 center점을 kd_tree 한 점

    int parking_center_point_index; // parking_center_point_k의 index

    vector<vector<double>> set_vec; // 백터의 집합

    vector<double> start_parking_point; // 주차를 위한 최적의 장소

    vector<double> start_parking_point_k; // start_parking_point를 kd_tree 한 점

    int start_parking_point_index; // start_parking_point_k의 index

    vector<double> parking_point; // 주차완료할 점

    int parking_point_index; // parking_point을 kd_tree 한 점의 인덱스

    vector<double> parking_center_point;

    //*****************************************/
    bool prl_stop = false;                // 평행 주차 중지 상태
    int key = 0;                          // if의 flag
    bool parking_situation_check = false; // 시간 체크 플래그

    //**************제어 조절***************/
    int SERIAL_E_STOP = 0;   // E-STOP
    int SERIAL_NOMAL = 2;    // 일반주행
    int SERIAL_SLOW = 1;     // 천천히 가기
    int SERIAL_REVERSE = -1; // 후진
    int s_m = SERIAL_NOMAL;
    // int SERIAL_DILEMMA = 3;
    // int SERIAL_SLOW_2 = 4;
    // int SERIAL_AACCELERATION = 5;

    //**************타임 조절*****************/
    double STOP_TIME = 2;
    double STOP_PARKING_TIME = 10.0;
    std::chrono::system_clock::time_point prl_time;

    State current_state_ = State::INIT;

    Local &local_;
    Lidar &lidar_;
    Control &control_;
    Path &local_path_;

    PrlAutoParking pap;

    // temp

public:
    ParkingClass(Local &local, Lidar &lidar, Control &control, Path &local_path) : local_(local), lidar_(lidar), control_(control), local_path_(local_path)
    {
    }

    void prl_mission_func()
    {
        const double *car_utm = local_.getAddCurCarUTMPos();
        // cout << current_state_<< endl;

        const std::vector<std::vector<double>> &local_pos_arr = local_path_.getRefPosArr();
        switch (current_state_)
        {
        case State::INIT:
        {
            control_.setTargetVelocity(2);
            if (lidar_.getIsParkingStop())
            {
                cout << "주차공간 확인" << endl;
                control_.setTargetVelocity(SERIAL_E_STOP);
                current_state_ = State::MAKE_READY_PATH;
            }
            cout << "주차공간 확인" << endl;
            break;
        }
        case State::MAKE_READY_PATH:
        {
            cout << "주차공간 확인" << endl;
            control_.setTargetVelocity(SERIAL_E_STOP); // 0 설정*******************************************************************************
            ready_parking_point_k_index = local_.getCurClosestIndex(car_utm);
            if (!parking_situation_check)
            {
                prl_time = std::chrono::system_clock::now();
                parking_situation_check = true;
            }
            if (chrono::duration_cast<chrono::seconds>(chrono::system_clock::now() - prl_time).count() > STOP_TIME)
            {
                cout << "정지중" << endl;
                cout << "***************************" << s_m << "***************************" << endl;

                if (lidar_.getRefParkingPoint().size() > 0)
                {
                    ready_parking_point_k_index_10 = max(0, ready_parking_point_k_index - 10);

                    parking_center_point = lidar_.getRefParkingPoint();
                    // = pap.create_parking_center_point(lidar_.getRefParkingEdgeOut());

                    parking_center_point_index = local_.getCurClosestIndex(parking_center_point);

                    parking_center_point_distance = local_.getCurClosestDistance(parking_center_point);

                    parking_center_point_k = local_.getCurClosestPos(parking_center_point);

                    set_vec = pap.create_vec(parking_center_point_index, parking_center_point_distance, parking_center_point, parking_center_point_k);

                    start_parking_point = pap.create_start_parking_point(parking_center_point, set_vec);

                    start_parking_point_k = local_.getCurClosestPos(start_parking_point);

                    start_parking_point_index = local_.getCurClosestIndex(start_parking_point);

                    parking_point = pap.create_parking_point(parking_center_point, set_vec);

                    parking_point_index = local_.getCurClosestIndex(parking_point);

                    cout << "ready parking path 생성 중" << endl;

                    // vector<vector<double>> ready_parking_path_point = pap.ready_parking_path(ready_parking_point_k_index, ready_parking_point_k_index_10, start_parking_point, set_vec, local_pos_arr);

                    vector<vector<double>> ready_parking_path_point = pap.ready_parking_path(ready_parking_point_k_index, ready_parking_point_k_index_10, start_parking_point, parking_point, local_pos_arr);
                    // generateTargetCourseFlat 호출
                    auto t = generateSplinePath(ready_parking_path_point);

                    // tuple의 각 요소에 접근
                    std::vector<std::vector<double>> path = std::get<0>(t);
                    std::vector<double> ryaw = std::get<1>(t);
                    std::vector<double> rk = std::get<2>(t);

                    // path 객체 생성
                    Path ready_parking_path("ready_parking_path", path, ryaw, rk);

                    // path update
                    local_path_.updatePath(ready_parking_point_k_index_10, local_pos_arr.size(), ready_parking_path, 0, std::get<0>(t).size());

                    parking_situation_check = false;
                    current_state_ = State::GO_STARTING_PARKING_POINT;
                }
            }
            else
            {
                cout << chrono::duration_cast<chrono::seconds>(chrono::system_clock::now() - prl_time).count() << "초 정지하는중" << endl;
            }
            break;
        }
        case State::GO_STARTING_PARKING_POINT:
        {
            cout << "2초 정지 완료, 주차 시작점으로 " << s_m << " m/s로 " << "이동 중" << endl;
            control_.setTargetVelocity(SERIAL_SLOW);

            double dis = sqrt(pow(car_utm[0] - start_parking_point[0], 2) + pow(car_utm[1] - start_parking_point[1], 2));

            cout << "start_parking_point 까지 남은 거리" << dis << endl;

            if (dis <= 0.3)
            {
                cout << "주차시작점 도착." << endl;
                s_m = SERIAL_E_STOP;
                control_.setTargetVelocity(SERIAL_E_STOP);
                current_state_ = State::MAKE_PARKING_PATH;
            }
            break;
        }
        case State::MAKE_PARKING_PATH:
        {
            if (!parking_situation_check)
            {
                prl_time = std::chrono::system_clock::now();
                parking_situation_check = true;
            }
            else if (parking_situation_check && chrono::duration_cast<chrono::seconds>(chrono::system_clock::now() - prl_time).count() > 5)
            {
                std::vector<std::vector<double>> parking_path_point = pap.parking_path(parking_point, parking_center_point, parking_center_point_k, set_vec);
                // 의심1 parking_point,
                // 의심2 parking_center_point,
                // 의심1 parking_center_point_k,

                // generateTargetCourseFlat 호출
                auto t = generateSplinePath(parking_path_point);

                // tuple의 각 요소에 접근
                std::vector<std::vector<double>> path_2 = std::get<0>(t);
                std::vector<double> ryaw_2 = std::get<1>(t);
                std::vector<double> rk_2 = std::get<2>(t);

                // path 객체 생성
                Path parking_path("parking_path_point", path_2, ryaw_2, rk_2);
                local_path_.updatePath(0, local_pos_arr.size(), parking_path, 0, std::get<0>(t).size());
                parking_situation_check = false;
                current_state_ = State::GO_PARKING_POINT;
            }
            else
            {
                cout << chrono::duration_cast<chrono::seconds>(chrono::system_clock::now() - prl_time).count() << "정지중, parking path 생성중 " << endl;
            }
            break;
        }
        case State::GO_PARKING_POINT:
        {
            cout << "주차 후진 중" << endl;
            control_.setTargetVelocity(SERIAL_REVERSE);
            cout << "***************************" << SERIAL_REVERSE << "***************************" << endl;

            auto dis = sqrt(pow(car_utm[0] - parking_point[0], 2) + pow(car_utm[1] - parking_point[1], 2));
            if (dis <= 1.4)
            {
                cout << "주차 끝" << endl;
                s_m = SERIAL_E_STOP;
                control_.setTargetVelocity(SERIAL_E_STOP);
                current_state_ = State::MAKE_OUT_PATH;
            }
            break;
        }
        case State::MAKE_OUT_PATH:
        {
            if (!parking_situation_check)
            {
                prl_time = std::chrono::system_clock::now();
                parking_situation_check = true;
            }
            else if (parking_situation_check && chrono::duration_cast<chrono::seconds>(chrono::system_clock::now() - prl_time).count() > STOP_PARKING_TIME)
            {
                vector<vector<double>> out_parking_path_point = pap.out_parking_path(start_parking_point_index, {car_utm[0], car_utm[1]}, parking_point, set_vec, local_.getRefCurGlobalPath().getRefPosArr());

                // generateTargetCourseFlat 호출
                auto t = generateSplinePath(out_parking_path_point);

                // tuple의 각 요소에 접근
                std::vector<std::vector<double>> path = std::get<0>(t);
                std::vector<double> ryaw = std::get<1>(t);
                std::vector<double> rk = std::get<2>(t);

                // path 객체 생성
                Path out_parking_path("asd", path, ryaw, rk);

                // Path update
                local_path_.updatePath(0, local_path_.getRefPosArr().size(), out_parking_path, 0, std::get<0>(t).size());

                // Path update2
                local_path_.updatePath(local_path_.getRefPosArr().size() - 1, local_path_.getRefPosArr().size(), local_.getRefCurGlobalPath(), start_parking_point_index - 1, local_.getRefCurGlobalPath().getRefPosArr().size());

                cout << "10초 정지 완료" << endl;
                parking_situation_check = false;
                current_state_ = State::DONE;
            }
            else
            {
                cout << "정지중, 주차 완료, out_parking_path 생성 중" << endl;
                cout << "10초 정지 하는 중" << endl;
                cout << chrono::duration_cast<chrono::seconds>(chrono::system_clock::now() - prl_time).count() << endl;
            }
            break;
        }
        case State::DONE:
        {
            s_m = SERIAL_NOMAL;
            control_.setTargetVelocity(SERIAL_NOMAL);
            cout << "탈출중" << endl;

            auto dis = sqrt(pow(car_utm[0] - start_parking_point[0], 2) + pow(car_utm[1] - start_parking_point[1], 2));
            if (dis <= 0.1)
            {
                cout << "탈출 완료 " << endl;
                current_state_ = State::DONE;
            }
            break;
        }
        }
    }
};

#endif
