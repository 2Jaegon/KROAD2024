/***
 * 작성자:  김호진
 * 설명: 정지선 & 신호등 미션 & 방지턱
 * Style
 * -클래스: PascalCase
 * -멤버변수: snake_case_   (snake_case 뒤에 밑줄)(파이썬 사용 x)
 * -함수: camelCase
 * -변수: snake_case
 * -상수: kPascalCase   (PascalCase앞에 k 접두어)
 ***/

#ifndef STOP_TRAFFIC_H_
#define STOP_TRAFFIC_H_

#include <vector>
#include <iostream>
#include <chrono>
#include <thread>

#include "local.hpp"
#include "vision.hpp"
#include "control.hpp"
#include "path.hpp"
#include "control.hpp"

class StopTfraffic
{
private:
    std::vector<std::vector<double>> STOP_LINE_LIST_ = {
        {346897.62102963787, 4069946.1614505025},
        {346897.62102963787, 4069946.1614505025},
        {346897.62102963787, 4069946.1614505025},
        {346897.62102963787, 4069946.1614505025},
        {346897.62102963787, 4069946.1614505025},
        {346897.62102963787, 4069946.1614505025},
        {346897.62102963787, 4069946.1614505025},
        {346897.62102963787, 4069946.1614505025},
        {346897.62102963787, 4069946.1614505025},
        {346897.62102963787, 4069946.1614505025},
        {346897.62102963787, 4069946.1614505025},
        {346897.62102963787, 4069946.1614505025},
        {346897.62102963787, 4069946.1614505025},
        {346897.62102963787, 4069946.1614505025},
        {346897.62102963787, 4069946.1614505025},
        {346897.62102963787, 4069946.1614505025},
        {346897.62102963787, 4069946.1614505025},
        {346897.62102963787, 4069946.1614505025},
        {346897.62102963787, 4069946.1614505025},
        {346897.62102963787, 4069946.1614505025},
        {346897.62102963787, 4069946.1614505025},
        {346897.62102963787, 4069946.1614505025},
        {346897.62102963787, 4069946.1614505025},
        {0, 0}};
    // 신호등 정지선 임시 변수
    bool time_check_ = false;
    bool stop_ing_ = false;
    bool late_2_stop_ = false;
    double STOP_DISTANCE_ = 4.5;
    double STOP_DISTANCE_PRE_ = 7;
    double LATE_DIS_ = 4.5;
    double WAIT_TIME_ = 3.2;
    double STOP_LINE_TIME_ = 3.3;
    std::chrono::time_point<std::chrono::high_resolution_clock> tf_time_;

    // 방지턱 임시 변수
    bool bump_flag_ = false;
    bool bump_in_flag_ = false;

    std::vector<double> bump_point_ = {346907.6816799486, 4069898.7344150976};

    Local &local_;
    Vision &vision_;
    Control &control_;
    Path &local_path_;

public:
    StopTfraffic(Local &local, Control &control, Path &local_path, Vision &vision)
        : local_(local), control_(control), local_path_(local_path), vision_(vision)
    {
    }

    void stopline_left()
    {
        const double *car_pos = local_.getAddCurCarUTMPos();
        std::vector<int> tf_light = vision_.getRefVLight();

        int mission_idx = local_.getCurMissionIndex();

        std::vector<double> stop_line_point = {STOP_LINE_LIST_[mission_idx][0], STOP_LINE_LIST_[mission_idx][1]};
        std::vector<double> stop_line_global_point = local_path_.getClosestPos(stop_line_point);
        std::vector<double> car_pos_global_point = local_path_.getClosestPos(car_pos);

        double line_dis = std::hypot(car_pos_global_point[0] - stop_line_global_point[0], car_pos_global_point[1] - stop_line_global_point[1]);

        std::cout << "----- 정지선까지 거리  :  " << line_dis << std::endl;

        std::cout << "----- 현재 신호  :  ";
        check_traffic_sign(tf_light);

        if (vision_.getRefVLight().empty())
        {
            std::cout << "인지가 신호 안줌" << std::endl;
        }

        if (!(late_2_stop_))
        {
            control_.setTargetVelocity(2);
            if (line_dis < STOP_DISTANCE_)
            {
                if (tf_light[2] == 1)
                {
                    control_.setTargetVelocity(2);
                    stop_ing_ = false;
                }
                else
                {
                    control_.setTargetVelocity(0);
                    stop_ing_ = true;
                }
            }
            else if (line_dis >= STOP_DISTANCE_ && line_dis < STOP_DISTANCE_PRE_)
            {
                if (tf_light[2] == 1)
                {
                    control_.setTargetVelocity(2);
                }
                else
                {
                    control_.setTargetVelocity(1);
                }
            }
        }
        else
        {
            control_.setTargetVelocity(2); // serial->serial_dilema ????
        }
        if (!(stop_ing_))
        {
            if (line_dis < LATE_DIS_)
            {
                late_2_stop_ = true;
            }
        }
    }

    void stopline_straight()
    {
        const double *car_pos = local_.getAddCurCarUTMPos();
        std::vector<int> tf_light = vision_.getRefVLight();
        int mission_idx = local_.getCurMissionIndex();

        std::vector<double> stop_line_point = {STOP_LINE_LIST_[mission_idx][0], STOP_LINE_LIST_[mission_idx][1]};
        std::vector<double> stop_line_global_point = local_path_.getClosestPos(stop_line_point);
        std::vector<double> car_pos_global_point = local_path_.getClosestPos(car_pos);

        double line_dis = std::hypot(car_pos_global_point[0] - stop_line_global_point[0], car_pos_global_point[1] - stop_line_global_point[1]);

        std::cout << "정지선까지 거리  :  " << line_dis << std::endl;

        std::cout << "현재 신호   :  ";
        check_traffic_sign(tf_light);

        if (vision_.getRefVLight().empty())
        {
            std::cout << "인지가 신호 안줌" << std::endl;
        }

        if (!(late_2_stop_))
        {
            control_.setTargetVelocity(2);
            if (line_dis < STOP_DISTANCE_)
            {
                if (tf_light[3] == 1)
                {
                    control_.setTargetVelocity(2); // serial -> normal
                    stop_ing_ = false;
                }
                else
                {
                    control_.setTargetVelocity(0); // serial->e-stop
                    stop_ing_ = true;
                }
            }
            if (line_dis >= STOP_DISTANCE_ && line_dis < STOP_DISTANCE_PRE_)
            {
                if (tf_light[3] == 1)
                {
                    control_.setTargetVelocity(2); // serial->normal
                }
                else
                {
                    control_.setTargetVelocity(1); // serial->slow
                }
            }
        }
        else
        {
            control_.setTargetVelocity(5); // serial->delemma
        }
        if (!(stop_ing_))
        {
            if (line_dis < LATE_DIS_)
            {
                late_2_stop_ = true;
            }
        }
    }

    void stopline_no_light()
    {
        const double *car_pos = local_.getAddCurCarUTMPos();
        int mission_idx = local_.getCurMissionIndex();
        std::vector<double> stop_line_point = {STOP_LINE_LIST_[mission_idx][0], STOP_LINE_LIST_[mission_idx][1]};
        std::vector<double> stop_line_global_point = local_path_.getClosestPos(stop_line_point);
        std::vector<double> car_pos_global_point = local_path_.getClosestPos(car_pos);

        double line_dis = std::hypot(car_pos_global_point[0] - stop_line_global_point[0], car_pos_global_point[1] - stop_line_global_point[1]);

        std::cout << " 정지선까지 거리 : " << line_dis << std::endl;

        if (!(stop_ing_))
        {
            if (line_dis < STOP_DISTANCE_)
            {
                control_.setTargetVelocity(2);
                std::cout << " *** 정지중 *** " << std::endl;
                control_.setTargetVelocity(0); // serial->e-stop
                if (!(time_check_))
                {
                    std::cout << "타이머 가동" << std::endl;
                    tf_time_ = std::chrono::high_resolution_clock::now();
                    time_check_ = true;
                    stop_ing_ = false;
                }
                else
                {
                    auto current_time = std::chrono::high_resolution_clock::now();
                    double elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(current_time - tf_time_).count();
                    if (elapsed_time > STOP_LINE_TIME_)
                    {
                        control_.setTargetVelocity(2); // serial->noral
                        stop_ing_ = true;
                        time_check_ = false;
                    }
                }
            }
            else if (line_dis >= STOP_DISTANCE_ && line_dis < STOP_DISTANCE_PRE_)
            {
                control_.setTargetVelocity(1); // serial->slow
                std::cout << "김속중 .. " << std::endl;
            }
            else
            {
                control_.setTargetVelocity(2); // serial->normal
                std::cout << "주행중 .. " << std::endl;
            }
        }
        else
        {
            control_.setTargetVelocity(2); // serial->normal
        }
    }
    void check_traffic_sign(std::vector<int> traffic_sign)
    {
        if (traffic_sign[0] == 1)
        {
            std::cout << "red" << std::endl;
        }
        else if (traffic_sign[1] == 1 && traffic_sign[2] == 1)
        {
            std::cout << "orang e & left" << std::endl;
        }
        else if (traffic_sign[1] == 1 && traffic_sign[3] == 2)
        {
            std::cout << "orange & green" << std::endl;
        }
        else if (traffic_sign[1] == 1)
        {
            std::cout << "orange" << std::endl;
        }
        else if (traffic_sign[2] == 1)
        {
            std::cout << "left" << std::endl;
        }
        else if (traffic_sign[3] == 1)
        {
            std::cout << "green" << std::endl;
        }
        else
        {
            std::cout << "인지가 표지판 안줌" << std::endl;
        }
    }

    void bump()
    {
        const double *car_pos = local_.getAddCurCarUTMPos();
        std::vector<double> bump_global_point = local_path_.getClosestPos(bump_point_);
        std::vector<double> car_pos_global_point = local_path_.getClosestPos(car_pos);

        double bump_dis = std::hypot(car_pos_global_point[0] - bump_global_point[0], car_pos_global_point[1] - bump_global_point[1]);
        std::cout << "--- 방지턱 까지 거리 : " << bump_dis << std::endl;
        std::cout << "--- 5m 전 감속 --- 2m 후 가속 ---" << std::endl;
        control_.setTargetVelocity(2); // serial->normal
        if (bump_dis < 5)
        {
            bump_flag_ = true;
        }
        else if ((bump_dis >= 1) && bump_flag_) // 방지턱 탈출 후 속도
        {
            bump_in_flag_ = true;
        }
        else if ((bump_dis >= 2) && bump_in_flag_) // 방지턱 탈출 후 속도
        {
            control_.setTargetVelocity(1.5);
        }
    }
};

#endif
