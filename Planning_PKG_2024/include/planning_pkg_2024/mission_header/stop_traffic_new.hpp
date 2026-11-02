/***
 * 작성자: 김호진
 * 설명: 신호등 관련 클래스
 * Style
 * -클래스: PascalCase
 * -멤버변수: snake_case_   (snake_case 뒤에 밑줄)(파이썬 사용 x)
 * -함수: camelCase
 * -변수: snake_case
 * -상수: kPascalCase   (PascalCase앞에 k 접두어)
 ***/

#ifndef STOP_TRAFFIC_NEW_H_
#define STOP_TRAFFIC_NEW_H_

#include <algorithm>
#include <chrono>
#include <thread>
#include "local.hpp"
#include "nlohmann/json.hpp"

#include "control.hpp"
#include "nlohmann/json.hpp"
#include "vision.hpp"

class StopTrafficNew
{
private:
    enum class TrafficState
    {
        INIT,
        EMERGENCY,
        GO_OR_STOP,
        GO,
        STOP,
        DONE
    };

    enum class StopState
    {
        INIT,
        APPROACH,
        STOP,
        DONE
    };

    enum class DoubleStopState
    {
        INIT,
        APPROACH,
        STOP,
        INIT_NEXT,
        APPORACH_NEXT,
        STOP_NEXT,
        DONE

    };

    enum class BumpState
    {
        INIT,
        APPROACH,
        SLOW,
        DONE
    };

    std::vector<std::vector<double>> STOP_LINE_LIST_;

    std::vector<std::vector<double>> TWO_STOP_LINE_;
    bool is_on_bump_ = false;

    Local &local_;
    Control &control_;
    Path &local_path_;
    Vision &vision_;

    TrafficState tf_current_state_ = TrafficState::INIT;
    StopState stop_current_state_ = StopState::INIT;
    TrafficState tf_prev_state_ = TrafficState::INIT;
    DoubleStopState double_stop_current_state_ = DoubleStopState::INIT;
    BumpState bump_current_state_ = BumpState::INIT;

    //////////////////stop line && traffic ///////////////////
    double STOP_DISTANCE_PRE_ = 8;
    double STOP_DISTANCE_ = 5;

    double STOP_LINE_TIME_ = 3.3;

    double tf_normal_target_spped_ = 4.0;
    double tf_straight_done_target_speed_ = 6.0;
    double tf_left_done_target_speed_ = 5.0;
    double tf_reduce_target_speed_ = 4.0;
    double tf_emergency_target_speed_ = 6.9;

    ///////////////////////////////////////////////////////////////

    /////////////////////////bump////////////////////////////////
    std::vector<double> bump_point_ = {346907.6816799486, 4069898.7344150976};

    double bump_init_velocity_;
    double bump_approach_velocity_;
    double bump_slow_velocity_;
    double bump_done_velocity_;

    double bump_approach_dis_;
    double bump_slow_dis_;
    double bump_escape_dis_;
    ///////////////////////////////////////////////////////////////
    std::chrono::time_point<std::chrono::high_resolution_clock> tf_time_;

public:
    StopTrafficNew(Local &local, Control &control, Path &local_path, Vision &vision, nlohmann::json &param)
        : local_(local), control_(control), local_path_(local_path), vision_(vision),
          STOP_DISTANCE_(param["stop_distance"]), STOP_DISTANCE_PRE_(param["stop_distance_pre"]),
          STOP_LINE_TIME_(param["stop_line_time"]),
          bump_point_(param["bump_point"]), STOP_LINE_LIST_(param["stop_line_list"]),
          bump_init_velocity_(param["bump_init_velocity"]),
          bump_slow_velocity_(param["bump_slow_velocity"]), bump_approach_velocity_(param["bump_approach_velocity"]),
          bump_done_velocity_(param["bump_done_velocity"]), bump_approach_dis_(param["bump_approach_dis"]),
          bump_slow_dis_(param["bump_slow_dis"]), bump_escape_dis_(param["bump_escape_dis"]),
          TWO_STOP_LINE_(param["two_stop_line"]), tf_normal_target_spped_(param["tf_normal_target_spped_"]),
          tf_straight_done_target_speed_(param["tf_straight_done_target_speed_"]), tf_left_done_target_speed_(param["tf_left_done_target_speed_"]),
          tf_reduce_target_speed_(param["tf_reduce_target_speed_"]), tf_emergency_target_speed_(param["tf_emergency_target_speed_"])

    {
    }

    void traffic_left()
    {
        std::vector<int> tf_light = vision_.getRefVLight();
        const double *car_pos = local_.getAddCurCarUTMPos();
        int mission_idx = local_.getCurMissionIndex();
        std::vector<double> stop_line_point = {STOP_LINE_LIST_[mission_idx][0], STOP_LINE_LIST_[mission_idx][1]};
        std::vector<double> stop_line_global_point = local_path_.getClosestPos(stop_line_point);
        std::vector<double> car_pos_global_point = local_path_.getClosestPos(car_pos);
        double line_dis_ = std::hypot(car_pos_global_point[0] - stop_line_global_point[0], car_pos_global_point[1] - stop_line_global_point[1]);

        std::cout << "stop line distance : " << line_dis_ << std::endl;
        std::cout << "(0 : red) / (1 : orange) / (2 : left) / (3 : gree)" << std::endl;
        for (int i = 0; i < tf_light.size(); i++)
        {
            std::cout << tf_light[i] << " ";
        }
        std::cout << "\n";

        if (line_dis_ < 0.2)
        {
            tf_current_state_ = TrafficState::EMERGENCY;
        }

        switch (tf_current_state_)
        {

        case TrafficState::INIT:
        {
            control_.setTargetVelocity(tf_normal_target_spped_);
            std::cout << "현재상태 : INIT" << std::endl;

            if (line_dis_ >= STOP_DISTANCE_ && line_dis_ < STOP_DISTANCE_PRE_)
            {
                if ((tf_light[1] == 2))
                {
                    tf_current_state_ = TrafficState::INIT; // Stop-YOLO-Node
                    break;
                }
                else if (isAllZero(tf_light))
                {
                    tf_current_state_ = TrafficState::INIT; // No-Flir-Camera-Information
                    break;
                }
                else
                {
                    tf_current_state_ = TrafficState::GO_OR_STOP;
                    break;
                }
            }
            else if (line_dis_ < STOP_DISTANCE_)
            {
                tf_current_state_ = TrafficState::EMERGENCY;
                break;
            }

            break;
        }

        case TrafficState::GO_OR_STOP:
        {
            std::cout << "현재상태 : GO_OR_STOP" << std::endl;

            if (tf_light[2] == 1)
            {
                control_.setTargetVelocity(5);
                std::cout << "갈 길 갈 거 임 (좌회전 신호 o)" << std::endl;
            }
            else
            {
                control_.setTargetVelocity(0);
                std::cout << "정지중 (좌회전 신호 x)" << std::endl;
            }

            if (line_dis_ < STOP_DISTANCE_)
            {
                if (tf_light[2] == 1)
                {
                    tf_current_state_ = TrafficState::EMERGENCY;
                }
                else
                {
                    tf_current_state_ = TrafficState::STOP;
                }
            }
            break;
        }

        case TrafficState::STOP:
        {
            std::cout << "현재상태 : STOP" << std::endl;

            if (tf_light[2] == 1)
            {
                control_.setTargetVelocity(5);
                std::cout << "갈 길 갈 거 임 (좌회전 신호 o)" << std::endl;
            }
            else
            {
                control_.setTargetVelocity(0);
                std::cout << "정지중 (좌회전 신호 x)" << std::endl;
            }
            break;
        }

        case TrafficState::EMERGENCY:
        {
            std::cout << "!! EMERGENCY !!" << std::endl;
            control_.setTargetVelocity(tf_emergency_target_speed_);

            if (line_dis_ > 11)
            {
                tf_current_state_ = TrafficState::DONE;
            }
            break;
        }

        case TrafficState::DONE:
        {
            std::cout << "현재 상태 : DONE" << std::endl;
            if (local_.getCurMissionIndex() == 11)
            {
                control_.setTargetVelocity(6.9);
            }
            else
            {
                control_.setTargetVelocity(tf_left_done_target_speed_);
            }
            break;
        }
        }
    }

    void traffic_straight()
    {
        std::vector<int> tf_light = vision_.getRefVLight();
        const double *car_pos = local_.getAddCurCarUTMPos();
        int mission_idx = local_.getCurMissionIndex();
        std::vector<double> stop_line_point = {STOP_LINE_LIST_[mission_idx][0], STOP_LINE_LIST_[mission_idx][1]};
        std::vector<double> stop_line_global_point = local_path_.getClosestPos(stop_line_point);
        std::vector<double> car_pos_global_point = local_path_.getClosestPos(car_pos);
        double line_dis_ = std::hypot(car_pos_global_point[0] - stop_line_global_point[0], car_pos_global_point[1] - stop_line_global_point[1]);

        std::cout << "stop line distance : " << line_dis_ << std::endl;
        std::cout << "(0 : red) / (1 : orange) / (2 : left) / (3 : gree)" << std::endl;
        for (int i = 0; i < tf_light.size(); i++)
        {
            std::cout << tf_light[i] << " ";
        }
        std::cout << "\n";

        if (line_dis_ < 0.2)
        {
            tf_current_state_ = TrafficState::EMERGENCY;
        }

        switch (tf_current_state_)
        {

        case TrafficState::INIT:
        {
            control_.setTargetVelocity(tf_normal_target_spped_);
            std::cout << "현재상태 : INIT" << std::endl;

            if (line_dis_ >= STOP_DISTANCE_ && line_dis_ < STOP_DISTANCE_PRE_)
            {
                if ((tf_light[1] == 2))
                {
                    tf_current_state_ = TrafficState::INIT; // Stop-YOLO-Node
                    break;
                }
                else if (isAllZero(tf_light))
                {
                    tf_current_state_ = TrafficState::INIT; // No-Flir-Camera-Information
                    break;
                }
                else
                {
                    tf_current_state_ = TrafficState::GO_OR_STOP;
                    break;
                }
            }
            else if (line_dis_ < STOP_DISTANCE_)
            {
                tf_current_state_ = TrafficState::EMERGENCY;
                break;
            }
            break;
        }

        case TrafficState::GO_OR_STOP:
        {
            std::cout << "현재상태 : GO_OR_STOP" << std::endl;

            if (tf_light[3] == 1)
            {
                control_.setTargetVelocity(5);
                std::cout << "갈 길 갈 거 임 (직진 신호 o)" << std::endl;
            }
            else
            {
                control_.setTargetVelocity(0);
                std::cout << "정지중 (직진 신호 x)" << std::endl;
            }

            if (line_dis_ < STOP_DISTANCE_)
            {
                if (tf_light[3] == 1)
                {
                    tf_current_state_ = TrafficState::EMERGENCY;
                }
                else
                {
                    tf_current_state_ = TrafficState::STOP;
                }
            }
            break;
        }

        case TrafficState::STOP:
        {
            std::cout << "현재상태 : STOP" << std::endl;

            if (tf_light[3] == 1)
            {
                control_.setTargetVelocity(5);
                std::cout << "갈 길 갈 거 임 (직진 신호 o)" << std::endl;
            }
            else
            {
                control_.setTargetVelocity(0);
                std::cout << "정지중 (직진 신호 x)" << std::endl;
            }
            break;
        }

        case TrafficState::EMERGENCY:
        {
            std::cout << "!! EMERGENCY !!" << std::endl;
            control_.setTargetVelocity(tf_emergency_target_speed_);

            if (line_dis_ > 11)
            {
                tf_current_state_ = TrafficState::DONE;
            }
            break;
        }

        case TrafficState::DONE:
        {
            std::cout << "현재 상태 : DONE" << std::endl;
            control_.setTargetVelocity(tf_straight_done_target_speed_);
            break;
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
        switch (stop_current_state_)
        {
        case StopState::INIT:
        {
            std::cout << "현재 상태 : INIT" << std::endl;
            if (mission_idx == 13)
            {
                control_.setTargetVelocity(5);
                if (line_dis < 15)
                {
                    stop_current_state_ = StopState::APPROACH;
                    break;
                }
                else if (line_dis >= STOP_DISTANCE_PRE_)
                {
                    stop_current_state_ = StopState::INIT;
                    break;
                }
            }
            else
            {
                control_.setTargetVelocity(5);
                if (line_dis < STOP_DISTANCE_PRE_)
                {
                    stop_current_state_ = StopState::APPROACH;
                    break;
                }
                else if (line_dis >= STOP_DISTANCE_PRE_)
                {
                    stop_current_state_ = StopState::INIT;
                    break;
                }
            }
            break;
        }

        case StopState::APPROACH:
        {
            std::cout << "현재 상태 : APPROACH" << std::endl;
            control_.setTargetVelocity(5);
            std::cout << "김속중 .. " << std::endl;
            if (line_dis <= STOP_DISTANCE_)
            {
                tf_time_ = std::chrono::high_resolution_clock::now();
                stop_current_state_ = StopState::STOP;
                break;
            }
        }
        break;

        case StopState::STOP:
        {
            std::cout << "현재 상태 : STOP" << std::endl;
            control_.setTargetVelocity(0);
            auto current_time = std::chrono::high_resolution_clock::now();
            double elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(current_time - tf_time_).count();
            std::cout << "타이머 가동 : " << elapsed_time << std::endl;
            if (elapsed_time > STOP_LINE_TIME_)
            {
                stop_current_state_ = StopState::DONE;
                break;
            }
            break;
        }

        case StopState::DONE:
        {

            control_.setTargetVelocity(5);

            std::cout << "현재 상태 : DONE" << std::endl;
            break;
        }
        }
    }

    void double_stopline_no_light()
    {
        const double *car_pos = local_.getAddCurCarUTMPos();
        int mission_idx = local_.getCurMissionIndex();
        std::vector<double> stop_line_point = {TWO_STOP_LINE_[0][0], TWO_STOP_LINE_[0][1]};
        std::vector<double> next_stop_line_point = {TWO_STOP_LINE_[1][0], TWO_STOP_LINE_[1][1]};
        std::vector<double> stop_line_global_point = local_path_.getClosestPos(stop_line_point);
        std::vector<double> car_pos_global_point = local_path_.getClosestPos(car_pos);
        std::vector<double> next_stop_line_global_point = local_path_.getClosestPos(next_stop_line_point);

        double line_dis = std::hypot(car_pos_global_point[0] - stop_line_global_point[0], car_pos_global_point[1] - stop_line_global_point[1]);
        double next_line_dis = std::hypot(car_pos_global_point[0] - next_stop_line_global_point[0], car_pos_global_point[1] - next_stop_line_global_point[1]);

        std::cout << " 정지선까지 거리 : " << line_dis << std::endl;
        std::cout << "  다음 정지선까지 거리 : " << next_line_dis << std::endl;

        switch (double_stop_current_state_)
        {
        case DoubleStopState::INIT:
        {
            std::cout << "현재 상태 : INIT" << std::endl;
            control_.setTargetVelocity(5); // serial->slow
            if (line_dis < STOP_DISTANCE_PRE_)
            {
                double_stop_current_state_ = DoubleStopState::APPROACH;
                break;
            }
            break;
        }
        case DoubleStopState::APPROACH:
        {
            std::cout << "현재 상태 : APPROACH" << std::endl;
            control_.setTargetVelocity(4); // serial->slow
            std::cout << "김속중 .. " << std::endl;
            if (line_dis < STOP_DISTANCE_)
            {
                tf_time_ = std::chrono::high_resolution_clock::now();
                double_stop_current_state_ = DoubleStopState::STOP;
                break;
            }
            break;
        }
        case DoubleStopState::STOP:
        {
            std::cout << "현재 상태 : STOP" << std::endl;
            control_.setTargetVelocity(0);
            auto current_time = std::chrono::high_resolution_clock::now();
            double elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(current_time - tf_time_).count();
            std::cout << "타이머 가동 : " << elapsed_time << std::endl;
            if (elapsed_time > STOP_LINE_TIME_)
            {
                double_stop_current_state_ = DoubleStopState::APPORACH_NEXT;
                break;
            }
            break;
        }

        case DoubleStopState::APPORACH_NEXT:
        {
            std::cout << "현재 상태 : APPROACH_NEXT" << std::endl;
            control_.setTargetVelocity(2);
            std::cout << "김속중 .. " << std::endl;
            if (next_line_dis < 5.4)
            {
                tf_time_ = std::chrono::high_resolution_clock::now();
                double_stop_current_state_ = DoubleStopState::STOP_NEXT;
                break;
            }
            break;
        }

        case DoubleStopState::STOP_NEXT:
        {
            std::cout << "현재 상태 : STOP_NEXT" << std::endl;
            control_.setTargetVelocity(0);
            auto current_time = std::chrono::high_resolution_clock::now();
            double elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(current_time - tf_time_).count();
            std::cout << "타이머 가동 : " << elapsed_time << std::endl;
            if (elapsed_time > STOP_LINE_TIME_)
            {
                double_stop_current_state_ = DoubleStopState::DONE;
                break;
            }
            break;
        }

        case DoubleStopState::DONE:
        {
            control_.setTargetVelocity(5);
            std::cout << "현재 상태 : DONE" << std::endl;
            break;
        }
        }
    }

    void bump()
    {
        const double *car_pos = local_.getAddCurCarUTMPos();
        std::vector<double> bump_global_point = local_path_.getClosestPos(bump_point_);
        std::vector<double> car_pos_global_point = local_path_.getClosestPos(car_pos);

        double bump_dis = std::hypot(car_pos_global_point[0] - bump_global_point[0], car_pos_global_point[1] - bump_global_point[1]);
        std::cout << "방지턱 거리 : " << bump_dis << std::endl;

        switch (bump_current_state_)
        {
        case BumpState::INIT:
        {
            std::cout << "state : Init" << std::endl;
            control_.setTargetVelocity(bump_init_velocity_);
            if (bump_dis < bump_approach_dis_)
            {
                bump_current_state_ = BumpState::APPROACH;
                break;
            }
            break;
        }
        case BumpState::APPROACH:
        {
            std::cout << "state : Apporach" << std::endl;
            control_.setTargetVelocity(bump_approach_velocity_);
            if (bump_dis < bump_slow_dis_)
            {
                bump_current_state_ = BumpState::SLOW;
                break;
            }
            break;
        }
        case BumpState::SLOW:
        {

            std::cout << "state : Slow" << std::endl;
            control_.setTargetVelocity(bump_slow_velocity_);

            if (bump_dis <= 0.1)
            {
                is_on_bump_ = true;
            }

            if (bump_dis >= bump_escape_dis_ && (is_on_bump_))
            {
                bump_current_state_ = BumpState::DONE;
                break;
            }
            break;
        }
        case BumpState::DONE:
        {
            std::cout << "state : Done" << std::endl;
            control_.setTargetVelocity(bump_done_velocity_);
        }
        break;
        }
    }

    void resetStatus()
    {
        tf_current_state_ = TrafficState::INIT;
        tf_prev_state_ = TrafficState::INIT;
        stop_current_state_ = StopState::INIT;
        double_stop_current_state_ = DoubleStopState::INIT;
        bump_current_state_ = BumpState::INIT;
    }

    bool isAllZero(const std::vector<int> &vec) // if all of value is zero -> true
    {
        return std::all_of(vec.begin(), vec.end(), [](int i)
                           { return i == 0; });
    }
};

#endif
