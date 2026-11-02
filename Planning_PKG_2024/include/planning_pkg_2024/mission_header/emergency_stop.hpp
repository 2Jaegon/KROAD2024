/***
 * 작성자: 조윤서
 * 설명: 동적 장애물 회피 판단
 * Style
 * -클래스: PascalCase
 * -멤버변수: snake_case_   (snake_case 뒤에 밑줄)(파이썬 사용 x)
 * -함수: camelCase
 * -변수: snake_case
 * -상수: kPascalCase   (PascalCase앞에 k 접두어)
 ***/

/*
DWA 사용하여 작성 예정.
분석 중에 있음.
*/
#ifndef EMG_STOP_HPP
#define EMG_STOP_HPP

#include <iostream>
#include <chrono>
#include "control.hpp"

class EmergencyStop
{
private:
    Control &control_;

    bool stop_trigger_ = false;
    std::chrono::system_clock::time_point start_t;
    bool time_check_ = false;

    bool stop_more_than_once_ = false;

public:
    EmergencyStop(Control &ctrl) : control_(ctrl) {}
    ~EmergencyStop() {};

    bool getStopTrigger()
    {
        return stop_trigger_;
    }

    bool getStopMoreThanOnce()
    {
        return stop_more_than_once_;
    }

    void setStopTrigger(bool stop)
    {
        stop_trigger_ = stop;
    }

    void stopMomentFunc()
    {
        if (stop_trigger_)
        {
            if (!time_check_)
            {
                start_t = std::chrono::system_clock::now();
                time_check_ = true;
                control_.setTargetVelocity(Velocity::STOP);
            }
            // 시리얼 모드 스탑으로 변경
            std::cout << "디버깅_현재 속도 -> " << control_.getTargetVelocity() << "\n";

            if (stopOrStart())
            {
                // 시리얼 모드 노멀로 변경
                control_.setTargetVelocity(2.5);
                std::cout << "디버깅_현재 속도 -> " << control_.getTargetVelocity() << "\n";
                time_check_ = false;
                stop_trigger_ = false;
                // if (!stop_more_than_once_)
                // {
                //     stop_more_than_once_ = true;
                // }
            }
        }
    }

    bool stopOrStart()
    {
        std::chrono::duration<double> wait_t = std::chrono::system_clock::now() - start_t;
        std::cout << "현재까지 대기한 시간 : " << wait_t.count() << "\n";
        return wait_t.count() > 5;
    }
};

#endif
