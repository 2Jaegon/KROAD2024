/***
 * 작성자: 조윤서
 * 설명: std::chrono를 사용하여 차량이 잠시 멈춘 후 출발하게 하는 클래스
 * Style
 * -클래스: PascalCase
 * -멤버변수: snake_case_   (snake_case 뒤에 밑줄)(파이썬 사용 x)
 * -함수: camelCase
 * -변수: snake_case
 * -상수: kPascalCase   (PascalCase앞에 k 접두어)
 ***/

#ifndef CAR_WAIT_TIMER_H_
#define CAR_WAIT_TIMER_H_

#include <iostream>
#include <chrono>
#include "control.hpp"

class CarWaitTimer
{
private:
    Control &control_;
    bool time_check_ = false;
    std::chrono::system_clock::time_point start_time_;

public:
    CarWaitTimer(Control &control) : control_(control) {}
    ~CarWaitTimer() {}

    bool waitASecFunc(double v = 0, int during = 1)
    { // 특정 시간동안 멈추는 함수
        if (!time_check_)
        {
            start_time_ = std::chrono::system_clock::now();
            time_check_ = true;
            control_.setTargetVelocity(Velocity::STOP);
        }
        // 시리얼 모드 스탑으로 변경
        std::cout << "디버깅_현재 속도 -> " << control_.getTargetVelocity() << "\n";

        return checkASecFunc(start_time_, during, &time_check_, v); // time_check false로 풀어줘서 다시 함수 쓸 수 있게 함.
    }

    // 특정 시간이 지났는지 확인해주는 함수.
    // auto로 지정했지만 std::chrono 타입으로 넘겨주세요...
    bool checkASecFunc(auto start_time, int standard = 1, bool *stop_trigger = nullptr, double v = -2024)
    {
        std::chrono::seconds wait_time = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - start_time);
        if (wait_time.count() >= standard)
        {
            if (stop_trigger != nullptr)
            {
                *stop_trigger = false;
            } // 초기화해야할 스탑 트리거가 있다면 초기화.
            if (v != -2024)
            {
                control_.setTargetVelocity(v);
            } // 설정해줄 속도 값이 있다면 속도 설정
            return true;
        }
        else
        {
            std::cout << "대기한 시간(sec) : " << wait_time.count() << "\n";
            return false;
        }
    }
};
#endif

/*
사용 예시 첨부합니다~

waitASecFunc 사용 예시

#include "control.hpp"

int main(void)
{
    CarWaitTimer ct;
    bool its = true;
    bool my_flag1 = true;
    bool my_flag2 = false;
    bool my_flag3 = false;


    while(1){ //로스 스핀 돌리기 귀찮아서염
        if (my_flag1){
            if (ct.waitASecFunc(5)){
                my_flag1 = false;
                my_flag2 = true;
            }
        }
        if (my_flag2){
            if (ct.waitASecFunc(3)){
                my_flag2 = false;
                my_flag3 = true;
            }
        }
        if (my_flag3){
            if (ct.waitASecFunc(7)){
                my_flag3 = false;
                std::cout << "종료!" << "\n";
            }
        }
    }
}
*/