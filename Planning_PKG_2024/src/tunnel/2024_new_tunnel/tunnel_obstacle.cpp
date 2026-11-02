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

#include "2024_new_tunnel.hpp"

void Tunnel2024NEW::dynamicObjFunc() // 동적 장애물 코드
{
    // 동적 장애물 감지했고 정지 상태 아니면
    if (lidar_.getIsDynamicStop() && (!stop_trigger_))
    {
        stop_trigger_ = true; // 정지 신호 true
    }

    if (stop_trigger_) // 정지 신호 들어왔으면
    {
        if (control_.waitAndDepart(5, kVelocityDynamic)) // 5초 기다려야 정지 신호 해제
        {
            stop_trigger_ = false;
        }
    }
}

void Tunnel2024NEW::checkStaticToDynamic()
{
    switch (StoD_)
    {
    case 0:
    {
        StoD_ += (in_tunnel_) ? 1 : 0; // 터널이면 동적 준비
        break;
    }
    case 1:
    {
        std::cout << "🍎🍎🍎🍎🍎 정적 소형 하는 중!!! : 모드 1 🍎🍎🍎🍎🍎" << "\n";
        // 시간에 따른 동적 장애물 변환 체크 : 임계값 6초
        if (last_time_for_small_ != std::chrono::system_clock::time_point())
        {
            // 첫 진입하고나서 10초간 아무것도 못 보면 바로 동적 전환. 아닐 시 마지막 장애물에서부터 4초 감지
            int wait_time = ((last_time_for_small_ == tunnel_entry_t_) ? 30 : 6);
            std::cout << "동적 대기 시간은 얼마나??? : " << wait_time << "\n";

            if (small_ing_ && control_.checkWaitTime(last_time_for_small_, wait_time)) // 처음 동적으로 전환될 때.
            {
                small_ing_ = false;
                lidar_.setDynamicMode(true);
                StoD_ = 3;
            }
        }
        std::cout << "🍎🍎🍎🍎🍎🍎🍎🍎🍎🍎🍎🍎🍎🍎🍎🍎🍎" << "\n";
        break;
    }
    case 2:
    {
        std::cout << "🍎🍎🍎🍎🍎 정적 소형 하는 중!!! : 모드 2 🍎🍎🍎🍎🍎" << "\n";
        if (small_static_flag_ >= 5)
        {
            std::cout << "잘 가나?????" << "\n";
            if (small_ing_) // 처음 동적으로 전환될 때.
            {
                small_ing_ = false;
                lidar_.setDynamicMode(true);
                StoD_++;
            }
        }
        std::cout << "🍎🍎🍎🍎🍎🍎🍎🍎🍎🍎🍎🍎🍎🍎🍎🍎🍎" << "\n";
        break;
    }

    case 3: // 동적으로 무사히 넘어갔을 때.
    {
        control_.setTargetVelocity(kVelocityDynamic);
        StoD_++; // 속도 딱 한 번만 초기화하도록.
        break;
    }

    default:
    {
        break;
    }
    }
}

void Tunnel2024NEW::checkSSStatus(int now)
{ // small static 상태 변화
    switch (small_static_flag_)
    {
    case 0:
    {
        if (now == 1) // x0 -> 01
        {
            small_static_flag_++;
            StoD_ = 2;
        }
        else // 감지 실패
        {
            small_static_flag_ = 0;
            StoD_ = 1;
        }
        break;
    }

    case 1:
    {
        if (now == 2) // 01 -> 12
        {
            small_static_flag_++;
        }
        if (now == 1) // 현상 유지
        {
            return;
        }
        if (now == 0) // 회피 실패
        {
            small_static_flag_ = 0;
        }
        break;
    }

    case 2:
    {
        if (now == 1) // 12 -> 21
        {
            small_static_flag_++;
        }
        if (now == 2) // 현상 유지
        {
            return;
        }
        if (now == 0) // 회피 실패
        {
            small_static_flag_ = 0;
        }
        break;
    }

    case 3:
    {
        if (now == 0) // 21 -> 10
        {
            small_static_flag_++;
        }
        else // 현상 유지. 또한 회피할 때 2개 볼 수 있음.
        {
            return;
        }
        break;
    }

    case 4:
    {
        if (!dynamic_check_init) // 10 -> 00
        {
            dynamic_check_init = true;
            last_time_for_small_ = std::chrono::system_clock::now();
        }

        if (control_.checkWaitTime(last_time_for_small_, 1))
        {
            small_static_flag_++;
        }

        break;
    }

    default:
    {
        break;
    }
    }
}