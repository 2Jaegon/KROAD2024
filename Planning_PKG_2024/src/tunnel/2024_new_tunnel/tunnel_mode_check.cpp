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

void Tunnel2024NEW::modeCheck()
{
    /*
    터널이 어떤 모드로 주행할 지 판별하는 함수입니다.
    1순위 : GPS 모드 (DriveStatus::GPS)
    2순위 : 벽 주행 모드 (DriveStatus::WALL)
    3순위 : 차선 센터 포인트 주행 모드 (DriveStatus:LANE)
    4순위 : NONE 모드 (DriveStatus::NOMODE) ->해당 모드로 전환 시 터널 미션은 실패한 것으로 간주.

    INIT : 초기화 함수
    */
    switch (now_status_)
    {
    case DriveStatus::INIT:
    {
        control_.setTargetVelocity(kVelocityInit); // 초기 속도 설정
        // std::cout << "여기까지 들어가나요?" << "\n";
        now_status_ = DriveStatus::NOMODE;
        break;
    }
    case DriveStatus::NOMODE:
    {
        if (checkGPS())
        {
            // GPS 모드 잠정적 폐기. Lane mode 사용.
            now_status_ = DriveStatus::LANE;
            change_gps_ = true;
        }
        else if (checkWall())
        { // 벽 감지 O : WALL모드로 변경
            now_status_ = DriveStatus::WALL;
        }
        else if (checkCenterLane())
        { // 벽 감지 X + 차선 감지 O : LANE 모드로 변경
            now_status_ = DriveStatus::LANE;
        }
        else
        {
            return;
        } // 벽감지 X + 차선 감지 X : 변경사항 없음. return.
        break;
    }

    case DriveStatus::LANE:
    {
        if (checkGPS())
        {
            //     // LANE 적용 할 때
            //     if (!gps_ok_)
            //     {
            //         gps_ok_time_ = std::chrono::system_clock::now();
            //         gps_ok_ = true;
            //     }

            //     if (gps_ok_)
            //     {
            //         if (control_.checkWaitTime(gps_ok_time_, 20))
            //         {
            //             now_status_ = DriveStatus::GPS;
            //             change_gps_ = true;
            //         }
            //     }

            //     // LANE 적용 안 할 때
            //     // now_status_ = DriveStatus::GPS;
            //     // change_gps_ = true;

            return; // 계속 Lane Mode로 가는게 더 안정성 있음. GPS 모드 잠정적 폐기
        }
        else if (checkWall())
        { // 벽 감지 O : WALL모드로 변경
            now_status_ = DriveStatus::WALL;
            gps_ok_ = false;
        }
        else if (checkCenterLane())
        {
            gps_ok_ = false;
            return;
        } // 벽 감지 X + 차선 감지 계속 : 변경사항 없음 return
        else
        { // 벽 감지 X + 차선 감지 x : NOMODE로 변경
            gps_ok_ = false;
            now_status_ = DriveStatus::NOMODE;
        }
        break;
    }

    case DriveStatus::WALL:
    {
        if (checkGPS()) // 바로 GPS 적용할 때
        // if (false) // GPS 전 LANE 적용할 때
        {
            // GPS모드 잠정 폐지. Lane mode 확정.
            now_status_ = DriveStatus::LANE;
            change_gps_ = true;
        }
        else if (checkWall())
        {
            return;
        } // 벽 감지 계속 : 변경사항 없음. return.
        else if (checkCenterLane())
        { // 벽 감지 X + 차선 감지 O : LANE 모드로 변경
            // 무조건 GPS 가기 전에 LANE으로 갈 수 있도록 변경
            now_status_ = DriveStatus::LANE;
        }
        else
        { // 벽 감지 X + 차선 감지 x : NOMODE로 변경
            now_status_ = DriveStatus::NOMODE;
        }
        break;
    }
    case DriveStatus::GPS:
    {
        if (checkGPS())
        {
            return;
        }
        else if (checkWall())
        { // 벽 감지 O : WALL모드로 변경
            now_status_ = DriveStatus::WALL;
            change_gps_ = false;
        }
        else if (checkCenterLane())
        { // 벽 감지 X + 차선 감지 O : LANE 모드로 변경
            now_status_ = DriveStatus::LANE;
            change_gps_ = false;
        }
        else
        { // 벽 감지 X + 차선 감지 x : NOMODE로 변경
            now_status_ = DriveStatus::NOMODE;
            change_gps_ = false;
        }
        break;
    }
    }

    return;
}

bool Tunnel2024NEW::checkGPS()
{
    // // 기존 코드 (2023)
    if (in_tunnel_)
    {
        tunnel_ceiling_ = lidar_.getTunnelCeiling();
        if (tunnel_ceiling_ >= 0)
        {
            std::cout << "천장 길이 : " << tunnel_ceiling_ << "\n";
            if (tunnel_ceiling_ > 30 && lidar_.getRefTunnelLWall().size() >= kFindNum && lidar_.getRefTunnelRWall().size() >= kFindNum) // 같다 포함되는지 아닌지가 헷갈리는데 일단 함. 참고.
            {
                return false;
            }

            else
            {
                if (tunnel_ceiling_ <= 0)
                {
                    // 8초 이상 감지 안되면 wall mode 해제
                    if (no_ceil_t_ == std::chrono::system_clock::time_point())
                    {
                        no_ceil_t_ = std::chrono::system_clock::now();
                    }

                    if (control_.checkWaitTime(no_ceil_t_, 4))
                    {
                        return true;
                    }
                    else
                    {
                        return false;
                    }
                }

                return false;
            }
        }
    }

    return false;

    // return lidar_.getIsTunnelEnd(); // PointCloud 개수
}

bool Tunnel2024NEW::checkWall()
{
    /* 벽이 제대로 감지되는 상황인가? */
    double L_len = lidar_.getTunnelLLen();
    double R_len = lidar_.getTunnelRLen();
    if (L_len >= 12 && R_len >= 8) // 터널이 확실히 감지되는 지점 (L:12, R:8)
    {
        // // 디버깅용
        // std::cout << "LLLLLLLLLLL : " << L_len << "\n";
        // std::cout << "RRRRRRRRRRR : " << R_len << "\n";
        if (tunnel_start_t_ == std::chrono::system_clock::time_point())
        { // 벽 처음 보기 시작할 때
            tunnel_start_t_ = std::chrono::system_clock::now();
            return false;
        }
        if (control_.checkWaitTime(tunnel_start_t_, 2))
        {
            tunnel_ceiling_ = lidar_.getTunnelCeiling();
            if (tunnel_ceiling_ >= 0)
            {
                std::cout << "천장 길이 : " << tunnel_ceiling_ << "\n";
                if (!in_tunnel_ && tunnel_ceiling_ > 30 && lidar_.getRefTunnelLWall().size() >= kFindNum && lidar_.getRefTunnelRWall().size() >= kFindNum) // 같다 포함되는지 아닌지가 헷갈리는데 일단 함. 참고.
                {
                    in_tunnel_ = true; // 터널 첫진입
                    tunnel_entry_t_ = std::chrono::system_clock::now();
                    last_time_for_small_ = tunnel_entry_t_;
                    return true;
                }

                else
                {
                    if (tunnel_ceiling_ <= 0)
                    {
                        // 8초 이상 감지 안되면 wall mode 해제
                        if (no_ceil_t_ == std::chrono::system_clock::time_point())
                        {
                            no_ceil_t_ = std::chrono::system_clock::now();
                        }

                        if (control_.checkWaitTime(no_ceil_t_, 8))
                        {
                            return false;
                        }
                        else
                        {
                            return true;
                        }
                    }

                    else if (tunnel_ceiling_ < 30 || R_len < 8 || L_len < 12 || lidar_.getRefTunnelLWall().size() < kFindNum || lidar_.getRefTunnelRWall().size() < kFindNum)
                    {
                        // 벽모드가 실행 불가한 상황.
                        no_ceil_t_ = std::chrono::system_clock::time_point();
                        if (no_tunnel_t_ == no_ceil_t_)
                        {
                            no_tunnel_t_ = std::chrono::system_clock::now();
                        } // gps 초기화 한 김에 또 에폭 불러내는 것보단 초기화한 gps_t와 비교하는 게 낫다고 생각하여 gps_t와 비교하니 참고바람.

                        if (control_.checkWaitTime(no_tunnel_t_, 2))
                        {
                            return false;
                        } // 2초이상 안잡히면 월모드해제~, 기존 2
                        else
                        {
                            return true;
                        }
                    }

                    else
                    {
                        no_ceil_t_ = std::chrono::system_clock::time_point();
                        no_tunnel_t_ = no_ceil_t_; // 이미 gps_t가 초기화된 상태이므로...
                        return true;
                    }
                }
            }
        }
    }
    else
    {
        if (tunnel_start_t_ != std::chrono::system_clock::time_point())
        {
            std::cout << "-----초기화-----" << "\n";
            tunnel_start_t_ = std::chrono::system_clock::time_point(); // epoch(1970.01.01)으로 초기화
        } // 바뀐 게 있을 때만 초기화 진행.

        return false;
    }

    return false; // 모든 조건에도 걸리지 않는다면 기본적으로 false
}

bool Tunnel2024NEW::checkCenterLane()
{                                                  // 차선 인식 되는지 안되는지
    if ((lidar_.getRefTunnelCenter()).size() >= 4) // 최소 4개여야 경로 생성됨.
    {
        lidar_center_ = lidar_.getRefTunnelCenter();
        return true;
    } // 차선 센터 인식 OK
    else
    {
        return false;
    } // 차선 센터 인식 XXX

    return false; // 모든 조건에도 걸리지 않는다면 기본적으로 false
}