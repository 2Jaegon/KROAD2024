#ifndef CAMERA_ID_H
#define CAMERA_ID_H

#include <string>
#include <cstdlib>
#include <iostream>

// 전역 변수 선언
std::string video_device[3];

// 함수 선언
void setCameraIds()
{
    const char *team_env = std::getenv("KROAD_TEAM");
    std::string team = "not set";

    // 환경변수 있는지 확인~ 없으면 직접 입력하기
    if (team_env == nullptr)
    {
        std::cout << "KROAD_TEAM environment variable not set." << std::endl;

        while (team != "A" && team != "B")
        {
            std::cout << "Enter your Team: (A/B)" << std::endl;
            std::cin >> team;

            if (team == "A" || team == "a")
                team = "A";
            else if (team == "B" || team == "b")
                team = "B";
            else
                std::cout << "Unknown team name: " << team << ", try again👿👿👿" << std::endl;
        }
    }
    else
        team = std::string(team_env);

    // 팀 출력
    std::cout << "Your team is " << team;
    if (team == "A")
        std::cout << "🐘 K-BUB 🐘" << std::endl;
    else if (team == "B")
        std::cout << "🦒 KORUS 🦒" << std::endl;

    // 팀에 따른 카메라 경로
    if (team == "A")
    {
        // 왼 오 중
        video_device[0] = "/dev/v4l/by-id/usb-046d_罗技高清网络摄像机_C930c_554D90DE-video-index0";
        video_device[1] = "/dev/v4l/by-id/usb-046d_罗技高清网络摄像机_C930c-video-index0";
        video_device[2] = "/dev/v4l/by-id/usb-046d_罗技高清网络摄像机_C930c_E776BDCE-video-index0";
    }
    else
    {
        video_device[0] = "/dev/v4l/by-id/usb-046d_罗技高清网络摄像机_C930c_744E4AAE-video-index0";
        video_device[1] = "/dev/v4l/by-id/usb-046d_Logitech_Webcam_C930e_964D8E9E-video-index0";
        video_device[2] = "/dev/v4l/by-id/usb-046d_罗技高清网络摄像机_C930c_A7B93AAE-video-index0";
    }
}

#endif // CAMERA_ID_H
