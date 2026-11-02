// 이미지 a. 카메라로 찍고 퍼블리시하는 코드 (수정된 코드)
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include <chrono>
#include <string>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include "std_msgs/msg/string.hpp"

#include "camera/qos_utils.hpp"
#include "camera/camera_id.hpp"

using namespace std;
using namespace cv;
using namespace std::chrono_literals;

Mat frame[3];

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("tri_cam");

    setCameraIds(); // camera id 설정 - camera_id.cpp 파일 참조

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher[3];
    for (int i = 0; i < 3; i++)
        publisher[i] = node->create_publisher<sensor_msgs::msg::Image>("video" + to_string(i + 1), camera::best_effort_qos());

    VideoCapture cap[3];

    for (int i = 0; i < 3; i++)
    {
        cap[i].open(video_device[i], CAP_V4L2);
        cap[i].set(CAP_PROP_FRAME_WIDTH, 640);
        cap[i].set(CAP_PROP_FRAME_HEIGHT, 480);
        // cap[i].set(CAP_PROP_FPS, 30);
        // cap[i].set(CAP_PROP_BRIGHTNESS, 10);

        if (cap[i].isOpened())
            std::cout
                << "video" << i + 1 << " is opened ✅" << std::endl;
        else
            std::cout << "video" << i + 1 << " is not opened ❌" << std::endl;

        // for (int j = 0; j < 30; j++)
        //     cap[i].grab();
    }

    while (rclcpp::ok())
    {
        for (int i = 0; i < 3; i++)
        {
            cap[i] >> frame[i];
        }

        // 동일한 타임스탬프 사용
        auto current_time = node->now();

        std_msgs::msg::Header header[3];
        sensor_msgs::msg::Image msg[3];

        for (int i = 0; i < 3; i++)
        {
            header[i] = std_msgs::msg::Header();
            header[i].stamp = current_time; // 동일한 시간을 타임스탬프로 설정
            auto msg = cv_bridge::CvImage(header[i], "bgr8", frame[i]).toImageMsg();
            publisher[i]->publish(*msg.get());
        }

        rclcpp::spin_some(node);
    }

    return 0;
}
