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
// 정찬우
using namespace std;
using namespace cv;
using namespace std::chrono_literals;

Mat frame3, frame4;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("multi_cam");
    auto publisher3 = node->create_publisher<sensor_msgs::msg::Image>("video3", camera::best_effort_qos()); // deli

    auto publisher4 = node->create_publisher<sensor_msgs::msg::Image>("video4", camera::best_effort_qos()); // traffic

    std::string gvideo_deli = "/dev/v4l/by-id/usb-046d_罗技高清网络摄像机_C930c_A7B93AAE-video-index0";
    std::string gvideo_traffic = "/dev/v4l/by-id/usb-046d_Logitech_Webcam_C930e_BFFAF6DE-video-index0";

    std::string evideo_deli = "/dev/v4l/by-id/usb-046d_罗技高清网络摄像机_C930c_E776BDCE-video-index0";
    std::string evideo_traffic = "/dev/v4l/by-id/usb-046d_Logitech_Webcam_C930e-video-index0";

    VideoCapture cap3(gvideo_deli, CAP_V4L2);    // deli
    VideoCapture cap4(gvideo_traffic, CAP_V4L2); // traiffic

    if (cap3.isOpened())
        std::cout << "deli_cam is opened" << std::endl;
    if (cap4.isOpened())
        std::cout << "traffic_cam is opened" << std::endl;

    while (rclcpp::ok())
    {
        cap3 >> frame3;
        cap4 >> frame4;

        // 동일한 타임스탬프 사용
        // deli
        auto current_time = node->now();
        auto header3 = std_msgs::msg::Header();
        header3.stamp = current_time; // 동일한 시간을 타임스탬프로 설정
        auto msg3 = cv_bridge::CvImage(header3, "bgr8", frame3).toImageMsg();
        publisher3->publish(*msg3.get());

        // traffic
        auto header4 = std_msgs::msg::Header();
        header4.stamp = current_time; // 동일한 시간을 타임스탬프로 설정
        auto msg4 = cv_bridge::CvImage(header4, "bgr8", frame4).toImageMsg();
        publisher4->publish(*msg4.get());

        rclcpp::spin_some(node);
    }

    return 0;
}
