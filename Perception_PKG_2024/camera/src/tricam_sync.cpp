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

using namespace std;
using namespace cv;
using namespace std::chrono_literals;

Mat frame1, frame2, frame3;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("tri_cam");

    auto publisher1 = node->create_publisher<sensor_msgs::msg::Image>("video1", camera::best_effort_qos());
    auto publisher2 = node->create_publisher<sensor_msgs::msg::Image>("video2", camera::best_effort_qos());
    auto publisher3 = node->create_publisher<sensor_msgs::msg::Image>("video3", camera::best_effort_qos());

    std::string evideo1 = "/dev/v4l/by-id/usb-046d_罗技高清网络摄像机_C930c_554D90DE-video-index0";
    std::string evideo2 = "/dev/v4l/by-id/usb-046d_罗技高清网络摄像机_C930c_4366BDCE-video-index0";
    std::string evideo3 = "/dev/v4l/by-id/usb-046d_罗技高清网络摄像机_C930c_E776BDCE-video-index0";

    std::string gvideo2b = "/dev/v4l/by-id/usb-046d_Logitech_Webcam_C930e_964D8E9E-video-index0";
    std::string gvideo1b = "/dev/v4l/by-id/usb-046d_罗技高清网络摄像机_C930c_744E4AAE-video-index0";
    std::string gvideo3 = "/dev/v4l/by-id/usb-046d_罗技高清网络摄像机_C930c_A7B93AAE-video-index0";

    std::string gvideo2f = "/dev/v4l/by-id/usb-046d_Logitech_Webcam_C930e_BFFAF6DE-video-index0";
    std::string gvideo1f = "/dev/v4l/by-id/usb-046d_Logitech_Webcam_C930e-video-index0";

    VideoCapture cap1(evideo1,CAP_V4L2); //왼
    cap1.set(CAP_PROP_FRAME_WIDTH, 640);
    cap1.set(CAP_PROP_FRAME_HEIGHT, 480);

    VideoCapture cap2(evideo2,CAP_V4L2); //오
    cap2.set(CAP_PROP_FRAME_WIDTH, 640);
    cap2.set(CAP_PROP_FRAME_HEIGHT, 480);

    VideoCapture cap3(evideo3,CAP_V4L2); //중
    cap3.set(CAP_PROP_FRAME_WIDTH, 640);
    cap3.set(CAP_PROP_FRAME_HEIGHT, 480);

    if(cap1.isOpened()) std::cout<<"video1 is opend"<<std::endl;
    if(cap2.isOpened()) std::cout<<"video2 is opend"<<std::endl;
    if(cap3.isOpened()) std::cout<<"video3 is opend"<<std::endl;

    while (rclcpp::ok())
    {
        cap1 >> frame1;
        cap2 >> frame2;
        cap3 >> frame3;

        // 동일한 타임스탬프 사용
        auto current_time = node->now();

        auto header1 = std_msgs::msg::Header();
        header1.stamp = current_time; // 동일한 시간을 타임스탬프로 설정
        auto msg1 = cv_bridge::CvImage(header1, "bgr8", frame1).toImageMsg();
        publisher1->publish(*msg1.get());

        auto header2 = std_msgs::msg::Header();
        header2.stamp = current_time; // 동일한 시간을 타임스탬프로 설정
        auto msg2 = cv_bridge::CvImage(header2, "bgr8", frame2).toImageMsg();
        publisher2->publish(*msg2.get());

        auto header3 = std_msgs::msg::Header();
        header3.stamp = current_time; // 동일한 시간을 타임스탬프로 설정
        auto msg3 = cv_bridge::CvImage(header3, "bgr8", frame3).toImageMsg();
        publisher3->publish(*msg3.get());

        rclcpp::spin_some(node);
    }

    return 0;
}