// 이미지 b. 두 이미지를 병합하고 퍼블리시하는 코드 (수정된 코드)
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

// QoS
#include "fusion/qos_utils.hpp"

using namespace std;
using namespace cv;

// 전역 변수로 이미지와 타임스탬프 저장
Mat image1, image2;
bool image1_received = false, image2_received = false;
rclcpp::Time timestamp1, timestamp2;

// 첫 번째 이미지 콜백 함수
void image1_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  try {
    // ROS 이미지 메시지를 OpenCV 이미지로 변환
    image1 = cv_bridge::toCvCopy(msg, "bgr8")->image;
    // 타임스탬프 저장
    timestamp1 = msg->header.stamp;
    image1_received = true;
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("image_subscriber"), "cv_bridge 예외: %s", e.what());
  }
}

// 두 번째 이미지 콜백 함수
void image2_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  try {
    // ROS 이미지 메시지를 OpenCV 이미지로 변환
    image2 = cv_bridge::toCvCopy(msg, "bgr8")->image;
    // 타임스탬프 저장
    timestamp2 = msg->header.stamp;
    image2_received = true;
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("image_subscriber"), "cv_bridge 예외: %s", e.what());
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("image_subscriber");

  // 첫 번째와 두 번째 이미지 토픽 구독
  auto subscription1 = node->create_subscription<sensor_msgs::msg::Image>(
    "/video1", fusion::best_effort_qos(), image1_callback);

  auto subscription2 = node->create_subscription<sensor_msgs::msg::Image>(
    "/video2", fusion::best_effort_qos(), image2_callback);

  // 병합된 이미지를 퍼블리시할 퍼블리셔 생성
  auto pub = node->create_publisher<sensor_msgs::msg::Image>("correct_image", fusion::best_effort_qos());

  Mat merged;

  while(rclcpp::ok())
  {
    // 두 이미지가 모두 수신되었는지 확인
    if (image1_received && image2_received)
    {
      // 두 이미지를 수직으로 병합
      vconcat(image1, image2, merged);

      // 두 이미지의 타임스탬프가 동일하다고 가정 (이미지 a에서 동일하게 설정)
      rclcpp::Time fused_time = timestamp1; // 또는 timestamp2, 둘이 동일해야 함

      // 새로운 헤더 생성 및 타임스탬프 설정
      std_msgs::msg::Header header;
      header.stamp = fused_time;
      header.frame_id = "merged_frame"; // 선택 사항: 프레임 ID 설정

      // 병합된 OpenCV 이미지를 ROS 이미지 메시지로 변환
      cv_bridge::CvImage cv_image(header, "bgr8", merged);
      
      // `auto` 키워드를 사용하여 올바른 포인터 타입 추론
      auto image_msg = cv_image.toImageMsg();

      // SharedPtr을 역참조하여 publish 함수에 전달
      pub->publish(*image_msg);

      // 다음 이미지를 위해 플래그 초기화
      image1_received = false;
      image2_received = false;
    }

    // 콜백 함수 처리
    rclcpp::spin_some(node);
  }

  rclcpp::shutdown();
  return 0;
}
