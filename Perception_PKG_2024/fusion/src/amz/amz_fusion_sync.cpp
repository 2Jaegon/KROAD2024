#include <rclcpp/rclcpp.hpp>
#include <deque>
#include <memory>
#include <chrono>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "geometry_msgs/msg/pose_array.hpp"

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/image_encodings.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// YOLO headers
#include <std_msgs/msg/int16.hpp>
#include <vision_msgs/msg/bounding_box3_d.hpp>
// 기타 필요한 헤더들...

#include "fusion/qos_utils.hpp"

using namespace std::chrono_literals;

// 클러스터 박스 구조체
struct Box
{
    float x;
    float y;
    float z;

    float size_x;
    float size_y;
    float size_z;
};

// 클러스터 박스의 8개의 꼭지점을 저장하는 구조체
struct Box_points
{
    float x1, y1, z1;
    float x2, y2, z2;
    float x3, y3, z3;
    float x4, y4, z4;
};

// YOLO 박스를 저장하는 구조체
struct Box_yolo
{
    float x1;
    float x2;
    float y1;
    float y2;
    int color;
};

// IoU 계산 함수
float get_iou(const Box_yolo &a, const Box_yolo &b)
{
    float x_left = std::max(a.x1, b.x1);
    float y_top = std::max(a.y1, b.y1);
    float x_right = std::min(a.x2, b.x2);
    float y_bottom = std::min(a.y2, b.y2);

    if (x_right < x_left || y_bottom < y_top)
        return 0.0f;

    float intersection_area = (x_right - x_left) * (y_bottom - y_top);

    float area1 = (a.x2 - a.x1) * (a.y2 - a.y1);
    float area2 = (b.x2 - b.x1) * (b.y2 - b.y1);

    float iou = intersection_area / (area1 + area2 - intersection_area);

    return iou;
}

class ImageLiDARFusion : public rclcpp::Node
{
public:
    // 파라미터 관련 변수
    std::vector<double> CameraExtrinsic_vector_right;
    std::vector<double> CameraExtrinsic_vector_left;
    std::vector<double> CameraMat_vector_right;
    std::vector<double> CameraMat_vector_left;
    std::vector<double> DistCoeff_vector_right;
    std::vector<double> DistCoeff_vector_left;

    cv::Mat transformMat_right;
    cv::Mat transformMat_left;

    ImageLiDARFusion()
        : Node("amz_fusion_sync")
    {
        RCLCPP_INFO(this->get_logger(), "----------- initialize -----------");

        // 파라미터 선언 및 설정
        this->declare_parameter("CameraExtrinsicMat_right", std::vector<double>());
        this->CameraExtrinsic_vector_right = this->get_parameter("CameraExtrinsicMat_right").as_double_array();
        this->declare_parameter("CameraMat_right", std::vector<double>());
        this->CameraMat_vector_right = this->get_parameter("CameraMat_right").as_double_array();
        this->declare_parameter("DistCoeff_right", std::vector<double>());
        this->DistCoeff_vector_right = this->get_parameter("DistCoeff_right").as_double_array();

        this->declare_parameter("CameraExtrinsicMat_left", std::vector<double>());
        this->CameraExtrinsic_vector_left = this->get_parameter("CameraExtrinsicMat_left").as_double_array();
        this->declare_parameter("CameraMat_left", std::vector<double>());
        this->CameraMat_vector_left = this->get_parameter("CameraMat_left").as_double_array();
        this->declare_parameter("DistCoeff_left", std::vector<double>());
        this->DistCoeff_vector_left = this->get_parameter("DistCoeff_left").as_double_array();

        set_param();

        // 구독자 설정
        LiDAR_sub_ = this->create_subscription<vision_msgs::msg::Detection3DArray>(
            "/lidar_bbox", fusion::best_effort_qos(),
            std::bind(&ImageLiDARFusion::lidarCallback, this, std::placeholders::_1));

        yolo_detect_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
            "/yolo_detect", fusion::best_effort_qos(),
            std::bind(&ImageLiDARFusion::yoloCallback, this, std::placeholders::_1));

        // 퍼블리셔 설정
        LiDAR_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("test_LiDAR", fusion::best_effort_qos());
        cone_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/LiDAR/center_color", fusion::best_effort_qos());
        pose_array_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/LiDAR/center_color2", fusion::best_effort_qos()); // 추가된 퍼블리셔

        // 타이머 콜백 설정 (예: 20Hz)
        auto timer_callback = [this]() -> void
        { FusionCallback(); };
        timer_ = create_wall_timer(50ms, timer_callback); // 20Hz

        RCLCPP_INFO(this->get_logger(), "------------ initialize end------------");
    }

    ~ImageLiDARFusion()
    {
        RCLCPP_INFO(this->get_logger(), "ImageLiDARFusion Node has been terminated");
    }

private:
    // 메시지 큐
    std::deque<vision_msgs::msg::Detection3DArray::SharedPtr> lidar_queue_;
    std::deque<vision_msgs::msg::Detection2DArray::SharedPtr> yolo_queue_;

    // 큐 관리 관련 상수
    const double MAX_TIME_DIFF_MS = 45.0; // 최대 시간 차이 30ms
    const size_t QUEUE_MAX_SIZE = 100;    // 큐의 최대 크기 (1초 동안 100Hz 가정)

    // 퍼블리셔 및 서브스크라이버
    rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr LiDAR_sub_;
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr yolo_detect_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr LiDAR_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cone_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_pub_; // 추가된 퍼블리셔
    rclcpp::TimerBase::SharedPtr timer_;

    // 파라미터 설정 함수
    void set_param()
    {
        // 받아온 파라미터
        cv::Mat CameraExtrinsicMat_right(3, 4, CV_64F, CameraExtrinsic_vector_right.data());
        cv::Mat CameraExtrinsicMat_left(3, 4, CV_64F, CameraExtrinsic_vector_left.data());
        cv::Mat CameraMat_right(3, 3, CV_64F, CameraMat_vector_right.data());
        cv::Mat CameraMat_left(3, 3, CV_64F, CameraMat_vector_left.data());
        cv::Mat DistCoeffMat_right(1, 4, CV_64F, DistCoeff_vector_right.data());
        cv::Mat DistCoeffMat_left(1, 4, CV_64F, DistCoeff_vector_left.data());

        // 재가공 : transformMat
        // 3x4 행렬을 기대하므로 CameraExtrinsicMat_right의 상위 3x4를 사용
        cv::Mat CameraExtrinsicMat_right_3x4 = CameraExtrinsicMat_right(cv::Rect(0, 0, 4, 3));
        this->transformMat_right = CameraMat_right * CameraExtrinsicMat_right_3x4;
        this->transformMat_left = CameraMat_left * CameraExtrinsicMat_left;

        std::cout << "transformMat_right : " << this->transformMat_right << "\n"
                  << "transformMat_left : " << this->transformMat_left << "\n";
        RCLCPP_INFO(this->get_logger(), "parameter set end");
    }

    // LiDAR 메시지 콜백
    void lidarCallback(const vision_msgs::msg::Detection3DArray::SharedPtr msg)
    {
        // 큐에 메시지 추가
        lidar_queue_.push_back(msg);

        // 큐 크기 제한
        if (lidar_queue_.size() > QUEUE_MAX_SIZE)
        {
            lidar_queue_.pop_front();
        }
    }

    // YOLO 메시지 콜백
    void yoloCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
    {
        // 큐에 메시지 추가
        yolo_queue_.push_back(msg);

        // 큐 크기 제한
        if (yolo_queue_.size() > QUEUE_MAX_SIZE)
        {
            yolo_queue_.pop_front();
        }
    }

    // 클러스터 박스의 8개의 꼭지점을 구하는 함수
    Box_points calcBox_points(const geometry_msgs::msg::Point &center, const geometry_msgs::msg::Vector3 &size)
    {
        Box_points vertices;
        if (center.y >= 0)
        {
            vertices.x1 = center.x - (size.x) / 2;
            vertices.y1 = center.y + (size.y) / 2;
            vertices.z1 = center.z - (size.z) / 2;

            vertices.x2 = center.x + (size.x) / 2;
            vertices.y2 = center.y - (size.y) / 2;
            vertices.z2 = center.z - (size.z) / 2;

            vertices.x3 = center.x + (size.x) / 2;
            vertices.y3 = center.y - (size.y) / 2;
            vertices.z3 = center.z + (size.z) / 2;

            vertices.x4 = center.x - (size.x) / 2;
            vertices.y4 = center.y + (size.y) / 2;
            vertices.z4 = center.z + (size.z) / 2;
        }
        else
        {
            vertices.x1 = center.x + (size.x) / 2;
            vertices.y1 = center.y + (size.y) / 2;
            vertices.z1 = center.z - (size.z) / 2;

            vertices.x2 = center.x - (size.x) / 2;
            vertices.y2 = center.y - (size.y) / 2;
            vertices.z2 = center.z - (size.z) / 2;

            vertices.x3 = center.x - (size.x) / 2;
            vertices.y3 = center.y - (size.y) / 2;
            vertices.z3 = center.z + (size.z) / 2;

            vertices.x4 = center.x + (size.x) / 2;
            vertices.y4 = center.y + (size.y) / 2;
            vertices.z4 = center.z + (size.z) / 2;
        }
        return vertices;
    }

    // 퓨전 콜백
    void FusionCallback()
    {
        // 큐가 비어있지 않은지 확인
        if (lidar_queue_.empty() || yolo_queue_.empty())
        {
            RCLCPP_WARN(this->get_logger(), "큐 사이즈가 비었습니다! 라이다: %d, 욜로: %d",
                        lidar_queue_.empty(), yolo_queue_.empty());
            return;
        }

        // 두 큐의 시작 인덱스
        size_t lidar_idx = 0;
        size_t yolo_idx = 0;

        // 최적의 매칭을 찾기 위한 변수
        double best_time_diff = MAX_TIME_DIFF_MS + 1.0;
        size_t best_lidar_idx = 0;
        size_t best_yolo_idx = 0;

        // 두 큐를 순회하며 가장 가까운 매칭 찾기
        while (lidar_idx < lidar_queue_.size() && yolo_idx < yolo_queue_.size())
        {
            auto lidar_msg = lidar_queue_[lidar_idx];
            auto yolo_msg = yolo_queue_[yolo_idx];

            rclcpp::Time lidar_time = lidar_msg->header.stamp;
            rclcpp::Time yolo_time = yolo_msg->header.stamp;

            double time_diff_ms = (lidar_time - yolo_time).seconds() * 1000.0;

            if (std::abs(time_diff_ms) < std::abs(best_time_diff))
            {
                best_time_diff = time_diff_ms;
                best_lidar_idx = lidar_idx;
                best_yolo_idx = yolo_idx;
            }

            // 두 메시지의 시간 순서에 따라 인덱스 증가
            if (lidar_time < yolo_time)
            {
                lidar_idx++;
            }
            else
            {
                yolo_idx++;
            }
        }

        // 가장 작은 시간 차이가 MAX_TIME_DIFF_MS 이내인지 확인
        if (std::abs(best_time_diff) <= MAX_TIME_DIFF_MS)
        {
            RCLCPP_INFO(this->get_logger(), "큐 매치 : %.2f ms", best_time_diff);
            auto lidar_msg = lidar_queue_[best_lidar_idx];
            auto yolo_msg = yolo_queue_[best_yolo_idx];

            // 타임스탬프 일치
            rclcpp::Time fused_stamp = lidar_msg->header.stamp;

            // 포인트클라우드 생성
            sensor_msgs::msg::PointCloud2 pointcloud_msg;

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr coord_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

            // LiDAR 박스 처리 (필터링 및 포인트클라우드 생성)
            std::vector<Box> lidar_boxes;
            for (const auto &detection : lidar_msg->detections)
            {
                // 박스 크기 필터링
                if (detection.bbox.size.y > 0.1 && detection.bbox.size.y < 0.4 && detection.bbox.size.z < 1)
                {
                    Box box = {
                        detection.bbox.center.position.x,
                        detection.bbox.center.position.y,
                        detection.bbox.center.position.z,
                        detection.bbox.size.x,
                        detection.bbox.size.y * 1.2f,
                        detection.bbox.size.z * 1.5f};
                    lidar_boxes.push_back(box);
                }
            }

            // YOLO 박스 처리
            std::vector<Box_yolo> yolo_boxes;
            for (const auto &detection : yolo_msg->detections)
            {
                if (detection.results.empty())
                    continue;

                std::string class_name = detection.results[0].hypothesis.class_id;
                int color = 0;
                if (class_name == "blue")
                    color = 1;
                else if (class_name == "orange")
                    color = 2;

                // Compute 2D bounding box
                float yolo_x1 = detection.bbox.center.position.x - (detection.bbox.size_x * 1.2f / 2.0f);
                float yolo_x2 = detection.bbox.center.position.x + (detection.bbox.size_x * 1.2f / 2.0f);
                float yolo_y1 = detection.bbox.center.position.y - (detection.bbox.size_y / 2.0f);
                float yolo_y2 = detection.bbox.center.position.y + (detection.bbox.size_y / 2.0f);

                Box_yolo box_yolo = {yolo_x1, yolo_x2, yolo_y1, yolo_y2, color};
                yolo_boxes.push_back(box_yolo);
            }

            // IoU 매칭 및 클래스 할당
            std_msgs::msg::Float64MultiArray cone_msg;
            geometry_msgs::msg::PoseArray pose_array_msg; // PoseArray 메시지 생성
            pose_array_msg.header.stamp = fused_stamp;    // 타임스탬프 설정
            pose_array_msg.header.frame_id = "velodyne";  // 프레임 ID 설정 (필요에 따라 변경 가능)

            for (size_t i = 0; i < lidar_boxes.size(); ++i)
            {
                const auto &lidar_box = lidar_boxes[i];

                // center Point 및 size Vector3 생성
                geometry_msgs::msg::Point center;
                center.x = lidar_box.x;
                center.y = lidar_box.y;
                center.z = lidar_box.z;

                geometry_msgs::msg::Vector3 size;
                size.x = lidar_box.size_x;
                size.y = lidar_box.size_y;
                size.z = lidar_box.size_z;

                // Box_points 계산
                Box_points vertices = calcBox_points(center, size);

                // 투영 및 2D bounding box 생성
                Box_yolo projected_box;
                if (lidar_box.y >= 0)
                {
                    // Left camera
                    double box_1[4] = {vertices.x1, vertices.y1, vertices.z1, 1.0};
                    double box_2[4] = {vertices.x2, vertices.y2, vertices.z2, 1.0};
                    double box_3[4] = {vertices.x3, vertices.y3, vertices.z3, 1.0};
                    double box_4[4] = {vertices.x4, vertices.y4, vertices.z4, 1.0};

                    cv::Mat pos1(4, 1, CV_64F, box_1);
                    cv::Mat pos2(4, 1, CV_64F, box_2);
                    cv::Mat pos3(4, 1, CV_64F, box_3);
                    cv::Mat pos4(4, 1, CV_64F, box_4);

                    cv::Mat newpos1 = this->transformMat_left * pos1;
                    cv::Mat newpos2 = this->transformMat_left * pos2;
                    cv::Mat newpos3 = this->transformMat_left * pos3;
                    cv::Mat newpos4 = this->transformMat_left * pos4;

                    // Compute image coordinates
                    float x1_img = static_cast<float>(newpos1.at<double>(0, 0) / newpos1.at<double>(2, 0));
                    float y1_img = static_cast<float>(newpos1.at<double>(1, 0) / newpos1.at<double>(2, 0));

                    float x2_img = static_cast<float>(newpos2.at<double>(0, 0) / newpos2.at<double>(2, 0));
                    float y2_img = static_cast<float>(newpos2.at<double>(1, 0) / newpos2.at<double>(2, 0));

                    float x3_img = static_cast<float>(newpos3.at<double>(0, 0) / newpos3.at<double>(2, 0));
                    float y3_img = static_cast<float>(newpos3.at<double>(1, 0) / newpos3.at<double>(2, 0));

                    float x4_img = static_cast<float>(newpos4.at<double>(0, 0) / newpos4.at<double>(2, 0));
                    float y4_img = static_cast<float>(newpos4.at<double>(1, 0) / newpos4.at<double>(2, 0));

                    // 2D Bounding Box 계산
                    projected_box = {
                        std::min({x1_img, x2_img, x3_img, x4_img}),
                        std::max({x1_img, x2_img, x3_img, x4_img}),
                        std::min({y1_img, y2_img, y3_img, y4_img}),
                        std::max({y1_img, y2_img, y3_img, y4_img}),
                        0 // placeholder, will be set based on IoU
                    };
                }
                else
                {
                    // Right camera
                    double box_1[4] = {vertices.x1, vertices.y1, vertices.z1, 1.0};
                    double box_2[4] = {vertices.x2, vertices.y2, vertices.z2, 1.0};
                    double box_3[4] = {vertices.x3, vertices.y3, vertices.z3, 1.0};
                    double box_4[4] = {vertices.x4, vertices.y4, vertices.z4, 1.0};

                    cv::Mat pos1(4, 1, CV_64F, box_1);
                    cv::Mat pos2(4, 1, CV_64F, box_2);
                    cv::Mat pos3(4, 1, CV_64F, box_3);
                    cv::Mat pos4(4, 1, CV_64F, box_4);

                    cv::Mat newpos1 = this->transformMat_right * pos1;
                    cv::Mat newpos2 = this->transformMat_right * pos2;
                    cv::Mat newpos3 = this->transformMat_right * pos3;
                    cv::Mat newpos4 = this->transformMat_right * pos4;

                    // Compute image coordinates
                    float x1_img = static_cast<float>(newpos1.at<double>(0, 0) / newpos1.at<double>(2, 0));
                    float y1_img = static_cast<float>(newpos1.at<double>(1, 0) / newpos1.at<double>(2, 0) + 480.0f);

                    float x2_img = static_cast<float>(newpos2.at<double>(0, 0) / newpos2.at<double>(2, 0));
                    float y2_img = static_cast<float>(newpos2.at<double>(1, 0) / newpos2.at<double>(2, 0) + 480.0f);

                    float x3_img = static_cast<float>(newpos3.at<double>(0, 0) / newpos3.at<double>(2, 0));
                    float y3_img = static_cast<float>(newpos3.at<double>(1, 0) / newpos3.at<double>(2, 0) + 480.0f);

                    float x4_img = static_cast<float>(newpos4.at<double>(0, 0) / newpos4.at<double>(2, 0));
                    float y4_img = static_cast<float>(newpos4.at<double>(1, 0) / newpos4.at<double>(2, 0) + 480.0f);

                    // 2D Bounding Box 계산
                    projected_box = {
                        std::min({x1_img, x2_img, x3_img, x4_img}),
                        std::max({x1_img, x2_img, x3_img, x4_img}),
                        std::min({y1_img, y2_img, y3_img, y4_img}),
                        std::max({y1_img, y2_img, y3_img, y4_img}),
                        0 // placeholder, will be set based on IoU
                    };
                }

                // IoU 매칭
                float max_iou = 0.0f;
                int matched_class_id = 0;

                for (const auto &yolo_box : yolo_boxes)
                {
                    float iou = get_iou(projected_box, yolo_box);
                    if (iou > max_iou)
                    {
                        max_iou = iou;
                        matched_class_id = yolo_box.color;
                    }
                }

                if (max_iou > 0.2f)
                {
                    // YOLO 매칭된 클래스 ID 할당
                    cone_msg.data.push_back(static_cast<double>(lidar_box.x));
                    cone_msg.data.push_back(static_cast<double>(lidar_box.y));
                    cone_msg.data.push_back(static_cast<double>(matched_class_id));

                    // PoseArray에 포즈 추가
                    geometry_msgs::msg::Pose pose;
                    pose.position.x = lidar_box.x;
                    pose.position.y = lidar_box.y;
                    pose.position.z = static_cast<float>(matched_class_id);
                    // Orientation은 기본값 (identity quaternion)
                    pose.orientation.x = 0.0f;
                    pose.orientation.y = 0.0f;
                    pose.orientation.z = 0.0f;
                    pose.orientation.w = 1.0f;
                    pose_array_msg.poses.push_back(pose);

                    pcl::PointXYZRGB pointRGB;
                    pointRGB.x = lidar_box.x;
                    pointRGB.y = lidar_box.y;
                    pointRGB.z = lidar_box.z;

                    if (matched_class_id == 1) // blue
                    {
                        pointRGB.r = 0;
                        pointRGB.g = 0;
                        pointRGB.b = 255;
                    }
                    else if (matched_class_id == 2) // orange
                    {
                        pointRGB.r = 255;
                        pointRGB.g = 255; // 더 정확한 오렌지 색상
                        pointRGB.b = 0;
                    }
                    coord_cloud->push_back(pointRGB);
                }
                else
                { // 카메라 못봐도 강제로 주변에 할당
                    if (lidar_box.y > -1.51 && lidar_box.y < 1.51 && lidar_box.x > -1 && lidar_box.x < 0.8)
                    {
                        // 매칭되지 않은 경우 위치 기반 클래스 할당 (예: 좌우 카메라 기준)
                        int class_id = (lidar_box.y >= 0) ? 2 : 1; // 1: blue, 2: orange

                        cone_msg.data.push_back(static_cast<double>(lidar_box.x));
                        cone_msg.data.push_back(static_cast<double>(lidar_box.y));
                        cone_msg.data.push_back(static_cast<double>(class_id));

                        // PoseArray에 포즈 추가
                        geometry_msgs::msg::Pose pose;
                        pose.position.x = lidar_box.x;
                        pose.position.y = lidar_box.y;
                        pose.position.z = static_cast<float>(class_id);
                        // Orientation은 기본값 (identity quaternion)
                        pose.orientation.x = 0.0f;
                        pose.orientation.y = 0.0f;
                        pose.orientation.z = 0.0f;
                        pose.orientation.w = 1.0f;
                        pose_array_msg.poses.push_back(pose);

                        pcl::PointXYZRGB pointRGB;
                        pointRGB.x = lidar_box.x;
                        pointRGB.y = lidar_box.y;
                        pointRGB.z = lidar_box.z;

                        if (class_id == 1) // blue
                        {
                            pointRGB.r = 55;
                            pointRGB.g = 155;
                            pointRGB.b = 255;
                        }
                        else if (class_id == 2) // orange
                        {
                            pointRGB.r = 255;
                            pointRGB.g = 165; // 더 정확한 오렌지 색상
                            pointRGB.b = 55;
                        }
                        coord_cloud->push_back(pointRGB);
                    }
                    else
                    {
                        // 카메라 못보고 멀리있는 건 모르겠다고 pub
                        cone_msg.data.push_back(static_cast<double>(lidar_box.x));
                        cone_msg.data.push_back(static_cast<double>(lidar_box.y));
                        cone_msg.data.push_back(static_cast<double>(0));

                        // PoseArray에 포즈 추가
                        geometry_msgs::msg::Pose pose;
                        pose.position.x = lidar_box.x;
                        pose.position.y = lidar_box.y;
                        pose.position.z = 0.0f; // class_id가 없음을 의미
                        // Orientation은 기본값 (identity quaternion)
                        pose.orientation.x = 0.0f;
                        pose.orientation.y = 0.0f;
                        pose.orientation.z = 0.0f;
                        pose.orientation.w = 1.0f;
                        pose_array_msg.poses.push_back(pose);

                        pcl::PointXYZRGB pointRGB;
                        pointRGB.x = lidar_box.x;
                        pointRGB.y = lidar_box.y;
                        pointRGB.z = lidar_box.z;
                        pointRGB.r = 55;
                        pointRGB.g = 255;
                        pointRGB.b = 55;
                        coord_cloud->push_back(pointRGB);
                    }
                }
            }

            // 퍼블리시
            // 포인트 클라우드 퍼블리시
            coord_cloud->width = coord_cloud->points.size();
            coord_cloud->height = 1;
            pcl::toROSMsg(*coord_cloud, pointcloud_msg);
            pointcloud_msg.header.frame_id = "velodyne";
            pointcloud_msg.header.stamp = fused_stamp;
            LiDAR_pub_->publish(pointcloud_msg);

            // cone_msg 퍼블리시
            cone_pub_->publish(cone_msg);

            // PoseArray 퍼블리시
            pose_array_pub_->publish(pose_array_msg);

            // 큐 정리 (동기화된 메시지 제거)
            lidar_queue_.erase(lidar_queue_.begin(), lidar_queue_.begin() + best_lidar_idx + 1);
            yolo_queue_.erase(yolo_queue_.begin(), yolo_queue_.begin() + best_yolo_idx + 1);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "큐 논 매치 : %.2f ms", best_time_diff);
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageLiDARFusion>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
