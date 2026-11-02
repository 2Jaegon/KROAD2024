// 기본
#include <rclcpp/rclcpp.hpp>

// 자료형
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>

// pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/surface/mls.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/prosac.h>
#include <pcl/sample_consensus/sac_model_line.h>

// 터널 끝점 검출
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>

// 시각화
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// 계산
#include <cmath>
#include <vector>

#include <pcl/common/transforms.h> // 길요한
#include <pcl/common/common.h>     // 길요한

#include "lidar_lane/qos_utils.hpp"

using PointT = pcl::PointXYZI;
using namespace std;

class LiDAR_lane : public rclcpp::Node
{
public:
    LiDAR_lane() : Node("LiDAR_lane")
    {
        mission_flag_sub_ = this->create_subscription<std_msgs::msg::Int16>(
            "/Planning/mission", lidar::best_effort_qos(),
            [this](const std_msgs::msg::Int16::SharedPtr msg) -> void
            {
                mission_flag_cb(msg);
            });

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),

            std::bind(&LiDAR_lane::timer_callback, this)); // 1초마다 timer_callback 함수 호출

        lane_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/Marker/center_lane", lidar::best_effort_qos());
        lane_dot_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/Marker/lane_dot", lidar::best_effort_qos());
        coord_dot_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/Marker/ERP_center_dot", lidar::best_effort_qos());
        crop_box_ = this->create_publisher<visualization_msgs::msg::Marker>("/Marker/range_tunnel_end", lidar::best_effort_qos());
        lane_dot_MA = this->create_publisher<std_msgs::msg::Float64MultiArray>("/LiDAR/lane", lidar::best_effort_qos());
        center_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/LiDAR/center_lane", lidar::best_effort_qos());
        wall_distance = this->create_publisher<std_msgs::msg::Float64MultiArray>("/LiDAR/wall_dist", lidar::best_effort_qos());
        ceiling_end_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/LiDAR/ceiling_end", lidar::best_effort_qos());
        pub_cluster = this->create_publisher<sensor_msgs::msg::PointCloud2>("/Cluster/tunnel_line", lidar::best_effort_qos());
        pub_ceiling = this->create_publisher<sensor_msgs::msg::PointCloud2>("/Cluster/tunnel_ceiling", lidar::best_effort_qos());
        wall_cluster = this->create_publisher<sensor_msgs::msg::PointCloud2>("/Cluster/tunnel_wall", lidar::best_effort_qos());
        zone_center = this->create_publisher<visualization_msgs::msg::Marker>("/Marker/orange_lane_zone", lidar::best_effort_qos());
        tunnel_end_ = this->create_publisher<std_msgs::msg::Bool>("/LiDAR/tunnel_end", lidar::best_effort_qos());
        tunnel_center_roi_ = this->create_publisher<visualization_msgs::msg::Marker>("/Marker/center_roi", lidar::best_effort_qos());

        if (tunnel_end == 1)
        {
            lidar_2d_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "scan", rclcpp::SensorDataQoS(), std::bind(&LiDAR_lane::lidar_cb, this, std::placeholders::_1));
        }
        else if (tunnel_end == 2 || tunnel_end == 5)
        {
            image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                "/flir_camera/image_raw", 10, std::bind(&LiDAR_lane::image_cb, this, std::placeholders::_1));
        }

        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "velodyne_points", 1, std::bind(&LiDAR_lane::cloud_cb, this, std::placeholders::_1));

        // RCLCPP_INFO(this->get_logger(), "LiDAR_lane node has started.");
    }

private:
    int mission_flag = 0;
    float lane_intensity_threshold = 70.0;
    bool ceiling_nan = false;

    float ceiling_end_check = 0; // 천장 끝점 검출 확인하는 변수

    int mission_check = -1; // 미션 들어오는 확인하는 변수
    int mission_count = 0;  // 몇 번 동안 띄울지 결정하는 함수

    int tunnel_end = 6;
    /*
    1 : 2d lidar를 활용한 터널 끝점 검출
    2 : 카메라를 활용한 터널 끝점 검출
    3 : 3d lidar로 사이드 조사
    4 : 3d lidar로 시간 조사
    5 : 카메라를 활용한 터널 끝점 검출
    */

    int chk = 0;
    /*
    0 : 현재 터널 밖
    1 : 현재 터널 안
    2 : 현재 터널 끝
    */

    std::chrono::steady_clock::time_point start_time_;
    int tunnel_end_time = 0;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cluster;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ceiling;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr lane_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr lane_dot_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr coord_dot_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr lane_dot_MA;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr center_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wall_distance;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr wall_cluster;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr zone_center;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr crop_box_;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr mission_flag_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_2d_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr tunnel_end_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr tunnel_center_roi_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ceiling_end_;

    // rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ceiling_end_;

    rclcpp::TimerBase::SharedPtr timer_;

    void timer_callback()
    {
        if (mission_check == -1)
        {
            mission_count = 0;
            RCLCPP_INFO(this->get_logger(), "\n 🚨🚨 터널 아닌가 ? 터널 아닌가 ? 🚨🚨");
        }
        else if (mission_check > 0 && mission_count < 1)
        {
            RCLCPP_INFO(this->get_logger(), "\n 터널 안 인가 ? 🥇 터널 안 인가 ? 🥇");
            mission_count = 1;
        }
        mission_check = -1;
    }

    void mission_flag_cb(const std_msgs::msg::Int16::SharedPtr msg)
    {
        this->mission_flag = msg->data;
        if (mission_flag == 15 || mission_flag == 17)
        {
            mission_check = 1;
        }
    }

    void cloud_cb(const sensor_msgs::msg::PointCloud2::SharedPtr input)
    {
        if (mission_flag == 15)
        {
            // sensor_msgs/PointCloud2 data 를 pcl/PointCloud로 변환
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::fromROSMsg(*input, *cloud);

            /////////////////////////////////<벽 보는 코드>////////////////////////////////////////////////////////////////////
            // VoxelGrid filter를 이용하여 다운샘플링
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZI>());
            down_sampling(cloud, cloud_downsampled);

            // crop the wall
            pcl::PointCloud<pcl::PointXYZI>::Ptr wall_filtered(new pcl::PointCloud<pcl::PointXYZI>);
            seethewall(cloud_downsampled, wall_filtered);

            // 벽을 정사영하여 x,y만 남김
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZI>());
            project_to_2d_plane(wall_filtered, cloud_projected);

            pcl::PointCloud<pcl::PointXYZI>::Ptr largest_clusters_cloud(new pcl::PointCloud<pcl::PointXYZI>());
            find_two_largest_clusters(cloud_projected, largest_clusters_cloud);

            // 클러스터를 이용하여 벽의 벡터를 RANSAC으로 생성 //벽까지 거리 보냄 (왼xmin,왼y1,왼xmax,왼y2,오xmin,오y1,오xmax,오y2)
            vector_wall(largest_clusters_cloud);
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            ////////////////////////////////<중앙 차선 코드>////////////////////////////////////////////////////////////////////
            // crop the ground
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointXYZI>);
            cropGround(cloud, cloud_filtered2);

            // Detect lanes using intensity values
            pcl::PointCloud<pcl::PointXYZI>::Ptr lanes_cloud(new pcl::PointCloud<pcl::PointXYZI>());
            extractLanesByIntensity(cloud_filtered2, lanes_cloud, lane_intensity_threshold);

            clusterLanes(lanes_cloud, 0.2, 3, 50); // 여기 바꿔
            //////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            ///////////////////////////////<천장 보는 코드>//////////////////////////////////////////////////////////////////////
            pcl::PointCloud<pcl::PointXYZI>::Ptr ceiling_filtered(new pcl::PointCloud<pcl::PointXYZI>);
            I_want_know_tunnel_end(cloud, ceiling_filtered);

            if (ceiling_nan)
            {
                pcl::PointCloud<pcl::PointXYZI>::Ptr ceiling_projected(new pcl::PointCloud<pcl::PointXYZI>);
                clustering_tunnel_ceiling(ceiling_filtered, ceiling_projected);
            }

            if (tunnel_end == 1)
            {
                RCLCPP_INFO(this->get_logger(), "2D 라이다를 이용한 터널 끝 감지 로직 실행 중");
            }
            else if (tunnel_end == 2)
            {
                RCLCPP_INFO(this->get_logger(), "카메라(hsv)를 이용한 터널 끝 감지 로직 실행 중");
            }
            else if (tunnel_end == 3)
            {
                pcl::PointCloud<pcl::PointXYZI>::Ptr ceiling_filtered(new pcl::PointCloud<pcl::PointXYZI>);
                lidar_delete_z(cloud, ceiling_filtered);
                pcl::PointCloud<pcl::PointXYZI>::Ptr ceiling_projected(new pcl::PointCloud<pcl::PointXYZI>);
                search_tunnel_end(ceiling_filtered, ceiling_projected);
            }

            else if (tunnel_end == 4)
            {
                if (tunnel_end_time == 0)
                {
                    start_time_ = std::chrono::steady_clock::now();
                    tunnel_end_time = 1;
                }
                if (ceiling_end_check < 19 && tunnel_end_time == 1)
                {
                    auto current_time = std::chrono::steady_clock::now();
                    auto duration = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time_).count();
                    if (duration >= 7.5)
                    {
                        auto msg = std_msgs::msg::Bool();
                        msg.data = true;
                        tunnel_end_->publish(msg);
                        RCLCPP_INFO(this->get_logger(), "Published true after 7 seconds.");

                        // 시간을 다시 초기화하여 반복적으로 7초 지연 후 publish
                        // start_time_ = std::chrono::steady_clock::now();
                    }
                }
                else if (ceiling_end_check > 19 && tunnel_end_time == 1)
                {
                    tunnel_end_time = 0;
                }
                else
                {
                    auto msg = std_msgs::msg::Bool();
                    msg.data = false;
                    tunnel_end_->publish(msg);
                }
            }
            else if (tunnel_end == 5)
            {
                RCLCPP_INFO(this->get_logger(), "카메라(명암)를 이용한 터널 끝 감지 로직 실행 중");
            }
            else if (tunnel_end == 6)
            {
                // std::cout << "tunnel_end_check : " << "ceiling_end_check" << std::endl;
                pcl::PointCloud<pcl::PointXYZI>::Ptr tunnel_end_cloud(new pcl::PointCloud<pcl::PointXYZI>);
                crop_tunnel_end_cloud(cloud, tunnel_end_cloud);
            }
            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            // std::cout << "tunnel_end_check : " << ceiling_end_check << std::endl;
            // 정사영 된 포인트 클라우드를 출력으로 바꾸어 publish
            sensor_msgs::msg::PointCloud2 output_wall;
            pcl::toROSMsg(*largest_clusters_cloud, output_wall);
            output_wall.header = input->header;
            wall_cluster->publish(output_wall);

            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "velodyne";
            marker.ns = "my_roi";
            marker.id = 0;
            marker.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.scale.x = 8;
            marker.scale.y = 4;
            marker.scale.z = 1;
            marker.pose.position.x = 4;
            marker.pose.position.y = 2;
            marker.pose.position.z = -0.5;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 0.3;
            zone_center->publish(marker);
        }
    }

    void down_sampling(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_downsampled)
    {
        pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
        voxel_grid.setInputCloud(cloud);
        voxel_grid.setLeafSize(0.1f, 0.1f, 0.1f);
        voxel_grid.filter(*cloud_downsampled);
    }

    void crop_tunnel_end_cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr tunnel_end_cloud)
    {
        // std::cout << "tunnel_end_cloud" << std::endl;
        // Remove points close to the ground plane
        pcl::CropBox<pcl::PointXYZI> cropFilter;
        cropFilter.setInputCloud(cloud);

        cropFilter.setMin(Eigen::Vector4f(-2, -20, 2, 1)); // 여기 바꿔
        cropFilter.setMax(Eigen::Vector4f(0, 20, 3, 1));

        cropFilter.filter(*tunnel_end_cloud); // 여기에 저장

        std::cout << tunnel_end_cloud->points.size() << std::endl;

        auto msg = std_msgs::msg::Bool();
        msg.data = false;
        if (tunnel_end_cloud->points.size() >= 100)
        {
            chk = 1;
        }
        else if (tunnel_end_cloud->points.size() < 45 && chk == 1)
        {
            msg.data = true;
            chk = 2;
        }
        tunnel_end_->publish(msg);

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "velodyne";
        marker.ns = "my_roi";
        marker.id = 0;
        marker.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.scale.x = cropFilter.getMax()[0] - cropFilter.getMin()[0];
        marker.scale.y = cropFilter.getMax()[1] - cropFilter.getMin()[1];
        marker.scale.z = cropFilter.getMax()[2] - cropFilter.getMin()[2];
        marker.pose.position.x = (cropFilter.getMax()[0] + cropFilter.getMin()[0]) / 2;
        marker.pose.position.y = (cropFilter.getMax()[1] + cropFilter.getMin()[1]) / 2;
        marker.pose.position.z = (cropFilter.getMax()[2] + cropFilter.getMin()[2]) / 2;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 0.3;
        crop_box_->publish(marker);
    }

    void cropGround(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered2)
    {

        // Remove points close to the ground plane
        pcl::CropBox<pcl::PointXYZI> cropFilter;
        cropFilter.setInputCloud(cloud);
        if (chk != 2)
        {
            cropFilter.setMin(Eigen::Vector4f(-0, -10, -1.7, 1)); // 여기 바꿔
            cropFilter.setMax(Eigen::Vector4f(8, 10, -0.45, 1));
        }
        else
        {
            cropFilter.setMin(Eigen::Vector4f(-0, -3, -1.7, 1)); // 여기 바꿔
            cropFilter.setMax(Eigen::Vector4f(8, 3, -0.45, 1));
        }

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "velodyne";
        marker.ns = "my_roi";
        marker.id = 0;
        marker.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.scale.x = cropFilter.getMax()[0] - cropFilter.getMin()[0];
        marker.scale.y = cropFilter.getMax()[1] - cropFilter.getMin()[1];
        marker.scale.z = cropFilter.getMax()[2] - cropFilter.getMin()[2];
        marker.pose.position.x = (cropFilter.getMax()[0] + cropFilter.getMin()[0]) / 2;
        marker.pose.position.y = (cropFilter.getMax()[1] + cropFilter.getMin()[1]) / 2;
        marker.pose.position.z = (cropFilter.getMax()[2] + cropFilter.getMin()[2]) / 2;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 0.2;
        tunnel_center_roi_->publish(marker);

        cropFilter.filter(*cloud_filtered2); // 여기에 저장
    }

    void seethewall(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr wall_filtered)
    {
        // Remove points close to the ground plane
        pcl::CropBox<pcl::PointXYZI> cropFilter;
        cropFilter.setInputCloud(cloud);

        cropFilter.setMin(Eigen::Vector4f(-20, -20, 0.8, 0)); // x, y, z, min (m)
        // cropFilter.setMax(Eigen::Vector4f(100, 20, 2.0, 0));  // sanhak
        cropFilter.setMax(Eigen::Vector4f(100, 20, 3.0, 0)); // kcity

        cropFilter.filter(*wall_filtered);
    }

    // 2d lidar로 0도 추출
    void lidar_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // 각도가 0인 인덱스를 계산
        int zero_angle_index = static_cast<int>((0.0 - msg->angle_min) / msg->angle_increment);

        std::cout << "2d lidar" << std::endl;

        // 인덱스가 유효한지 확인
        if (zero_angle_index >= 0 && zero_angle_index < msg->ranges.size())
        {
            // 각도 0도에서의 거리 값 추출
            float distance_at_zero_angle = msg->ranges[zero_angle_index];
            RCLCPP_INFO(this->get_logger(), "Distance at 0 degrees: %f", distance_at_zero_angle);

            // 터널 감지 로직
            if (distance_at_zero_angle < 20 && chk == 0)
            {
                chk = 1; // 터널 안에 있음
                RCLCPP_INFO(this->get_logger(), "터널 시작 감지");
            }
            else if (distance_at_zero_angle > 20 && chk == 1)
            {
                chk = 2; // 터널 끝 근접

                // Bool 메시지 생성 및 퍼블리시
                std_msgs::msg::Bool msg;
                msg.data = true;
                tunnel_end_->publish(msg);

                RCLCPP_INFO(this->get_logger(), "터널 끝");
            }
            else if (distance_at_zero_angle < 20 && chk == 2)
            {
                RCLCPP_INFO(this->get_logger(), "장애물 보는중............");
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Index out of range for 0 degrees.");
        }
    }

    void image_cb(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (tunnel_end = 2)
        {
            // OpenCV를 사용하여 sensor_msgs::Image 메시지를 cv::Mat 형식으로 변환
            cv_bridge::CvImagePtr cv_ptr;
            try
            {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            }
            catch (cv_bridge::Exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
                return;
            }

            // 이미지를 HSV 색상 공간으로 변환
            cv::Mat hsv_image;
            cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);

            // HSV 이미지에서 특정 색상 범위를 찾기 위한 코드
            // 초록색 범위 설정 (H: 40~80, S: 100~255, V: 50~255)
            cv::Mat mask;
            cv::inRange(hsv_image, cv::Scalar(40, 100, 50), cv::Scalar(80, 255, 255), mask); // 초록색
            // cv::inRange(hsv_image, cv::Scalar(90, 100, 100), cv::Scalar(120, 255, 255), mask); // 하늘색
            // cv::inRange(hsv_image, cv::Scalar(20, 100, 100), cv::Scalar(30, 255, 255), mask); // 노란색

            // 마스크된 이미지에서 특정 색상 범위에 포함된 픽셀 수를 카운트
            int non_zero_count = cv::countNonZero(mask);
            RCLCPP_INFO(this->get_logger(), "Green pixel count: %d", non_zero_count);

            // 특정 픽셀 수 이상이면 터널 안이라고 판단
            if (non_zero_count < 10000 && chk == 0)
            { // 기준값은 상황에 맞게 조정
                chk = 1;
                RCLCPP_INFO(this->get_logger(), "Outside tunnel (Green pixel count: %d)", non_zero_count);
            }
            else if (non_zero_count > 10000 && chk == 1)
            { // 기준값은 상황에 맞게 조정
                chk = 2;
                RCLCPP_INFO(this->get_logger(), "Tunnel detected (Green pixel count: %d)", non_zero_count);
            }
            else if (non_zero_count < 10000 && chk == 2)
            {
                // Bool 메시지 생성 및 퍼블리시
                std_msgs::msg::Bool msg;
                msg.data = true;
                tunnel_end_->publish(msg);

                RCLCPP_INFO(this->get_logger(), "Outside tunnel (Green pixel count: %d)", non_zero_count);
            }
        }
        else if (tunnel_end == 5)
        {
            // OpenCV를 사용하여 sensor_msgs::Image 메시지를 cv::Mat 형식으로 변환
            cv_bridge::CvImagePtr cv_ptr;
            try
            {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            }
            catch (cv_bridge::Exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
                return;
            }

            // 이미지를 그레이스케일로 변환
            cv::Mat gray_image;
            cv::cvtColor(cv_ptr->image, gray_image, cv::COLOR_BGR2GRAY);

            // 그레이스케일 이미지에서 특정 명암 범위를 찾기 위한 코드
            // 예를 들어, 어두운 픽셀 범위 설정 (0~100)
            cv::Mat mask;
            cv::inRange(gray_image, cv::Scalar(0), cv::Scalar(100), mask);

            // 마스크된 이미지에서 특정 명암 범위에 포함된 픽셀 수를 카운트
            int non_zero_count = cv::countNonZero(mask);
            RCLCPP_INFO(this->get_logger(), "Dark pixel count: %d", non_zero_count);

            // 특정 픽셀 수 이상이면 터널 안이라고 판단
            if (non_zero_count < 10000 && chk == 0)
            { // 기준값은 상황에 맞게 조정
                chk = 1;
                RCLCPP_INFO(this->get_logger(), "Outside tunnel (Dark pixel count: %d)", non_zero_count);
            }
            else if (non_zero_count > 10000 && chk == 1)
            { // 기준값은 상황에 맞게 조정
                chk = 2;
                RCLCPP_INFO(this->get_logger(), "Tunnel detected (Dark pixel count: %d)", non_zero_count);
            }
            else if (non_zero_count < 10000 && chk == 2)
            {
                // Bool 메시지 생성 및 퍼블리시
                std_msgs::msg::Bool msg;
                msg.data = true;
                tunnel_end_->publish(msg);

                RCLCPP_INFO(this->get_logger(), "Outside tunnel (Dark pixel count: %d)", non_zero_count);
            }
        }
    }

    void I_want_know_tunnel_end(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr ceiling_projected)
    {

        // Remove points close to the ground plane
        pcl::PointCloud<pcl::PointXYZI>::Ptr ceiling_filtered(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::CropBox<pcl::PointXYZI> cropFilter;
        cropFilter.setInputCloud(cloud);

        cropFilter.setMin(Eigen::Vector4f(15, -30, 3.0, 1));
        cropFilter.setMax(Eigen::Vector4f(30, 30, 10.0, 1));

        cropFilter.filter(*ceiling_filtered);

        // Z 축 값이 0이 되도록 변환 행렬을 생성합니다.
        Eigen::Matrix4f projection_matrix = Eigen::Matrix4f::Identity();
        /*
        | 1 0 0 x |
        | 0 1 0 y |
        | 0 0 1 z |
        | 0 0 0 1 |
        (2,2)값에 0을 넣어서 z값을 삭제합니다.
        */
        projection_matrix(2, 2) = 0;
        // std::cout << "before" << std::endl;
        std::vector<int> indices_remove; // NaN 값을 제거한 후 유지할 포인트 인덱스를 저장
        pcl::removeNaNFromPointCloud(*ceiling_filtered, *ceiling_filtered, indices_remove);
        // std::cout << "after" << std::endl;
        // 변환 행렬을 적용합니다.
        if (!ceiling_filtered->empty())
        {
            ceiling_nan = true;
            pcl::transformPointCloud(*ceiling_filtered, *ceiling_projected, projection_matrix);
        }
        else
        {
            ceiling_nan = false;
            std::cout << "No valid points after removing NaNs." << std::endl;
        }
    }

    void clustering_tunnel_ceiling(pcl::PointCloud<pcl::PointXYZI>::Ptr ceiling_projected, pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_projected)
    {
        // 클러스터링을 위한 KDTree 오브젝트 생성
        pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZI>);
        kdtree->setInputCloud(ceiling_projected);

        // 클러스터링 결과를 저장할 벡터
        std::vector<pcl::PointIndices> ceiling_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        ec.setClusterTolerance(20.0);
        ec.setMinClusterSize(10);
        // ec.setMaxClusterSize(25000);
        ec.setSearchMethod(kdtree);
        ec.setInputCloud(ceiling_projected);
        ec.extract(ceiling_indices);

        int ii = 0;
        pcl::PointCloud<PointT> TotalCloud;
        std::vector<pcl::PointCloud<PointT>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<PointT>::Ptr>> clusters;
        pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
        // std::cout << "before" << std::endl;
        for (vector<pcl::PointIndices>::const_iterator it = ceiling_indices.begin(); it != ceiling_indices.end(); ++it, ++ii)
        {
            for (vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            {
                cluster->points.push_back(ceiling_projected->points[*pit]);
                PointT pt = ceiling_projected->points[*pit];
                PointT pt2;
                pt2.x = pt.x, pt2.y = pt.y, pt2.z = pt.z;
                pt2.intensity = (float)(ii + 1);
                TotalCloud.push_back(pt2);
            }
            cluster->width = cluster->size();
            cluster->height = 1;
            cluster->is_dense = true;
            clusters.push_back(cluster);
        }
        // std::cout << "mid" << std::endl;
        pcl::PCLPointCloud2 cloud_p;
        pcl::toPCLPointCloud2(TotalCloud, cloud_p);
        sensor_msgs::msg::PointCloud2 clu;
        pcl_conversions::fromPCL(cloud_p, clu);
        clu.header.frame_id = "velodyne";
        pub_ceiling->publish(clu);

        // std::cout << "after" << std::endl;

        std_msgs::msg::Float64MultiArray ceiling_end;
        for (int i = 0; i < int(clusters.size()); i++)
        {
            Eigen::Vector4f centroid, min_p, max_p;
            pcl::compute3DCentroid(*clusters[i], centroid);
            pcl::getMinMax3D(*clusters[i], min_p, max_p);

            geometry_msgs::msg::Point center_point;
            center_point.x = centroid[0];
            center_point.y = centroid[1];
            center_point.z = centroid[2];

            geometry_msgs::msg::Point min_point;
            min_point.x = min_p[0];
            min_point.y = min_p[1];
            min_point.z = min_p[2];

            geometry_msgs::msg::Point max_point;
            max_point.x = max_p[0];
            max_point.y = max_p[1];
            max_point.z = max_p[2];
            float max_px = max_point.x;
            ceiling_end.data.push_back(max_px);

            ceiling_end_check = max_px;
        }
        // ceiling_end.data.push_back(50.0);
        // ceiling_end_->publish(ceiling_end);
        ceiling_end_->publish(ceiling_end);
        ceiling_end.data.clear();
    }

    void lidar_delete_z(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr ceiling_projected)
    {

        // Remove points close to the ground plane
        pcl::PointCloud<pcl::PointXYZI>::Ptr ceiling_filtered(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::CropBox<pcl::PointXYZI> cropFilter;
        cropFilter.setInputCloud(cloud);

        cropFilter.setMin(Eigen::Vector4f(-1, -10, 1.75, 1));
        cropFilter.setMax(Eigen::Vector4f(4, 10, 23, 1));

        cropFilter.filter(*ceiling_filtered);

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "velodyne";
        marker.ns = "my_roi";
        marker.id = 0;
        marker.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.scale.x = cropFilter.getMax()[0] - cropFilter.getMin()[0];
        marker.scale.y = cropFilter.getMax()[1] - cropFilter.getMin()[1];
        marker.scale.z = cropFilter.getMax()[2] - cropFilter.getMin()[2];
        marker.pose.position.x = (cropFilter.getMax()[0] + cropFilter.getMin()[0]) / 2;
        marker.pose.position.y = (cropFilter.getMax()[1] + cropFilter.getMin()[1]) / 2;
        marker.pose.position.z = (cropFilter.getMax()[2] + cropFilter.getMin()[2]) / 2;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 0.3;
        crop_box_->publish(marker);

        // Z 축 값이 0이 되도록 변환 행렬을 생성합니다.
        Eigen::Matrix4f projection_matrix = Eigen::Matrix4f::Identity();
        /*
        | 1 0 0 x |
        | 0 1 0 y |
        | 0 0 1 z |
        | 0 0 0 1 |
        (2,2)값에 0을 넣어서 z값을 삭제합니다.
        */
        // projection_matrix(0, 0) = 0;
        // // std::cout << "before" << std::endl;
        // std::vector<int> indices_remove; // NaN 값을 제거한 후 유지할 포인트 인덱스를 저장
        // pcl::removeNaNFromPointCloud(*ceiling_filtered, *ceiling_filtered, indices_remove);
        // // std::cout << "after" << std::endl;
        // // 변환 행렬을 적용합니다.
        // if (!ceiling_filtered->empty())
        // {
        //     ceiling_nan = true;
        //     pcl::transformPointCloud(*ceiling_filtered, *ceiling_projected, projection_matrix);
        // }
        // else
        // {
        //     ceiling_nan = false;
        //     std::cout << "No valid points after removing NaNs." << std::endl;
        // }
    }

    void search_tunnel_end(pcl::PointCloud<pcl::PointXYZI>::Ptr ceiling_projected, pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_projected)
    {

        // 클러스터링이 끝난 ceiling_projected 포인트 클라우드 데이터에 대해 RANSAC을 사용해 선을 찾음
        pcl::SACSegmentation<pcl::PointXYZI> seg;
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_LINE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.01); // 선에 대한 허용 오차 설정
        seg.setInputCloud(ceiling_projected);

        int line_count = 0; // 추출된 선의 개수를 저장
        pcl::ExtractIndices<pcl::PointXYZI> extract;

        std_msgs::msg::Bool msg;
        msg.data = false;

        if (ceiling_projected->points.empty())
        {
            std::cerr << "ceiling_projected is empty!" << std::endl;
        }
        else
        {
            std::cout << "Number of points in ceiling_projected: " << ceiling_projected->points.size() << std::endl;
        }

        if (ceiling_end_check < 19)
        {
            while (ceiling_projected->points.size() > 0)
            {
                // 선을 찾아 inliers와 coefficients에 저장
                seg.segment(*inliers, *coefficients);
                if (inliers->indices.size() == 0)
                {
                    break; // 더 이상 선이 없으면 종료
                }
                std::cout << inliers->indices.size() << std::endl;
                // 추출된 포인트가 10개 미만이면 publish
                if (inliers->indices.size() < 10)
                {
                    // Bool 메시지 생성 및 퍼블리시
                    msg.data = true;
                    break;
                }

                line_count++; // 선 개수 증가

                // 추출된 선의 포인트들을 제거
                extract.setInputCloud(ceiling_projected);
                extract.setIndices(inliers);
                extract.setNegative(true); // inliers 제거
                extract.filter(*ceiling_projected);

                // 만약 선이 2개 미만이라면 true 메시지 publish
                if (line_count < 2)
                {
                    // Bool 메시지 생성 및 퍼블리시
                    msg.data = true;
                }
            }
        }
        tunnel_end_->publish(msg);
    }

    void project_to_2d_plane(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_projected)
    {
        // Z 축 값이 0이 되도록 변환 행렬을 생성합니다.
        Eigen::Matrix4f projection_matrix = Eigen::Matrix4f::Identity();

        projection_matrix(2, 2) = 0;
        std::vector<int> indices_remove; // NaN 값을 제거한 후 유지할 포인트 인덱스를 저장
        pcl::removeNaNFromPointCloud(*cloud, *cloud, indices_remove);
        // 변환 행렬을 적용합니다.
        if (!cloud->empty())
        {
            ceiling_nan = true;
            pcl::transformPointCloud(*cloud, *cloud_projected, projection_matrix);
        }
        else
        {
            ceiling_nan = false;
            std::cout << "No valid points after removing NaNs." << std::endl;
        }
    }

    void find_two_largest_clusters(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr largest_clusters_cloud)
    {
        // 클러스
        // 터링을 위한 KDTree 오브젝트 생성
        pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZI>);
        kdtree->setInputCloud(cloud);

        // 클러스터링 결과를 저장할 벡터
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        ec.setClusterTolerance(0.5);
        ec.setMinClusterSize(10);
        // ec.setMaxClusterSize(25000);
        ec.setSearchMethod(kdtree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        // 클러스터의 길이를 저장할 벡터
        std::vector<std::pair<size_t, size_t>> cluster_lengths;

        // 클러스터의 길이 계산
        for (size_t i = 0; i < cluster_indices.size(); ++i)
        {
            cluster_lengths.emplace_back(cluster_indices[i].indices.size(), i); // emplace_back 함수는 stl에서 제공하는 벡터 등의 컨테이너에서 사용할 수 있는 멤버 함수 like push_back
        } // 첫번째 size_t는 길이 , 두번째 size_t는 index

        // 여기 수정했음
        if (cluster_lengths.size() > 2)
        {
            // 클러스터 길이를 기준으로 정렬(내림차순)
            std::sort(cluster_lengths.begin(), cluster_lengths.end(), std::greater<>()); // greater<>는 첫번째 클러스터의 길이를 비교하여 첫번째가 마지막보다 클 때 참값 반환->내림차순

            size_t num_clusters_to_publish = std::min<size_t>(2, cluster_lengths.size());
            // 가장 긴 두 개의 클러스터에 속한 포인트를 하나의 포인트 클라우드로 병합
            for (size_t i = 0; i < num_clusters_to_publish; ++i)
            {
                size_t cluster_idx = cluster_lengths[i].second;
                for (const auto &point_idx : cluster_indices[cluster_idx].indices)
                {                                                           // 클러스터에 포함된 점들의 인덱스에 대해 반복
                    largest_clusters_cloud->push_back((*cloud)[point_idx]); // law data에서 해당 인덱스에 해당하는 point들 가져옴.
                }
            }
        }
        else
        {
            std::cout << "wall line 수 부족!!!!!!!!!!!!!!!!" << std::endl;
        }
    }

    void vector_wall(pcl::PointCloud<pcl::PointXYZI>::Ptr largest_clusters_cloud)
    {
        // Create KD-tree object for efficient clustering
        pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZI>);
        kdtree->setInputCloud(largest_clusters_cloud);

        // Euclidean clustering
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        ec.setClusterTolerance(0.5);
        ec.setMinClusterSize(10);
        // ec.setMaxClusterSize(50);
        ec.setSearchMethod(kdtree);
        ec.setInputCloud(largest_clusters_cloud);
        ec.extract(cluster_indices);

        std_msgs::msg::Float64MultiArray wall_d;

        // For each of the largest clusters...

        for (const auto &indices : cluster_indices)
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>);
            for (const auto &index : indices.indices)
            {
                cluster->points.push_back(largest_clusters_cloud->points[index]);
            }

            // Find the min and max points along x-axis (or whichever axis represents your 'horizontal')
            // x축을 기준으로 가장 가까운 점과 큰 점을 찾고 그에 대응하는 y값 저장
            float x_min = std::numeric_limits<float>::max();
            float y_at_x_min = 0;

            float x_max = std::numeric_limits<float>::min();
            float y_at_x_max = 0;

            for (const auto &point : cluster->points) // 클러스터 내의 포인터들에 대해 반복
            {
                if (point.x < x_min)
                {
                    x_min = point.x;
                    y_at_x_min = point.y;
                }

                if (point.x > x_max)
                {
                    x_max = point.x;
                    y_at_x_max = point.y;
                }
            }
            wall_d.data.push_back(x_min);
            wall_d.data.push_back(y_at_x_min);
            wall_d.data.push_back(x_max);
            wall_d.data.push_back(y_at_x_max);
        }
        wall_distance->publish(wall_d);
        wall_d.data.clear();
    }

    void extractLanesByIntensity(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered2, pcl::PointCloud<pcl::PointXYZI>::Ptr lanes_cloud, float lane_intensity_threshold)
    {
        for (const auto &point : cloud_filtered2->points)
        {
            if (point.intensity >= lane_intensity_threshold)
            {
                lanes_cloud->points.push_back(point);
            }
        }
    }

    void clusterLanes(pcl::PointCloud<pcl::PointXYZI>::Ptr lanes_cloud, double tolerance, int min_size, int max_size)
    {
        // Create KD-tree for efficient clustering
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
        tree->setInputCloud(lanes_cloud);

        // Perform Euclidean clustering
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        ec.setClusterTolerance(tolerance);
        ec.setMinClusterSize(min_size);
        ec.setMaxClusterSize(max_size);
        ec.setSearchMethod(tree);
        ec.setInputCloud(lanes_cloud);

        // Extract the clusters out of the input data
        ec.extract(cluster_indices); // 여기에 저장

        int ii = 0;
        pcl::PointCloud<PointT> TotalCloud;
        std::vector<pcl::PointCloud<PointT>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<PointT>::Ptr>> clusters; // allocator 메모리 정렬용
        pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
        for (vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it, ++ii)
        {
            for (vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            {
                cluster->points.push_back(lanes_cloud->points[*pit]);
                PointT pt = lanes_cloud->points[*pit];
                PointT pt2;
                pt2.x = pt.x, pt2.y = pt.y, pt2.z = pt.z;
                pt2.intensity = (float)(ii + 1);
                TotalCloud.push_back(pt2);
            }
            cluster->width = cluster->size();
            cluster->height = 1;
            cluster->is_dense = true;
            clusters.push_back(cluster);
        }

        pcl::PCLPointCloud2 cloud_p;
        pcl::toPCLPointCloud2(TotalCloud, cloud_p);
        sensor_msgs::msg::PointCloud2 clu;
        pcl_conversions::fromPCL(cloud_p, clu);
        clu.header.frame_id = "velodyne";
        pub_cluster->publish(clu);

        // Cluster centers and clusters for visualization
        pcl::PointCloud<pcl::PointXYZI>::Ptr centers(new pcl::PointCloud<pcl::PointXYZI>);

        visualization_msgs::msg::MarkerArray marker_array1;
        visualization_msgs::msg::MarkerArray marker_array2;
        visualization_msgs::msg::MarkerArray marker_array3;

        std_msgs::msg::ColorRGBA color;
        color.r = 1.f;
        color.g = 0.f;
        color.b = 0.f;
        color.a = 1.f;

        std_msgs::msg::ColorRGBA color2;
        color2.r = 0.f;
        color2.g = 0.f;
        color2.b = 1.f;
        color2.a = 1.f;

        std_msgs::msg::Float64MultiArray lane_coord;

        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
        {
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*lanes_cloud, it->indices, centroid);

            pcl::PointXYZI center_point;
            center_point.x = centroid[0];
            center_point.y = centroid[1];
            center_point.z = centroid[2];

            lane_coord.data.push_back(center_point.x);
            lane_coord.data.push_back(center_point.y);

            centers->points.push_back(center_point);

            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "velodyne";
            marker.ns = "clusters";                                     // namespace
            marker.id = it - cluster_indices.begin();                   // 현재 클러스터의 인덱스
            marker.type = visualization_msgs::msg::Marker::SPHERE_LIST; // marker 시각화
            marker.action = visualization_msgs::msg::Marker::ADD;       // marker 동작 설정 여기서는 추가하는 동작
            marker.lifetime.nanosec = 100000000;                        // 0.1초가 지나면 marker 사라짐

            geometry_msgs::msg::Point point_msg;
            point_msg.x = center_point.x;
            point_msg.y = center_point.y;
            point_msg.z = center_point.z;

            marker.points.push_back(point_msg);
            marker.colors.push_back(color);

            marker.scale.x = 0.5;
            marker.scale.y = 0.5;
            marker.scale.z = 0.5;

            marker_array1.markers.push_back(marker);
        }

        lane_dot_MA->publish(lane_coord); // 클러스터링 된 lane의 중심점 좌표 pub
        lane_coord.data.clear();

        lane_dot_->publish(marker_array1); // 클러스터링 된 lane의 중심점 좌표 시각화
        marker_array1.markers.clear();

        pcl::PointCloud<pcl::PointXYZI>::Ptr centercenters(new pcl::PointCloud<pcl::PointXYZI>); // 중앙선만 따로 뽑아낸 pointcloud
        pcl::CropBox<pcl::PointXYZI> cropFilter;
        cropFilter.setInputCloud(centers);

        cropFilter.setMin(Eigen::Vector4f(-0, 0, -1.7, 1)); // 여기 바꿔
        cropFilter.setMax(Eigen::Vector4f(8, 4, -0.45, 1));

        cropFilter.filter(*centercenters);

        if (centercenters->points.size() > 2) // 여기 바꿔
        {
            std_msgs::msg::Float64MultiArray center_lane;
            std_msgs::msg::Float64MultiArray your_point;
            for (int i = 0; i < 1; ++i)
            {
                pcl::SampleConsensusModelLine<PointT>::Ptr line_model(new pcl::SampleConsensusModelLine<PointT>(centercenters));
                pcl::ProgressiveSampleConsensus<PointT> ransac(line_model);
                // 샘플 컨센서스 모델을 사용해서 포인트 클라우드에서 선을 추정하기 위한 모델을 생성

                //////////////////////////////////////////////////////////////{
                if (chk != 2)
                {
                    ransac.setDistanceThreshold(0.5); // 여기 바꿔
                }
                else
                {
                    ransac.setDistanceThreshold(0.8); // 여기 바꿔
                }
                //////////////////////////////////////////////////////////////

                std::vector<int> inliers_;
                printf("ransac 생성\n");
                if (ransac.computeModel())
                {
                    printf("ransac 성공\n");
                    ransac.getInliers(inliers_); // 인라이어를 inliers_에 저장

                    bool isTooFar = false;

                    for (size_t j = 1; j < inliers_.size(); ++j)
                    {
                        const auto &prev_point = centercenters->points[inliers_[j - 1]];
                        const auto &curr_point = centercenters->points[inliers_[j]];

                        float segment_length = std::sqrt(std::pow(curr_point.x - prev_point.x, 2) +
                                                         std::pow(curr_point.y - prev_point.y, 2) +
                                                         std::pow(curr_point.z - prev_point.z, 2));

                        if (segment_length > 3.0)
                        // 이전 점이랑 현재 점이랑 거리가 임계값을 넘으면 인라이어 다시 잡음
                        {
                            printf("거리 너무 길어\n");
                            isTooFar = true;
                            break;
                        }
                    }
                    if (!isTooFar)
                    {
                        printf("선 생성 해줄게\n");
                        // Create and add markers to visualize the line segment
                        visualization_msgs::msg::Marker line_marker;
                        line_marker.header.frame_id = "velodyne";
                        line_marker.ns = "clusters";
                        line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
                        line_marker.action = visualization_msgs::msg::Marker::ADD;
                        line_marker.scale.x = 0.2;
                        line_marker.pose.orientation.w = 1.0;
                        line_marker.lifetime.nanosec = 100000000;

                        // Set the color of the line segment marker
                        std_msgs::msg::ColorRGBA color_line_segment;
                        color_line_segment.r = 0.f;
                        color_line_segment.g = 1.f;
                        color_line_segment.b = 0.f;
                        color_line_segment.a = 1.0;

                        std::vector<geometry_msgs::msg::Point> points;
                        for (const auto &index : inliers_)
                        {
                            // printf("선에 좌표 부여 해줄게\n");
                            geometry_msgs::msg::Point point_msg;
                            point_msg.x = centercenters->points[index].x;
                            point_msg.y = centercenters->points[index].y;
                            point_msg.z = centercenters->points[index].z;

                            geometry_msgs::msg::Point point_yours;
                            point_yours.x = point_msg.x;
                            point_yours.y = point_msg.y - 2.0;
                            point_yours.z = point_msg.z;

                            points.push_back(point_yours); // 차선에서 오른쪽으로 2만큼 옮긴 점들(erp 점)

                            line_marker.points.push_back(point_msg);
                            line_marker.colors.push_back(color_line_segment);

                            // Assign a unique id for each cluster
                            static int marker_id_counter = 0;
                            line_marker.id = marker_id_counter++;

                            visualization_msgs::msg::Marker marker_yours;
                            marker_yours.header.frame_id = "velodyne";
                            marker_yours.ns = "coord";
                            marker_yours.id = marker_id_counter++;
                            marker_yours.type = visualization_msgs::msg::Marker::SPHERE_LIST;
                            marker_yours.action = visualization_msgs::msg::Marker::ADD;
                            marker_yours.lifetime.nanosec = 100000000;

                            geometry_msgs::msg::Point point_coord;
                            point_coord.x = point_yours.x;
                            point_coord.y = point_yours.y;
                            point_coord.z = point_yours.z;

                            marker_yours.points.push_back(point_coord);
                            marker_yours.colors.push_back(color2);

                            marker_yours.scale.x = 0.5;
                            marker_yours.scale.y = 0.5;
                            marker_yours.scale.z = 0.5;

                            marker_array3.markers.push_back(marker_yours);
                        }

                        std::sort(points.begin(), points.end(), [](const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b)
                                  { return a.x < b.x; });

                        for (const auto &sorted_point : points)
                        {
                            center_lane.data.push_back(sorted_point.x);
                            center_lane.data.push_back(sorted_point.y);
                        }
                        center_->publish(center_lane);
                        center_lane.data.clear();

                        marker_array2.markers.push_back(line_marker);
                        lane_->publish(marker_array2);
                        marker_array2.markers.clear();

                        coord_dot_->publish(marker_array3);
                        marker_array3.markers.clear();
                    }
                }
            }
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LiDAR_lane>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}