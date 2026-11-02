#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <limits>
#include "std_msgs/msg/int16.hpp"
// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h> // 길요한이 수정함
#include <pcl/common/common.h>   // 길요한이 수정함
#include <vision_msgs/msg/detection2_d_array.hpp>

// etc
#include <cmath>
#include <vector>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/bool.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/surface/mls.h>
#include <filesystem>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <typeinfo>
#include <cstdio>
#include <fstream>
#include <visualization_msgs/msg/marker.hpp>
#include <pcl/common/centroid.h> // 길요한
#include <pcl/common/common.h>   // 길요한
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include "fusion/qos_utils.hpp"

using namespace std;
using namespace pcl;
using PointT = pcl::PointXYZI;
int planning_flag = 0;
std::vector<pcl::PointCloud<PointT>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<PointT>::Ptr>> clusters;

class VelodynePreprocess : public rclcpp::Node
{
public:
  sensor_msgs::msg::PointCloud2 output;

public:
  VelodynePreprocess()
      : Node("velodyne_preprocess")
  {
    RCLCPP_INFO(this->get_logger(), "Velodyne Preprocess Node has been started");
    planning_mission_sub = this->create_subscription<std_msgs::msg::Int16>("/Planning/mission", fusion::best_effort_qos(), std::bind(&VelodynePreprocess::mission_cb, this, std::placeholders::_1));

    LiDAR_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/velodyne_points_filtered", fusion::best_effort_qos());

    LiDAR_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/velodyne_points", fusion::best_effort_qos(),
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) -> void
        {
          LiDARCallback(msg);
        });

    deli_marker = this->create_publisher<visualization_msgs::msg::MarkerArray>("/Marker/deli_signs", fusion::best_effort_qos());

    // 클러스터 개수 퍼블리셔 추가
    cluster_count_pub_ = this->create_publisher<std_msgs::msg::Int16>("/cluster_count", fusion::best_effort_qos());

    class_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/deli_signs", fusion::best_effort_qos(),
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) -> void
        {
          class_data_ = *msg;
          handleClassData(msg);
        });
    psm_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
        "/deli_detect", fusion::best_effort_qos(),
        [this](const vision_msgs::msg::Detection2DArray::SharedPtr msg) -> void
        {
          psm(msg); // msg를 인자로 psm 함수 호출
        });
  }
  ~VelodynePreprocess()
  {
    RCLCPP_INFO(this->get_logger(), "Velodyne Preprocess Node has been terminated");
  }

public:
  void LiDARCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void handleClassData(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void psm(const vision_msgs::msg::Detection2DArray::SharedPtr msg);
  void mission_cb(const std_msgs::msg::Int16::SharedPtr msgs)
  {
    planning_flag = msgs->data;
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr LiDAR_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr LiDAR_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr deli_sign;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr deli_marker;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr cluster_count_pub_; // 클러스터 개수 퍼블리셔 추가
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr class_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr planning_mission_sub;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr psm_sub_;

  int pre_class = -1;
  std_msgs::msg::Float64MultiArray class_data_;
};

// 클러스터링을 거리순으로 정렬하는 함수
void sortClustersByDistance(std::vector<pcl::PointCloud<PointT>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<PointT>::Ptr>> &clusters)
{
  std::vector<std::pair<float, pcl::PointCloud<PointT>::Ptr>> distance_cluster_pairs;

  for (const auto &cluster : clusters)
  {
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cluster, centroid);
    float distance = std::sqrt(centroid[0] * centroid[0] + centroid[1] * centroid[1] + centroid[2] * centroid[2]);
    distance_cluster_pairs.push_back(std::make_pair(distance, cluster));
  }

  // 거리 기준으로 정렬
  std::sort(distance_cluster_pairs.begin(), distance_cluster_pairs.end(),
            [](const auto &a, const auto &b)
            { return a.first < b.first; });

  // 정렬된 클러스터를 다시 clusters 벡터에 저장
  clusters.clear();
  for (const auto &pair : distance_cluster_pairs)
  {
    clusters.push_back(pair.second);
  }
}
int yolo_cnt = 0;
void VelodynePreprocess::psm(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
{
  if (yolo_cnt > 50)
  {
    RCLCPP_INFO(this->get_logger(), "💕💕💕욜로는 성공적으로 도는중💕💕💕");
    yolo_cnt = 0;
  }
  else
    yolo_cnt++;
}

void VelodynePreprocess::handleClassData(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  // 클러스터링 데이터 처리를 위한 퍼블리셔 설정
  deli_sign = this->create_publisher<std_msgs::msg::Float64MultiArray>("/coord_xy", 1);

  // Define the file path, using the absolute path as requested
  std::string file_path = "/ros2_ws/src/perception_ws/fusion/src/lidar_pre/class_data.txt";
  if (!std::filesystem::exists(file_path))
  {
    RCLCPP_ERROR(this->get_logger(), "🆘🆘🆘File path %s does not exist!🆘🆘🆘", file_path.c_str());
    return;
  }
  if ((planning_flag == 7) || (planning_flag == 9))
  {
    std::cout << planning_flag << std::endl;
    // Open a file in append mode to store class data
    std::ofstream outfile(file_path, std::ios::app);
    if (!outfile.is_open())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open %s for writing", file_path.c_str());
      return;
    }

    // Write the received class data to the file
    RCLCPP_INFO(this->get_logger(), "💕💕💕욜로 돌아서 txt파일에 기록하는중💕💕💕");
    for (double value : msg->data)
    {
      int class_value = static_cast<int>(value);
      if (class_value == 1 || class_value == 2 || class_value == 3)
      {
        outfile << class_value << "\n";
      }
    }
    outfile.close();

    // Now, read the data back from the file to determine the most frequent class
    std::ifstream infile(file_path);
    if (!infile.is_open())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open %s for reading", file_path.c_str());
      return;
    }

    std::map<int, int> class_count;
    int class_value;

    RCLCPP_INFO(this->get_logger(), "💕💕💕욜로 돌아서 txt파일에서 읽고있는중💕💕💕");
    while (infile >> class_value)
    {
      class_count[class_value]++;
    }
    infile.close();

    // Find the class with the maximum occurrences
    int max_count = 0;
    int most_frequent_class = -1;

    for (const auto &pair : class_count)
    {
      if (pair.second > max_count)
      {
        max_count = pair.second;
        most_frequent_class = pair.first;
      }
    }

    pre_class = most_frequent_class;

    if (pre_class != -1)
    { // If a valid class was found
      RCLCPP_INFO(this->get_logger(), "💙💙💙Catch: %d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d💙💙💙", pre_class, pre_class, pre_class, pre_class, pre_class, pre_class, pre_class, pre_class, pre_class, pre_class, pre_class, pre_class, pre_class, pre_class, pre_class, pre_class, pre_class, pre_class, pre_class, pre_class, pre_class);
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "🔇🔇🔇txt파일에 클래스 없어유🔇🔇🔇");
    }
  }
}

void VelodynePreprocess::LiDARCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{

  if (pre_class != -1)
  { // If a valid class was found
    RCLCPP_INFO(this->get_logger(), "💙💙💙💙💙💙💙💙💙Catch: %d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d💙💙💙💙💙💙💙💙💙", pre_class, pre_class, pre_class, pre_class, pre_class, pre_class, pre_class, pre_class, pre_class, pre_class, pre_class, pre_class, pre_class, pre_class, pre_class, pre_class, pre_class, pre_class, pre_class, pre_class, pre_class, pre_class, pre_class, pre_class, pre_class, pre_class, pre_class, pre_class);
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "😚😚😚😚😚😚😚잠깐! 잠깐! txt파일에 확인했나요?.?😚😚😚😚😚😚😚");
  }

  pcl::PointCloud<PointT>::Ptr cloud_origin(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_down(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_crop(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_cluter(new pcl::PointCloud<PointT>);

  pcl::fromROSMsg(*msg, *cloud_origin);

  // 다운샘플링
  pcl::VoxelGrid<PointT> VG;
  VG.setInputCloud(cloud_origin);
  VG.setLeafSize(0.05f, 0.05f, 0.05f);
  VG.filter(*cloud_down);

  // 크롭필터
  pcl::CropBox<PointT> cropFilter;
  cropFilter.setInputCloud(cloud_down);
  cropFilter.setMin(Eigen::Vector4f(1.5, -3.0, 0.1, 1.0)); // x,y,z,1
  cropFilter.setMax(Eigen::Vector4f(25.0, 2.0, 1.2, 1.0));
  cropFilter.filter(*cloud_crop);

  // 클러스터링
  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  tree->setInputCloud(cloud_crop);
  vector<pcl::PointIndices> cluster_indices;

  pcl::EuclideanClusterExtraction<PointT> ECE;
  ECE.setInputCloud(cloud_crop);
  ECE.setClusterTolerance(0.6); // 1m
  ECE.setMinClusterSize(7);
  ECE.setMaxClusterSize(1000);
  ECE.setSearchMethod(tree);
  ECE.extract(cluster_indices);

  // 클러스터 개수를 계산하여 퍼블리시
  int cluster_count = cluster_indices.size();
  std_msgs::msg::Int16 cluster_count_msg;
  cluster_count_msg.data = cluster_count;
  cluster_count_pub_->publish(cluster_count_msg); // 클러스터 개수 퍼블리시

  int ii = 0;
  pcl::PointCloud<PointT> TotalCloud;
  clusters.clear();
  for (vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it, ++ii)
  {
    pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
    for (vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
    {
      cluster->points.push_back(cloud_crop->points[*pit]);
      PointT pt = cloud_crop->points[*pit];
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
  // 클러스터 거리기준정렬 psm수정
  sortClustersByDistance(clusters);

  pcl::PCLPointCloud2 cloud_p;
  pcl::toPCLPointCloud2(TotalCloud, cloud_p);
  pcl_conversions::fromPCL(cloud_p, output);
  output.header.frame_id = "velodyne";
  output.header.stamp = msg->header.stamp;
  LiDAR_pub_->publish(output);

  std_msgs::msg::Float64MultiArray obj;
  std_msgs::msg::Float32MultiArray p;

  if (class_data_.data.size() > 0 && class_data_.data.size() == clusters.size()) // 빅성민이 수정함
  {
    for (int i = 0; i < int(clusters.size()); i++)
    {
      Eigen::Vector4f centroid, min_p, max_p;
      pcl::compute3DCentroid(*clusters[i], centroid);
      pcl::getMinMax3D(*clusters[i], min_p, max_p);

      geometry_msgs::msg::Point center_point;
      center_point.x = centroid[0];
      center_point.y = centroid[1];
      center_point.z = centroid[2];

      float cen_x = center_point.x;
      float cen_y = center_point.y;

      p.data.push_back(cen_x);
      p.data.push_back(cen_y);

      // 클래스 정보가 충분한지 확인 후 추가
      if ((i < class_data_.data.size() && class_data_.data[i] == pre_class + 3) || class_data_.data[i] < 4) // i를 기반으로 유효한 클래스 데이터가 있는지 확인
      // if (i < class_data_.data.size())
      {
        obj.data.push_back(cen_x);
        obj.data.push_back(cen_y);
        obj.data.push_back(class_data_.data[i]);
        // 클래스 정보를 추가

        obj.data.push_back(-1000);
        deli_sign->publish(obj);
        obj.data.clear();
      }
    }
  }

  p.data.push_back(-1000);
  p.data.clear();

  visualization_msgs::msg::MarkerArray p_array;
  geometry_msgs::msg::Point obj1;
  visualization_msgs::msg::Marker marker1;
  marker1.ns = "deli_signs";
  marker1.action = visualization_msgs::msg::Marker::ADD;
  marker1.type = visualization_msgs::msg::Marker::POINTS;
  marker1.id = 0;
  marker1.pose.orientation.w = 1.0;
  marker1.scale.x = 0.5;
  marker1.scale.y = 0.5;
  marker1.color.a = 1.0;
  marker1.color.r = 1.0f;
  obj1.x = p.data[0];
  obj1.y = p.data[1];
  obj1.z = 0.0;
  marker1.points.push_back(obj1);
  marker1.header.frame_id = "velodyne";
  p_array.markers.push_back(marker1);

  geometry_msgs::msg::Point obj2;
  visualization_msgs::msg::Marker marker2;
  marker2.ns = "deli_signs";
  marker2.action = visualization_msgs::msg::Marker::ADD;
  marker2.type = visualization_msgs::msg::Marker::POINTS;
  marker2.id = 1;
  marker2.pose.orientation.w = 1.0;
  marker2.scale.x = 0.5;
  marker2.scale.y = 0.5;
  marker2.color.a = 1.0;
  marker2.color.b = 1.0f;
  obj2.x = p.data[2];
  obj2.y = p.data[3];
  obj2.z = 0.0;
  marker2.points.push_back(obj2);
  marker2.header.frame_id = "velodyne";
  p_array.markers.push_back(marker2);

  geometry_msgs::msg::Point obj3;
  visualization_msgs::msg::Marker marker3;
  marker3.ns = "deli_signs";
  marker3.action = visualization_msgs::msg::Marker::ADD;
  marker3.type = visualization_msgs::msg::Marker::POINTS;
  marker3.id = 2;
  marker3.pose.orientation.w = 1.0;
  marker3.scale.x = 0.5;
  marker3.scale.y = 0.5;
  marker3.color.a = 1.0;
  marker3.color.g = 1.0f;
  obj3.x = p.data[4];
  obj3.y = p.data[5];
  obj3.z = 0.0;
  marker3.points.push_back(obj3);
  marker3.header.frame_id = "velodyne";
  p_array.markers.push_back(marker3);

  deli_marker->publish(p_array);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelodynePreprocess>());
  rclcpp::shutdown();
  return 0;
}