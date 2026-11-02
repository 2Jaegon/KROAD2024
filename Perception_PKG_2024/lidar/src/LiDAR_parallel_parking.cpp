#include "lidar/LiDAR_parallel_parking.hpp"

using PointT = pcl::PointXYZI;
using namespace std;

// 1030 girin bag

LiDAR_parallel_parking::LiDAR_parallel_parking()
    : Mission("LiDAR_parallel_parking")
{
  // lidar::best_effort_qos()
  velodyne_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/Cluster/parallel", lidar::best_effort_qos());
  object_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/LiDAR/parking_edge_out", lidar::best_effort_qos());
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/Marker/parking_cen_mark", lidar::best_effort_qos());
  marker_pub2_ = this->create_publisher<visualization_msgs::msg::Marker>("/Marker/parallel_roi", lidar::best_effort_qos());
  park_ok2_ = this->create_publisher<std_msgs::msg::Bool>("/LiDAR/park_ok2", lidar::best_effort_qos());

  RCLCPP_INFO(this->get_logger(), "LiDAR_parallel_parking is ready");
}

int LiDAR_parallel_parking::get_status_code()
{
  return 14;
}

void LiDAR_parallel_parking::callback(const sensor_msgs::msg::PointCloud2::SharedPtr input)
{
  if (check_pointcloud == 0)
  {
    RCLCPP_INFO(this->get_logger(), "LiDAR_parallel_parking node sub pointcloud");
    check_pointcloud = 1;
  }
  // cout << "라이다 콜백" << endl;
  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
  pcl::fromROSMsg(*input, *cloud);

  pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
  pcl::VoxelGrid<PointT> VG;
  VG.setInputCloud(cloud);
  VG.setLeafSize(0.1f, 0.1f, 0.1f);
  VG.filter(*cloud_filtered);
  static bool left = false;
  pcl::PointCloud<PointT>::Ptr cloud_filtered2(new pcl::PointCloud<PointT>);
  pcl::CropBox<PointT> cropFilter;
  cropFilter.setInputCloud(cloud_filtered);

  //////////////////////// 라이다 고정되면 수정 필요 /////////////////////////////////////////
  if (park_area == false)
  {

    //[kcity 코끼리]
    // cropFilter.setMin(Eigen::Vector4f(2.5, -6.5, -0.5, 0.0));
    // cropFilter.setMax(Eigen::Vector4f(3.5, -0.5, 0.5, 0.0));
    //[kcity 기린]
    cropFilter.setMin(Eigen::Vector4f(2.8, -6.5, -0.5, 0.0));
    cropFilter.setMax(Eigen::Vector4f(4.1, -0.5, 0.5, 0.0));
    //[옛날코드]
    // cropFilter.setMin(Eigen::Vector4f(2.5, -6.5, -0.5, 0.0));
    // cropFilter.setMax(Eigen::Vector4f(3.5, -0.5, 0.5, 0.0));
  }

  else if (park_area == true && park_ok2 == false)
  {
    cout << "max_y_point.y :" << max_y_point.y << endl;
    //[kcity 코끼리]
    // cropFilter.setMin(Eigen::Vector4f(1.3, max_y_point.y - 0.5, -0.5, 0.0));
    // cropFilter.setMax(Eigen::Vector4f(4.5, max_y_point.y + 1.0, 0.5, 0.0));
    //[kcity 기린]
    cropFilter.setMin(Eigen::Vector4f(2.0, max_y_point.y - 0.5, -0.5, 0.0));
    cropFilter.setMax(Eigen::Vector4f(4.5, max_y_point.y + 1.0, 0.5, 0.0));
    // [옛날꺼]
    // cropFilter.setMin(Eigen::Vector4f(1.3, max_y_point.y-0.5, -0.5, 0.0));
    // cropFilter.setMax(Eigen::Vector4f(4.5, max_y_point.y+1.0 , 0.5, 0.0));
  }
  else if (park_ok2 == true)
  {
    cropFilter.setMin(Eigen::Vector4f(-3, max_y_point.y - 3.8, -0.5, 0.0));
    cropFilter.setMax(Eigen::Vector4f(8.0, max_y_point.y + 1.0, 0.5, 0.0));
  }

  ///////////////////////////////////////////////////////////////////////////////////////

  //////////////////////// < ROI를 나타내는 직육면체 마커 > //////////////////////////////////
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
  marker_pub2_->publish(marker);
  /////////////////////////////////////////////////////////////////////////////////////

  cropFilter.filter(*cloud_filtered2);

  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
  tree->setInputCloud(cloud_filtered2);
  vector<pcl::PointIndices> cluster_indices;

  pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
  ec.setInputCloud(cloud_filtered2);

  if (park_area == false)
  {
    ec.setClusterTolerance(0.5);
    ec.setMinClusterSize(4);
    ec.setMaxClusterSize(1000);
    ec.setSearchMethod(tree);
    ec.extract(cluster_indices);
  }
  else if (park_area == true && park_ok2 == false)
  {
    ec.setClusterTolerance(0.5);
    ec.setMinClusterSize(4);
    ec.setMaxClusterSize(1000);
    ec.setSearchMethod(tree);
    ec.extract(cluster_indices);
  }
  else if (park_ok2 == true)
  {
    ec.setClusterTolerance(0.5);
    ec.setMinClusterSize(2);
    ec.setMaxClusterSize(1000);
    ec.setSearchMethod(tree);
    ec.extract(cluster_indices);
  }

  int ii = 0;
  pcl::PointCloud<PointT> TotalCloud;
  clusters.clear();
  for (vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it, ++ii)
  {
    pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
    for (vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
    {
      cluster->points.push_back(cloud_filtered2->points[*pit]);
      PointT pt = cloud_filtered2->points[*pit];
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
  sensor_msgs::msg::PointCloud2 output;
  pcl_conversions::fromPCL(cloud_p, output);
  output.header.frame_id = "velodyne";
  velodyne_pub_->publish(output);

  if (park_ok2 == false && park_ok3 == false)
  {
    roi_change();
  }

  else if (park_ok2 == true && park_ok3 == false)
  {
    if (ok2_count < 5)
    {
      park_ok2_msg.data = park_ok2;
      park_ok2_->publish(park_ok2_msg);
      ok2_count++;
    }
    if (ok2_count >= 5)
    {
      if (!parking_check)
      {
        park_time = std::chrono::system_clock::now();
        parking_check = true;
      }

      if (parking_check)
      {
        cout << "현재까지 대기한 시간 -> " << chrono::duration_cast<chrono::seconds>(chrono::system_clock::now() - park_time).count() << "\n";
      }

      if (chrono::duration_cast<chrono::seconds>(chrono::system_clock::now() - park_time).count() > 2)
      {
        parking_check = false;
        park_ok3 = true;
      }
    }
  }

  else if (park_ok3 == true)
  {
    find_parking_cen();
  }
}

void LiDAR_parallel_parking::roi_change()
{
  // cout << "cluster.size(): " << clusters.size() << endl;
  if (clusters.size() > 0)
  {
    int start_cluster = 0;
    for (size_t i = 0; i < clusters.size(); i++)
    {
      Eigen::Vector4f centroid, min_p, max_p;
      Eigen::Vector3f scale_;
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

      if (park_area == false)
      {
        ///// 첫 ROI 콘 사이즈 /////
        //////////////////////// 라이다 고정되면 수정 필요 /////////////////////////////////////////
        if ((max_point.y - min_point.y) < 0.20 && (max_point.y - min_point.y) > 0.05 &&
            (max_point.z - min_point.z) > 0.13 && (max_point.z - min_point.z) < 0.6 &&
            max_point.z < 0.2 && max_point.z > -0.3)
        ///////////////////////////////////////////////////////////////////////////////////////
        {
          start_cluster = start_cluster + 1; // 주차시작 부분을 감지하고 ROI 변경을 위한 부분
        }
        cout << "max_point.y - min_point.y: " << max_point.y - min_point.y << endl;
        cout << "max_point.z - min_point.z: " << max_point.z - min_point.z << "    max_point.z: " << max_point.z << endl;
        cout << "start_cluster: " << start_cluster << endl;
      }
    }

    double ok2_min_y = std::numeric_limits<double>::max();
    double ok2_max_y = std::numeric_limits<double>::lowest();
    std::vector<geometry_msgs::msg::Point> dynamic_list;

    if (park_area == false && start_cluster >= 2)
    {
      std::cout << "check:2 2️⃣ 2️⃣ 2️⃣ 2️⃣" << std::endl;
      std::cout << "check:2 2️⃣ 2️⃣ 2️⃣ 2️⃣" << std::endl;
      // if (point.y < ok2_min_y)
      //     ok2_min_y = point.y;
      // if (point.y > ok2_max_y)
      //     ok2_max_y = point.y;

      for (size_t i = 0; i < clusters.size(); i++)
      {
        Eigen::Vector4f centroid, min_p, max_p;
        pcl::compute3DCentroid(*clusters[i], centroid);
        pcl::getMinMax3D(*clusters[i], min_p, max_p);
        geometry_msgs::msg::Point center_point;
        center_point.x = centroid[0];
        center_point.y = centroid[1];
        center_point.z = centroid[2];

        dynamic_list.push_back(center_point);

        for (const auto &point : dynamic_list)
        {
          obj.data.push_back(point.x);
          obj.data.push_back(point.y);
          std::cout << "point : " << point.x << " ,  " << point.y << endl;
        }
      }
      cout << "dynamic_list(size) : " << dynamic_list.size() << endl;

      float max_y = std::numeric_limits<float>::lowest();

      // 모든 점들을 순회하며 y값이 가장 작은 점 찾기
      for (const auto &point : dynamic_list)
      {
        if (point.y > max_y)
        {
          max_y = point.y;
          max_y_point = point;
        }
      }

      cout << "Point with max y value: ("
           << max_y_point.x << ", "
           << max_y_point.y << ")" << endl;

      park_area = true;
      park_ok1 = true;
    }
  }

  /////// 클러스터가 없으면 check3로 넘어가는데 가끔 클러스터가 있는데 사라지는 순간이 있어 방지하기 위해 추가 //////////
  if (park_area == true && clusters.size() == 0 && cluster_count <= 2)
  {
    cluster_count = cluster_count + 1;
    std::cout << "cluster_count: " << cluster_count << std::endl;
  }
  ///////////////////////////////////////////////////////////////////////////////////////////////////

  else if (park_area == true && cluster_count > 2)
  {
    std::cout << "check:3 3️⃣ 3️⃣ 3️⃣ 3️⃣" << std::endl;
    std::cout << "check:3 3️⃣ 3️⃣ 3️⃣ 3️⃣" << std::endl;
    std::cout << "check:3 3️⃣ 3️⃣ 3️⃣ 3️⃣" << std::endl;
    std::cout << "check:3 3️⃣ 3️⃣ 3️⃣ 3️⃣" << std::endl;
    park_ok2 = true;
  }
}

void LiDAR_parallel_parking::find_parking_cen()
{

  std::vector<geometry_msgs::msg::Point> point_list;
  std::vector<geometry_msgs::msg::Point> P_list;
  // 퍼블리시를 5번까지만 수행
  if (pub_count < 1)
  {
    if (clusters.size() > 0)
    {
      for (size_t i = 0; i < clusters.size(); i++)
      {
        Eigen::Vector4f centroid, min_p, max_p;
        pcl::compute3DCentroid(*clusters[i], centroid);
        // pcl::getMinMax3D(*clusters[i], min_p, max_p);
        geometry_msgs::msg::Point center_point;
        center_point.x = centroid[0];
        center_point.y = centroid[1];
        center_point.z = centroid[2];
        point_list.push_back(center_point);
      }
    }

    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double min_y = std::numeric_limits<double>::max();
    double max_y = std::numeric_limits<double>::lowest();
    double min_z = std::numeric_limits<double>::max();
    double max_z = std::numeric_limits<double>::lowest();

    // 벡터를 순회하며 최대, 최소값을 찾음
    for (const auto &point : point_list)
    {
      if (point.x < min_x)
        min_x = point.x;
      if (point.x > max_x)
        max_x = point.x;

      if (point.y < min_y)
        min_y = point.y;
      if (point.y > max_y)
        max_y = point.y;

      if (point.z < min_z)
        min_z = point.z;
      if (point.z > max_z)
        max_z = point.z;
    }

    geometry_msgs::msg::Point center;
    center.x = (min_x + max_x) / 2;
    center.y = max_y;
    center.z = (min_z + max_z) / 2;

    geometry_msgs::msg::Point short_point;
    double length = std::numeric_limits<double>::max();

    for (size_t i = 0; i < clusters.size(); i++)
    {
      Eigen::Vector4f centroid, min_p, max_p;
      pcl::compute3DCentroid(*clusters[i], centroid);
      pcl::getMinMax3D(*clusters[i], min_p, max_p);
      geometry_msgs::msg::Point center_point;
      center_point.x = centroid[0];
      center_point.y = centroid[1];
      center_point.z = centroid[2];

      double compute = sqrt(pow(center_point.x - center.x, 2) + pow(center_point.y - center.y, 2)); // sqrt(dx * dx + dy * dy)
      if (length > compute)
      {
        length = compute;
        short_point = center_point;
      }
    }
    // cout << "center x : " << center_point.x << " y : " << center_point.y << endl;
    // minpoint.y = min_y;
    geometry_msgs::msg::Point p1;
    geometry_msgs::msg::Point p2;
    geometry_msgs::msg::Point p3;
    geometry_msgs::msg::Point p4;
    geometry_msgs::msg::Point closest_point;

    if (short_point.x < center.x)
    {
      p1 = short_point;

      p4.x = short_point.x + 5;
      p4.y = short_point.y;
      p4.z = short_point.z;
      std::cout << "p1 먼저잡음 p1 : " << p1.x << " ,  " << p1.y << std::endl;

      float closest_distance = std::numeric_limits<float>::max(); // 초기화

      for (size_t j = 0; j < clusters.size(); j++)
      {
        Eigen::Vector4f centroid, min_p, max_p;
        pcl::compute3DCentroid(*clusters[j], centroid);
        geometry_msgs::msg::Point center_point;
        center_point.x = centroid[0];
        center_point.y = centroid[1];
        center_point.z = centroid[2];

        // 유클리드 거리 계산
        float distance = std::sqrt(
            std::pow(center_point.x - p4.x, 2) +
            std::pow(center_point.y - p4.y, 2));

        // 가장 가까운 클러스터의 중심점을 찾음
        if (distance < closest_distance)
        {
          closest_distance = distance;
          closest_point = center_point;
        }
      }
      p4 = closest_point;
    }
    else
    {
      p4 = short_point;

      p1.x = short_point.x - 5;
      p1.y = short_point.y;
      p1.z = short_point.z;

      std::cout << "p4 먼저잡음 p4 : " << p4.x << " ,  " << p4.y << short_point.x << std::endl;

      geometry_msgs::msg::Point closest_point;
      float closest_distance = std::numeric_limits<float>::max(); // 초기화

      for (size_t j = 0; j < clusters.size(); j++)
      {
        Eigen::Vector4f centroid, min_p, max_p;
        pcl::compute3DCentroid(*clusters[j], centroid);
        geometry_msgs::msg::Point center_point;
        center_point.x = centroid[0];
        center_point.y = centroid[1];
        center_point.z = centroid[2];

        // 유                                 클리드 거리 계산
        float distance = std::sqrt(
            std::pow(center_point.x - p1.x, 2) +
            std::pow(center_point.y - p1.y, 2));

        // 가장 가까운 클러스터의 중심점을 찾음
        if (distance < closest_distance)
        {
          closest_distance = distance;
          closest_point = center_point;
        }
      }
      p1 = closest_point;
    }

    p2.x = p1.x;
    p2.y = p1.y - 3;
    p2.z = p1.z;

    p3.x = p4.x;
    p3.y = p4.y - 3;
    p3.z = p4.z;

    std::vector<geometry_msgs::msg::Point> find_p3_list = {p3};
    P_list = {p1, p2};

    for (size_t i = 0; i < find_p3_list.size(); i++)
    {
      geometry_msgs::msg::Point p3_closest_point;
      float closest_distance = std::numeric_limits<float>::max(); // 초기화

      for (size_t j = 0; j < clusters.size(); j++)
      {
        Eigen::Vector4f centroid, min_p, max_p;
        pcl::compute3DCentroid(*clusters[j], centroid);
        geometry_msgs::msg::Point center_point;
        center_point.x = centroid[0];
        center_point.y = centroid[1];
        center_point.z = centroid[2];

        // 유클리드 거리 계산
        float distance = std::sqrt(
            std::pow(center_point.x - find_p3_list[i].x, 2) +
            std::pow(center_point.y - find_p3_list[i].y, 2));

        // 가장 가까운 클러스터의 중심점을 찾음
        if (distance < closest_distance)
        {
          closest_distance = distance;
          p3_closest_point = center_point;
        }
      }

      // 가장 가까운 점을 리스트에 추가
      P_list.push_back(p3_closest_point);
    }
    // P_list 에 {p1, p2, p3, p4}
    P_list.push_back(p4);

    // 잡은 p1,p2,p3,p4 출력
    std::cout << "P1 : " << P_list[0].x << " ,  " << P_list[0].y << endl;
    std::cout << "P2 : " << P_list[1].x << " ,  " << P_list[1].y << endl;
    std::cout << "P3 : " << P_list[2].x << " ,  " << P_list[2].y << endl;
    std::cout << "P4 : " << P_list[3].x << " ,  " << P_list[3].y << endl;

    // 메시지 생성 및 퍼블리시
    std_msgs::msg::Float64MultiArray obj;
    obj.data.clear(); // 초기화

    /////////////////////////////민주 추가////////////////////////////////////

    std::vector<float> list3;

    // 두 점 사이의 거리를 계산하고, 조건에 맞는 점을 찾는 루프
    for (size_t i = 0; i < P_list.size(); i++)
    {
      float dx;
      float dy;
      if (i < 3)
      {
        // 두 점 사이의 거리 계산 (z축 고려 안 함)
        dx = P_list[i].x - P_list[i + 1].x;
        dy = P_list[i].y - P_list[i + 1].y;
      }

      else // P_list[3]-P_list[0]
      {
        // 두 점 사이의 거리 계산 (z축 고려 안 함)
        dx = P_list[i].x - P_list[0].x;
        dy = P_list[i].y - P_list[0].y;
      }

      float len = std::sqrt(dx * dx + dy * dy);
      std::cout << "len : " << len << std::endl;

      list3.push_back(len);
    }

    if (list3[0] > 2.7 && list3[0] < 3.3 &&
        list3[1] > 4.7 && list3[1] < 5.3 &&
        list3[2] > 2.7 && list3[2] < 3.3 &&
        list3[3] > 4.7 && list3[3] < 5.3)
    {
      std::cout << "✅✅✅✅4점 잡기 성공티비!!✅✅✅✅" << endl;
      std::cout << "✅✅✅✅4점 잡기 성공티비!!✅✅✅✅" << endl;
      for (const auto &point : P_list)
      {
        obj.data.push_back(point.x);
        obj.data.push_back(point.y);
        std::cout << "point : " << point.x << " ,  " << point.y << endl;
      }
    }
    else
    {

      // Eigen::Vector2f vec(p4.x - p1.x, p4.y - p1.y);  // 벡터 초기화
      // vec.normalize();  // 벡터 정규화
      // Eigen::Vector2f vec_normal = {vec[1], vec[0]};

      std::cout << "🤬🤬🤬🤬faillllllllllllllll🤬🤬🤬🤬" << std::endl;

      geometry_msgs::msg::Point offset_p2;
      geometry_msgs::msg::Point offset_p3;
      obj.data.push_back(P_list[0].x);
      obj.data.push_back(P_list[0].y);

      offset_p2.x = p1.x;
      offset_p2.y = p1.y - 3;
      offset_p2.z = p1.z;
      obj.data.push_back(offset_p2.x);
      obj.data.push_back(offset_p2.y);
      P_list[1] = offset_p2;
      // std::cout << "offset value p1_y - 3 = p2_y : " << p1.y << " -3 =  " << offset_p2.y << endl;

      offset_p3.x = p4.x;
      offset_p3.y = p4.y - 3;
      offset_p3.z = p4.z;
      obj.data.push_back(offset_p3.x);
      obj.data.push_back(offset_p3.y);
      P_list[2] = offset_p3;
      // std::cout << "offset value p4_y - 3 = p3_y : " << p4.y << " -3 =  " << offset_p3.y << endl;

      obj.data.push_back(P_list[3].x);
      obj.data.push_back(P_list[3].y);

      std::cout << "P1 push : " << P_list[0].x << " ,  " << P_list[0].y << endl;
      std::cout << "P2 push : " << P_list[1].x << " ,  " << P_list[1].y << endl;
      std::cout << "P3 push : " << P_list[2].x << " ,  " << P_list[2].y << endl;
      std::cout << "P4 push : " << P_list[3].x << " ,  " << P_list[3].y << endl;
    }

    std::cout << "P_list size: " << P_list.size() << std::endl; // P_list의 사이즈를 출력
    object_pub_->publish(obj);                                  // 퍼블리시
    pub_count++;                                                // 퍼블리시 횟수 증가
  }
  ///////////////////////////////////////////////////////////////////////

  // p1, p2, p3, p4의 마커 생성 및 퍼블리시 (MarkerArray 사용)

  std::vector<std::array<float, 3>> colors = {{0.0, 1.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, {1.0, 1.0, 0.0}};

  visualization_msgs::msg::Marker marker;
  visualization_msgs::msg::MarkerArray marker_array;
  geometry_msgs::msg::Point p;

  for (size_t i = 0; i < P_list.size(); ++i)
  {
    // 마커 설정
    marker.header.frame_id = "velodyne";
    marker.ns = "my_marker";
    marker.id = 9 + i; // 각 마커마다 고유한 id 할당
    marker.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.lifetime.nanosec = 1000000000;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.color.r = colors[i][0];
    marker.color.b = colors[i][1];
    marker.color.g = colors[i][2];
    marker.color.a = 1.0;

    // 점 추가
    p.x = P_list[i].x;
    p.y = P_list[i].y;
    marker.points.push_back(p);

    // 마커를 배열에 추가
    marker_array.markers.push_back(marker);
  }

  // 마커 배열을 퍼블리시
  marker_pub_->publish(marker_array);
}