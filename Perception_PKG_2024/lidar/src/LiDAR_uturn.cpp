#include "lidar/LiDAR_uturn.hpp"
#include "lidar/utils.hpp"

LiDAR_uturn::LiDAR_uturn()
    : Mission("LiDAR_uturn")
{

    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/Cluster/uturn", lidar::best_effort_qos());
    pub_center_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/LiDAR/center_point", lidar::best_effort_qos());
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/Marker/uturn_cen_mark", lidar::best_effort_qos());
    marker_pub2_ = this->create_publisher<visualization_msgs::msg::Marker>("/Marker/uturn_roi", lidar::best_effort_qos());

    RCLCPP_INFO(this->get_logger(), "LiDAR_uturn is ready");
}

int LiDAR_uturn::get_status_code()
{
    return 16;
}

void LiDAR_uturn::callback(const sensor_msgs::msg::PointCloud2::SharedPtr input)
{
    if (check_pointcloud == 0)
    {
        RCLCPP_INFO(this->get_logger(), "LiDAR_uturn node sub pointcloud");
        check_pointcloud = 1;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*input, *cloud);

    pcl::VoxelGrid<pcl::PointXYZI> VG;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    VG.setInputCloud(cloud);
    VG.setLeafSize(0.1f, 0.1f, 0.1f);
    VG.filter(*cloud_filtered);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::CropBox<pcl::PointXYZI> cropFilter;
    cropFilter.setInputCloud(cloud_filtered);

    //====================[ 수정 할 곳 ]====================  R O I 영 역

    cropFilter.setMin(Eigen::Vector4f(0, -3, -0.7, 0)); // x, y, z, i
    cropFilter.setMax(Eigen::Vector4f(15, 1, 1, 0));

    // cropFilter.setMin(Eigen::Vector4f(-0, -4, -1, 0)); // x, y, z, i
    // cropFilter.setMax(Eigen::Vector4f(30, 4, 1.0, 0));
    //=======================================================clustering=================

    cropFilter.filter(*cloud_filtered2);

    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(cloud_filtered2);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setInputCloud(cloud_filtered2);

    ec.setClusterTolerance(0.5); // 여기도 수정해야 함.
    ec.setMinClusterSize(3);
    // ec.setMaxClusterSize(1000);
    ec.setSearchMethod(tree);
    ec.extract(cluster_indices);

    //==========================================================point cloud 색깔 입히는 과정================
    int ii = 0;
    pcl::PointCloud<pcl::PointXYZI> TotalCloud;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZI>::Ptr>> clusters;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it, ++ii)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        {
            cluster->points.push_back(cloud_filtered2->points[*pit]);
            pcl::PointXYZI pt = cloud_filtered2->points[*pit];
            pcl::PointXYZI pt2;
            pt2.x = pt.x, pt2.y = pt.y, pt2.z = pt.z;
            pt2.intensity = (float)(ii + 1);
            TotalCloud.push_back(pt2);
        }
        cluster->width = cluster->size();
        cluster->height = 1;
        cluster->is_dense = true;
        clusters.push_back(cluster);
    }
    std_msgs::msg::Float64MultiArray cone_center;
    geometry_msgs::msg::Point center_point;

    for (int i = 0; i < clusters.size(); i++)
    {
        Eigen::Vector4f centroid, min_p, max_p;
        pcl::compute3DCentroid(*clusters[i], centroid);
        pcl::getMinMax3D(*clusters[i], min_p, max_p);
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

        float ob_size = sqrt(pow(max_point.x - min_point.x, 2) + pow(max_point.y - min_point.y, 2));
        float z_size = (max_point.z - min_point.z);
        if ((max_point.y - min_point.y) < 0.5 && (max_point.z - min_point.z) > 0.2 && max_point.x > 0.5)
        {
            cone_center.data.push_back(center_point.x);
            cone_center.data.push_back(center_point.y);
        }
    }
    /////////////////// < cone 중심을 나타내는 점 마커 > ///////////////
    visualization_msgs::msg::Marker marker;
    visualization_msgs::msg::MarkerArray markerArray;
    geometry_msgs::msg::Point p;
    marker.header.frame_id = "velodyne";
    marker.ns = "my_marker";
    marker.id = 9;
    marker.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.lifetime.nanosec = 100000000;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    p.x = center_point.x;
    p.y = center_point.y;
    // p.x = x_right;
    // p.y = y_right;
    marker.points.push_back(p);
    markerArray.markers.push_back(marker);
    marker_pub_->publish(markerArray);

    //////////////////////// < ROI를 나타내는 직육면체 마커 > //////////////////////////////////
    visualization_msgs::msg::Marker marker2;
    marker2.header.frame_id = "velodyne";
    marker2.ns = "my_roi";
    marker2.id = 0;
    marker2.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
    marker2.action = visualization_msgs::msg::Marker::ADD;
    marker2.type = visualization_msgs::msg::Marker::CUBE;
    marker2.scale.x = cropFilter.getMax()[0] - cropFilter.getMin()[0];
    marker2.scale.y = cropFilter.getMax()[1] - cropFilter.getMin()[1];
    marker2.scale.z = cropFilter.getMax()[2] - cropFilter.getMin()[2];
    marker2.pose.position.x = (cropFilter.getMax()[0] + cropFilter.getMin()[0]) / 2;
    marker2.pose.position.y = (cropFilter.getMax()[1] + cropFilter.getMin()[1]) / 2;
    marker2.pose.position.z = (cropFilter.getMax()[2] + cropFilter.getMin()[2]) / 2;
    marker2.color.r = 0.0;
    marker2.color.g = 1.0;
    marker2.color.b = 0.0;
    marker2.color.a = 0.3;
    marker_pub2_->publish(marker2);
    /////////////////////////////////////////////////////////////////////////////////////

    pub_center_->publish(cone_center);
    cone_center.data.clear();

    pcl::PCLPointCloud2 cloud_p;
    pcl::toPCLPointCloud2(TotalCloud, cloud_p);
    sensor_msgs::msg::PointCloud2 output;
    pcl_conversions::fromPCL(cloud_p, output);
    output.header.frame_id = "velodyne";
    pub_->publish(output);
}