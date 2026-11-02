#include "lidar/LiDAR_tunnel.hpp"

LiDAR_tunnel::LiDAR_tunnel()
    : Mission("LiDAR_tunnel")
{
    pub_s = this->create_publisher<sensor_msgs::msg::PointCloud2>("/Cluster/small_static_tunnel", lidar::best_effort_qos());                      // 정적 클러스터
    object_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/LiDAR/object_cen", lidar::best_effort_qos());                            // 정적 중점
    marker_array_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/Marker/small_static_tunnel_object", lidar::best_effort_qos()); // 정적 마커
    pub_d = this->create_publisher<sensor_msgs::msg::PointCloud2>("/Cluster/dynamic_tunnel", lidar::best_effort_qos());                           // 동적 클러스터
    dynamic_stop_ = this->create_publisher<std_msgs::msg::Bool>("/LiDAR/dynamic_stop", lidar::best_effort_qos());                                 // 플래닝에게 줄 동적 stop 신호
    dynamic_mark_ = this->create_publisher<visualization_msgs::msg::Marker>("/Marker/dynamic_tunnel_object", lidar::best_effort_qos());           // 동적 마커
    velodyne_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/Cluster/tunnel_downsampling", lidar::best_effort_qos());

    bool_mission_flag = this->create_subscription<std_msgs::msg::Bool>(
        "/Planning/is_dynamic", lidar::best_effort_qos(), // 동적 미션 불리언 신호
        std::bind(&LiDAR_tunnel::bool_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "LiDAR_tunnel is ready");
}

void LiDAR_tunnel::bool_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    mission_bool = msg->data; // mission_bool 변수를 설정
    // RCLCPP_INFO(this->get_logger(), "boolean flag get! Value: %s", mission_bool ? "true" : "false");
}

int LiDAR_tunnel::get_status_code()
{
    return 15;
}

void LiDAR_tunnel::callback(const sensor_msgs::msg::PointCloud2::SharedPtr input)
{
    if (check_pointcloud == 0)
    {
        RCLCPP_INFO(this->get_logger(), "LiDAR_tunnel node sub pointcloud");
        check_pointcloud = 1;
    }
    // RCLCPP_INFO(this->get_logger(), "IT is RRRRRRRRRUNing!!!");

    // RCLCPP_INFO(this->get_logger(), "Running : STATIC");
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

    if (mission_bool)
    {
        cropFilter.setMin(Eigen::Vector4f(0, -1, -0.4, 0)); // 수평 라이다
        cropFilter.setMax(Eigen::Vector4f(11, 1, 1.5, 0));
    }
    else
    {
        cropFilter.setMin(Eigen::Vector4f(0, -2, -0.4, 0)); // 수평 라이다
        cropFilter.setMax(Eigen::Vector4f(11, 4, 1.5, 0));
    }

    //=======================================================clustering=================

    cropFilter.filter(*cloud_filtered2);

    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(cloud_filtered2);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setInputCloud(cloud_filtered2);

    ec.setClusterTolerance(0.5);
    ec.setMinClusterSize(3);
    ec.setMaxClusterSize(2000);
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
        // std::cout << cluster->size() << std::endl;
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

    std_msgs::msg::Float64MultiArray obj;
    std_msgs::msg::Float32MultiArray p;

    if (mission_bool)
    {
        RCLCPP_INFO(this->get_logger(), "Running Dynamic");
        auto msg = std_msgs::msg::Bool();
        msg.data = false;
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

            float cen_x = center_point.x;
            float cen_y = center_point.y;

            float distance = sqrt(pow(center_point.x, 2) + pow(center_point.y, 2));

            if ((max_point.z - min_point.z) >= 0.35 && distance > 3)
            {
                std::cout << "distance : " << distance << std::endl;
                std::cout << "max : " << max_point.z << " min : " << min_point.z << std::endl;
                std::cout << "object 찾았다!!!!!!!!!!!!!!!!!!" << std::endl;
                p.data.push_back(cen_x);
                p.data.push_back(cen_y);

                geometry_msgs::msg::Point obj1;
                visualization_msgs::msg::Marker marker1;
                marker1.ns = "points_and_lines";
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
                dynamic_mark_->publish(marker1);
                if (distance < 8.0)
                {
                    std::cout << "영역 내 object 검출!!!!!!!!!!!!!!!!!!!!!" << std::endl;
                    msg.data = true;
                }
            }
        }
        p.data.clear();
        dynamic_stop_->publish(msg);
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Running Static");
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

            float cen_x = center_point.x;
            float cen_y = center_point.y;

            if ((max_point.y - min_point.y) < 1.5 && (max_point.y - min_point.y) > 0.5 && max_point.z < 1.2)
            {

                std::cout << "max : " << max_point.z << " min : " << min_point.z << std::endl;
                obj.data.push_back(cen_x);
                obj.data.push_back(cen_y);
                p.data.push_back(cen_x);
                p.data.push_back(cen_y);

                p.data.push_back(-1000);
                visualization_msgs::msg::MarkerArray p_array;

                geometry_msgs::msg::Point obj1;
                visualization_msgs::msg::Marker marker1;
                marker1.ns = "points_and_lines";
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
                marker2.ns = "points_and_lines";
                marker2.action = visualization_msgs::msg::Marker::ADD;
                marker2.type = visualization_msgs::msg::Marker::POINTS;
                marker2.id = 1;
                marker2.pose.orientation.w = 1.0;
                marker2.scale.x = 0.5;
                marker2.scale.y = 0.5;
                marker2.color.a = 1.0;
                marker2.color.g = 1.0f;
                obj2.x = p.data[2];
                obj2.y = p.data[3];
                obj2.z = 0.0;
                marker2.points.push_back(obj2);
                marker2.header.frame_id = "velodyne";
                p_array.markers.push_back(marker2);

                marker_array_->publish(p_array);
                p.data.clear();

                pcl::PCLPointCloud2 cloud_p;
                pcl::toPCLPointCloud2(TotalCloud, cloud_p);
                sensor_msgs::msg::PointCloud2 output;
                pcl_conversions::fromPCL(cloud_p, output);
                output.header.frame_id = "velodyne";
                pub_s->publish(output);
            }
        }
        obj.data.push_back(-1000);
        object_->publish(obj);
        obj.data.clear();
    }
}