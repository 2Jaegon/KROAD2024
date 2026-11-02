#include "mission.hpp"

class LiDAR_uturn : public Mission
{
public:
    LiDAR_uturn();

private:
    // func
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr input) override;
    int get_status_code() override;

    // pub
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_center_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub2_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    // var
    int check_pointcloud = 0;
};