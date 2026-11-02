#include "narrow_2_utm/narrow_2_UTM.hpp"

Relative_2_UTM::Relative_2_UTM() : Node("relative_2_utm")
{
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliability(rclcpp::ReliabilityPolicy::BestEffort); // best_effort

    // sub
    heading_sub = this->create_subscription<std_msgs::msg::Float64>("/Local/heading", qos, std::bind(&Relative_2_UTM::heading_cb, this, std::placeholders::_1));                              // 현재 차량 헤딩(라디안)
    car_utm_sub = this->create_subscription<geometry_msgs::msg::PointStamped>("/Local/utm", qos, std::bind(&Relative_2_UTM::car_UTM_cb, this, std::placeholders::_1));                        // 현재 차량 UTM
    narrow_obstacle_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>("/LiDAR/center_color", qos, std::bind(&Relative_2_UTM::narrow_object_cb, this, std::placeholders::_1)); // 장애물 상대좌표
    // pub
    narrow_obstacle_UTM = this->create_publisher<std_msgs::msg::Float64MultiArray>("/Planning/narrow_object_UTM", qos); // 장애물 절대좌표
    fill_n(check_cb_sub, 2, false);
    auto timer_callback = [this]() -> void
    { calc_relative_2_UTM(); };
    timer_ = create_wall_timer(10ms, timer_callback); // 10hz
}

void Relative_2_UTM::calc_relative_2_UTM()
{

    if (!check_cb_sub[0] || !check_cb_sub[1])
    {
        cout << "local 안들어옴" << endl;
        return;
    }
    if (obj_cen.size() == 0)
    {
        return;
    }
    for (int i = 0; i < obj_cen.size(); i++)
    {
        x = inter_sensor_distanse[0] + obj_cen[i][0];
        y = inter_sensor_distanse[1] + obj_cen[i][1];
        c = obj_cen[i][2];

        tmp_x = x * cos(c_heading) - y * sin(c_heading);
        tmp_y = x * sin(c_heading) + y * cos(c_heading);

        tmp_UTM.push_back(tmp_x + c_UTM[0]);
        tmp_UTM.push_back(tmp_y + c_UTM[1]);
        tmp_UTM.push_back(c);
        obj_UTM.push_back(tmp_UTM);

        tmp_UTM.clear();
    }

    array.data.clear();

    for (int i = 0; i < obj_UTM.size(); i++)
    {
        array.data.push_back(obj_UTM[i][0]);
        array.data.push_back(obj_UTM[i][1]);
        array.data.push_back(obj_UTM[i][2]);
    }
    narrow_obstacle_UTM->publish(array);
    obj_cen.clear();
    obj_UTM.clear();
}

void Relative_2_UTM::heading_cb(const std_msgs::msg::Float64::SharedPtr msg)
{
    c_heading = msg->data;
    check_cb_sub[0] = true;
}

void Relative_2_UTM::car_UTM_cb(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    c_UTM[0] = msg->point.x;
    c_UTM[1] = msg->point.y;
    check_cb_sub[1] = true;
}

void Relative_2_UTM::narrow_object_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    int cluster_count = 0;
    obj_cen.clear();
    tmp_obj_cen;
    if (msg->data.size() == 0)
    {
        return;
    }

    for (int i = 0; i < msg->data.size(); i++)
    {
        if (i % 3 != 2)
        { // x,y
            tmp_obj_cen.push_back(msg->data[i]);
        }
        else
        { // color : 2, 5, 8, 11 ---
            tmp_obj_cen.push_back(msg->data[i]);
            obj_cen.push_back(tmp_obj_cen);
            tmp_obj_cen.clear();
        }
    }
    tmp_obj_cen.clear();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Relative_2_UTM>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
