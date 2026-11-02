#include "relative_2_utm/relative_2_UTM.hpp"

std::vector<double> calculateVectorSubtraction(const std::vector<double> &v1, const std::vector<double> &v2)
{
    std::vector<double> result(2);
    result[0] = v1[0] - v2[0];
    result[1] = v1[1] - v2[1];
    return result;
}

double calculateDistance(const std::vector<double> &v1, const std::vector<double> &v2)
{
    std::vector<double> diff = calculateVectorSubtraction(v1, v2);
    return sqrt(diff[0] * diff[0] + diff[1] * diff[1]);
}

Relative_2_UTM::Relative_2_UTM() : Node("relative_2_utm")
{
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliability(rclcpp::ReliabilityPolicy::BestEffort); // best_effort

    // sub
    heading_sub = this->create_subscription<std_msgs::msg::Float64>("/Local/heading", qos, std::bind(&Relative_2_UTM::heading_cb, this, std::placeholders::_1));       // 현재 차량 헤딩(라디안)
    car_utm_sub = this->create_subscription<geometry_msgs::msg::PointStamped>("/Local/utm", qos, std::bind(&Relative_2_UTM::car_UTM_cb, this, std::placeholders::_1)); // 현재 차량 UTM
    big_obstacle_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>("/LiDAR/bigcone_point", qos, std::bind(&Relative_2_UTM::big_object_cb, this, std::placeholders::_1));
    mission_sub = this->create_subscription<std_msgs::msg::Int16>("/Planning/mission", qos, std::bind(&Relative_2_UTM::mission_cb, this, std::placeholders::_1));
    obq_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>("/Vision/parking_points", qos, std::bind(&Relative_2_UTM::obq_cb, this, std::placeholders::_1));
    prl_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>("/LiDAR/parking_cen", qos, std::bind(&Relative_2_UTM::prl_cb, this, std::placeholders::_1));
    prl_edge_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>("/LiDAR/parking_edge_out", qos, std::bind(&Relative_2_UTM::prl_edge_cb, this, std::placeholders::_1));
    small_obstacle_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>("/LiDAR/object_cen", qos, std::bind(&Relative_2_UTM::small_object_cb, this, std::placeholders::_1));
    u_turn_point_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>("/LiDAR/center_point", qos, std::bind(&Relative_2_UTM::u_turn_point_cb, this, std::placeholders::_1));
    deli_flag_point_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>("/coord_xy", qos, std::bind(&Relative_2_UTM::deli_fusion_cb, this, std::placeholders::_1));

    // pub
    big_obstacle_UTM = this->create_publisher<std_msgs::msg::Float64MultiArray>("/Convert/big_object_UTM", qos);     // 장애물 절대좌표
    obq_obstacle_UTM = this->create_publisher<std_msgs::msg::Float64MultiArray>("/Convert/obq_points", qos);         // 장애물 절대좌표
    prl_point_UTM = this->create_publisher<std_msgs::msg::Float64MultiArray>("/Convert/prl_points", qos);            // 주차위치 절대좌표
    parking_edge_UTM = this->create_publisher<std_msgs::msg::Float64MultiArray>("/Convert/parking_edge", qos);       // 주차 4점 절대좌표
    small_obstacle_UTM = this->create_publisher<std_msgs::msg::Float64MultiArray>("/Convert/small_object_UTM", qos); // 장애물 절대좌표
    u_turn_point_UTM = this->create_publisher<std_msgs::msg::Float64MultiArray>("/Convert/u_turn_point", qos);       // 장애물 절대좌표
    deli_flag_point_UTM = this->create_publisher<std_msgs::msg::Float64MultiArray>("/Convert/deli_UTM", qos);        // 표지판 절대좌표

    fill_n(mission_state, 30, false);
    loop_duration = std::chrono::milliseconds(20);
}

void Relative_2_UTM::calc_relative_2_UTM()
{
    while (rclcpp::ok())
    {

        rclcpp::spin_some(shared_from_this());
        cout << "노드 실행" << endl;
        std::this_thread::sleep_for(loop_duration);
        // cout << check_cb_sub[0] << check_cb_sub[1] << endl;
        if (!check_cb_sub[0] || !check_cb_sub[1])
        {
            continue;
        } // waiting for utm and heading
        if (mission == 0)
        {
            continue;
        } // waiting for mission number
        if (!mission_state[mission])
        {
            continue;
        } // waiting for mission points
        switch (mission)
        {
        case 6:
            cout << "obq_points" << endl; // 사선주차
            obq_cal_obj_UTM(c_heading, inter_sensor_distanse, obj_cen, c_UTM);
            obq_point_UTM_Pub(obq_obstacle_UTM);
            fill_n(check_cb_sub, 2, false);
            break;

        case 7:
            cout << "deli_a" << endl; // 배달A
            deli_flag_point_cal_UTM(c_heading, inter_sensor_distanse, obj_cen, c_UTM);
            deli_flag_point_UTM_Pub(deli_flag_point_UTM);
            fill_n(check_cb_sub, 2, false);
            break;

        case 9:
            cout << "deli_b" << endl; // 배달B
            deli_flag_point_cal_UTM(c_heading, inter_sensor_distanse, obj_cen, c_UTM);
            deli_flag_point_UTM_Pub(deli_flag_point_UTM);
            fill_n(check_cb_sub, 2, false);
            break;

        case 11:
            cout << "small_obj" << endl; // 정적소형
            small_cal_obj_UTM(c_heading, inter_sensor_distanse, obj_cen, c_UTM);
            small_obstacle_UTM_Pub(small_obstacle_UTM);
            fill_n(check_cb_sub, 2, false);
            break;

        case 12:
            cout << "big_obj" << endl; // 정적대형
            big_cal_obj_UTM(c_heading, inter_sensor_distanse, obj_cen, c_UTM);
            big_obstacle_UTM_Pub(big_obstacle_UTM);
            fill_n(check_cb_sub, 2, false);
            break;

        case 13:
            cout << "prl_points" << endl; // 평행주차
            prl_cal_obj_UTM(c_heading, inter_sensor_distanse, obj_cen, c_UTM);
            prl_parking_UTM_Pub(prl_point_UTM);
            fill_n(check_cb_sub, 2, false);
            break;

        case 14:
            cout << "parking_edge" << endl; // 평행주차
            parking_edge_obj_UTM(c_heading, inter_sensor_distanse, obj_cen, c_UTM);
            parking_edge_UTM_Pub(parking_edge_UTM);
            fill_n(check_cb_sub, 2, false);
            break;

        case 16:
            cout << "u_turn" << endl; // 유턴
            u_turn_point_cal_UTM(c_heading, inter_sensor_distanse, obj_cen, c_UTM);
            u_turn_point_UTM_Pub(u_turn_point_UTM);
            fill_n(check_cb_sub, 2, false);
            break;

        default:
            cout << "no mission" << endl;
        }
        fill_n(mission_state, 30, false);
    }

    printf("finish\n");
    return;
}

void Relative_2_UTM::big_obstacle_UTM_Pub(std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> big_obstacle_UTM)
{
    std_msgs::msg::Float64MultiArray array;
    array.data.clear();

    for (int i = 0; i < obj_UTM.size(); i++)
    {
        array.data.push_back(obj_UTM[i][0]);
        array.data.push_back(obj_UTM[i][1]);
    }

    big_obstacle_UTM->publish(array);

    obj_UTM.clear();
}

void Relative_2_UTM::obq_point_UTM_Pub(std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> obq_obstacle_UTM)
{
    std_msgs::msg::Float64MultiArray array;
    array.data.clear();

    for (int i = 0; i < obj_UTM.size(); i++)
    {
        array.data.push_back(obj_UTM[i][0]);
        array.data.push_back(obj_UTM[i][1]);
    }

    obq_obstacle_UTM->publish(array);

    obj_UTM.clear();
}

void Relative_2_UTM::prl_parking_UTM_Pub(std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> prl_point_UTM)
{
    std_msgs::msg::Float64MultiArray array;
    array.data.clear();

    for (int i = 0; i < obj_UTM.size(); i++)
    {
        array.data.push_back(obj_UTM[i][0]);
        array.data.push_back(obj_UTM[i][1]);
    }

    prl_point_UTM->publish(array);

    obj_UTM.clear();
}

void Relative_2_UTM::parking_edge_UTM_Pub(std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> parking_edge_UTM)
{
    std_msgs::msg::Float64MultiArray array;
    array.data.clear();

    for (int i = 0; i < obj_UTM.size(); i++)
    {
        array.data.push_back(obj_UTM[i][0]);
        array.data.push_back(obj_UTM[i][1]);
    }

    parking_edge_UTM->publish(array);

    obj_UTM.clear();
}

void Relative_2_UTM::small_obstacle_UTM_Pub(std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> small_obstacle_UTM)
{
    std_msgs::msg::Float64MultiArray array;
    array.data.clear();

    for (int i = 0; i < obj_UTM.size(); i++)
    {
        array.data.push_back(obj_UTM[i][0]);
        array.data.push_back(obj_UTM[i][1]);
    }

    small_obstacle_UTM->publish(array);

    obj_UTM.clear();
}

void Relative_2_UTM::u_turn_point_UTM_Pub(std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> u_turn_point_UTM)
{
    std_msgs::msg::Float64MultiArray array;
    array.data.clear();

    for (int i = 0; i < obj_UTM.size(); i++)
    {
        array.data.push_back(obj_UTM[i][0]);
        array.data.push_back(obj_UTM[i][1]);
    }

    u_turn_point_UTM->publish(array);

    obj_UTM.clear();
}

void Relative_2_UTM::deli_flag_point_UTM_Pub(std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> deli_flag_point_UTM)
{
    std_msgs::msg::Float64MultiArray array;
    array.data.clear();

    for (int i = 0; i < obj_UTM.size(); i++)
    {
        array.data.push_back(obj_UTM[i][0]);
        array.data.push_back(obj_UTM[i][1]);
        array.data.push_back(obj_UTM[i][2]); // id
    }
    // cout << array << endl;
    deli_flag_point_UTM->publish(array);

    obj_UTM.clear();
}

void Relative_2_UTM::heading_cb(const std_msgs::msg::Float64::SharedPtr msg)
{
    c_heading = msg->data;
    // cout << "heading come" << endl;
    check_cb_sub[0] = true;
}

void Relative_2_UTM::car_UTM_cb(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    c_UTM[0] = msg->point.x;
    c_UTM[1] = msg->point.y;
    // cout << "UTM come" << endl;
    check_cb_sub[1] = true;
}

void Relative_2_UTM::big_object_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    int cluster_count = 0;
    obj_cen.clear();
    vector<double> tmp_obj_cen;

    for (int i = 0; i < msg->data.size(); i++)
    {
        if (i % 2 == 0)
        { // x

            tmp_obj_cen.push_back(msg->data[i]);
        }
        else
        { // y
            tmp_obj_cen.push_back(msg->data[i]);
            obj_cen.push_back(tmp_obj_cen);
            tmp_obj_cen.clear();
        }
    }
    // if(msg->data[0] == 777.0){
    //     mission_state[12] = false;
    // }
    // else{
    //     mission_state[12] = true;
    // }
    mission_state[12] = true;
}

void Relative_2_UTM::obq_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    int cluster_count = 0;
    obj_cen.clear();
    vector<double> tmp_obj_cen;

    for (int i = 0; i < msg->data.size(); i++)
    {
        if (i % 2 == 0)
        { // x

            tmp_obj_cen.push_back(msg->data[i]);
        }
        else
        { // y
            tmp_obj_cen.push_back(msg->data[i]);
            obj_cen.push_back(tmp_obj_cen);
            tmp_obj_cen.clear();
        }
    }
    mission_state[6] = true;
}

void Relative_2_UTM::prl_edge_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    int cluster_count = 0;
    obj_cen.clear();
    vector<double> tmp_obj_cen;

    for (int i = 0; i < msg->data.size(); i++)
    {
        if (i % 2 == 0)
        { // x

            tmp_obj_cen.push_back(msg->data[i]);
        }
        else
        { // y
            tmp_obj_cen.push_back(msg->data[i]);
            obj_cen.push_back(tmp_obj_cen);
            tmp_obj_cen.clear();
        }
    }
    mission_state[14] = true;
}

void Relative_2_UTM::prl_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    int cluster_count = 0;
    obj_cen.clear();
    vector<double> tmp_obj_cen;

    for (int i = 0; i < msg->data.size(); i++)
    {
        if (i % 2 == 0)
        { // x

            tmp_obj_cen.push_back(msg->data[i]);
        }
        else
        { // y
            tmp_obj_cen.push_back(msg->data[i]);
            obj_cen.push_back(tmp_obj_cen);
            tmp_obj_cen.clear();
        }
    }
    mission_state[13] = true;
}

void Relative_2_UTM::small_object_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    int cluster_count = 0;
    obj_cen.clear();
    vector<double> tmp_obj_cen;

    for (int i = 0; i < msg->data.size(); i++)
    {
        if (i % 2 == 0)
        { // x

            tmp_obj_cen.push_back(msg->data[i]);
        }
        else
        { // y
            tmp_obj_cen.push_back(msg->data[i]);
            obj_cen.push_back(tmp_obj_cen);
            tmp_obj_cen.clear();
        }
    }
    mission_state[11] = true;
}

void Relative_2_UTM::u_turn_point_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    int cluster_count = 0;
    obj_cen.clear();
    vector<double> tmp_obj_cen;

    for (int i = 0; i < msg->data.size(); i++)
    {
        if (i % 2 == 0)
        { // x

            tmp_obj_cen.push_back(msg->data[i]);
        }
        else
        { // y
            tmp_obj_cen.push_back(msg->data[i]);
            obj_cen.push_back(tmp_obj_cen);
            tmp_obj_cen.clear();
        }
    }
    mission_state[16] = true;
}

void Relative_2_UTM::deli_fusion_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    obj_cen.clear();
    vector<double> tmp_obj_cen;
    cout << 'gogogo' << endl;
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
    mission_state[7] = true; // 배달을 두개로 나누어야 하는지 고민
    mission_state[9] = true;
}

void Relative_2_UTM::big_cal_obj_UTM(float c_heading, double sensor_distanse[2], vector<vector<double>> object_cen, double utm_gps[2])
{
    cout << c_heading << endl;

    for (int i = 0; i < object_cen.size(); i++)
    {
        double x = sensor_distanse[0] + object_cen[i][0];
        double y = sensor_distanse[1] + object_cen[i][1];

        double tmp_x = x * cos(c_heading) - y * sin(c_heading);
        double tmp_y = x * sin(c_heading) + y * cos(c_heading);

        vector<double> tmp_UTM;
        tmp_UTM.push_back(tmp_x + c_UTM[0]);
        tmp_UTM.push_back(tmp_y + c_UTM[1]);
        obj_UTM.push_back(tmp_UTM);

        cout << x << ", " << y << endl;
    }
    cout << "------------------ " << endl
         << endl;
}

void Relative_2_UTM::obq_cal_obj_UTM(float c_heading, double sensor_distanse[2], vector<vector<double>> object_cen, double utm_gps[2])
{
    cout << c_heading << endl;

    for (int i = 0; i < object_cen.size(); i++)
    {
        double x = sensor_distanse[0] + object_cen[i][0];
        double y = sensor_distanse[1] + object_cen[i][1];

        double tmp_x = x * cos(c_heading) - y * sin(c_heading);
        double tmp_y = x * sin(c_heading) + y * cos(c_heading);

        vector<double> tmp_UTM;
        tmp_UTM.push_back(tmp_x + c_UTM[0]);
        tmp_UTM.push_back(tmp_y + c_UTM[1]);
        obj_UTM.push_back(tmp_UTM);

        cout << x << ", " << y << endl;
    }
    cout << "------------------ " << endl
         << endl;
}

void Relative_2_UTM::prl_cal_obj_UTM(float c_heading, double sensor_distanse[2], vector<vector<double>> object_cen, double utm_gps[2])
{
    cout << c_heading << endl;

    for (int i = 0; i < object_cen.size(); i++)
    {
        double x = sensor_distanse[0] + object_cen[i][0];
        double y = sensor_distanse[1] + object_cen[i][1];

        double tmp_x = x * cos(c_heading) - y * sin(c_heading);
        double tmp_y = x * sin(c_heading) + y * cos(c_heading);

        vector<double> tmp_UTM;
        tmp_UTM.push_back(tmp_x + c_UTM[0]);
        tmp_UTM.push_back(tmp_y + c_UTM[1]);
        obj_UTM.push_back(tmp_UTM);

        cout << x << ", " << y << endl;
    }
    cout << "------------------ " << endl
         << endl;
}

void Relative_2_UTM::parking_edge_obj_UTM(float c_heading, double sensor_distanse[2], vector<vector<double>> object_cen, double utm_gps[2])
{
    cout << c_heading << endl;

    for (int i = 0; i < object_cen.size(); i++)
    {
        double x = sensor_distanse[0] + object_cen[i][0];
        double y = sensor_distanse[1] + object_cen[i][1];

        double tmp_x = x * cos(c_heading) - y * sin(c_heading);
        double tmp_y = x * sin(c_heading) + y * cos(c_heading);

        vector<double> tmp_UTM;
        tmp_UTM.push_back(tmp_x + c_UTM[0]);
        tmp_UTM.push_back(tmp_y + c_UTM[1]);
        obj_UTM.push_back(tmp_UTM);

        cout << x << ", " << y << endl;
    }
    cout << "------------------ " << endl
         << endl;
}

void Relative_2_UTM::small_cal_obj_UTM(float c_heading, double sensor_distanse[2], vector<vector<double>> object_cen, double utm_gps[2])
{
    cout << c_heading << endl;

    for (int i = 0; i < object_cen.size(); i++)
    {
        double x = sensor_distanse[0] + object_cen[i][0];
        double y = sensor_distanse[1] + object_cen[i][1];

        double tmp_x = x * cos(c_heading) - y * sin(c_heading);
        double tmp_y = x * sin(c_heading) + y * cos(c_heading);

        vector<double> tmp_UTM;
        tmp_UTM.push_back(tmp_x + c_UTM[0]);
        tmp_UTM.push_back(tmp_y + c_UTM[1]);
        obj_UTM.push_back(tmp_UTM);

        cout << x << ", " << y << endl;
    }
    cout << "------------------ " << endl
         << endl;
}

void Relative_2_UTM::u_turn_point_cal_UTM(float c_heading, double sensor_distanse[2], vector<vector<double>> object_cen, double utm_gps[2])
{
    cout << c_heading << endl;

    for (int i = 0; i < object_cen.size(); i++)
    {
        double x = sensor_distanse[0] + object_cen[i][0];
        double y = sensor_distanse[1] + object_cen[i][1];

        double tmp_x = x * cos(c_heading) - y * sin(c_heading);
        double tmp_y = x * sin(c_heading) + y * cos(c_heading);

        vector<double> tmp_UTM;
        tmp_UTM.push_back(tmp_x + c_UTM[0]);
        tmp_UTM.push_back(tmp_y + c_UTM[1]);
        obj_UTM.push_back(tmp_UTM);

        cout << x << ", " << y << endl;
        cout << tmp_UTM[0] << tmp_UTM[1] << endl;
    }
    cout << "------------------ " << endl
         << endl;
}

void Relative_2_UTM::deli_flag_point_cal_UTM(float c_heading, double sensor_distanse[2], vector<vector<double>> object_cen, double utm_gps[2])
{
    cout << c_heading << endl;

    for (int i = 0; i < object_cen.size(); i++)
    {
        double x = sensor_distanse[0] + object_cen[i][0];
        double y = sensor_distanse[1] + object_cen[i][1];
        double id = object_cen[i][2];

        double tmp_x = x * cos(c_heading) - y * sin(c_heading);
        double tmp_y = x * sin(c_heading) + y * cos(c_heading);

        vector<double> tmp_UTM;
        tmp_UTM.push_back(tmp_x + c_UTM[0]);
        tmp_UTM.push_back(tmp_y + c_UTM[1]);
        tmp_UTM.push_back(id);
        obj_UTM.push_back(tmp_UTM);

        cout << x << ", " << y << endl;
    }
    cout << "------------------ " << endl
         << endl;
}

// 미션 번호 callback
void Relative_2_UTM::mission_cb(const std_msgs::msg::Int16::SharedPtr msg)
{
    mission = msg->data;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Relative_2_UTM>();
    node->calc_relative_2_UTM();

    rclcpp::shutdown();
    return 0;
}
