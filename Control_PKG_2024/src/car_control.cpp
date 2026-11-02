#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int16.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "control/callback_data_manage.hpp"
#include "control/PIDController.hpp"
#include "control/lat_control.hpp"
#include "control/lon_control.hpp"
#include "control/mission_param.hpp"

using namespace std;

bool IS_PRINT = true;

float wheel_adaptation(float Yawrate_rps, float delta_offset_pre)
{

    float delta_offset = delta_offset_pre;

    if (Yawrate_rps > 0.01)
    {                                             // need tune (0.01rad)
        delta_offset = delta_offset_pre - 0.0001; // need tune (plus -0.0001rad)
    }
    else if (Yawrate_rps < -0.01)
    {                                             // need tune (-0.01rad)
        delta_offset = delta_offset_pre + 0.0001; // need tune (plus 0.0001rad)
    }
    return delta_offset;
}

class ERPControl : public rclcpp::Node
{
private:
    std::shared_ptr<CallbackClass> callback_data_ptr;
    CallbackClass *cb_data;

    std::shared_ptr<CombinedSteer> lat_control_ptr;
    CombinedSteer *lat_control;
    std::shared_ptr<ReversePurePursuit> R_lat_control_ptr;
    ReversePurePursuit *R_lat_control;

    std::shared_ptr<LonController> lon_control_ptr;
    LonController *lon_control;

    std::shared_ptr<ControlGainTuning> param_manage_ptr;
    ControlGainTuning *param_manage;

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr lo_odom_sub;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr lo_curr_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr lo_imu__sub;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr lo_beta_sub;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr pl_loca_sub;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr pl_miss_sub;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr pl_cont_sub;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr pl_pyaw_sub;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr pl_pyaws_sub;

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr er_data_sub;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr ERP_data_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr tmp_data_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr Rel_path_pub;

    rclcpp::TimerBase::SharedPtr timer_;

    float pre_offset_delta = 0;
    GasAndBrake acc_val;

public:
    ERPControl()
        : Node("erp_control")
    {
        ////////////////////////////////////////        Qos          ////////////////////////////////////////
        // auto qos = rclcpp::QoS(rclcpp::KeepLast(10)); // reliable
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliability(rclcpp::ReliabilityPolicy::BestEffort); // best_effort
        ////////////////////////////////////////        Qos          ////////////////////////////////////////

        ////////////////////////////////////////   클레스들 인스턴스화   ////////////////////////////////////////
        callback_data_ptr = std::make_shared<CallbackClass>();
        cb_data = callback_data_ptr.get();

        lat_control_ptr = std::make_shared<CombinedSteer>(cb_data);
        lat_control = lat_control_ptr.get();
        R_lat_control_ptr = std::make_shared<ReversePurePursuit>();
        R_lat_control = R_lat_control_ptr.get();

        lon_control_ptr = std::make_shared<LonController>();
        lon_control = lon_control_ptr.get();

        param_manage_ptr = std::make_shared<ControlGainTuning>(cb_data, lat_control, R_lat_control, lon_control);
        param_manage = param_manage_ptr.get();
        ////////////////////////////////////////   클레스들 인스턴스화   ////////////////////////////////////////

        ////////////////////////////////////////   <SUBSCRIBER>   ////////////////////////////////////////
        // Local
        lo_odom_sub = this->create_subscription<geometry_msgs::msg::PointStamped>("/Local/utm", qos, std::bind(&CallbackClass::lo_odom_cb, cb_data, std::placeholders::_1));
        lo_curr_sub = this->create_subscription<std_msgs::msg::Float64>("/Local/heading", qos, std::bind(&CallbackClass::lo_yaw_cb, cb_data, std::placeholders::_1));
        lo_imu__sub = this->create_subscription<sensor_msgs::msg::Imu>("/imu", qos, std::bind(&CallbackClass::lo_imu_cb, cb_data, std::placeholders::_1));
        lo_beta_sub = this->create_subscription<std_msgs::msg::Float64>("/beta", qos, std::bind(&CallbackClass::lo_beta_cb, cb_data, std::placeholders::_1));

        // Planning
        pl_loca_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>("/Planning/local_path", qos, std::bind(&CallbackClass::pl_local_path_cb, cb_data, std::placeholders::_1));
        pl_miss_sub = this->create_subscription<std_msgs::msg::Int16>("/Planning/mission", qos, std::bind(&CallbackClass::pl_mission_num_cb, cb_data, std::placeholders::_1));
        pl_cont_sub = this->create_subscription<std_msgs::msg::Float64>("/Planning/target_velocity", qos, std::bind(&CallbackClass::pl_control_switch_cb, cb_data, std::placeholders::_1));
        pl_pyaws_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>("/Planning/path_yaw", qos, std::bind(&CallbackClass::pl_local_path_yaws_cb, cb_data, std::placeholders::_1));

        // ERP Serial Data
        er_data_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>("/ERP/serial_data", qos, std::bind(&CallbackClass::co_ERP_data_cb, cb_data, std::placeholders::_1));
        ////////////////////////////////////////   <SUBSCRIBER>   ////////////////////////////////////////

        ////////////////////////////////////////   <PUBLISHER>  ////////////////////////////////////////
        ERP_data_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>("/Control/serial_data", 1);
        tmp_data_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/Control/tmp_plot_val", 1);
        Rel_path_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/Control/rel_path", 1);
        ////////////////////////////////////////   <PUBLISHER>  ////////////////////////////////////////

        timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
                                         std::bind(&ERPControl::timer_callback, this));
    }

    void timer_callback()
    {
        // 가독성을 위해 새로운 변수로 저장
        float curr_speed = cb_data->get_speed();
        float yaw = cb_data->get_yaw();
        Point odom = cb_data->get_odom();
        double lat_error = cb_data->calc_n_get_lat_error();
        double yaw_rate = cb_data->get_yawrate();
        double pd_path_yaw = cb_data->get_pd_path_yaw();
        double path_curature = 0.0, curv_1 = 0.0, curv_2 = 0.0;

        // lat_error가 404면 뭔가 잘못됨.
        if (lat_error == 404 || pd_path_yaw == 404)
        {
            // // pub E-STOP
            // auto ERP_data_msg = std::make_shared<std_msgs::msg::Float32MultiArray>();
            // ERP_data_msg->data = {1, 1, 0, 0, 0, 0, 0};
            // ERP_data_pub->publish(*ERP_data_msg);
            cout << "404!!!" << endl;
            return;
        }

        ////////////////////////////////////////   계산   ////////////////////////////////////////
        ////////////////////////////////////////   계산   ////////////////////////////////////////
        ////////////////////////////////////////   계산   ////////////////////////////////////////

        int changed_param_num = param_manage->set_mission_param();

        // Control Switch
        double cs = cb_data->get_control_switch();
        float steer = 0.0;
        int gear_state = 0;

        // 횡방향 계산
        if (cs < 0) // 후진
        {
            gear_state = 2;
            R_lat_control->set_R_PP_data(curr_speed, yaw, odom, cb_data->get_relative_path());
            steer = R_lat_control->calc_R_PP_steer();
        }
        else // 전진
        {
            lat_control->set_stanly_data(curr_speed, pd_path_yaw, yaw, lat_error);
            lat_control->set_pp_data(curr_speed, yaw, odom, cb_data->get_relative_path());
            steer = lat_control->calc_combined_steer();
        }

        // 종방향 계산
        if (cb_data->get_mission_num() == 998)
        {
            // 협로 첫 바퀴에는 곡률 데이터를 못믿는다.
            // 협로 두 번째 바퀴에서만 곡률에 따른 속도제어를 한다.
            curv_1 = cb_data->calc_path_curvature(0.5, 1.5); // 속도 제어시 제일 먼저 곡률을 계산해야함.
            curv_2 = cb_data->calc_path_curvature(0.1, 1.5); // S 커브와 같은 구간에서 불필요한 가속을 막기 위해
            path_curature = max(curv_1, curv_2);
            param_manage->target_speed_reducing_by_curature(path_curature);
        }

        param_manage->target_speed_reducing_by_steer(steer);

        if (cs < 0)
        {
            lon_control->set_lon_data(curr_speed);
            lon_control->set_lon_target_speed(1.5);
        }
        else if (cs == 0)
        {
            acc_val.gas = 0.0;
            acc_val.brake = 180;
        }
        else if (0.0 < cs && cs <= 6.9)
        {
            lon_control->set_lon_data(curr_speed);
            lon_control->set_lon_target_speed(cs);
        }
        else
        {
            cout << "pl_control_switch error" << endl;
            lon_control->set_lon_data(curr_speed);
            lon_control->set_lon_target_speed(1.0);
        }

        if (cb_data->get_serial_sub_flag() && cs != 0)
        {
            acc_val = lon_control->calc_gas_n_brake();
            cb_data->set_down_serial_sub_flag();
        }

        if (cb_data->get_mission_num() == 998)
        {
            acc_val.brake = clip(acc_val.brake, 50.0, 100.0);
        }

        // ---------------------------------------------------------- </계산>
        // <ros topic pub> -------------------------------------------------

        // 제어 값 pub
        auto ERP_data_msg = std::make_shared<std_msgs::msg::Float32MultiArray>();

        ERP_data_msg->data.push_back(1.0);           // 0 | control_mode
        ERP_data_msg->data.push_back(0.0);           // 1 | e_stop
        ERP_data_msg->data.push_back(gear_state);    // 2 | gear
        ERP_data_msg->data.push_back(acc_val.gas);   // 3 | speed
        ERP_data_msg->data.push_back(steer);         // 4 | steer
        ERP_data_msg->data.push_back(acc_val.brake); // 5 | brake
        ERP_data_msg->data.push_back(0.0);           // 6 | enc

        ERP_data_pub->publish(*ERP_data_msg);

        // 디버그용 토픽
        auto tmp_plot_val_msg = std::make_shared<std_msgs::msg::Float64MultiArray>();
        // tmp_plot_val_msg->data.push_back(R_lat_control->get_reverse_yaw()); // 0
        // tmp_plot_val_msg->data.push_back(lat_control->get_LA_distance());   // 1
        tmp_plot_val_msg->data.push_back(lat_error);                       // 0
        tmp_plot_val_msg->data.push_back(lon_control->get_target_speed()); // 1
        tmp_plot_val_msg->data.push_back(curv_1);                          // 2
        tmp_plot_val_msg->data.push_back(curv_2);                          // 3
        tmp_plot_val_msg->data.push_back(path_curature);                   // 4

        vector<double> preview_steers = lat_control->get_preview_steers();
        for (size_t i = 0; i < preview_steers.size(); i++)
        {
            tmp_plot_val_msg->data.push_back(preview_steers[i]); //
        }
        tmp_plot_val_msg->data.push_back(lat_control->get_PP_steer()); //

        vector<Point> predic_p = cb_data->get_stanley_predict_pos();
        for (size_t i = 0; i < predic_p.size(); i++)
        {
            Point tmp_p = cb_data->AxisTrans_rel2abs(predic_p[i]);
            tmp_plot_val_msg->data.push_back(tmp_p.x); //
            tmp_plot_val_msg->data.push_back(tmp_p.y); //
        }
        tmp_plot_val_msg->data.push_back(lat_control->get_stanley_integral_val());
        tmp_plot_val_msg->data.push_back(cb_data->get_speed());

        tmp_data_pub->publish(*tmp_plot_val_msg);

        // 상대경로를 plot을 위해 pub 하는 구문
        if (false)
        {
            vector<Point> Rel_path = cb_data->get_relative_path();

            auto Rel_path_msg = std::make_shared<std_msgs::msg::Float64MultiArray>();

            for (size_t i = 1; i < Rel_path.size(); i++)
            {
                Rel_path_msg->data.push_back(Rel_path[i].x);
                Rel_path_msg->data.push_back(Rel_path[i].y);
            }

            Rel_path_pub->publish(*Rel_path_msg);
        }

        // ------------------------------------------------ </ros topic pub>
        // <print> ---------------------------------------------------------

        if (IS_PRINT)
        {
            cout << "==================" << endl;
            cout << "changed_param_num : " << changed_param_num << endl;
            cout << "gas : " << acc_val.gas << endl;
            cout << "brake : " << acc_val.brake << endl;
            cout << "steer : " << steer << endl
                 << endl;

            cout << "yawrate : " << yaw_rate << endl;
            cout << "PP_steer : " << lat_control->get_PP_steer() * 180 / M_PI << " deg (" << lat_control->get_PP_steer() << " rad)" << endl;
            cout << "stanly_steer: " << lat_control->get_stanly_steer() * 180 / M_PI << "deg (" << lat_control->get_stanly_steer() << "rad)" << endl;
            cout << "curr_speed : " << curr_speed * 3.6 << "km/h (" << curr_speed << "m/s" << endl;
            cout << "yaw : " << yaw << endl;
            cout << "lat_error : " << lat_error << endl;
            cout << "LD distance : " << lat_control->get_LA_distance() << endl;
            cout << "target speed : " << lon_control->get_target_speed() * 3.6 << "km/h (" << lon_control->get_target_speed() << "m/s" << endl;
            cout << "path_yaw_nearest(rad) : " << pd_path_yaw << endl;
        }

        // ------------------------------------------------------- </print>
        return;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ERPControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}