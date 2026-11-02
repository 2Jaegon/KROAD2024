// #ifdef GPS_JAM_HPP_
// #define GPS_JAM_HPP_

// #include <iostream>
// #include "rclcpp/rclcpp.hpp"
// #include "sensor_msgs/msg/nav_sat_fix.hpp"
// #include "utility_function.hpp"

// /////////////////////////////////////////////////////////////////////////
// //                             미 완 성                                 //
// ////////////////////////////////////////////////////////////////////////

// class GPSJammer
// {
// public:
//     GPSJammer() : Node("gps_jammer")
//     {
//     }

//     void gpsJamming(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
//     {
//         // 변조된 NavSatFix 메시지 생성
//         auto jammed_msg = *msg;

//         // position_covariance 변조
//         for (auto &value : jammed_msg.position_covariance)
//         {
//             value += randomNumCreator(-70.0, 70.0); // 모든 값을 높은 값으로 변경 (예시)
//         }

//         // 변조된 메시지 퍼블리시
//         publisher_->publish(jammed_msg);
//     }
// };

// #endif