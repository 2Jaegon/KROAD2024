/***
 * 작성자: 박용현
 * 설명: Lidar 클래스 헤더파일
 * Style
 * -클래스: PascalCase
 * -멤버변수: snake_case_   (snake_case 뒤에 밑줄)(파이썬 사용 x)
 * -함수: camelCase
 * -변수: snake_case
 * -상수: kPascalCase   (PascalCase앞에 k 접두어)
 ***/

#ifndef LIDAR_H_
#define LIDAR_H_

#include <vector>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

#include "utility_function.hpp"
#include "static_clusters.hpp"
#include "delaunay_classifier.hpp"
#include "dynamic_clusters.hpp"

class Lidar
{
private:
    // 협로
    DynamicClassCluster traffic_cone_cluster_;

public:
    /* 생성자 */
    // 클러스터링하는 기준 epsilon, min_point_size, max_point_size 조정 필요!!!!
    Lidar(nlohmann::json &param) : traffic_cone_cluster_(param["traffic_cone_eps"], param["traffic_cone_min"], param["traffic_cone_max"])
    {
    }

    /* getter 함수 */
    const std::vector<std::vector<double>> &getRefTrafficCone()
    {
        return traffic_cone_cluster_.getRefClusterTrustPoints();
    }

    /* callback 함수 */
    void lidarTrafficConeCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) // traffic_cone
    {
        for (size_t i = 0; i < msg->data.size(); i += 3)
        {
            std::vector<double> point(3, 0);
            point[0] = msg->data[i];
            point[1] = msg->data[i + 1];
            point[2] = msg->data[i + 2];
            if (point[2] >= 1 && point[2] <= 2) // 1: blue, 2: yellow
            {
                traffic_cone_cluster_.update(point);
            }
        }
    }

    /* 기능 관련 함수 */
    void clearAllClusters()
    {
        traffic_cone_cluster_.clearCluster();
    }
};

#endif
