#include <iostream>
#include <vector>
#include <cmath>
#include <tuple>
#include "kdtree.hpp"
#include "local.hpp"
#include "lidar.hpp"
#include "control.hpp"
#include "utility_function.hpp"
#include <chrono>
#include <tuple>
#include <queue>
#include <limits>
using namespace std;

// 유턴 미션수행클래스
class UturnClass
{

private:
    int key = 0;
    // vector<vector<double>> lidar_cone;
    queue<vector<double>> lidar_cone;
    vector<vector<double>> cone;
    vector<double> front_cone;
    vector<double> front_cone_k1;
    vector<double> front_cone_k2;
    vector<double> uturn_front_car;
    vector<double> uturn_front_car_2;
    vector<double> mid_to_car_vec;
    vector<double> mid_to_car_vec_2;
    vector<double> mid_cone_point;
    vector<double> con1_to_con2_vec;
    vector<double> mid_to_lastcone_vec;
    vector<double> lastcone;
    vector<double> pre_offset;
    vector<double> norm_pre_offset;
    vector<double> near_in_cone;
    vector<vector<double>> in_cone_list;
    double cal_vec;
    vector<double> in_cone;
    double uturn_start_index_10;
    double uturn_end_index;
    double dist_cone_with_mid = numeric_limits<double>::max();
    vector<vector<double>> path_point;
    bool uturn_gogo = false;
    Local &local_;
    Lidar &lidar_;
    Control &control_;
    Path &local_path_;

    // 파라미터 상수
    const double kUturnVelocity_;
    const double kEndVelocity_;

public:
    UturnClass(Local &local, Lidar &lidar, Control &control, Path &local_path, nlohmann::json &param)
        : local_(local), lidar_(lidar), control_(control), local_path_(local_path),
          kUturnVelocity_(param["uturn_velocity"]), kEndVelocity_(param["end_velocity"])
    {
    }

    void uturnvoid()
    {
        const double *car_utm = local_.getAddCurCarUTMPos();
        const Path &uturn_nextpath = local_.getRefNextGlobalPath();
        cout << key << " : ";

        if (key == 0)
        {
            control_.setTargetVelocity(kUturnVelocity_);
            key = 1;
        }
        else if (!lidar_.getRefUTurnPoint().empty() && key == 1)
        {
            std::cout << "🐿️🐿️🐿️🐿️🐿️🐿️ 아직 경로 안만드는중 🐿️🐿️🐿️🐿️🐿️🐿️" << "\n";

            if (front_cone.size() > 0)
            {
                cout << "🇵🇹🇵🇹🇵🇹🇵🇹🇵🇹 front_cone : " << front_cone[0] << ", " << front_cone[1] << " 🇵🇹🇵🇹🇵🇹🇵🇹🇵🇹" << endl;
            }

            cone = lidar_.getRefUTurnPoint();

            for (int i = 0; i < cone.size(); i++)
            {
                if (local_.getCurClosestDistance(cone[i]) < 1.2)
                {
                    front_cone = cone[i];
                    uturn_gogo = true;
                    break;
                }
            }
            if (uturn_gogo)
            {
                cout << "🇵🇹🇵🇹🇵🇹🇵🇹🇵🇹 가는중~~~~~~ 🇵🇹🇵🇹🇵🇹🇵🇹🇵🇹" << endl;
                double dis = std::hypot(front_cone[0] - car_utm[0], front_cone[1] - car_utm[1]);
                cout << dis << endl;
                if (dis < 7)
                {
                    front_cone_k1 = local_.getCurClosestPos(front_cone);
                    front_cone_k2 = uturn_nextpath.getClosestPos(front_cone);
                    uturn_front_car = local_.getCurClosestPos(car_utm);
                    uturn_front_car_2 = uturn_nextpath.getClosestPos(car_utm);
                    uturn_start_index_10 = local_.getCurClosestIndex(car_utm) - 10;
                    uturn_end_index = uturn_nextpath.getClosestIndex(car_utm);
                    key = 2;
                }
            }
        }
        else if (key == 2)
        {
            const std::vector<std::vector<double>> &local_pos_arr = local_path_.getRefPosArr();
            const std::vector<std::vector<double>> &next_local_pos_arr = uturn_nextpath.getRefPosArr();

            mid_cone_point = {(front_cone_k1[0] + front_cone_k2[0]) / 2, (front_cone_k1[1] + front_cone_k2[1]) / 2};

            con1_to_con2_vec = {front_cone_k2[0] - front_cone_k1[0], front_cone_k2[1] - front_cone_k1[1]};

            mid_to_car_vec = {uturn_front_car[0] - mid_cone_point[0], uturn_front_car[1] - mid_cone_point[1]};
            mid_to_car_vec_2 = {uturn_front_car_2[0] - mid_cone_point[0], uturn_front_car_2[1] - mid_cone_point[1]};

            // 외적해서 구분하는 것

            for (int i = 0; i < cone.size() - 1; i++)
            {
                mid_to_lastcone_vec = {cone[i][0] - mid_cone_point[0], cone[i][1] - mid_cone_point[1]};

                double cross_product_1 = mid_to_car_vec[0] * mid_to_lastcone_vec[1] - mid_to_car_vec[1] * mid_to_lastcone_vec[0];
                double cross_product_2 = mid_to_car_vec_2[0] * mid_to_lastcone_vec[1] - mid_to_car_vec_2[1] * mid_to_lastcone_vec[0];

                if (cross_product_1 >= 0 and cross_product_2 <= 0)
                {
                    in_cone_list.push_back(cone[i]);
                }
            }

            for (int i = 0; i < in_cone_list.size(); i++)
            {
                double dist = hypot(in_cone_list[i][0] - mid_cone_point[0], in_cone_list[i][1] - mid_cone_point[1]);

                if (dist < dist_cone_with_mid)
                {
                    dist_cone_with_mid = dist;
                    near_in_cone = in_cone_list[i];
                }
            }

            if (near_in_cone.size() == 0)
            {
                key == 0;
            }
            else
            {
                vector<vector<double>> back_matrix = {{0, -1}, {1, 0}};
                vector<double> back_vec = {back_matrix[0][0] * con1_to_con2_vec[0] + back_matrix[0][1] * con1_to_con2_vec[1], back_matrix[1][0] * con1_to_con2_vec[0] + back_matrix[1][1] * con1_to_con2_vec[1]};
                back_vec = {back_vec[0] / sqrt(pow(back_vec[0], 2) + pow(back_vec[1], 2)), back_vec[1] / sqrt(pow(back_vec[0], 2) + pow(back_vec[1], 2))};

                pre_offset = {near_in_cone[0] - mid_cone_point[0], near_in_cone[1] - mid_cone_point[1]};
                norm_pre_offset = {pre_offset[0] / sqrt(pow(pre_offset[0], 2) + pow(pre_offset[1], 2)), pre_offset[1] / sqrt(pow(pre_offset[0], 2) + pow(pre_offset[1], 2))};
                cal_vec = sqrt(pow(near_in_cone[0] - mid_cone_point[0], 2) + pow(near_in_cone[1] - mid_cone_point[1], 2));

                if (cal_vec > 5.7)
                    mid_cone_point = {mid_cone_point[0] + 2 * norm_pre_offset[0], mid_cone_point[1] + 2 * norm_pre_offset[1]};
                else
                    mid_cone_point = {near_in_cone[0] + back_vec[0] * 2, near_in_cone[1] + back_vec[1] * 2};

                for (int i = 0; i < 10; i++)
                    path_point.push_back(local_pos_arr[uturn_start_index_10 + i]);

                path_point.push_back(mid_cone_point);

                for (int i = 0; i < 10; i++)
                    path_point.push_back(next_local_pos_arr[uturn_end_index + i]);

                auto t = generateSplinePath(path_point);

                // tuple의 각 요소에 접근
                std::vector<std::vector<double>> path = std::get<0>(t);
                std::vector<double> ryaw = std::get<1>(t);
                std::vector<double> rk = std::get<2>(t);
                // Path 객체 생성
                Path uturn_path("uturnPath", path, ryaw, rk);
                // local_path 업데이트
                local_path_.updatePath(uturn_start_index_10, local_pos_arr.size(), uturn_path, 0, std::get<0>(t).size());
                local_.changeNextPath();
                local_path_.updatePath(local_path_.getRefPosArr().size(), local_path_.getRefPosArr().size(), uturn_nextpath, uturn_end_index + 10, uturn_nextpath.getRefPosArr().size());
                key++;
            }
        }
        else if (key == 3)
        {
            double distance_to_nextpath = uturn_nextpath.getClosestDistance(car_utm);
            if (distance_to_nextpath < 0.5)
            {
                control_.setTargetVelocity(kEndVelocity_);
                key++;
            }
        }
    }
};
