/***
 * 작성자: 정성윤
 * 설명: 전역 경로를 기록, 생성하는 클래스
 * Style
 * -클래스: PascalCase
 * -멤버변수: snake_case_   (snake_case 뒤에 밑줄)(파이썬 사용 x)
 * -함수: camelCase
 * -변수: snake_case
 * -상수: kPascalCase   (PascalCase앞에 k 접두어)
 ***/

#ifndef GLOBAL_PATH_GENERATOR_HPP_
#define GLOBAL_PATH_GENERATOR_HPP_

#include <vector>
#include <algorithm>
#include "local.hpp"
#include "lidar.hpp"
#include "control.hpp"
#include "path.hpp"

class GlobalPathGenerator
{
private:
    Local &local_;
    Lidar &lidar_;
    Control &control_;
    Path &local_path_;

    std::vector<std::vector<double>> global_ref_points_;

    std::vector<double> start_point_;

    Path global_path_;

    int ref_spline_index_;

    const double kRecordDist_;
    const int kSplineNum_;

public:
    GlobalPathGenerator(Local &local, Lidar &lidar, Control &control, Path &local_path, nlohmann::json &param)
        : local_(local), lidar_(lidar), control_(control), local_path_(local_path),
          ref_spline_index_(0),
          global_path_("global_path"),
          kRecordDist_(param["record_distance"]),
          kSplineNum_(param["spline_num"])
    {
    }

    const Path &getGlobalPath()
    {
        return global_path_;
    }

    const std::vector<double> &getRefStartPoint()
    {
        return start_point_;
    }

    void recordPoint(const std::vector<double> &record_point)
    {
        if (global_ref_points_.empty())
        {
            global_ref_points_.push_back(record_point);
            start_point_ = record_point;
        }
        else
        {
            std::vector<double> &last_point = global_ref_points_.back();
            double distance = std::hypot(record_point[0] - last_point[0], record_point[1] - last_point[1]);

            std::cout << "ref_spline_index+kSplineNum: " << ref_spline_index_ + kSplineNum_ << "\n";
            std::cout << "global_path_points size: " << global_ref_points_.size() << "\n";

            if (distance > kRecordDist_)
            {
                global_ref_points_.push_back(record_point);
                std::cout << "**********************record***************\n";
                if (ref_spline_index_ + kSplineNum_ <= global_ref_points_.size())
                {
                    const std::vector<std::vector<double>> &global_path_pos_arr = global_path_.getRefPosArr();
                    std::vector<std::vector<double>> spline_points;

                    int spline_start_idx = std::clamp(static_cast<int>(global_path_pos_arr.size() - 10), 0, static_cast<int>(global_path_pos_arr.size())); // clamp로 spline_start_idx 제한
                    int spline_end_idx = std::clamp(spline_start_idx + 4, 0, static_cast<int>(global_path_pos_arr.size()));                                // clamp로 spline_end_idx를 제한

                    spline_points.insert(spline_points.end(), global_path_pos_arr.begin() + spline_start_idx, global_path_pos_arr.begin() + spline_end_idx);
                    spline_points.insert(spline_points.end(), global_ref_points_.begin() + ref_spline_index_, global_ref_points_.end());

                    // 스플라인 경로 생성
                    auto t = generateSplinePath(spline_points, 0.05);
                    Path new_global_path("newGlobalPath", std::get<0>(t), std::get<1>(t), std::get<2>(t));
                    global_path_.updatePath(spline_start_idx, global_path_pos_arr.size(), new_global_path, 0, std::get<0>(t).size());

                    ref_spline_index_ = global_ref_points_.size();
                }
            }
        }
    }

    void pathClose()
    {
        if (!global_ref_points_.empty())
        {
            const std::vector<std::vector<double>> &global_path_pos_arr = global_path_.getRefPosArr();
            std::vector<std::vector<double>> spline_points;

            int spline_start_idx = std::clamp(static_cast<int>(global_path_pos_arr.size() - 10), 0, static_cast<int>(global_path_pos_arr.size()) - 1); // clamp로 spline_start_idx 제한
            int spline_end_idx = std::clamp(spline_start_idx + 4, spline_start_idx, static_cast<int>(global_path_pos_arr.size()));                     // clamp로 spline_end_idx를 제한

            int spline_start_global_idx = 0;
            int spline_end_global_idx = spline_start_global_idx + 10;

            spline_points.insert(spline_points.end(), global_path_pos_arr.begin() + spline_start_idx, global_path_pos_arr.begin() + spline_end_idx);
            spline_points.insert(spline_points.end(), global_path_pos_arr.begin() + spline_start_global_idx, global_path_pos_arr.begin() + spline_end_global_idx);

            for (auto &p : spline_points)
            {
                std::cout << p[0] << " " << p[1] << "\n";
            }

            auto t = generateSplinePath(spline_points, 0.05);

            Path closing_global_path("closingGlobalPath", std::get<0>(t), std::get<1>(t), std::get<2>(t));

            int spline_end_global_closing_idx = std::clamp(spline_end_global_idx + 340, spline_end_global_idx, static_cast<int>(global_path_.getRefPosArr().size()));
            std::cout << global_path_.getRefPosArr().size() << "-------------" << spline_end_global_idx << "============" << spline_end_global_closing_idx << "\n";
            closing_global_path.updatePath(closing_global_path.getRefPosArr().size(), closing_global_path.getRefPosArr().size(), global_path_, spline_end_global_idx, spline_end_global_closing_idx);
            global_path_.updatePath(spline_start_idx, global_path_pos_arr.size(), closing_global_path, 0, closing_global_path.getRefPosArr().size());
        }
    }
};

#endif
