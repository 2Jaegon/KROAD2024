/***
 * 작성자: 정성윤
 * 설명: 협로 DWA 로컬 패스 주행 관련 클래스 헤더파일
 * Style
 * -클래스: PascalCase
 * -멤버변수: snake_case_   (snake_case 뒤에 밑줄)(파이썬 사용 x)
 * -함수: camelCase
 * -변수: snake_case
 * -상수: kPascalCase   (PascalCase앞에 k 접두어)
 ***/

#ifndef LOCAL_DWA_DRIVING_H_
#define LOCAL_DWA_DRIVING_H_

#include <vector>
#include "local.hpp"
#include "lidar.hpp"
#include "control.hpp"
#include "path.hpp"
#include "dwa.hpp"

class LocalDwaDriving
{
private:
    Local &local_;
    Lidar &lidar_;
    Control &control_;
    Path &local_path_;
    DynamicWindowApproach dwa_;

    /* 파라미터 */
    const double kValidFrontDist_;
    const double kValidBehindDist_;
    const double kFrontAngle_;

    const double kTargetVelocity_;

public:
    LocalDwaDriving(Local &local, Lidar &lidar, Control &control, Path &local_path, nlohmann::json &param)
        : local_(local), lidar_(lidar), control_(control), local_path_(local_path), dwa_(local_, lidar_, control_, local_path_, param),
          kValidFrontDist_(param["valid_front_dist"]),
          kValidBehindDist_(param["valid_behind_dist"]),
          kTargetVelocity_(param["target_velocity"]),
          kFrontAngle_(param["front_angle"])
    {
    }

    void doLocalDriving()
    {
        control_.setTargetVelocity(kTargetVelocity_);
        const std::vector<std::vector<double>> &all_cones = lidar_.getRefTrafficCone();

        if (all_cones.size() > 0)
        {
            // 차량의 헤딩 벡터를 얻음
            std::vector<double> heading_vector = getHeadingVector();

            // 유효한 콘들을 가져옴
            std::vector<std::vector<double>> valid_cones;
            getValidCone(all_cones, heading_vector, kValidFrontDist_, kValidBehindDist_, valid_cones);

            std::vector<double> ref_point = {0.0, 0.0};
            if (!valid_cones.empty())
            {
                // 앞쪽 콘의 x, y 좌표 평균 계산
                for (const auto &cone : valid_cones)
                {
                    ref_point[0] += cone[0];
                    ref_point[1] += cone[1];
                }
                ref_point[0] /= static_cast<double>(valid_cones.size());
                ref_point[1] /= static_cast<double>(valid_cones.size());
            }
            else
            {
                const double *cur_car_utm = local_.getAddCurCarUTMPos();
                double cur_yaw = local_.getCurCarYaw();
                ref_point[0] = cur_car_utm[0] + kValidFrontDist_ * std::cos(cur_yaw);
                ref_point[1] = cur_car_utm[1] + kValidFrontDist_ * std::sin(cur_yaw);
            }
            dwa_.doDynamicWindowApproach(ref_point, valid_cones);
        }
    }

    std::vector<double> getHeadingVector()
    {
        // 초기 헤딩 벡터 (x축 기준)
        std::vector<double> heading_vector = {1.0, 0.0};

        // 차량의 회전 행렬 계산
        double cos_yaw = std::cos(local_.getCurCarYaw());
        double sin_yaw = std::sin(local_.getCurCarYaw());

        // 회전 행렬을 적용하여 헤딩 벡터 계산
        double rotated_x = cos_yaw * heading_vector[0] - sin_yaw * heading_vector[1];
        double rotated_y = sin_yaw * heading_vector[0] + cos_yaw * heading_vector[1];

        return {rotated_x, rotated_y};
    }

    // 탐색에 가치있는 콘 거르는 함수
    void getValidCone(const std::vector<std::vector<double>> &all_cones, const std::vector<double> &heading_vector, double front_dist, double behind_dist, std::vector<std::vector<double>> &valid_cones)
    {
        // 헤딩 벡터의 수직 벡터 계산
        std::vector<double> heading_vector_ort = {-heading_vector[1], heading_vector[0]};

        const double *car_pos = local_.getAddCurCarUTMPos();
        std::vector<double> car_pos_behind_point = {
            car_pos[0] - behind_dist * heading_vector[0],
            car_pos[1] - behind_dist * heading_vector[1]};

        double frontDistSq = front_dist * front_dist; // 앞쪽 거리 제곱값

        for (const auto &cone : all_cones)
        {
            // 차량의 헤딩 벡터를 기준으로 왼쪽(90도) 내에 있는 콘만 탐지
            double dx = cone[0] - car_pos_behind_point[0];
            double dy = cone[1] - car_pos_behind_point[1];
            double cross = heading_vector_ort[0] * dy - heading_vector_ort[1] * dx;
            if (cross < 0)
            {
                // 차량과 콘 사이의 거리가 frontDist 이내일 때만 추가
                double distSq = (cone[0] - car_pos[0]) * (cone[0] - car_pos[0]) + (cone[1] - car_pos[1]) * (cone[1] - car_pos[1]);
                if (distSq < frontDistSq && isWithinAngle(car_pos, heading_vector, cone, kFrontAngle_))
                {
                    valid_cones.push_back(cone);
                }
            }
        }
    }

    bool isWithinAngle(const double *car_pos, const std::vector<double> &heading_vector,
                       const std::vector<double> &cone_pos, double max_angle_deg)
    {
        // 차량과 콘 사이의 벡터
        std::vector<double> cone_vector = {cone_pos[0] - car_pos[0], cone_pos[1] - car_pos[1]};

        // 내적 계산
        double dot_product = heading_vector[0] * cone_vector[0] + heading_vector[1] * cone_vector[1];

        // 벡터 크기 계산 (magnitude)
        double heading_mag = std::sqrt(heading_vector[0] * heading_vector[0] + heading_vector[1] * heading_vector[1]);
        double cone_mag = std::sqrt(cone_vector[0] * cone_vector[0] + cone_vector[1] * cone_vector[1]);

        // 두 벡터 사이의 각도 계산 (라디안 값으로)
        double angle_rad = std::acos(dot_product / (heading_mag * cone_mag));

        // 라디안 값을 각도로 변환
        double angle_deg = angle_rad * (180.0 / M_PI);

        // max_angle_deg 이내에 있는지 확인
        return angle_deg <= max_angle_deg;
    }

    const std::vector<double> getRefPoint() const
    {
        return dwa_.getRefPoint();
    }
};
#endif
