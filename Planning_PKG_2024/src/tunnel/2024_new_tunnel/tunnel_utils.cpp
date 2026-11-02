/***
 * 작성자: 조윤서
 * 설명: 터널 내부 주행
 * Style
 * -클래스: PascalCase
 * -멤버변수: snake_case_   (snake_case 뒤에 밑줄)(파이썬 사용 x)
 * -함수: camelCase
 * -변수: snake_case
 * -상수: kPascalCase   (PascalCase앞에 k 접두어)
 ***/

#include "2024_new_tunnel.hpp"

std::vector<double> Tunnel2024NEW::divPoint(const std::vector<double> R, const std::vector<double> L, double ratio)
{
    double x_r = R[0];
    double y_r = R[1];
    double x_l = L[0];
    double y_l = L[1];

    double x_internal = (x_l + ratio * x_r) / (1 + ratio);
    double y_internal = (y_l + ratio * y_r) / (1 + ratio);

    std::vector<double> sub = {x_internal, y_internal};
    return sub;
}

// void Tunnel2024NEW::pathFilter(std::vector<std::vector<double>> &result, const std::vector<std::vector<double>> origin)
// {
//     /* x좌표가 0이상인 애들만 남겨주는 필터용 함수입니다. */
//     /* 터널은 상대 좌표계를 쓰므로 무조건 좌표가 0 이상이어야 함!!! */

//     // result.clear(); // garbage 값 청소...
//     // for (const auto &point : origin)
//     // {
//     //     if (point[0] >= 0)
//     //     {
//     //         result.emplace_back(point);
//     //     }
//     // }

//     // ========================================================
//     result.clear(); // 이전 결과값 초기화

//     // 필터링 기준을 곡률 변화에 따라 완화합니다.
//     auto calcCurvature = [](const std::vector<double> &p1, const std::vector<double> &p2, const std::vector<double> &p3)
//     {
//         double angle1 = std::atan2(p2[1] - p1[1], p2[0] - p1[0]);
//         double angle2 = std::atan2(p3[1] - p2[1], p3[0] - p2[0]);
//         return std::abs(angle2 - angle1);
//     };

//     // 필터링 후 경로 포인트가 최소 3개 이상은 되도록 조정합니다.
//     if (origin.size() < 3)
//     {
//         for (const auto &point : origin)
//         {
//             if (point[0] >= 0)
//             {
//                 result.emplace_back(point);
//             }
//         }
//         return;
//     }

//     result.emplace_back(origin[0]);
//     for (size_t i = 1; i < origin.size() - 1; ++i)
//     {
//         const auto &p1 = origin[i - 1];
//         const auto &p2 = origin[i];
//         const auto &p3 = origin[i + 1];

//         // x좌표가 0 이상이고 곡률 변화가 크지 않은 경우에만 경로에 포함
//         if (p2[0] >= 0 && calcCurvature(p1, p2, p3) < M_PI / 3)
//         { // 곡률 변화 기준을 60도로 완화
//             result.emplace_back(p2);
//         }
//     }

//     if (origin.back()[0] >= 0)
//     {
//         result.emplace_back(origin.back());
//     }
// }

std::vector<int> Tunnel2024NEW::get_sorted_indices(const std::vector<double> &v)
{
    // 인덱스를 초기화합니다.
    std::vector<int> sorted_indices(v.size());
    iota(sorted_indices.begin(), sorted_indices.end(), 0);

    // lst의 값을 기준으로 인덱스를 정렬합니다.
    sort(sorted_indices.begin(), sorted_indices.end(), [&v](int a, int b)
         { return v[a] > v[b]; });

    return sorted_indices;
}

void Tunnel2024NEW::sort_coor_by_dist(std::vector<std::vector<double>> &ans, std::vector<std::vector<double>> &coor)
{
    sort(coor.begin(), coor.end(), [](const std::vector<double> &a, const std::vector<double> &b)
         { return std::hypot(a[0], a[1]) < std::hypot(b[0], b[1]); });
}

// ====================================================
// 혼합 보간법 적용 (선형 보간과 스플라인 보간 혼합)
// std::vector<std::vector<double>> hybridInterpolation(const std::vector<std::vector<double>> &points)
// {
//     std::vector<std::vector<double>> result;
//     for (size_t i = 0; i < points.size() - 1; ++i)
//     {
//         const auto &p1 = points[i];
//         const auto &p2 = points[i + 1];
//         double distance = std::hypot(p2[0] - p1[0], p2[1] - p1[1]);

//         // 곡률이 작은 구간에서는 선형 보간 적용
//         if (distance > 5.0)
//         { // 임계값 5.0은 상황에 맞게 조정 가능
//             for (double t = 0; t <= 1.0; t += 0.1)
//             {
//                 double x = (1 - t) * p1[0] + t * p2[0];
//                 double y = (1 - t) * p1[1] + t * p2[1];
//                 result.push_back({x, y});
//             }
//         }
//         else
//         {
//             // 곡률이 큰 구간에서는 기존의 3차 스플라인 보간 사용
//             result.push_back(p1);
//         }
//     }
//     result.push_back(points.back());
//     return result;
// }

// 곡률 기반 보간 최적화
std::vector<std::vector<double>> Tunnel2024NEW::curvatureOptimizedInterpolation(const std::vector<std::vector<double>> &points)
{
    std::vector<std::vector<double>> result;
    result.push_back(points[0]);

    for (size_t i = 1; i < points.size() - 1; ++i)
    {
        const auto &p1 = points[i - 1];
        const auto &p2 = points[i];
        const auto &p3 = points[i + 1];

        double curvature = std::abs(std::atan2(p3[1] - p2[1], p3[0] - p2[0]) - std::atan2(p2[1] - p1[1], p2[0] - p1[0]));

        // 곡률이 작은 경우 필터링 없이 포함
        if (curvature < M_PI / 6)
        { // 임계값은 상황에 맞게 조정
            result.push_back(p2);
        }
    }
    result.push_back(points.back());
    return result;
}

// 곡률 기반 필터링 조정
void Tunnel2024NEW::pathFilter(std::vector<std::vector<double>> &result, const std::vector<std::vector<double>> origin)
{
    result.clear();
    auto calcCurvature = [](const std::vector<double> &p1, const std::vector<double> &p2, const std::vector<double> &p3)
    {
        double angle1 = std::atan2(p2[1] - p1[1], p2[0] - p1[0]);
        double angle2 = std::atan2(p3[1] - p2[1], p3[0] - p2[0]);
        return std::abs(angle2 - angle1);
    };

    result.push_back(origin[0]);
    for (size_t i = 1; i < origin.size() - 1; ++i)
    {
        const auto &p1 = origin[i - 1];
        const auto &p2 = origin[i];
        const auto &p3 = origin[i + 1];

        if (p2[0] >= 0 && calcCurvature(p1, p2, p3) < M_PI / 4)
        { // 곡률 변화 기준을 45도로 조정
            result.push_back(p2);
        }
    }
    if (origin.back()[0] >= 0)
    {
        result.push_back(origin.back());
    }
}

// 5. 경로 업데이트 시 보간 간격 조정 및 필터링 조정
// auto csp_result = generateSplinePath(path_tmp, 0.01); // 보간 간격을 기존보다 더 작게 설정
// std::vector<std::vector<double>> path01 = std::get<0>(csp_result);

// std::vector<std::vector<double>> path02;
// pathFilter(path02, path01);