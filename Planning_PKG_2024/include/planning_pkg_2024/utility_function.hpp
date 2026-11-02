/***
 * 작성자: 조윤서
 * 설명: 기타 잡다한 응용 함수들
 * Style
 * -클래스: PascalCase
 * -멤버변수: snake_case_   (snake_case 뒤에 밑줄)(파이썬 사용 x)
 * -함수: camelCase
 * -변수: snake_case
 * -상수: kPascalCase   (PascalCase앞에 k 접두어)
 ***/

#ifndef UTIL_FUNC_H_
#define UTIL_FUNC_H_

#include <iostream>
#include <vector>
#include <cmath>
#include "local.hpp"
#include "cubic_spline.hpp"
#include <random>

// 스플라인 패스 반환
// TODO : 나중에 Path 객체 안에 Path 객체 생성자로 넣기, index 유효성 검사 추가하기
inline std::tuple<std::vector<std::vector<double>>, std::vector<double>, std::vector<double>> generateSplinePath(const std::vector<std::vector<double>> &path_raw, double ds = 0.1)
{
    std::vector<double> x, y;
    std::vector<double> last_point = {-1, -1};
    for (const auto &point : path_raw)
    {
        if (last_point == point)
            continue;
        x.push_back(point[0]);
        y.push_back(point[1]);
        last_point = point;
    }

    CubicSpline2D csp(x, y);
    std::vector<double> s;
    for (double i = 0; i < csp.calc_s(x, y).back(); i += ds)
    {
        s.push_back(i);
    }

    std::vector<std::vector<double>> path;
    std::vector<double> ryaw, rk;
    for (const auto &i_s : s)
    {
        auto pos = csp.calc_position(i_s);
        path.push_back({pos.first, pos.second});
        ryaw.push_back(csp.calc_yaw(i_s));
        rk.push_back(csp.calc_curvature(i_s));
    }

    return std::make_tuple(path, ryaw, rk);
}

/* 벡터 슬라이싱 */
// 파이썬의 list 슬라이싱과 기능 동일 (idx m부터 idx n-1까지)
template <typename T>
void vectorSlice(T &ans, T const &v, int m, int n)
{
    ans.clear();
    // 유효한 인덱스인지 체크 - 런타임 디버깅용
    // m = std::max(0, m);
    // n = std::min(static_cast<int>(v.size()), n);
    ans.insert(ans.end(), v.begin() + m, v.begin() + n);
}

/* Python NumPy vstack method 구현*/
template <typename T>
void vstackCPP(T &vstack, T &vec01, T &vec02, bool clear = true)
{
    if (clear)
    {
        vstack.clear();
    }
    vstack.reserve(vec01.size() + vec02.size());
    vstack.insert(vstack.end(), vec01.begin(), vec01.end());
    vstack.insert(vstack.end(), vec02.begin(), vec02.end());
    return;
}

/* Python NumPy flatten method 구현*/
template <typename T>
void flatten(T &result, std::vector<T> &vec2D)
{
    result.clear();
    for (const auto &inner_vec : vec2D)
    {
        result.insert(result.end(), inner_vec.begin(), inner_vec.end());
    }
}

/* 1차원 벡터 2차원으로 바꿔주는 벡터*/
template <typename T>
void reshape1Dto2D(std::vector<T> &result, T &vec1D)
{
    result.clear();
    int vsize = vec1D.size();
    if (vsize & 1)
    {
        vec1D.emplace_back(vec1D[0] - vec1D[0]);
    } // 홀수일 경우 강제적으로 짝수로 만들어줌.
      //  0이 아니라 첫번째 인덱스에서 첫번째 인덱스를 뺀 값을 넣는 이유는 vector 자료형 일치 때문.
    for (int i = 0; i < (vsize / 2); i++)
    {
        T pair = {vec1D[2 * i], vec1D[2 * i + 1]};
        result.emplace_back(pair);
    }
}

/* 2차원 벡터 출력~ */
template <typename T>
void print2Dvec(T &vec2D, std::string head_msg = " ")
{
    std::cout << head_msg << "/n";
    for (auto vec1D = vec2D.begin(); vec1D != vec2D.end(); vec1D++)
    {
        for (int idx = 0; idx != (*vec1D).size(); idx++)
        {
            std::cout << (*vec1D)[idx] << " ";
        }
        std::cout << "\n";
    }
}

inline double outerProduct(const std::vector<double> &obj_edge_point, const std::vector<double> &closest_global_point, const std::vector<double> &offset)
{
    // Calculate the components of the vectors
    double v1x = obj_edge_point[0] - closest_global_point[0];
    double v1y = obj_edge_point[1] - closest_global_point[1];

    double v2x = offset[0] - closest_global_point[0];
    double v2y = offset[1] - closest_global_point[1];

    // Calculate the cross product
    double cross_product = (v1x * v2y) - (v1y * v2x);

    return cross_product;
}

/* 난수 생성기 */
inline double randomNumCreator(double min, double max)
{
    std::random_device rd;  // 시드에 사용할 장치
    std::mt19937 gen(rd()); // Mersenne Twister 엔진 초기화
    std::uniform_real_distribution<double> rNum(min, max);
    return rNum(gen);
}

#endif
