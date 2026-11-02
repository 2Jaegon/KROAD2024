/***
 * 작성자: 정성윤
 * 설명: 정적 소형 회피 알고리즘 관련 클래스 헤더파일
 * Style
 * -클래스: PascalCase
 * -멤버변수: snake_case_   (snake_case 뒤에 밑줄)(파이썬 사용 x)
 * -함수: camelCase
 * -변수: snake_case
 * -상수: kPascalCase   (PascalCase앞에 k 접두어)
 ***/

#ifndef HARD_SMALL_STATIC_H_
#define HARD_SMALL_STATIC_H_

#include <iostream>
#include <vector>
#include "lidar.hpp"
#include "local.hpp"
#include "utility_function.hpp"

class HardSmallStatic
{
    Local &local_;
    Lidar &lidar_;
    Path &local_path_;

    // 함수 내 static 변수 선언
    bool first_trans_flag = false;  // 마지막 경로가 변경되었는지 여부를 저장하는 플래그
    bool second_trans_flag = false; // 첫번째 경로가 변경되었는지 여부를 저장하는 플래그
    bool second_flag = false;

public:
    HardSmallStatic(Local &local, Lidar &lidar, Path &local_path) : local_(local), lidar_(lidar), local_path_(local_path)
    {
    }

    void makePath()
    {
        // 오프셋 및 보간을 위한 상수 선언
        double move_first = 2.0;   // 첫 번째 목표 지점까지 이동할 거리
        double move_first2 = 2.0;  // 두 번째 목표 지점까지 이동할 거리
        double move_second = 2.0;  // 두 번째 목표 지점까지 이동할 거리
        int f_csp1 = 180;          // 첫 번째 이동 거리의 시작 오프셋
        int f_csp2 = 60;           // 첫 번째 이동 거리의 끝 오프셋
        int s_csp1 = 150;          // 두 번째 이동 거리의 시작 오프셋
        int s_csp2 = 180;          // 두 번째 이동 거리의 끝 오프셋
        double inner_ratio = 0.50; // 내부 보간을 위한 비율

        std::vector<double> inner_point(2, 0.0); // 두 목표 지점 사이의 보간 지점
        // 포인트 임시 변수 선언 및 초기화
        std::vector<double> f_target_p(2, 0.0); // 첫 번째 목표 지점
        std::vector<double> s_target_p(2, 0.0); // 두 번째 목표 지점

        // 글로벌 및 로컬 경로를 가져옴
        const Path &global_path = local_.getRefCurGlobalPath();                              // 글로벌 경로
        const std::vector<std::vector<double>> &global_pos_arr = global_path.getRefPosArr(); // 글로벌 경로의 위치 배열

        const double *car_utm = local_.getAddCurCarUTMPos();

        // 라이다 콘 데이터 확인
        if (!lidar_.getRefSmallStaticPoint().empty() && !second_trans_flag)
        {
            const std::vector<std::vector<double>> &cone = lidar_.getRefSmallStaticPoint(); // 라이다로 감지된 작은 고정 물체들
            std::vector<double> first_cone_pos, second_cone_pos;                            // 첫 번째 및 두 번째 유효한 콘 포인트

            std::vector<std::vector<double>> valid_cone_array; // 유효한 콘 포인트 배열
            std::vector<double> valid_cone_D_array;            // 유효한 콘의 거리 배열

            // 유효한 콘 필터링
            for (const auto &p : cone)
            {
                double obj_dis_to_path = global_path.getClosestDistance(p); // 콘과 경로 사이의 거리
                if (std::abs(obj_dis_to_path) <= 1.5)                       // 경로로부터 1.5m 이내의 콘만 유효
                {
                    double relative_cone_d = std::hypot(p[0] - local_.getAddCurCarUTMPos()[0], p[1] - local_.getAddCurCarUTMPos()[1]); // 현재 위치에서 콘까지의 거리
                    valid_cone_array.push_back(p);                                                                                     // 유효한 콘 포인트 추가
                    valid_cone_D_array.push_back(relative_cone_d);                                                                     // 유효한 콘 거리 추가
                }
                std::cout << "콘 필터링 완료\n";
            }

            if (!valid_cone_D_array.empty())
            {
                // 가장 가까운 콘 찾기
                auto min_element_it = std::min_element(valid_cone_D_array.begin(), valid_cone_D_array.end()); // 가장 작은 값 iterator
                size_t first_index = std::distance(valid_cone_D_array.begin(), min_element_it);               // 가장 작은 값 인덱스
                first_cone_pos = valid_cone_array[first_index];                                               // 첫 번째 유효한 콘 포인트

                std::cout << "콘 인덱스 찾음\n";

                valid_cone_array.erase(valid_cone_array.begin() + first_index); // 가장 가까운 콘 제거
                valid_cone_D_array.erase(valid_cone_D_array.begin() + first_index);

                if (!valid_cone_D_array.empty())
                {
                    min_element_it = std::min_element(valid_cone_D_array.begin(), valid_cone_D_array.end());
                    size_t second_index = std::distance(valid_cone_D_array.begin(), min_element_it);
                    second_cone_pos = valid_cone_array[second_index]; // 두 번째 유효한 콘 포인트
                }
                std::cout << "가까운 콘 찾기 완료\n";
            }

            if (!first_cone_pos.empty())
            {
                // 첫 번째 콘과의 거리 계산
                double small_d = std::hypot(first_cone_pos[0] - local_.getAddCurCarUTMPos()[0], first_cone_pos[1] - local_.getAddCurCarUTMPos()[1]);

                if (small_d <= 9.0 && !first_trans_flag)
                {
                    // 첫 번째 콘 근처 경로 변경
                    std::vector<double> fst_val = global_path.getClosestPos(first_cone_pos);
                    int fst_index = global_path.getClosestIndex(first_cone_pos);
                    static double fst_dist = std::hypot(first_cone_pos[0] - fst_val[0], first_cone_pos[1] - fst_val[1]);

                    std::cout << "콘 index 생성\n";

                    f_target_p[0] = first_cone_pos[0] - (first_cone_pos[0] - fst_val[0]) / std::abs(fst_dist) * move_first;
                    f_target_p[1] = first_cone_pos[1] - (first_cone_pos[1] - fst_val[1]) / std::abs(fst_dist) * move_first;

                    std::cout << "f_target_p: " << f_target_p[0] << " " << f_target_p[1] << "\n";

                    // 경로를 분할하여 새로운 경로 생성
                    std::vector<std::vector<double>> tmp_path1_1;
                    std::vector<std::vector<double>> tmp_path1_2;

                    int start_index1_1 = fst_index - f_csp1 - 10;
                    int end_index1_1 = fst_index - f_csp1;
                    int start_index1_2 = fst_index + f_csp2;
                    int end_index1_2 = fst_index + f_csp2 + 10;

                    // 범위가 유효한지 확인
                    if (start_index1_1 >= 0 && end_index1_1 < global_pos_arr.size())
                    {
                        tmp_path1_1.assign(global_pos_arr.begin() + start_index1_1, global_pos_arr.begin() + end_index1_1);
                    }

                    if (start_index1_2 >= 0 && end_index1_2 < global_pos_arr.size())
                    {
                        tmp_path1_2.assign(global_pos_arr.begin() + start_index1_2, global_pos_arr.begin() + end_index1_2);
                    }

                    std::cout << "새로운 경로 생성\n";

                    std::vector<std::vector<double>> small_path = tmp_path1_1;
                    small_path.push_back(f_target_p);
                    small_path.insert(small_path.end(), tmp_path1_2.begin(), tmp_path1_2.end());

                    // 스플라인 경로 생성
                    auto t = generateSplinePath(small_path, 0.05);

                    Path evade_path("evadePath", std::get<0>(t), std::get<1>(t), std::get<2>(t));
                    local_path_.updatePath(fst_index - f_csp1 - 10, fst_index + f_csp2 + 10, evade_path, 0, std::get<0>(t).size());

                    first_trans_flag = true; // 경로 변경 플래그 설정
                    std::cout << "첫번째 경로 변경 완료" << std::endl;
                }
                else if (small_d < 8.0 && !second_cone_pos.empty() && first_trans_flag)
                {
                    // 두 번째 콘 근처 경로 변경
                    std::vector<double> fst_val = global_path.getClosestPos(first_cone_pos);
                    int fst_index = global_path.getClosestIndex(first_cone_pos);
                    static double fst_dist = std::hypot(first_cone_pos[0] - fst_val[0], first_cone_pos[1] - fst_val[1]);

                    std::vector<double> sc_val = global_path.getClosestPos(second_cone_pos);
                    int sc_index = global_path.getClosestIndex(second_cone_pos);
                    static double sc_dist = std::hypot(second_cone_pos[0] - sc_val[0], second_cone_pos[1] - sc_val[1]);

                    inner_point[0] = inner_ratio * first_cone_pos[0] + (1 - inner_ratio) * second_cone_pos[0];
                    inner_point[1] = inner_ratio * first_cone_pos[1] + (1 - inner_ratio) * second_cone_pos[1];

                    f_target_p[0] = first_cone_pos[0] - (first_cone_pos[0] - fst_val[0]) / std::abs(fst_dist) * move_first2;
                    f_target_p[1] = first_cone_pos[1] - (first_cone_pos[1] - fst_val[1]) / std::abs(fst_dist) * move_first2;
                    s_target_p[0] = second_cone_pos[0] - (second_cone_pos[0] - sc_val[0]) / std::abs(sc_dist) * move_second;
                    s_target_p[1] = second_cone_pos[1] - (second_cone_pos[1] - sc_val[1]) / std::abs(sc_dist) * move_second;

                    // 경로를 분할하여 새로운 경로 생성
                    std::vector<std::vector<double>> tmp_path2_1;
                    std::vector<std::vector<double>> tmp_path2_2;

                    int start_index2_1 = fst_index - s_csp1 - 10;
                    int end_index2_1 = fst_index - s_csp1;
                    int start_index2_2 = sc_index + s_csp2;
                    int end_index2_2 = sc_index + s_csp2 + 10;

                    // 범위가 유효한지 확인
                    if (start_index2_1 >= 0 && end_index2_1 < global_pos_arr.size())
                    {
                        tmp_path2_1.assign(global_pos_arr.begin() + start_index2_1, global_pos_arr.begin() + end_index2_1);
                    }

                    if (start_index2_2 >= 0 && end_index2_2 < global_pos_arr.size())
                    {
                        tmp_path2_2.assign(global_pos_arr.begin() + start_index2_2, global_pos_arr.begin() + end_index2_2);
                    }

                    std::vector<std::vector<double>> small_path = tmp_path2_1;
                    small_path.push_back(f_target_p);
                    small_path.push_back(inner_point);
                    small_path.push_back(s_target_p);
                    small_path.insert(small_path.end(), tmp_path2_2.begin(), tmp_path2_2.end());

                    // 스플라인 경로 생성
                    auto t = generateSplinePath(small_path, 0.05);

                    Path evade_path("evadePath", std::get<0>(t), std::get<1>(t), std::get<2>(t));
                    local_path_.updatePath(fst_index - s_csp1 - 10, sc_index + s_csp2 + 10, evade_path, 0, std::get<0>(t).size());
                    second_trans_flag = true;
                    second_flag = true;
                }
                if (small_d < 3 && first_trans_flag)
                {
                    second_trans_flag = true; // 경로 변경 플래그 초기화
                    first_trans_flag = false;
                    std::cout << "최종 경로 변경 완료" << std::endl;
                }
            }
        }
        if (first_trans_flag)
            std::cout << "첫번쨰 경로 변경됨\n";
        if (second_flag)
            std::cout << "두번째 경로 변경됨\n";
        if (second_trans_flag)
            std::cout << "경로 변경 종료\n";
    }
};

#endif
