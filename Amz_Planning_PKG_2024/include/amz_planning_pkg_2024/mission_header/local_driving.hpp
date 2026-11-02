/***
 * 작성자: 정성윤
 * 설명: 협로 로컬 패스 주행 관련 클래스 헤더파일
 * Style
 * -클래스: PascalCase
 * -멤버변수: snake_case_   (snake_case 뒤에 밑줄)(파이썬 사용 x)
 * -함수: camelCase
 * -변수: snake_case
 * -상수: kPascalCase   (PascalCase앞에 k 접두어)
 ***/

#ifndef LOCAL_DRIVING_H_
#define LOCAL_DRIVING_H_

#include <iostream>
#include <vector>
#include <algorithm>
#include "nlohmann/json.hpp"
#include "local.hpp"
#include "vision.hpp"
#include "lidar.hpp"
#include "control.hpp"
#include "cubic_spline.hpp"
#include "utility_function.hpp"
#include "rrt.hpp"
#include "dwa.hpp"

class LocalDriving
{
private:
    Local &local_;
    Lidar &lidar_;
    Control &control_;
    Path &local_path_;

    /* 파라미터 */
    const double kTargetVelocity_;

    const double kFrontConesDist_;
    const double kConeObstacleSize_; // 콘의 장애물 크기
    const double kValidFrontDist_;
    const double kValidBehindDist_;
    const double kConeTargetsDistRatio_; // 콘 거리 비율 (0.5 = 50%)

    // rrt
    const int kIterationNumber_;   // 최대 노드 개수
    const double kPlanDistance_;   // 경로 계획 거리
    const double kExpandDistance_; // RRT에서 확장할 거리
    const double kExpandAngle_;    // RRT에서 확장할 각도

    /* private 함수 */
    // ccw 연산 함수
    bool ccw(const std::array<double, 2> &A, const std::array<double, 2> &B, const std::array<double, 2> &C)
    {
        return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0]);
    }
    // 선분 교차 여부를 확인하는 함수
    bool getLineSegmentIntersection(const std::array<double, 2> &a1, const std::array<double, 2> &a2,
                                    const std::array<double, 2> &b1, const std::array<double, 2> &b2)
    {
        return (ccw(a1, b1, b2) != ccw(a2, b1, b2)) && (ccw(a1, a2, b1) != ccw(a1, a2, b2));
    }

    // 선분 교차점 계산하는 함수
    std::pair<double, double> getLineIntersection(const std::array<double, 2> &a1,
                                                  const std::array<double, 2> &a2,
                                                  const std::array<double, 2> &b1,
                                                  const std::array<double, 2> &b2)
    {
        double x1 = a1[0], y1 = a1[1];
        double x2 = a2[0], y2 = a2[1];
        double x3 = b1[0], y3 = b1[1];
        double x4 = b2[0], y4 = b2[1];

        double denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);

        if (denominator == 0.0)
        {
            return {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
        }

        double x_numerator = ((x1 * y2 - y1 * x2) * (x3 - x4)) - ((x1 - x2) * (x3 * y4 - y3 * x4));
        double y_numerator = ((x1 * y2 - y1 * x2) * (y3 - y4)) - ((y1 - y2) * (x3 * y4 - y3 * x4));

        double x = x_numerator / denominator;
        double y = y_numerator / denominator;

        return {x, y};
    }

public:
    LocalDriving(Local &local, Lidar &lidar, Control &control, Path &local_path, nlohmann::json &param)
        : local_(local), lidar_(lidar), control_(control), local_path_(local_path),
          kConeObstacleSize_(param["cone_obstacle_size"]),
          kFrontConesDist_(param["front_cone_dist"]), kValidFrontDist_(param["valid_front_dist"]),
          kValidBehindDist_(param["valid_behind_dist"]), kTargetVelocity_(param["target_velocity"]),
          kConeTargetsDistRatio_(param["cone_target_dist_ratio"]),
          kIterationNumber_(param["iteration_number"]), kPlanDistance_(param["plan_distance"]),
          kExpandDistance_(param["expand_distance"]), kExpandAngle_(param["expand_angle"])
    {
    }

    void doLocalDriving()
    {
        const double *car_pos = local_.getAddCurCarUTMPos();
        double car_yaw = local_.getCurCarYaw();
        const std::vector<std::vector<double>> &all_cones = lidar_.getRefTrafficCone();

        if (all_cones.size() > 0)
        {
            // 차량의 헤딩 벡터를 얻음
            std::vector<double> heading_vector = getHeadingVector();

            // 유효한 콘들을 가져옴
            std::vector<std::vector<double>> valid_cones;
            getValidCone(all_cones, heading_vector, kValidFrontDist_, kValidBehindDist_, valid_cones);

            // 장애물 리스트와 RRT 타겟 리스트를 초기화
            std::vector<std::tuple<double, double, double>> cone_obstacle_list; // 콘 장애물 리스트
            std::vector<std::tuple<double, double, double>> rrt_target_cones;   // RRT에 사용할 타겟 리스트

            // 콘을 장애물 리스트와 RRT 타겟 리스트에 추가
            for (const auto &cone : valid_cones)
            {
                // 장애물 리스트에 추가
                cone_obstacle_list.emplace_back(cone[0], cone[1], kConeObstacleSize_);

                // 차량과 콘 사이의 거리 계산
                double cone_distance = std::hypot(car_pos[0] - cone[0], car_pos[1] - cone[1]);

                // 차량과 콘 사이의 거리가 설정한 기준보다 크다면, RRT 타겟으로 설정
                if (cone_distance > kFrontConesDist_ * kConeTargetsDistRatio_)
                {
                    rrt_target_cones.emplace_back(cone[0], cone[1], kConeObstacleSize_);
                }
            }

            // 차량 뒤쪽 지점을 계산
            std::vector<double> car_pos_behind_point = {
                car_pos[0] - kValidBehindDist_ * heading_vector[0],
                car_pos[1] - kValidBehindDist_ * heading_vector[1]};

            // 시작 위치를 설정
            std::vector<double> start = {car_pos_behind_point[0], car_pos_behind_point[1], car_yaw};

            // RRT 객체를 생성하고 경로 탐색을 시작
            RRT rrt(start, kPlanDistance_, cone_obstacle_list, kExpandDistance_, kExpandAngle_,
                    kIterationNumber_, rrt_target_cones);

            std::pair<std::vector<Node>, std::vector<Node>> rrt_result = rrt.Planning();
            std::vector<Node> node_list = rrt_result.first;
            std::vector<Node> leaf_nodes = rrt_result.second;

            // Delaunay 삼각분할을 수행하여 경계선을 얻음
            DelaunayClassifier delaunay_classifier_;
            for (auto &cone : valid_cones)
            {
                delaunay_classifier_.addPoint(cone);
            }
            delaunay_classifier_.classifyEdges();

            const std::vector<std::pair<std::vector<double>, std::vector<double>>> &delaunay_left = delaunay_classifier_.getClass1Edges();
            const std::vector<std::pair<std::vector<double>, std::vector<double>>> &delaunay_right = delaunay_classifier_.getClass2Edges();
            const std::vector<std::pair<std::vector<double>, std::vector<double>>> &delaunay_mix = delaunay_classifier_.getMixedClassEdges();

            // 최적의 브랜치를 찾음
            std::vector<Node> best_branch;
            findBestBranch(leaf_nodes, node_list, delaunay_left, delaunay_right, best_branch);

            // 최적 경로가 존재할 경우 웨이포인트를 생성
            if (!best_branch.empty())
            {
                std::vector<std::vector<double>> waypoints;
                std::vector<std::pair<std::array<double, 2>, std::array<double, 2>>> delaunay_edges;

                // Delaunay 경계선을 Edge 형태로 변환
                for (const auto &edge : delaunay_mix)
                {
                    std::array<double, 2> point1 = {edge.first[0], edge.first[1]};
                    std::array<double, 2> point2 = {edge.second[0], edge.second[1]};
                    delaunay_edges.emplace_back(point1, point2);
                }

                // 웨이포인트를 추출
                getWaypointsFromEdges(best_branch, delaunay_mix, waypoints);

                if (waypoints.size() > 1)
                {
                    for (auto &p : waypoints)
                    {
                        std::cout << p[0] << " " << p[1] << "\n";
                    }

                    std::vector<std::vector<double>> spline_points;
                    const std::vector<std::vector<double>> &local_path_pos_arr = local_path_.getRefPosArr();

                    int cur_closest_idx = local_path_.getClosestIndex(waypoints[0]);
                    int spline_start_idx = std::clamp(cur_closest_idx - 10, 0, static_cast<int>(local_path_pos_arr.size()) - 1);          // clamp로 spline_start_idx 제한
                    int spline_end_idx = std::clamp(spline_start_idx + 5, spline_start_idx, static_cast<int>(local_path_pos_arr.size())); // clamp로 spline_end_idx를 제한

                    std::cout << "start: " << spline_start_idx << "\n";
                    std::cout << "end: " << spline_end_idx << "\n";

                    std::cout << "local_path###########\n";
                    for (int i = (spline_start_idx - 4) >= 0 ? spline_start_idx - 4 : 0; i < (spline_end_idx + 4 < local_path_.getRefPosArr().size() ? spline_end_idx + 4 : local_path_.getRefPosArr().size()); ++i)
                    {
                        std::cout << local_path_.getRefPosArr()[i][0] << " " << local_path_.getRefPosArr()[i][1] << "\n";
                    }

                    spline_points.insert(spline_points.end(), local_path_pos_arr.begin() + spline_start_idx, local_path_pos_arr.begin() + spline_end_idx);
                    spline_points.insert(spline_points.end(), waypoints.begin(), waypoints.end());

                    for (auto &p : spline_points)
                    {
                        std::cout << "spline: " << p[0] << " " << p[1] << "\n";
                    }

                    // 스플라인 경로 생성
                    auto t = generateSplinePath(spline_points, 0.05);
                    Path front_local_path("frontLocalPath", std::get<0>(t), std::get<1>(t), std::get<2>(t));
                    // local_path_.updatePath(spline_start_idx, local_path_pos_arr.size(), front_local_path, 0, std::get<0>(t).size());
                    local_path_.updatePath(0, local_path_pos_arr.size(), front_local_path, 0, std::get<0>(t).size());

                    // std::cout << "update_local_path###########\n";
                    // for (int i = 0; i < local_path_.getRefPosArr().size(); ++i)
                    // {
                    //     std::cout << local_path_.getRefPosArr()[i][0] << " " << local_path_.getRefPosArr()[i][1] << "\n";
                    // }

                    // // 스플라인 경로 생성
                    // auto t = generateSplinePath(waypoints, 0.05);
                    // Path front_local_path("frontLocalPath", std::get<0>(t), std::get<1>(t), std::get<2>(t));
                    // local_path_.updatePath(0, local_path_.getRefPosArr().size(), front_local_path, 0, std::get<0>(t).size());
                }
            }
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
                if (distSq < frontDistSq)
                {
                    valid_cones.push_back(cone);
                }
            }
        }
    }

    // 최적의 경로를 찾는 함수 구현
    void findBestBranch(const std::vector<Node> &leaf_nodes, const std::vector<Node> &node_list,
                        const std::vector<std::pair<std::vector<double>, std::vector<double>>> &delaunay_left, const std::vector<std::pair<std::vector<double>, std::vector<double>>> &delaunay_right, std::vector<Node> &best_branch)
    {
        if (leaf_nodes.empty())
        {
            return;
        }

        std::vector<double> leaf_ratings; // 리프 노드에 대한 평가 값

        // 각 리프 노드에 대한 평가 수행
        for (const auto &leaf : leaf_nodes)
        {
            double branch_rating = 0.0; // 초기 브랜치 평가 값
            Node node = leaf;

            while (node.parent != -1) // 루트 노드까지 반복
            {
                // 왼쪽 콘 삼각형 경계 평가
                if (!delaunay_left.empty())
                {
                    for (const auto &edge : delaunay_left)
                    {
                        std::array<double, 2> a1 = {edge.first[0], edge.first[1]};
                        std::array<double, 2> a2 = {edge.second[0], edge.second[1]};
                        std::array<double, 2> b1 = {node.x, node.y};
                        const Node &parent_node = node_list[node.parent];
                        std::array<double, 2> b2 = {parent_node.x, parent_node.y};

                        if (getLineSegmentIntersection(a1, a2, b1, b2))
                        {
                            branch_rating = -std::numeric_limits<double>::infinity();
                            break; // 교차하면 더 이상 평가하지 않음
                        }
                        else
                        {
                            branch_rating += node.cost;
                        }
                    }
                }

                // 오른쪽 콘 삼각형 경계 평가
                if (!delaunay_right.empty())
                {
                    for (const auto &edge : delaunay_right)
                    {
                        std::array<double, 2> a1 = {edge.first[0], edge.first[1]};
                        std::array<double, 2> a2 = {edge.second[0], edge.second[1]};
                        std::array<double, 2> b1 = {node.x, node.y};
                        const Node &parent_node = node_list[node.parent];
                        std::array<double, 2> b2 = {parent_node.x, parent_node.y};

                        if (getLineSegmentIntersection(a1, a2, b1, b2))
                        {
                            branch_rating = -std::numeric_limits<double>::infinity();
                            break; // 교차하면 더 이상 평가하지 않음
                        }
                        else
                        {
                            branch_rating += node.cost;
                        }
                    }
                }

                node = node_list[node.parent]; // 부모 노드로 이동
            }

            leaf_ratings.push_back(branch_rating); // 리프 노드 평가 값 추가
        }

        if (leaf_ratings.empty())
        {
            return;
        }

        // 가장 높은 평가 값 찾기
        auto max_rating_iter = std::max_element(leaf_ratings.begin(), leaf_ratings.end());
        double max_rating = *max_rating_iter;

        if (max_rating < 0)
        { // 모든 평가 값이 음수인 경우
            return;
        }

        int max_rating_index = std::distance(leaf_ratings.begin(), max_rating_iter);

        Node node = leaf_nodes[max_rating_index];
        std::vector<Node> reverse_branch;
        reverse_branch.push_back(node);

        // 최적 경로를 역방향으로 추적
        while (node.parent != -1)
        {
            node = node_list[node.parent];
            reverse_branch.push_back(node);
        }

        best_branch.insert(best_branch.end(), reverse_branch.rbegin(), reverse_branch.rend());
    }

    // 수정된 getWaypointsFromEdges 함수
    void getWaypointsFromEdges(const std::vector<Node> &filtered_branch,
                               const std::vector<std::pair<std::vector<double>, std::vector<double>>> &delaunay_edges,
                               std::vector<std::vector<double>> &waypoints)
    {
        if (delaunay_edges.empty())
        {
            return;
        }

        for (size_t i = 0; i < filtered_branch.size() - 1; ++i)
        {
            const Node &node1 = filtered_branch[i];
            const Node &node2 = filtered_branch[i + 1];

            std::array<double, 2> a1 = {node1.x, node1.y};
            std::array<double, 2> a2 = {node2.x, node2.y};

            // 교차한 Edge의 정보를 저장할 구조체를 정의합니다.
            struct IntersectedEdgeInfo
            {
                std::vector<double> intersection; // 교차점
                std::vector<double> midpoint;     // Edge의 중간점
                double dist_sq;                   // node1에서 교차점까지의 제곱 거리
            };

            std::vector<IntersectedEdgeInfo> intersected_edges;

            for (const auto &edge : delaunay_edges)
            {
                const std::array<double, 2> &b1 = {edge.first[0], edge.first[1]};
                const std::array<double, 2> &b2 = {edge.second[0], edge.second[1]};

                // 두 선분이 교차하는지 확인
                if (getLineSegmentIntersection(a1, a2, b1, b2))
                {
                    // 교차점 계산
                    std::pair<double, double> intersection_pair = getLineIntersection(a1, a2, b1, b2);
                    std::vector<double> intersection = {intersection_pair.first, intersection_pair.second};

                    // Edge의 중간점 계산
                    std::vector<double> midpoint = {(b1[0] + b2[0]) / 2.0, (b1[1] + b2[1]) / 2.0};

                    double dx = intersection[0] - node1.x;
                    double dy = intersection[1] - node1.y;
                    double dist_sq = dx * dx + dy * dy;

                    // 교차한 Edge의 정보를 저장
                    intersected_edges.push_back({intersection, midpoint, dist_sq});
                }
            }

            if (!intersected_edges.empty())
            {
                // 이전에 추가된 점을 추적하기 위한 변수
                std::vector<double> last_point = {-1.0, -1.0}; // 초기값은 임의로 설정

                // 교차하는 선분이 하나일 경우, 그 선분의 중간점을 웨이포인트로 추가
                if (intersected_edges.size() == 1)
                {
                    const auto &edge_info = intersected_edges[0];
                    const std::vector<double> &current_point = edge_info.midpoint; // 현재 점

                    if (current_point != last_point) // 마지막으로 추가된 점과 비교하여 다를 때만 추가(cubic spline 오류 막기 위해)
                    {
                        waypoints.push_back(current_point);
                        last_point = current_point; // 마지막 점 업데이트
                    }
                }
                // 교차하는 선분이 여러 개일 경우, node1에서 교차점까지의 거리로 정렬 후 추가
                else
                {
                    // node1에서 교차점까지의 거리에 따라 정렬
                    std::sort(intersected_edges.begin(), intersected_edges.end(),
                              [](const IntersectedEdgeInfo &edge1, const IntersectedEdgeInfo &edge2)
                              {
                                  return edge1.dist_sq < edge2.dist_sq;
                              });

                    for (const auto &edge_info : intersected_edges)
                    {
                        const std::vector<double> &current_point = edge_info.midpoint; // 현재 점

                        if (current_point != last_point) // 마지막으로 추가된 점과 비교하여 다를 때만 추가(cubic spline 오류 막기 위해)
                        {
                            waypoints.push_back(current_point);
                            last_point = current_point; // 마지막 점 업데이트
                        }
                    }
                }
            }
        }
    }
};

#endif
