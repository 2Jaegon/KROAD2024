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

void Tunnel2024NEW::driveTunnel()
{
    // std::cout << "::: ------------------------- ::: 디버깅 ::: ------------------------- :::" << "\n";
    // std::cout << "왼쪽 벽 LLLLLLLLLL >>> " << lidar_.getTunnelLLen() << "\n";
    // std::cout << "오른쪽 벽 RRRRRRRRRR >>> " << lidar_.getTunnelRLen() << "\n";
    // std::cout << "::: ------------------------- ::: ----- ::: ------------------------- :::" << "\n";

    modeCheck();

    // --------------------- 이하 디버깅 코드~!! --------------------- //

    // std::cout << " 현재 주행 모드 : " << now_status_ << "\n";
    std::cout << " 정적 to 동적 상태 : " << StoD_ << "\n";
    std::cout << " 정적 flag 상태 : " << small_static_flag_ << "\n";

    if (tunnel_entry_t_ >= std::chrono::system_clock::time_point())
    {
        std::cout << "터널 진입한 시간" << std::chrono::duration_cast<chrono::seconds>(std::chrono::system_clock::now() - tunnel_entry_t_).count() << "\n";
    }

    // ------------------------------------------------------------ //

    switch (now_status_)
    {
    case (DriveStatus::GPS):
    { // GPS 주행 모드
        driveGPS();
        break;
    }
    case (DriveStatus::WALL):
    { // 벽 주행 모드
        driveWall();
        break;
    }
    case (DriveStatus::LANE):
    { // 차선 센터 주행 모드
        driveLaneCenter();
        break;
    }
    case (DriveStatus::NOMODE):
    { // No Mode
        std::cout << "XXX 인식되는 값 없음!!! XXX" << "\n";
        break;
    }
    case (DriveStatus::INIT):
    {
        // 속도 설정은 위에서 모드 체크하면서 하고 바꿉니다~
        break;
    }
    default:
    { // 습관성 default. nomode와 디버깅 메세지 말고 차이 없어요
        break;
    }
    }

    // 동적 장애물
    checkStaticToDynamic();
    if (!change_gps_ && lidar_.getDynamicMode())
    { // 다이나믹 모드라면 시작
        std::cout << "🟦🟦🟦🟦🟦 쉿! 동적 중 🟦🟦🟦🟦🟦" << "\n";
        dynamicObjFunc();
        // if (e_stop_.getStopMoreThanOnce()) // 한 번 멈췄으면 동적 실행 중지
        // {
        //     lidar_.setDynamicMode(false);
        // }
        std::cout << "🟦🟦🟦🟦🟦🟦🟦🟦🟦🟦🟦🟦🟦🟦🟦🟦" << "\n";
    }

    // // Plot용 (검증용) - 대회에서는 사용 않음
    switch (now_status_)
    {
    case (DriveStatus::GPS):
        /* code */
        local_.setRcsMode(false);
        break;

    default:
        local_.setRcsMode(true);
        break;
    }
}

void Tunnel2024NEW::tunnelJamming(bool start_sign)
{
    // if (!in_tunnel_ || local_.getJamStart())
    // {
    //     local_.setJamStart(start_sign);
    // }
}

void Tunnel2024NEW::driveGPS()
{ // 상대경로로 갈아끼워진 local path를 절대경로인 global path로 롤백
    std::cout << " ========= GPS 주행중 ========= " << "\n";
    const Path &g_path = local_.getRefCurGlobalPath();
    local_path_.updatePath(0, local_path_.getRefPosArr().size(), g_path, 0, g_path.getRefPosArr().size());
}

void Tunnel2024NEW::driveWall() // 벽 포인트 kFindNum 이상만큼은 있어야 오류없이 디버깅된다...!!!!
{
    std::cout << " ========= WALL 주행중 ========= " << "\n";
    L_wall_ = lidar_.getRefTunnelLWall(); // 업데이트... 해줘야 됨!!!
    R_wall_ = lidar_.getRefTunnelRWall();
    // 기울기와 절편 구하기
    double slope_l;
    double slope_r;
    try
    {
        // 예외 자체가 발생하면 안되는 부분이긴하지만... 혹시 몰라 처리해둡니다
        if (L_wall_[2] == L_wall_[0])
        {
            throw L_wall_[2];
        }
        if (R_wall_[2] == R_wall_[0])
        {
            throw R_wall_[2];
        }

        slope_l = (L_wall_[3] - L_wall_[1]) / (L_wall_[2] - L_wall_[0]);
        slope_r = (R_wall_[3] - R_wall_[1]) / (R_wall_[2] - R_wall_[0]);
    }
    catch (double e)
    { // division 0 오류 시 바로 cenDrive!!!
        std::cout << "에러 발생!!! 모드 긴급 전환 : " << e << "\n";
        std::cout << "에러 발생!!! 모드 긴급 전환 : " << e << "\n";
        std::cout << "에러 발생!!! 모드 긴급 전환 : " << e << "\n";
        driveLaneCenter();
        return;
    }

    double y_intercept_l = L_wall_[1] - (slope_l * L_wall_[0]); // b = y - ax
    double y_intercept_r = R_wall_[1] - (slope_r * R_wall_[0]);

    std::vector<std::vector<double>> l_points_ = {};
    std::vector<std::vector<double>> r_points_ = {};

    // 벽에 대한 포인트 구하기(세밀하게)
    for (double i = 0; i < 13; i += 0.5)
    { // 13 * 2
        // 각 좌표를 0부터 12.5까지 증가시킴 (터널은 상대좌표!!)
        // index 1의 값은 y = ax+b임.
        // x좌표는 동일한게 맞아요~
        std::vector<double> l_tmp = {i, slope_l * i + y_intercept_l};
        std::vector<double> r_tmp = {i, slope_r * i + y_intercept_r};

        l_points_.emplace_back(l_tmp);
        r_points_.emplace_back(r_tmp);
    }

    // KDTree로 가장 가까운 점들과 거리 찾기
    // l_points는 절대로 비어있지 않습니다. 디버깅시 참고.
    KDTree<std::vector<double>> l_tree(l_points_);
    KDTree<std::vector<double>> r_tree(r_points_); // 고로 KDTREE도 안 비어있어용~

    std::vector<std::vector<double>> wall_cen_point = {};

    for (int i = kFindNum; i > 0; i--)
    // 왜 뒤
    {
        wall_cen_point.emplace_back(divPoint(r_points_[r_points_.size() - i], l_points_[l_points_.size() - i], kRatio));
    }

    std::vector<double> wall_road_vec = {wall_cen_point[1][0] - wall_cen_point[0][0], wall_cen_point[1][1] - wall_cen_point[0][1]};

    // 단위 벡터 계산
    double wall_road_vec_size = std::hypot(wall_road_vec[0], wall_road_vec[1]); // 벡터의 크기
    std::vector<double> wall_road_norm_vec = {wall_road_vec[0] / wall_road_vec_size, wall_road_vec[1] / wall_road_vec_size};

    std::vector<double> wall_car_vec = {1, 0}; // 차량이 현재 주행 중인 방향
    double wall_yawvec = std::inner_product(wall_road_vec.begin(), wall_road_vec.end(), wall_car_vec.begin(), 0);
    double wall_cos_theta = wall_yawvec / wall_road_vec_size;
    double wall_car_yaw = acos(wall_cos_theta);

    double cross_product_wall = wall_road_vec[0] * wall_car_vec[1] - wall_road_vec[1] * wall_car_vec[0];
    if (cross_product_wall < 0)
    {
        local_.setCurYaw(-wall_car_yaw);
    }
    else
    {
        local_.setCurYaw(wall_car_yaw);
    }

    // // 아래에서 정적 소형 진행
    std::vector<std::vector<double>> rotation_matrix = {{0, -1},
                                                        {1, 0}};
    // 수작업으로 행렬 곱셈 수행
    std::vector<double> wall_small_offset_vec(2);
    wall_small_offset_vec[0] = rotation_matrix[0][0] * wall_road_norm_vec[0] + rotation_matrix[0][1] * wall_road_norm_vec[1];
    wall_small_offset_vec[1] = rotation_matrix[1][0] * wall_road_norm_vec[0] + rotation_matrix[1][1] * wall_road_norm_vec[1];

    // // 아래는 옛날 코드
    std::vector<double> mv_point01 = {};
    std::vector<double> mv_point02 = {};
    std::vector<std::vector<double>> go;

    // std::vector<double> &small_obj = lidar_.getRefTunnelSmallStaticPoint();
    if ((!(lidar_.getRefTunnelSmallStaticPoint().empty())) && (small_ing_))
    {
        // control_.setTargetVelocity(2.5);
        std::cout << "장애물 발견" << "\n";
        if (lidar_.getRefTunnelSmallStaticPoint().size() < 2)
        {
            vstackCPP(go, kCarTail, wall_cen_point);
            checkSSStatus(0); // 장애물 경로 생성 안 함!!!
        }

        else
        {
            control_.setTargetVelocity(kVelocitySmallStatic); // 회피 할 때만 속도 재정비~

            if (2 <= lidar_.getRefTunnelSmallStaticPoint().size() && lidar_.getRefTunnelSmallStaticPoint().size() < 4)
            {

                double car_dist = std::hypot(lidar_.getRefTunnelSmallStaticPoint()[0], lidar_.getRefTunnelSmallStaticPoint()[1]);
                std::cout << car_dist << "\n";
                std::cout << "장애물 한 개 발견" << "\n";
                std::cout << "그냥 장애물 거리 : " << car_dist << "\n";

                // 여기 밑에 evadePath 쓸 수 있으면 좋을듯? 지금은 적용 X
                std::vector<double> obj1 = {lidar_.getRefTunnelSmallStaticPoint()[0], lidar_.getRefTunnelSmallStaticPoint()[1]};

                if (car_dist < kMinDist)
                {
                    std::vector<double> obj1_closet_wall = r_tree.getClosestPos(obj1);
                    double dist_obj1 = std::hypot(obj1_closet_wall[0] - obj1[0], obj1_closet_wall[1] - obj1[1]);

                    std::cout << "장애물 1번 벽이랑 거리 : " << dist_obj1 << "\n";

                    std::vector<std::vector<double>> for_mv = {};
                    std::vector<std::vector<double>> tmp = {};
                    if ((2 <= dist_obj1) && (dist_obj1 < kSafetyDist))
                    {
                        mv_point01.emplace_back(obj1[0] + kWallOffset1 * wall_small_offset_vec[0]);
                        mv_point01.emplace_back(obj1[1] + kWallOffset1 * wall_small_offset_vec[1]);
                        for_mv.emplace_back(mv_point01);

                        if (!mv_point01.empty())
                        {
                            vstackCPP(tmp, kCarTail, for_mv);
                            vstackCPP(go, tmp, wall_cen_point);

                            checkSSStatus(1); // 장애물 하나 경로 생성 했어요~
                        }
                    }
                    else if ((kSafetyDist <= dist_obj1) && (dist_obj1 < 8))
                    {
                        mv_point01.emplace_back(obj1[0] - kWallOffset1 * wall_small_offset_vec[0]);
                        mv_point01.emplace_back(obj1[1] - kWallOffset1 * wall_small_offset_vec[1]);
                        for_mv.emplace_back(mv_point01);
                        if (!mv_point01.empty())
                        {
                            vstackCPP(tmp, kCarTail, for_mv);
                            vstackCPP(go, tmp, wall_cen_point);

                            checkSSStatus(1); // 장애물 하나 경로 생성 했어요~
                        }
                    }
                    else
                    {
                        vstackCPP(go, kCarTail, wall_cen_point);
                        checkSSStatus(0); // 얘는 경로 생성하는 장애물이 아니니까...
                    }
                }

                else
                {
                    vstackCPP(go, kCarTail, wall_cen_point);
                    checkSSStatus(0); // 아직 경로 생성 전
                }

                last_time_for_small_ = std::chrono::system_clock::now();
            }

            else if (4 <= lidar_.getRefTunnelSmallStaticPoint().size())
            {
                std::cout << "장애물 2개 발견" << "\n";
                std::vector<double> obj1 = {lidar_.getRefTunnelSmallStaticPoint()[0], lidar_.getRefTunnelSmallStaticPoint()[1]};
                std::vector<double> obj2 = {lidar_.getRefTunnelSmallStaticPoint()[2], lidar_.getRefTunnelSmallStaticPoint()[3]};

                std::vector<double> *fst_obj;
                std::vector<double> *scd_obj;

                if (std::hypot(obj1[0], obj1[1]) < std::hypot(obj2[0], obj2[1]))
                {
                    fst_obj = &obj1;
                    scd_obj = &obj2;
                }
                else
                {
                    fst_obj = &obj2;
                    scd_obj = &obj1;
                }

                double car_dist_fst = std::hypot((*fst_obj)[0], (*fst_obj)[1]);
                double car_dist_scd = std::hypot((*scd_obj)[0], (*scd_obj)[1]);

                // std::cout << "첫번째 장애물 거리 : " << car_dist_fst << "\n";
                // std::cout << "두번째 장애물 거리 : " << car_dist_scd << "\n";

                if (car_dist_fst < kMinDist)
                {
                    std::vector<double> obj1_closet_wall = r_tree.getClosestPos(*fst_obj);
                    double dist_fst = std::hypot(obj1_closet_wall[0] - (*fst_obj)[0], obj1_closet_wall[1] - (*fst_obj)[1]);

                    std::cout << "첫 번째 장애물 벽까지와 거리 : " << dist_fst << "\n";

                    if ((2 < dist_fst) && (dist_fst < kSafetyDist))
                    {
                        mv_point01.emplace_back((*fst_obj)[0] + kWallOffset1 * wall_small_offset_vec[0]);
                        mv_point01.emplace_back((*fst_obj)[1] + kWallOffset1 * wall_small_offset_vec[1]);
                    }
                    else if ((kSafetyDist < dist_fst) && (dist_fst < 8))
                    {
                        mv_point01.emplace_back((*fst_obj)[0] - kWallOffset1 * wall_small_offset_vec[0]);
                        mv_point01.emplace_back((*fst_obj)[1] - kWallOffset1 * wall_small_offset_vec[1]);
                    }
                }

                if (car_dist_scd < kMinDist)
                {
                    std::vector<double> obj2_closet_wall = r_tree.getClosestPos(*scd_obj);
                    double dist_scd = std::hypot(obj2_closet_wall[0] - (*scd_obj)[0], obj2_closet_wall[1] - (*scd_obj)[1]);

                    std::cout << "두 번째 장애물 벽까지와 거리 : " << dist_scd << "\n";

                    if ((2 < dist_scd) && (dist_scd < kSafetyDist))
                    {
                        mv_point02.emplace_back((*scd_obj)[0] + kWallOffset2 * wall_small_offset_vec[0]);
                        mv_point02.emplace_back((*scd_obj)[1] + kWallOffset2 * wall_small_offset_vec[1]);
                    }
                    else if ((kSafetyDist < dist_scd) && (dist_scd < 8))
                    {
                        mv_point02.emplace_back((*scd_obj)[0] - kWallOffset2 * wall_small_offset_vec[0]);
                        mv_point02.emplace_back((*scd_obj)[1] - kWallOffset2 * wall_small_offset_vec[1]);
                    }
                }

                // -----
                std::vector<std::vector<double>> for_mv01 = {};
                for_mv01.emplace_back(mv_point01);
                std::vector<std::vector<double>> for_mv02 = {};
                for_mv02.emplace_back(mv_point02);
                std::vector<std::vector<double>> tmp01;
                std::vector<std::vector<double>> tmp02;

                if ((mv_point01.size() > 0) && (mv_point02.size() > 0))
                {
                    vstackCPP(tmp01, kCarTail, for_mv01);
                    vstackCPP(tmp02, for_mv02, wall_cen_point);
                    vstackCPP(go, tmp01, tmp02);
                    checkSSStatus(2); // 장애물 두 개 경로 생성 했어요~
                }
                else if ((mv_point01.size() > 0) && (mv_point02.size() == 0))
                {
                    vstackCPP(tmp01, kCarTail, for_mv01);
                    vstackCPP(go, tmp01, wall_cen_point);
                    checkSSStatus(1);
                }
                else if ((mv_point01.size() == 0) && (mv_point02.size() > 0))
                {
                    vstackCPP(tmp02, kCarTail, for_mv02);
                    vstackCPP(go, tmp02, wall_cen_point);
                    checkSSStatus(1);
                }
                else
                {
                    vstackCPP(go, kCarTail, wall_cen_point);
                    checkSSStatus(0);
                }

                last_time_for_small_ = std::chrono::system_clock::now();
            }
        }
    }

    else
    {
        if (local_.getCurMissionNumber() != 17)
        {
            std::cout << "장애물 미발견" << "\n";
        }
        vstackCPP(go, kCarTail, wall_cen_point);

        if (small_ing_) // 정적 소형 중일 때만 동작하도록 ...
        {
            checkSSStatus(0); // 장애물 경로 생성 안함!!!
        }
    }

    if (go.size() >= 1)
    {
        // 플롯을 위한 publish
        // 보간
        auto csp_result = generateSplinePath(go, 0.05);

        std::vector<std::vector<double>> path01 = std::get<0>(csp_result);
        std::vector<std::vector<double>> path02;
        pathFilter(path02, path01);
        vstackCPP(wall_path_, kTunnelMark, path02);

        std::vector<double> wall_yaw = std::get<1>(csp_result);
        std::vector<double> wall_k = std::get<2>(csp_result);

        // Path update
        Path tunnel_wall_("wallDrive_tunnel", wall_path_, wall_yaw, wall_k);

        // const double *utm_point = local_.getAddCurCarUTMPos();
        // int start_idx = local_path_.getClosestIndex(utm_point);
        // end index : 교체할 부분의 범위가 삭제할 부분의 최대 범위를 넘어가면 삭제
        // int end_idx = local_path_.getRefPosArr().size() - start_idx < (wall_path_.size() - 1) ? local_path_.getRefPosArr().size() : start_idx + (wall_path_.size() - 1); // 오류고 자시고 로직을 좀 바꾸는 게 좋을 것 같습니다.

        local_path_.updatePath(0, local_path_.getRefPosArr().size(), tunnel_wall_, 0, tunnel_wall_.getRefPosArr().size());
    }
    return;
}

void Tunnel2024NEW::driveLaneCenter()
{
    lidar_center_ = lidar_.getRefTunnelCenter();

    if (lidar_center_.size() >= 4)
    {
        std::cout << " ========= LANE CENTER 주행중 ========= " << "\n";

        // 라이다 데이터 크기 및 좌표 확인
        std::cout << "라이다 데이터 크기: " << lidar_center_.size() << "\n";

        std::vector<std::vector<double>> center_p;
        reshape1Dto2D(center_p, lidar_center_);

        // std::cout << "제발 여기 오류이게해주세요!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << "\n";
        // print2Dvec(center_p);
        // std::cout << "제발 여기 오류이게해주세요!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << "\n";

        int cen_p_final_i = center_p.size() - 1;

        // 마지막 두 포인트 간 벡터 및 단위 벡터 계산
        std::vector<double> further_vec = {center_p[cen_p_final_i][0] - center_p[cen_p_final_i - 1][0],
                                           center_p[cen_p_final_i][1] - center_p[cen_p_final_i - 1][1]};
        double further_vec_size = std::hypot(further_vec[0], further_vec[1]);
        std::vector<double> further_norm_vec = {further_vec[0] / further_vec_size, further_vec[1] / further_vec_size};

        // 새로운 포인트 추가 (곡선 보정을 위한 점 추가)
        std::vector<std::vector<double>> further_p = {{center_p[cen_p_final_i][0] + further_norm_vec[0] * 10,
                                                       center_p[cen_p_final_i][1] + further_norm_vec[1] * 10}};

        std::vector<std::vector<double>> path_tmp;
        std::vector<std::vector<double>> tmp_tmp;

        // 차량 꼬리 좌표와 센터 포인트 결합
        vstackCPP(tmp_tmp, kCarTail, center_p);
        vstackCPP(path_tmp, tmp_tmp, further_p);

        // 큐빅 스플라인 보간을 사용한 경로 생성
        auto csp_result = generateSplinePath(path_tmp, 0.05);
        std::vector<std::vector<double>> path01 = std::get<0>(csp_result);

        // 필터링 적용
        std::vector<std::vector<double>> path02;
        pathFilter(path02, path01);

        // 최종 경로 설정
        vstackCPP(cen_path_, kTunnelMark, path02);

        std::vector<double> cen_yaw = std::get<1>(csp_result);
        std::vector<double> cen_k = std::get<2>(csp_result);

        // 경로 업데이트 및 경로 크기 확인
        std::cout << "경로 업데이트 시작\n";
        Path tunnel_center("centerDrive_tunnel", cen_path_, cen_yaw, cen_k);
        std::cout << "경로 포인트 개수: " << tunnel_center.getRefPosArr().size() << "\n";

        if (tunnel_center.getRefPosArr().size() > 0)
        {
            local_path_.updatePath(0, local_path_.getRefPosArr().size(), tunnel_center, 0, tunnel_center.getRefPosArr().size());
            std::cout << "경로 업데이트 완료\n";
        }
        else
        {
            std::cerr << "경로 업데이트 실패: 경로 포인트가 없습니다.\n";
        }
    }
    else
    {
        std::cerr << "라이다 데이터 부족 (최소 4개의 좌표 필요)\n";
    }
}

void Tunnel2024NEW::laneDrive()
{ // 나아아아중에 --- 이제 가자!!
  // ---여기서부터할거임 아마두???
  // lidar.tunnel_lane을 받아오는데

    // 아래는 일단 무시하세염~

    // std::vector<double> lane = lidar_.getRefTunnelLane();
    // if (!(lane.empty())){
    //     tunnel_lane_2D_.clear();
    //     for(int i=0; i < (lane.size() / 2); i++){
    //         std::vector<double> lane_pair = {lane[2*i], lane[2*i+1]};
    //         tunnel_lane_2D_.emplace_back(lane_pair);
    //     }

    //         // DBscan을 위한 파이썬 모듈 불러내기 --- c++ 끌어와서 하기

    //         //DBSCAN 시작~ ... 그냥 pcl 깡으로 받아서 하면 좋을 것 같은데 지금 pcl이 되나?
    //         Point *p = new Point;
    //         std::vector<Point> p_forDS;

    //         for (size_t i = 0; i < tunnel_lane_2D_.size(); i++) {
    //             p[i].x = tunnel_lane_2D_[i][0];
    //             p[i].y = tunnel_lane_2D_[i][1];
    //             p[i].clusterID = UNCLASSIFIED;

    //             p_forDS.emplace_back(p[i]);
    //         }

    //         DBSCAN ds(m_samples, eps, p_forDS);
    //         ds.run();

    //         std::vector<std::vector<double>> fst;
    //         std::vector<std::vector<double>> scd;
    //         std::vector<std::vector<double>> thd;

    //         for(int i=0;i < ds.m_points.size(); i++){
    //             if (ds.m_points[i].clusterID == 0){fst.emplace_back(tunnel_lane_2D[i]);}
    //             else if (ds.m_points[i].clusterID == 1){scd.emplace_back(tunnel_lane_2D[i]);}
    //             else if (ds.m_points[i].clusterID == 2){thd.emplace_back(tunnel_lane_2D[i]);}
    //         }

    //             std::vector<int> indices = get_sorted_indices({fst[0][1], scd[0][1], thd[0][1]});
    //             std::tuple<int, int, int> result = make_tuple(indices[0], indices[1], indices[2]);

    //             std::map<std::tuple<int, int, int>, std::vector<std::vector<std::vector<double>>>> order_mapping;
    //             order_mapping[make_tuple(0, 1, 2)] = {fst, scd, thd};
    //             order_mapping[make_tuple(0, 2, 1)] = {fst, thd, scd};
    //             order_mapping[make_tuple(1, 0, 2)] = {scd, fst, thd};
    //             order_mapping[make_tuple(1, 2, 0)] = {scd, thd, fst};
    //             order_mapping[make_tuple(2, 0, 1)] = {thd, fst, scd};
    //             order_mapping[make_tuple(2, 1, 0)] = {thd, scd, fst};

    //             std::vector<std::vector<std::vector<double>>> order = order_mapping[result];
    //             std::vector<std::vector<double>> fst_order; std::vector<double> fst_order_flat;
    //             sort_coor_by_dist(fst_order, order[0]);
    //             flatten(fst_order_flat, fst_order);

    //             std::vector<std::vector<double>> scd_order; std::vector<double> scd_order_flat;
    //             sort_coor_by_dist(scd_order, order[1]);
    //             flatten(scd_order_flat, scd_order);

    //             std::vector<std::vector<double>> thd_order; std::vector<double> thd_order_flat;
    //             sort_coor_by_dist(thd_order, order[2]);
    //             flatten(thd_order_flat, thd_order);

    //             // 선형회귀는 python binding하여야함...

    //             // 아직이용...
    //             int ind;
    //             double dist = 0.0; int near_index = -1; //코드에서 사용하진 않지만 디버깅용으로 남김
    //             for(int i = 0; i < kFindNum; i++){
    //                 ind = kFindNum - i;
    //             }
    // }
    // else {
    //     std::cout << "지금 tunnel lane도 없어용 ..." << "\n";
    // }
}