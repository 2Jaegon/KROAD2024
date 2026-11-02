/***
 * 작성자: 조아진
 * 설명: 평행 관련 클래스
 * Style
 * -클래스: PascalCase
 * -멤버변수: snake_case_   (snake_case 뒤에 밑줄)(파이썬 사용 x)
 * -함수: camelCase
 * -변수: snake_case
 * -상수: kPascalCase   (PascalCase앞에 k 접두어)
 ***/

#include <iostream>
#include <vector>
#include <cmath>
#include "local.hpp"
#include "lidar.hpp"
#include "control.hpp"
#include "path.hpp"

using namespace std;

class newPrlParking
{
private:
    Local &local_;
    Lidar &lidar_;
    Control &control_;
    Path &local_path_;

public:
    newPrlParking(Local &local, Lidar &lidar, Control &control, Path &local_path) : local_(local), lidar_(lidar), control_(control), local_path_(local_path)
    {
    }

    // 고점을 지나고 다른 원과 접하는 원의 반지름 도출하는 함수
    double calculateContactRadiusUp(const vector<double> &center1, double radius1, const vector<double> &circlePoint, double unit_dx_p3p4, double unit_dy_p3p4)
    {
        double b = 2 * ((-1) * unit_dx_p3p4 * (circlePoint[0] - center1[0]) + (-1) * unit_dy_p3p4 * (circlePoint[1] - center1[1]) - radius1);
        double c = pow(circlePoint[0] - center1[0], 2) + pow(circlePoint[1] - center1[1], 2) - pow(radius1, 2);

        double r2 = -c / b;
        return r2;
    }

    // 원 위의 두 점을 이용해 각도를 도출하는 함수
    double calculateAngle(const vector<double> &center, const vector<double> &point1, const vector<double> &point2)
    {
        double angle1 = atan2(point1[1] - center[1], point1[0] - center[0]);
        double angle2 = atan2(point2[1] - center[1], point2[0] - center[0]);
        cout << "angle1 : " << angle1 << endl;
        cout << "angle2 : " << angle2 << endl;
        double angle = angle2 - angle1;

        if (angle < 0) // angle이 음수인 경우 360도(2π 라디안)를 더해서 양수로 변환
            angle += 2 * M_PI;

        if (angle > M_PI) // angle이 180도(π 라디안)보다 크면 작은 각도로 변환
            angle = 2 * M_PI - angle;

        return angle; // 각도를 도로 ra로 변환
    }

    // 경로 시작점(기준점)과 각도와 반지름 센터점을 이용해 경로 생성을 위한 점들을 찍고 반환하는 함수

    vector<vector<double>> generateCirclePathPoints(const vector<double> &center, const vector<double> &startPoint, const vector<double> &endPoint, double angle, double angleStep)
    {
        vector<vector<double>> pathPoints;

        double start_dx = startPoint[0] - center[0];
        double start_dy = startPoint[1] - center[1]; // 시작점 단위벡터
        double startAngle = atan2(start_dy, start_dx);
        double end_dx = endPoint[0] - center[0];
        double end_dy = endPoint[1] - center[1]; // 끝점 단위벡터
        double radius = sqrt(start_dx * start_dx + start_dy * start_dy);

        bool clockwise;
        double cross_product = start_dx * end_dy - start_dy * end_dx; // 수정된 교차 곱 계산
        clockwise = (cross_product < 0);

        double angleStepRad = angleStep * M_PI / 180.0; // 각도를 라디안으로 변환
        double angleStepRad_dir = clockwise ? -angleStepRad : angleStepRad;

        for (double i = 0; i < angle; i += angleStepRad)
        {
            double angleInRadians = startAngle + (i * (clockwise ? -1 : 1)); // i를 라디안으로 해석
            cout << angleInRadians << endl;
            double x = center[0] + radius * cos(angleInRadians);
            double y = center[1] + radius * sin(angleInRadians);
            pathPoints.push_back({x, y});
        }

        return pathPoints;
    }

    // 두 점을 이은 직선 위에 일정 간격으로 점을 찍는 함수
    vector<vector<double>> generateLinePathPoints(const vector<double> &point1, const vector<double> &point2, double interval)
    {
        vector<vector<double>> pathPoints;
        double distance = sqrt(pow(point1[0] - point2[0], 2) + pow(point1[1] - point2[1], 2));
        int numPoints = static_cast<int>(distance / interval); // 찍을 점의 개수 계산
        double dx = (point2[0] - point1[0]) / numPoints;
        double dy = (point2[1] - point1[1]) / numPoints;
        for (int i = 0; i <= numPoints; ++i)
        {
            double x = point1[0] + i * dx;
            double y = point1[1] + i * dy;
            pathPoints.push_back({x, y}); // 점들을 생성
        }
        return pathPoints;
    }

    vector<vector<double>> ready_parking_path(vector<vector<double>> ready_line_points, vector<vector<double>> path_points_o4, vector<vector<double>> path_points_o3, vector<vector<double>> offset_line_points)
    {
        vector<vector<double>> ready_parking_path_points;

        // car_stop_point에서 gpath_point_o4까지 직선 위 점들
        for (int i = 0; i < ready_line_points.size() - 1; i++)
        {
            ready_parking_path_points.push_back(ready_line_points[i]);
            cout << "ready_parking_path_points : " << ready_parking_path_points[i][0] << ready_parking_path_points[i][1] << endl;
        }

        // 원4 위 점들
        for (int i = 0; i < path_points_o4.size() - 1; i++)
        {
            ready_parking_path_points.push_back(path_points_o4[i]);
            cout << "path_points_o4 : " << path_points_o4[i][0] << path_points_o4[i][1] << endl;
        }

        // 원3 위 점들
        for (int i = 0; i < path_points_o3.size() - 1; i++)
        {
            ready_parking_path_points.push_back(path_points_o3[i]);
            cout << "path_points_o3 : " << path_points_o3[i][0] << path_points_o3[i][1] << endl;
        }

        // pre_parking_start_point 부터 pre_parking_start_point_offset까지 직선 위 점들
        for (int i = 0; i < offset_line_points.size(); i++)
        {
            ready_parking_path_points.push_back(offset_line_points[i]);
            cout << "offset_line_points : " << offset_line_points[i][0] << offset_line_points[i][1] << endl;
        }

        return ready_parking_path_points; // 글로벌 패스 위 직선 점들, 원 4의 점들, 원 3의 점들 차례로 pushback
    }

    vector<vector<double>> reverse_parking_path(vector<vector<double>> path_points_psps, vector<vector<double>> path_points_o2, vector<vector<double>> path_points_o1, vector<vector<double>> reverse_line_points, vector<double> reverse_heading_point)
    {
        vector<vector<double>> reverse_parking_path_points;

        // psp점과 pre_psp점 사이의 직선 위에 path points
        for (int i = 0; i < path_points_psps.size() - 1; i++)
            reverse_parking_path_points.push_back(path_points_psps[i]);

        // 원2 위 점들
        for (int i = 0; i < path_points_o2.size() - 1; i++)
            reverse_parking_path_points.push_back(path_points_o2[i]);

        // 원1 위 점들
        for (int i = 0; i < path_points_o1.size() - 1; i++)
            reverse_parking_path_points.push_back(path_points_o1[i]);

        // for (int i = 0; i < reverse_line_points.size() - 1; i++)
        //     reverse_parking_path_points.push_back(reverse_line_points[i]);

        reverse_parking_path_points.push_back(reverse_heading_point);

        return reverse_parking_path_points;
    }

    vector<vector<double>> final_parking_path(vector<vector<double>> p1p2_offset_points)
    {
        vector<vector<double>> final_parking_path_points;

        for (int i = 0; i < p1p2_offset_points.size(); i++)
            final_parking_path_points.push_back(p1p2_offset_points[i]);

        return final_parking_path_points;
    }

    vector<vector<double>> out_parking_path(int pre_parking_start_point_index, vector<double> car_utm, const vector<vector<double>> &local_pos_arr)
    {
        vector<vector<double>> spline_points;

        spline_points.push_back(car_utm);

        for (int i = 0; i < 10; i++)
            spline_points.push_back(local_pos_arr[pre_parking_start_point_index + i]);

        return spline_points;
    }

    vector<vector<double>> fail_path(int car_stop_point_index, int pre_parking_start_point_index, vector<double> car_utm, const vector<vector<double>> &local_pos_arr)
    {
        vector<vector<double>> spline_points;

        spline_points.push_back(car_utm);

        for (int i = 0; i < 10; i++)
            spline_points.push_back(local_pos_arr[car_stop_point_index + i]);

        for (int i = 0; i < 10; i++)
            spline_points.push_back(local_pos_arr[pre_parking_start_point_index + i]);

        return spline_points;
    }
};

class ParkingNewClass
{
private:
    Local &local_;
    Lidar &lidar_;
    Path &local_path_;
    Control &control_;

    newPrlParking npp_;

    /****************변수들..****************/
    vector<vector<double>> parking_edge_out; // 주차꼭짓점 4개
    vector<double> parking_point1;
    vector<double> parking_point2;
    vector<double> parking_point3;
    vector<double> parking_point4;

    vector<double> parking_start_point;     // 주차 공간 들어가기 전 간보는 점..?
    vector<double> pre_parking_start_point; // x축 방향으로 offset
    vector<double> pre_parking_start_point_offset;

    vector<double> car_stop_point;    // 처음에 인지한테 점 받으면 멈추는 점 좌표값 저장하기 위한 변수
    vector<double> center_point_p1p4; // gpath_point_o4 찍기 위한 p1p4중점
    vector<double> gpath_point_o4;    // 글로벌 패스 점들 중에서 parking_start_point점과 가장 가까운 점
    vector<double> car_utm_KD;

    // 각 원들의 path points
    vector<vector<double>> ready_line_points; // car_stop_point부터 gpath_point_o4 까지 직선에 쭉 찍히는 점
    vector<vector<double>> path_points_o1;
    vector<vector<double>> path_points_o2;
    vector<vector<double>> path_points_o3;
    vector<vector<double>> path_points_o4;

    vector<vector<double>> offset_line_points;
    vector<vector<double>> reverse_line_points;

    vector<vector<double>> path_points_psps;   // psp점과 pre_psp점 사이의 직선 위에 path points
    vector<vector<double>> p1p2_offset_points; // p1p2 offset한 직전경로
    double radius_min = 2;                     // 원의 최소회전반경 반지름

    // 원과 원의 접점
    vector<double> vec_center_o1o2;
    vector<double> vec_center_o3o4;
    vector<double> unit_vec_center_o1o2;
    vector<double> unit_vec_center_o3o4;
    vector<double> contect_point_o1o2;
    vector<double> contect_point_o3o4;

    /// p1p2 사이 o1 지나는 점
    vector<double> mid_point_p1p2;
    vector<double> mid_point_p1p2_back;
    int center_point_p1p4_index;
    vector<double> center_point_p1p2;
    vector<double> center_point_p1p2_offset;
    vector<double> reverse_heading_point;

    // 각 원들의 각도
    double angle_o1;
    double angle_o2;
    double angle_o3;
    double angle_o4;

    // readypath
    int car_stop_point_index;
    int gpath_point_o4_index;

    /* path 생성용 파라미터*/
    const double CENTER_O1_Y_UP;
    const double PARKING_POINT_X_DOWN;
    const double PARKING_START_POINT_Y_UP;
    const double PRE_PARKING_START_POINT_X_UP;
    const double RADIUS_O1_SIZE;
    const double RADIUS_O3_SIZE;

    /* 정지거리 파라미터 */
    const double READY_PARKING_STOP_DIS;
    const double REVERSE_PARKING_STOP_DIS;

    /* 속도값 파라미터 */
    const double SERIAL_E_STOP;
    const double SERIAL_NOMAL;
    // const double SERIAL_SLOW;
    const double SERIAL_AACCELERATION;
    const double SERIAL_REVERSE;

    int s_m = SERIAL_NOMAL;
    int key = 0;

    // Define states for the state machine
    enum class State
    {
        INIT,
        WAIT_FOR_PK_OK,
        WAIT_FOR_PERCEPTION,
        SET_PARKING_POINT,
        CALCULATE_POINT,
        GENERATE_PRE_PARKING_PATH,
        GO_TO_PRE_POINT,
        GENERATE_PARKING_PATH,
        WAIT_FOR_REVERSE,
        REVERSE,
        WAIT_FOR_GENERATE_E_PATH,
        GENERATE_ESCAPE_PATH,
        WAIT_FOR_ESCAPE,
        ESCAPE,
        DONE
    };

    // 상태변수
    State current_state_ = State::INIT;

public:
    ParkingNewClass(Local &local, Lidar &lidar, Control &control, Path &local_path, nlohmann::json &param)
        : local_(local), lidar_(lidar), control_(control), local_path_(local_path), npp_(local, lidar, control, local_path) // npp_ 초기화 기존의 app는 안에 이걸 써줬어서 했어야됨!@!#
          ,
          CENTER_O1_Y_UP(param["center_o1_y_up"]),
          PARKING_POINT_X_DOWN(param["parking_point_x_down"]),
          PARKING_START_POINT_Y_UP(param["parking_start_point_y_up"]),
          PRE_PARKING_START_POINT_X_UP(param["pre_parking_start_point_x_up"]),
          RADIUS_O1_SIZE(param["radius_o1_size"]),
          RADIUS_O3_SIZE(param["radius_o3_size"]), // JSON의 키에 맞게 수정
          READY_PARKING_STOP_DIS(param["ready_parking_stop_dis"]),
          REVERSE_PARKING_STOP_DIS(param["reverse_parking_stop_dis"]),
          SERIAL_E_STOP(param["serial_e_stop"]),
          SERIAL_NOMAL(param["serial_nomal"]),
          SERIAL_AACCELERATION(param["serial_aacceleration"]),
          SERIAL_REVERSE(param["serial_reverse"])
    {
    }
    void new_prl_mission_func()
    {
        cout << "key : " << static_cast<int>(current_state_) << endl;
        const double *car_utm = local_.getAddCurCarUTMPos();                      // 현재 차량의 위치
        const vector<vector<double>> &local_pos_arr = local_path_.getRefPosArr(); // 로컬패스 쫙 받아오기
        const Path &global_path = local_.getRefCurGlobalPath();                   // 현재 글로벌 패스 객체
        const Path &global_path_another = local_.getRefNextGlobalPath();          // 다른 차선의 글로벌 패스

        switch (current_state_)
        {
        case State::INIT:
        {
            control_.setTargetVelocity(SERIAL_NOMAL);
            current_state_ = State::WAIT_FOR_PK_OK;
            break;
        }
        case State::WAIT_FOR_PK_OK:
        {
            if (lidar_.getIsParkingStop())
            {
                cout << "주차공간 확인" << endl;
                control_.setTargetVelocity(SERIAL_E_STOP);

                current_state_ = State::WAIT_FOR_PERCEPTION;
            }
            break;
        }
        case State::WAIT_FOR_PERCEPTION:
        {
            if (control_.waitAndDepart(3, SERIAL_E_STOP))
            {
                car_stop_point = local_.getCurClosestPos(car_utm);
                current_state_ = State::SET_PARKING_POINT;
            }
            break;
        }
        case State::SET_PARKING_POINT:
        {
            if (lidar_.getRefParkingEdgeOut().size() > 0)
            {
                cout << "여기까진 왔니?" << endl;
                parking_edge_out = lidar_.getRefParkingEdgeOut(); // 인지에게서 꼭짓점 4개 받아오기
                parking_point1 = parking_edge_out[0];

                parking_point3 = parking_edge_out[2];
                parking_point4 = parking_edge_out[3];

                current_state_ = State::CALCULATE_POINT;
            }
            break;
        }
        case State::CALCULATE_POINT:
        {
            // cout << "2 여기까진 왔니?" << endl;
            double distance_p3p4 = sqrt(pow(parking_point3[0] - parking_point4[0], 2) + pow(parking_point3[1] - parking_point4[1], 2)); /*************************************************************************************************/
            double dx_p3p4 = parking_point4[0] - parking_point3[0];
            double dy_p3p4 = parking_point4[1] - parking_point3[1];
            double unit_dx_p3p4 = dx_p3p4 / distance_p3p4;
            double unit_dy_p3p4 = dy_p3p4 / distance_p3p4;

            double distance_p1p4 = sqrt(pow(parking_point1[0] - parking_point4[0], 2) + pow(parking_point1[1] - parking_point4[1], 2)); /**************************************************************************************************************************/
            double dx_p1p4 = parking_point4[0] - parking_point1[0];
            double dy_p1p4 = parking_point4[1] - parking_point1[1];
            double unit_dx_p1p4 = dx_p1p4 / distance_p1p4;
            double unit_dy_p1p4 = dy_p1p4 / distance_p1p4;

            parking_point2 = {parking_point1[0] + distance_p3p4 * unit_dx_p3p4 * (-1), parking_point1[1] + distance_p3p4 * unit_dy_p3p4 * (-1)};
            vector<double> parking_offset_point1 = {parking_point1[0] + unit_dx_p1p4 * (PARKING_POINT_X_DOWN), parking_point1[1] + unit_dy_p1p4 * (PARKING_POINT_X_DOWN)}; // 1m offset 시키기
            vector<double> parking_offset_point3 = {parking_point3[0] + unit_dx_p1p4 * (PARKING_POINT_X_DOWN), parking_point3[1] + unit_dy_p1p4 * (PARKING_POINT_X_DOWN)};
            vector<double> parking_offset_point4 = {parking_point4[0] + unit_dx_p1p4 * (PARKING_POINT_X_DOWN), parking_point4[1] + unit_dy_p1p4 * (PARKING_POINT_X_DOWN)};
            parking_start_point = {parking_offset_point4[0] + PARKING_START_POINT_Y_UP * unit_dx_p3p4, parking_offset_point4[1] + PARKING_START_POINT_Y_UP * unit_dy_p3p4}; // P4기준으로 y축으로 1m offset

            pre_parking_start_point = {parking_start_point[0] + PRE_PARKING_START_POINT_X_UP * unit_dx_p1p4, parking_start_point[1] + PRE_PARKING_START_POINT_X_UP * unit_dy_p1p4}; // parking_start_point 기준으로 x축으로 1.5m offset
            pre_parking_start_point_offset = {{pre_parking_start_point[0] + 1.8 * unit_dx_p1p4, pre_parking_start_point[1] + 1.8 * unit_dy_p1p4}};
            center_point_p1p2 = {(parking_point1[0] + parking_point2[0]) * 0.5, (parking_point1[1] + parking_point2[1]) * 0.5}; // reverse 에 사용
            center_point_p1p2_offset = {{center_point_p1p2[0] + 1 * (-1) * unit_dx_p1p4, center_point_p1p2[1] + 1 * (-1) * unit_dy_p1p4}};

            center_point_p1p4 = {(parking_offset_point1[0] + parking_offset_point4[0]) * (0.5), (parking_offset_point1[1] + parking_offset_point4[1]) * (0.5)};
            gpath_point_o4 = local_.getCurClosestPos(center_point_p1p4);
            gpath_point_o4_index = local_.getCurClosestIndex(gpath_point_o4);

            double radius_o1 = RADIUS_O1_SIZE;
            vector<double> center_o1 = {parking_offset_point1[0] + CENTER_O1_Y_UP * unit_dx_p3p4, parking_offset_point1[1] + CENTER_O1_Y_UP * unit_dy_p3p4}; // 처음그려지는 원 중심 x축 위로 오프셋 해보면 좋을듯?

            double radius_o2 = npp_.calculateContactRadiusUp(center_o1, radius_o1, parking_start_point, unit_dx_p3p4, unit_dy_p3p4);
            vector<double> center_o2 = {parking_start_point[0] + radius_o2 * (-1) * unit_dx_p3p4, parking_start_point[1] + radius_o2 * (-1) * unit_dy_p3p4};

            double radius_o3 = RADIUS_O3_SIZE;
            vector<double> center_o3 = {pre_parking_start_point[0] + RADIUS_O3_SIZE * unit_dx_p3p4, pre_parking_start_point[1] + RADIUS_O3_SIZE * unit_dy_p3p4};

            double radius_o4 = npp_.calculateContactRadiusUp(center_o3, radius_o3, gpath_point_o4, unit_dx_p3p4, unit_dy_p3p4);
            vector<double> center_o4 = {gpath_point_o4[0] + radius_o4 * (-1) * unit_dx_p3p4, gpath_point_o4[1] + radius_o4 * (-1) * unit_dy_p3p4};
            // 변수 값 출력
            cout << "radius_o1: " << radius_o1 << endl;
            cout << "center_o1: (" << center_o1[0] << ", " << center_o1[1] << ")" << endl;

            cout << "radius_o2: " << radius_o2 << endl;
            cout << "center_o2: (" << center_o2[0] << ", " << center_o2[1] << ")" << endl;

            cout << "radius_o3: " << radius_o3 << endl;
            cout << "center_o3: (" << center_o3[0] << ", " << center_o3[1] << ")" << endl;

            cout << "radius_o4: " << radius_o4 << endl;
            cout << "center_o4: (" << center_o4[0] << ", " << center_o4[1] << ")" << endl;

            vec_center_o1o2 = {center_o2[0] - center_o1[0], center_o2[1] - center_o1[1]};
            unit_vec_center_o1o2 = {vec_center_o1o2[0] / (radius_o1 + radius_o2), vec_center_o1o2[1] / (radius_o1 + radius_o2)};
            contect_point_o1o2 = {center_o1[0] + radius_o1 * unit_vec_center_o1o2[0], center_o1[1] + radius_o1 * unit_vec_center_o1o2[1]};

            vec_center_o3o4 = {center_o4[0] - center_o3[0], center_o4[1] - center_o3[1]};
            unit_vec_center_o3o4 = {vec_center_o3o4[0] / (radius_o3 + radius_o4), vec_center_o3o4[1] / (radius_o3 + radius_o4)};

            contect_point_o3o4 = {center_o3[0] + radius_o3 * unit_vec_center_o3o4[0], center_o3[1] + radius_o3 * unit_vec_center_o3o4[1]};

            mid_point_p1p2 = {center_o1[0] + radius_o1 * (-1) * unit_dx_p3p4, center_o1[1] + radius_o1 * (-1) * unit_dy_p3p4}; // 원1이랑 p1p2직선과 만나는 점
            mid_point_p1p2_back = {mid_point_p1p2[0] + unit_dx_p1p4 * (-1) * 1, mid_point_p1p2[1] + unit_dy_p1p4 * (-1) * 1};
            reverse_heading_point = {parking_offset_point1[0] + radius_min * (-1) * unit_dx_p1p4 + unit_dx_p3p4 * (1.5), parking_offset_point1[1] + radius_min * (-1) * unit_dy_p1p4 + unit_dy_p3p4 * (1.5)};

            angle_o1 = npp_.calculateAngle(center_o1, contect_point_o1o2, mid_point_p1p2);
            angle_o2 = npp_.calculateAngle(center_o2, parking_start_point, contect_point_o1o2);
            angle_o3 = npp_.calculateAngle(center_o3, contect_point_o3o4, pre_parking_start_point);
            angle_o4 = npp_.calculateAngle(center_o4, gpath_point_o4, contect_point_o3o4);
            cout << "angle_o1 : " << angle_o1 << endl;
            cout << "angle_o2 : " << angle_o2 << endl;
            cout << "angle_o3 : " << angle_o3 << endl;
            cout << "angle_o4 : " << angle_o4 << endl;

            ready_line_points = npp_.generateLinePathPoints(car_stop_point, gpath_point_o4, 0.05);
            offset_line_points = npp_.generateLinePathPoints(pre_parking_start_point, pre_parking_start_point_offset, 0.05);
            reverse_line_points = npp_.generateLinePathPoints(mid_point_p1p2, mid_point_p1p2_back, 0.05);

            // 각 원들의 path points
            path_points_o1 = npp_.generateCirclePathPoints(center_o1, contect_point_o1o2, mid_point_p1p2, angle_o1, 5.0); // anglestep = 5.0 => 5도 마다 쪼갠다.
            path_points_o2 = npp_.generateCirclePathPoints(center_o2, parking_start_point, contect_point_o1o2, angle_o2, 5.0);
            path_points_o3 = npp_.generateCirclePathPoints(center_o3, contect_point_o3o4, pre_parking_start_point, angle_o3, 5.0);
            path_points_o4 = npp_.generateCirclePathPoints(center_o4, gpath_point_o4, contect_point_o3o4, angle_o4, 5.0);

            // psp점과 pre_psp점 사이의 직선 위에 path points
            path_points_psps = npp_.generateLinePathPoints(pre_parking_start_point_offset, parking_start_point, 0.2);

            // p1p2 offset한 직전경로
            cout << "center_o3 : " << center_o3[0] << ", " << center_o3[1] << endl;
            cout << "gpath_point_o4 : " << gpath_point_o4[0] << ", " << gpath_point_o4[1] << endl;
            cout << "gpath_point_o4 : " << unit_dx_p3p4 << ", " << unit_dy_p3p4 << endl;

            current_state_ = State::GENERATE_PRE_PARKING_PATH;

            break;
        }
        case State::GENERATE_PRE_PARKING_PATH:
        {

            cout << "ready_parking_path 생성중" << endl;

            vector<vector<double>> ready_parking_path_points = npp_.ready_parking_path(ready_line_points, path_points_o4, path_points_o3, offset_line_points);

            // ready_parking_path_points들 보간
            auto t = generateSplinePath(ready_parking_path_points);

            car_stop_point_index = local_.getCurClosestIndex(car_stop_point);
            // tuple의 각 요소에 접근
            vector<vector<double>> path = get<0>(t);
            vector<double> ryaw = get<1>(t);
            vector<double> rk = get<2>(t);

            // path 객체 생성
            Path ready_parking_path("ready_parking_path", path, ryaw, rk);
            // path update (매개변수 글로벌패스 첫덱스, 막인덱스, 바꿔낄 패스 이름, 로컬패스 첫인덱스, 막인덱스)
            local_path_.updatePath(car_stop_point_index, local_pos_arr.size(), ready_parking_path, 0, get<0>(t).size());

            current_state_ = State::GO_TO_PRE_POINT;
            break;
        }
        case State::GO_TO_PRE_POINT:
        {
            control_.setTargetVelocity(SERIAL_NOMAL);

            car_utm_KD = local_path_.getClosestPos(car_utm);
            pre_parking_start_point = local_path_.getClosestPos(pre_parking_start_point);

            double dis = sqrt(pow(car_utm_KD[0] - pre_parking_start_point[0], 2) + pow(car_utm_KD[1] - pre_parking_start_point[1], 2));

            cout << "pre_parking_start_point 까지 남은 거리" << dis << endl;

            if (dis <= READY_PARKING_STOP_DIS)
            {
                control_.setTargetVelocity(0);
                current_state_ = State::WAIT_FOR_REVERSE;
            }

            break;
        }
        case State::WAIT_FOR_REVERSE:
        {
            if (control_.waitAndDepart(2, SERIAL_E_STOP))
            {
                current_state_ = State::GENERATE_PARKING_PATH;
            }
            break;
        }
        case State::GENERATE_PARKING_PATH:
        {
            cout << "reverse_parking_path 생성중" << endl;

            vector<vector<double>> reverse_parking_path_points = npp_.reverse_parking_path(path_points_psps, path_points_o2, path_points_o1, reverse_line_points, reverse_heading_point);
            // reverse_parking_path_points들 보간
            auto t = generateSplinePath(reverse_parking_path_points);
            // tuple의 각 요소에 접근
            vector<vector<double>> path = get<0>(t);
            vector<double> ryaw = get<1>(t);
            vector<double> rk = get<2>(t);
            // path 객체 생성
            Path reverse_parking_path("reverse_parking_path", path, ryaw, rk);

            local_path_.updatePath(0, local_pos_arr.size(), reverse_parking_path, 0, get<0>(t).size());

            control_.setTargetVelocity(SERIAL_REVERSE);
            current_state_ = State::REVERSE;
            break;
        }
        case State::REVERSE:
        {
            Path &global_path = local_.getRefIdxPath(0);
            car_utm_KD = global_path.getClosestPos(car_utm);
            // car_utm_KD = {car_utm[0], car_utm[1]};

            std::vector<double> path_pos = global_path.getClosestPos(center_point_p1p2);
            double dis = sqrt(pow(car_utm_KD[0] - path_pos[0], 2) + pow(car_utm_KD[1] - path_pos[1], 2));
            cout << "center_point_p1p2 까지 남은 거리" << dis << endl;

            double cur_yaw = local_.getCurCarYaw();
            double dx = parking_point4[0] - parking_point1[0];
            double dy = parking_point4[1] - parking_point1[1];
            double target_yaw = atan2(dy, dx);
            if (dis < REVERSE_PARKING_STOP_DIS + 1 && (dis <= REVERSE_PARKING_STOP_DIS || std::abs(cur_yaw - target_yaw) < 0.005))
            {
                cout << "center_point_p1p2 도착." << endl;
                control_.setTargetVelocity(0);
                current_state_ = State::WAIT_FOR_GENERATE_E_PATH;
            }

            // if (dis <= REVERSE_PARKING_STOP_DIS)
            // {
            //     cout << "center_point_p1p2 도착." << endl;
            //     control_.setTargetVelocity(0);
            //     current_state_ = State::GENERATE_ESCAPE_PATH;
            // }
            break;
        }
        case State::WAIT_FOR_GENERATE_E_PATH:
        {
            if (control_.waitAndDepart(3, 0))
            {
                current_state_ = State::GENERATE_ESCAPE_PATH;
            }
            break;
        }
        case State::GENERATE_ESCAPE_PATH:
        {
            int pre_parking_start_point_index = global_path_another.getClosestIndex(car_utm) + 20;
            vector<vector<double>> out_parking_path_points = npp_.out_parking_path(pre_parking_start_point_index, {car_utm[0], car_utm[1]}, global_path_another.getRefPosArr());
            // out_parking_path_points들 보간
            auto t = generateSplinePath(out_parking_path_points);
            // tuple의 각 요소에 접근
            vector<vector<double>> path = get<0>(t);
            vector<double> ryaw = get<1>(t);
            vector<double> rk = get<2>(t);
            // path 객체 생성
            Path out_parking_path("out_parking_path", path, ryaw, rk);

            local_path_.updatePath(0, local_path_.getRefPosArr().size(), out_parking_path, 0, get<0>(t).size());
            local_path_.updatePath(local_path_.getRefPosArr().size() - 1, local_path_.getRefPosArr().size(), global_path_another, pre_parking_start_point_index - 1, global_path_another.getRefPosArr().size());

            current_state_ = State::WAIT_FOR_ESCAPE;
            break;
        }
        case State::WAIT_FOR_ESCAPE:
        {
            if (control_.waitAndDepart(1, SERIAL_NOMAL))
            {
                cout << "생성\n";
                current_state_ = State::ESCAPE;
            }

            cout << "out_parking_path 생성중" << endl;
            break;
        }
        case State::ESCAPE:
        {
            vector<double> pre_parking_start_point_pos = global_path_another.getClosestPos(pre_parking_start_point);
            car_utm_KD = global_path_another.getClosestPos(car_utm);
            double dis = sqrt(pow(car_utm_KD[0] - pre_parking_start_point_pos[0], 2) + pow(car_utm_KD[1] - pre_parking_start_point_pos[1], 2));
            cout << "pre_parking_start_point_pos 까지 남은 거리" << dis << endl;

            if (dis <= 0.5)
            {
                cout << "pre_parking_start_point_pos 도착." << endl;
                current_state_ = State::DONE;
            }
            break;
        }
        case State::DONE:
        {
            control_.setTargetVelocity(SERIAL_AACCELERATION);
            cout << "결승선으로~~~~" << endl;
            break;
        }
        }
    }
};
