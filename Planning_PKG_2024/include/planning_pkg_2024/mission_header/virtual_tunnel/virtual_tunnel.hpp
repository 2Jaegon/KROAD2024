// /***
//  * 작성자: 조윤서
//  * 설명: 터널 가벽+가천장 설정 및 음영구간 세팅해주는 클래스
//  * Style
//  * -클래스: PascalCase
//  * -멤버변수: snake_case_   (snake_case 뒤에 밑줄)(파이썬 사용 x)
//  * -함수: camelCase
//  * -변수: snake_case
//  * -상수: kPascalCase   (PascalCase앞에 k 접두어)
//  ***/

// /*
// 해당 함수는 대회에서 쓰는 거 절대 아님!!!
// */

// ////////////////////////////////////////////////////////////////////////
// //                             구 버 전                                 //
// ////////////////////////////////////////////////////////////////////////

// #ifndef VIRTUAL_TN_HPP_
// #define VIRTUAL_TN_HPP_

// #include <iostream>
// #include <stdlib.h>
// #include <random>
// #include "local.hpp"
// #include "utility_function.hpp"

// class VirtualTunnel
// {
// private:
//     Local &local_;

// public:
//     VirtualTunnel(Local &local) : local_(local) {};
//     // ~VirtualTunnel(){};

//     void setShadedSection()
//     { // 음영 구간을 설정해주는 함수(GPS 튀게 해줌)
//         double newX = local_.getAddCurCarUTMPos()[0] + randomNumCreator(-100.0, 100.0);
//         double newY = local_.getAddCurCarUTMPos()[1] + randomNumCreator(-100.0, 100.0);
//         local_.setRecentGPS(newX, newY);
//         return;
//     }

//     // 산단 차선 9.3M

//     void setVirtualWall(){}
// };

// #endif