/*
*
*	This the Library for the Atlas Aerospace Kalman filter.
* This library may be modified to suit your specific use-case.
*
*
*	Written by:
*
*	Atlas Aerospace 03/08/2021
*
*
*	Some use cases may be:
*
*	- Filtering IMU data
*	- Filtering Barometer data
*	- Filtering Ultrasonic Distance sensors
*
*
*	My YouTube:
*
*	https://www.youtube.com/channel/UCWd6oqc8nbL-EX3Cxxk8wFA
*/

#pragma once
#include "BasicLinearAlgebra.h"
#include "math.h"

// Kalman Filter Struct
struct Kalman{

  // State space dynamics (EDIT TO MATCH YOUR STATE SPACE DYNAMICS)
  BLA::Matrix<2,2> A;
  BLA::Matrix<2> B;
  BLA::Matrix<2,2> Q;
  BLA::Matrix<1> R;
  BLA::Matrix<2> x;
  BLA::Matrix<1> u;
  BLA::Matrix<2,2> P = {0.0f, 0.0f, 0.0f, 0.0f};
  BLA::Matrix<2> K = {0.0f, 0.0f};
  BLA::Matrix<1,2> H = {0.0f, 1.0f};
  BLA::Matrix<2,2> I = {0.0f, 0.0f, 0.0f, 0.0f};
  BLA::Matrix<2> xPredicted;
  BLA::Matrix<2,2> pMinus;
  BLA::Matrix<1> S;
  
  // Filter
  void updateState(BLA::Matrix<1> y, BLA::Matrix<1> u, float dt){

    // Update the Kalman filter
    xPredicted = A * x + B * u;
    xPredicted = x + xPredicted * dt;
    pMinus = A * P * ~A + Q;

    // Correct the Kalman filter error
    S = H * pMinus * ~H + R;
    this-> K = pMinus * ~H * BLA::Invert(S);
    this-> x = xPredicted + K * (y - H * xPredicted);
    this-> P = (I - K * H) * pMinus;
  }
};
