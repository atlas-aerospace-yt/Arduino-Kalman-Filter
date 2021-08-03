/*
*
*	This the Library for the Atlas Aerospace Kalman filter.
*   This library may be modified to suit your specific use-case.
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

struct Kalman {

  // variables for the filter

  double  A, B, C;
  double  Q, R;
  double  U = 0.0, Y = 0.0;
  double  x_hat = 0.0, P = 0.0;

  // kalman loop function

  double Kalman::Kalman_Filter_Update() {

    double x_hat_minus_1 = A * x_hat + B * U;

    P = A * P * A + Q;

    double K = P * C * (1.0f / (C * P * C + R));

    x_hat_minus_1 += K * (Y - C * x_hat_minus_1);

    P = (1 - K *  C) *  P;

    x_hat = x_hat_minus_1;

    return x_hat;
  }
};
