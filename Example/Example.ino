/*
*	
*	This the example code for the Atlas Aerospace Kalman filter.
*	
* 
*	Written by:
* 
*	Atlas Aerospace 03/08/2021
*	
* 
*	This is the example file which shows you how to properly use 
*	the library.
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


#include "Filter.h"

// Declaration of your system dynamics.
// A B C Q R
// Q and R affect how aggressive the filtering is.

Kalman x_axis = { 1, 1, 1, 5, 80 };
Kalman y_axis = { 1, 1, 1, 5, 80 };
Kalman z_axis = { 1, 1, 1, 5, 80 };

void setup() {

}

void loop() {

	// This will depend on what imu you are using
	// and the syntax for your specific library.

	your_gyro.update();
	

	// The x_axis.Y is being set to the angle of the imu.
	// Y is the only variable you need to update here.
	// Again, the "your_gyro.get_angle_x()" will depend on your use case!

	x_axis.Y = your_gyro.get_angle_x();
	y_axis.Y = your_gyro.get_angle_y();
	z_axis.Y = your_gyro.get_angle_z();


	// This gets the filtered angle from the kalman filter by 
	// using the .Kalman_Filter_Update() function.

	your_gyro.x = x.Kalman_Filter_Update();
	your_gyro.y = y.Kalman_Filter_Update();
	your_gyro.z = z.Kalman_Filter_Update();
}