#ifndef KAI_MESSAGE_FRAME
#define KAI_MESSAGE_FRAME

#include "mbed.h"

#include "ros.h"
#include "sensor_msgs/BatteryState.h"

#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/NavSatStatus.h"

#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/IMU.h"




class CarState {
protected:

public:
	float f_wheel_velocity_FL_ms = 0;
	float f_wheel_velocity_FR_ms = 0;
	float f_wheel_velocity_RL_ms = 0;
	float f_wheel_velocity_RR_ms = 0;
	
	float f_car_velocity_ms = 0;
	
	float f_motor_torque_FL_Nm = 0;
	float f_motor_torque_FR_Nm = 0;
	float f_motor_torque_RL_Nm = 0;
	float f_motor_torque_RR_Nm = 0;
};


class CarControlSignal {

public:
	float brake = 0;			    // is it right???
	float steering = 0; 		  // is it right???
	float acceleration = 0;		// is it right???
}


#endif	// KAI_MESSAGE_FRAME
