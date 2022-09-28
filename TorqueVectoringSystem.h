#ifndef TORQUE_VECTORING_SYSTEM_H
#define TORQUE_VECTORING_SYSTEM_H

#include "mbed.h"
#include <math.h>

#include "HallSensor.h"
#include "MPU6050.h"

#define MOTOR_POLE_PAIR                 7


#define WHEEL_RADIUS                    (0.53 / 2.0) // ev wheel
#define GEAR_RATIO                      5.27
//#define PI                              3.141592


#define DEFAULT_VOLTAGE_INPUT           0.5
#define MAX_RESISTOR_ANGLE              300.0
#define MAX_RESISTOR_LIMITED_ANGLE      240.0
#define MAX_HANDLE_ANGLE                300.0
#define MAX_STEERING_BIG_ANGLE          67.238
#define MAX_STEERING_SMALL_ANGLE        55.267
#define MAX_STEERING_ANGLE              (MAX_STEERING_BIG_ANGLE + MAX_STEERING_SMALL_ANGLE) / 2.0


#define WHEEL_BASE                      1.371
#define TRACK                           1.300
#define ALPHA                           0.85


#define FL                              0
#define FR                              1
#define RL                              2
#define RR                              3
#define MAX_TORQUE                      17.0
#define ACTUAL_MAX_TORQUE_NY            10.0
#define ACTUAL_MAX_TORQUE_SW            8.0
#define ACTUAL_MAX_TORQUE_SH            12.0    
#define ACTUAL_MAX_TORQUE_CH            9.0
#define CONTROLLER_INPUT_VOLT_RANGE     5.0


#define KP_FOR_TORQUE_FL                0.1
#define KP_FOR_TORQUE_FR                0.1
#define KP_FOR_TORQUE_RL                0.1
#define KP_FOR_TORQUE_RR                0.1
#define KT                              17/148  //motor torque constant

#define CURRENT_SENSOR_VALUE_m50A       1.1906
#define CURRENT_SENSOR_VALUE_100A       2.7056

#define AMP_RATE_MOTOR                  400.0
#define AMP_RATE_BATTERY                100.0
#define SHUNT_R                         0.00005
#define ANALOG_RANGE                    3.3

#define TORQUE_VECTORING_RATE           0
#define FLOAT_MIN                       -1000.0

#define KP_FOR_THROTTLE_FL              0.5
#define KP_FOR_THROTTLE_FR              0.5
#define KP_FOR_THROTTLE_RL              0.5
#define KP_FOR_THROTTLE_RR              0.5


#define LOWER_BOUND                     0.0
#define UPPER_BOUND                     1.0

#define FR_OPAMP_GAIN                   1.515
#define FL_OPAMP_GAIN                   1.515
#define RR_OPAMP_GAIN                   1.515
#define RL_OPAMP_GAIN                   1.515
#define IDEAL_OPAMP_GAIN                1.515


#define PEDAL_MIN_VALUE                 0.213       // mbed analogread value
#define PEDAL_MAX_VALUE                 0.68        // mbed analogread value


#define THROTTLE_MIN                    0
#define THROTTLE_MAX                    3.3            

#define PRESENT                         1
#define PREVIOUS                        0

#define PWM_PERIOD_US                   25
#define PWM_PERIOD_MS                   1

#define CONTROLLER_IN_MIN               0.1
#define CONTROLLER_IN_MAX               0.8

#define TVS_ON                          1
#define TVS_OFF                         0
#define MOTOR_ON                        1
#define MOTOR_OFF                       0




class TorqueVectoringSystem {

public:
    TorqueVectoringSystem(
        PinName TVS_SWITCH_PIN, PinName FL_HALL_PIN, PinName FR_HALL_PIN, PinName RL_HALL_PIN, PinName RR_HALL_PIN, 
        PinName HANDLE_SENSOR_PIN, PinName MPU_SDA, PinName MPU_SCL, PinName PEDAL_SENSOR_PIN,
        PinName FL_CURRENT_SENSOR_PIN, PinName FR_CURRENT_SENSOR_PIN, PinName RL_CURRENT_SENSOR_PIN, PinName RR_CURRENT_SENSOR_PIN,
        PinName FL_OUTPUT_THROTTLE_PIN, PinName FR_OUTPUT_THROTTLE_PIN, PinName RL_OUTPUT_THROTTLE_PIN, PinName RR_OUTPUT_THROTTLE_PIN);


    //HallSensor FL_Hall_A(FL_HALL_PIN);
    //HallSensor FR_Hall_A(FR_HALL_PIN);
    HallSensor RL_Hall_A;
    HallSensor RR_Hall_A;

    MPU6050 mpu;

    AnalogIn Handle_Sensor;

    AnalogIn FL_Current_OUT;
    AnalogIn FR_Current_OUT;
    AnalogIn RL_Current_OUT;
    AnalogIn RR_Current_OUT;

    AnalogIn Pedal_Sensor;
    
    PwmOut FL_Throttle_PWM;
    PwmOut FR_Throttle_PWM;
    PwmOut RL_Throttle_PWM;
    PwmOut RR_Throttle_PWM;



    float f_motor_current_FL_A, f_motor_current_FR_A, f_motor_current_RL_A, f_motor_current_RR_A;

    float f_motor_RPM_FL;
    float f_motor_RPM_FR;
    float f_motor_RPM_RL;
    float f_motor_RPM_RR;

    //pedal box
    float f_pedal_sensor_value;
    float f_pedal_modified_sensor_value;

    int i_PWR_percentage;
    //handle
    float f_steering_sensor_value;


    float f_yaw_rate_meas_filtered_degs;

    float f_wheel_angle_deg;

    float f_vel_FL_ms;
    float f_vel_FR_ms;
    float f_vel_RL_ms;
    float f_vel_RR_ms;

    float f_vehicle_vel_ms;

    float f_yawrate_input_deg;

    float f_wheel_torque_FL_Nm;
    float f_wheel_torque_FR_Nm;
    float f_wheel_torque_RL_Nm;
    float f_wheel_torque_RR_Nm;

    float f_PID_yaw_rate2torque_FL_Nm, f_PID_yaw_rate2torque_FR_Nm, f_PID_yaw_rate2torque_RL_Nm, f_PID_yaw_rate2torque_RR_Nm;
    float f_measured_torque_FL_Nm, f_measured_torque_FR_Nm, f_measured_torque_RL_Nm, f_measured_torque_RR_Nm;

    float f_torque_FL_Nm;
    float f_torque_FR_Nm;
    float f_torque_RL_Nm;
    float f_torque_RR_Nm;

    float f_output_throttle_FL, f_output_throttle_FR, f_output_throttle_RL, f_output_throttle_RR;
    

    float f_PID_throttle_FL, f_PID_throttle_FR, f_PID_throttle_RL, f_PID_throttle_RR;
    float f_PWM_input_FL, f_PWM_input_FR, f_PWM_input_RL, f_PWM_input_RR;


    // IMU센서 관련 값
    float IMU_gx, IMU_gy, IMU_gz, IMU_ax, IMU_ay, IMU_az;
    float f_yawrate_meas_degs;


    float CvtRPM2Vel(float f_motor_RPM);
    float CalAvgVel(float f_velocity1_ms, float f_velocity2_ms);
    float CalHandlingVolt2WheelSteeringAngle(float f_handling_sensor_value);
    float CalInputYawRate(float f_wheel_steering_angle_deg, float f_avg_vel_ms);
    float IMUFilter(float i_IMU_yaw_rate_radps);

    bool WheelSteeringAngle2Torque(float f_wheel_steering_angle_deg, float f_pedal_sensor_value,
        float& f_wheel_torque_FL_Nm, float& f_wheel_torque_FR_Nm,
        float& f_wheel_torque_RL_Nm, float& f_wheel_torque_RR_Nm);

    void PIDYawRate2Torque(float f_input_yaw_rate_radps, float f_filtered_yaw_rate_radps,
        float& f_PID_yaw_rate2torque_FL_Nm, float& f_PID_yaw_rate2torque_FR_Nm,
        float& f_PID_yaw_rate2torque_RL_Nm, float& f_PID_yaw_rate2torque_RR_Nm);

    float OpAmp2Current(float f_opamp_ADC);
    float ReadCurrentSensor(float current_sensor_value);

    float CvtCurrent2Torque(float f_motor_current_A);

    float Torque2Throttle(float f_torque_Nm);

    float PIDforThrottle(float f_torque_Nm, float f_measured_torque_Nm, int direction);

    float SumFFandPID(float f_output_throttle, float f_PID_throttle);
    
    float ModifyPedalThrottle(float input, float in_min, float in_max, float out_min, float out_max);

    float map_f(float input, float in_min, float in_max, float out_min, float out_max);


    void process_accel();

    void process_accel(float accel_value);

    void process_off(
    PinName PEDAL_SENSOR_PIN, PinName FL_OUTPUT_THROTTLE_PIN, PinName FR_OUTPUT_THROTTLE_PIN, PinName RL_OUTPUT_THROTTLE_PIN, PinName RR_OUTPUT_THROTTLE_PIN);


   
};




#endif