/*
         _____                              _   __     ___      _____    
        |_   _|                            | | / /    / _ \    |_   _|   
          | |    ___   __ _  _ __ ___      | |/ /    / /_\ \     | |     
          | |   / _ \ / _` || '_ ` _ \     |    \    |  _  |     | |     
          | |  |  __/| (_| || | | | | |    | |\  \ _ | | | | _  _| |_  _ 
          \_/   \___| \__,_||_| |_| |_|    \_| \_/(_)\_| |_/(_) \___/ (_)

 _____  _____  _____  _____  ______           _                 _____  _   _  ______                     _                                
/ __  \|  _  |/ __  \/ __  \ | ___ \         (_)               |  ___|| | | | |  _  \                   | |                               
`' / /'| |/' |`' / /'`' / /' | |_/ /  __ _    _   __ _  ______ | |__  | | | | | | | |  ___ __   __  ___ | |  ___   _ __    ___  _ __  ___ 
  / /  |  /| |  / /    / /   | ___ \ / _` |  | | / _` ||______||  __| | | | | | | | | / _ \\ \ / / / _ \| | / _ \ | '_ \  / _ \| '__|/ __|
./ /___\ |_/ /./ /___./ /___ | |_/ /| (_| |  | || (_| |        | |___ \ \_/ / | |/ / |  __/ \ V / |  __/| || (_) || |_) ||  __/| |   \__ \
\_____/ \___/ \_____/\_____/ \____/  \__,_|  | | \__,_|        \____/  \___/  |___/   \___|  \_/   \___||_| \___/ | .__/  \___||_|   |___/
                                            _/ |                                                                  | |                     
                                           |__/                                                                   |_|                     

    Torque Vectoring System for 4-wheel independent driving car
    
    + ROS publisher for management system
    + Autonomous System State Indicator publisher

    Developers : 
        박성훈
        이나영
        김현우
        전민경

    Especially thanks to Jiwon Seok, A.K.A. PowerRanger RED
*/

#include <iostream>
#include "TorqueVectoringSystem.h"

#include "ros.h"
#include "CarState.h"


// #include "AutonomousMessage.h"
// #include "std_msgs/Float32.h"
// #include "std_msgs/Int8.h"




float f_global_vel_RL_ms = 0.0;
float f_global_vel_RR_ms = 0.0;
float f_global_vehicle_vel_ms = 0.0;
int i_global_PWR_percentage = 0;



void CarDriving() {


    PinName TVS_SWITCH_PIN = p30;

    PinName FL_HALL_PIN = p11;
    PinName FR_HALL_PIN = p12;
    PinName RL_HALL_PIN = p13;
    PinName RR_HALL_PIN = p14;

    PinName HANDLE_SENSOR_PIN = p19;

    PinName FL_CURRENT_SENSOR_PIN = p15;
    PinName FR_CURRENT_SENSOR_PIN = p16;
    PinName RL_CURRENT_SENSOR_PIN = p17;
    PinName RR_CURRENT_SENSOR_PIN = p18;
    

    PinName MPU_SDA = p28;
    PinName MPU_SCL = p27;
    
    PinName PEDAL_SENSOR_PIN = p20;

    PinName FL_OUTPUT_THROTTLE_PIN = p21;
    PinName FR_OUTPUT_THROTTLE_PIN = p22;
    PinName RL_OUTPUT_THROTTLE_PIN = p23;
    PinName RR_OUTPUT_THROTTLE_PIN = p24;

    


    TorqueVectoringSystem TVS(
        TVS_SWITCH_PIN, FL_HALL_PIN, FR_HALL_PIN, RL_HALL_PIN, RR_HALL_PIN, 
        HANDLE_SENSOR_PIN, MPU_SDA, MPU_SCL, PEDAL_SENSOR_PIN,
        FL_CURRENT_SENSOR_PIN, FR_CURRENT_SENSOR_PIN, RL_CURRENT_SENSOR_PIN, RR_CURRENT_SENSOR_PIN,
        FL_OUTPUT_THROTTLE_PIN, FR_OUTPUT_THROTTLE_PIN, RL_OUTPUT_THROTTLE_PIN, RR_OUTPUT_THROTTLE_PIN
    );
    // mpu 시작했는지 안했는지 표시해주는 함수 작성해야되는데 귀찮음
    TVS.mpu.start();


        
    while(1) {
        
        TVS.process_accel();
        f_global_vel_RL_ms = TVS.f_vel_RL_ms;
        f_global_vel_RR_ms = TVS.f_vel_RR_ms;
        f_global_vehicle_vel_ms = TVS.f_vehicle_vel_ms;
        i_global_PWR_percentage = TVS.i_PWR_percentage;
    }
}




void ROSPub() {

    kai_msgs::CarState kai_msg;
    ros::Publisher carstate("carstate", &kai_msg);

    ros::NodeHandle nh;
    nh.initNode();
    nh.advertise(carstate);

    kai_msg.f_wheel_velocity_FL_ms = 0.0;
    kai_msg.f_wheel_velocity_FR_ms = 0.0;
    kai_msg.f_motor_torque_FL_Nm = 0.0;
    kai_msg.f_motor_torque_FR_Nm = 0.0;
    kai_msg.f_motor_torque_RL_Nm = 0.0;
    kai_msg.f_motor_torque_RR_Nm = 0.0;
    kai_msg.c_torque_mode_flag = TVS_ON;
    kai_msg.c_motor_mode_flag[FL] = MOTOR_ON;
    kai_msg.c_motor_mode_flag[FR] = MOTOR_ON;
    kai_msg.c_motor_mode_flag[RL] = MOTOR_ON;
    kai_msg.c_motor_mode_flag[RR] = MOTOR_ON;

    while(1) 
    {
        kai_msg.f_wheel_velocity_RL_ms = f_global_vel_RL_ms;
        kai_msg.f_wheel_velocity_RR_ms = f_global_vel_RR_ms;
        kai_msg.f_car_velocity_ms = f_global_vehicle_vel_ms;
        kai_msg.i_throttle = i_global_PWR_percentage;

        carstate.publish (&kai_msg);
        nh.spinOnce();
        wait_ms(50);
    }

} 




int main(){

    Thread thread_accel;
    thread_accel.start(CarDriving);
    
    ROSPub();
}



/*
void ros_thread(){

    ros::NodeHandle nh;
    kai_msgs::CarState kai_msg;
    ros::Publisher carstate("carstate", &kai_msg);

    while(true){
        
        kai_msg.f_wheel_velocity_FL_ms=TVS.f_vel_FL_ms;
        kai_msg.f_wheel_velocity_FR_ms=TVS.f_vel_FR_ms;
        kai_msg.f_wheel_velocity_RL_ms=TVS.f_vel_RL_ms;
        kai_msg.f_wheel_velocity_RR_ms=TVS.f_vel_RR_ms;
        kai_msg.f_car_velocity_ms=TVS.f_vehicle_vel_ms;
        kai_msg.f_motor_torque_FL_Nm=TVS.f_measured_torque_FL_Nm;
        kai_msg.f_motor_torque_FR_Nm=TVS.f_measured_torque_FR_Nm;
        kai_msg.f_motor_torque_RL_Nm=TVS.f_measured_torque_RL_Nm;
        kai_msg.f_motor_torque_RR_Nm=TVS.f_measured_torque_RR_Nm;
        kai_msg.i_throttle = TVS.i_PWR_percentage;
        kai_msg.c_torque_mode_flag = TVS_ON;
        kai_msg.c_motor_mode_flag[FL] = MOTOR_ON;
        kai_msg.c_motor_mode_flag[FR] = MOTOR_ON;
        kai_msg.c_motor_mode_flag[RL] = MOTOR_ON;
        kai_msg.c_motor_mode_flag[RR] = MOTOR_ON;
        carstate.publish( & kai_msg );
        nh.spinOnce();
        wait_ms(125);
    }
} */







// void throttleCallback(const std_msgs::Float32& msg)
// {
//     global_accel_value = msg.data;
//     // nh.loginfo("throttle \r\n");
// }

// void brakeCallback(const std_msgs::Int8& msg)
// {
//     global_stop_trig = msg.data;
//     // nh.loginfo("throttle \r\n");
// }





