#include "TorqueVectoringSystem.h"


//========================== Mbed to PC ROS Communication Thread =======================//
//#include "CarState.h"
//#include <ros.h>



/*
ros::NodeHandle nh;
//kai_msgs::CarState kai_msg;
//ros::Publisher carstate("carstate", &kai_msg);
//float f_temp=0.0;
//int cnt=0;
//Thread thread;
//Thread thread1, thread2;
//TorqueVectoringSystem TVS;
Thread thread1, thread2;
/*
void ros_thread(){
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
}
*/

//========================== Mbed to PC ROS Communication Thread =======================//

//TorqueVectoringSystem TVS;
//========================== Torque Vectoring System Thread =======================//


void system_thread() {

    Serial pc_main(USBTX, USBRX, 115200);

    TorqueVectoringSystem TVS;

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




    //HallSensor FL_Hall_A(FL_HALL_PIN);
    //HallSensor FR_Hall_A(FR_HALL_PIN);
    HallSensor RL_Hall_A(RL_HALL_PIN);
    HallSensor RR_Hall_A(RR_HALL_PIN);

    MPU6050 mpu(MPU_SDA, MPU_SCL);                  // (sda, scl)
    mpu.start();
    pc_main.printf("mpu6050 started!\r\n");

    AnalogIn Handle_Sensor(HANDLE_SENSOR_PIN);

    AnalogIn FL_Current_OUT(FL_CURRENT_SENSOR_PIN);
    AnalogIn FR_Current_OUT(FR_CURRENT_SENSOR_PIN);
    AnalogIn RL_Current_OUT(RL_CURRENT_SENSOR_PIN);
    AnalogIn RR_Current_OUT(RR_CURRENT_SENSOR_PIN);

    AnalogIn Pedal_Sensor(PEDAL_SENSOR_PIN);
    
    PwmOut FL_Throttle_PWM(FL_OUTPUT_THROTTLE_PIN);
    PwmOut FR_Throttle_PWM(FR_OUTPUT_THROTTLE_PIN);
    PwmOut RL_Throttle_PWM(RL_OUTPUT_THROTTLE_PIN);
    PwmOut RR_Throttle_PWM(RR_OUTPUT_THROTTLE_PIN);
    
        
    FL_Throttle_PWM.period_us(PWM_PERIOD_US);
    FR_Throttle_PWM.period_us(PWM_PERIOD_US);
    RL_Throttle_PWM.period_us(PWM_PERIOD_US);
    RR_Throttle_PWM.period_us(PWM_PERIOD_US);
    /*
    FL_Throttle_PWM.period_ms(PWM_PERIOD_MS);
    FR_Throttle_PWM.period_ms(PWM_PERIOD_MS);
    RL_Throttle_PWM.period_ms(PWM_PERIOD_MS);
    RR_Throttle_PWM.period_ms(PWM_PERIOD_MS);
    */
    
    while(1) {
        TVS.process_accel(
            RL_Hall_A, RR_Hall_A, mpu, 
            Handle_Sensor, FL_Current_OUT, FR_Current_OUT, RL_Current_OUT, RR_Current_OUT,
            Pedal_Sensor, FL_Throttle_PWM, FR_Throttle_PWM, RL_Throttle_PWM, RR_Throttle_PWM)

    }
    
}
//========================== Torque Vectoring System Thread =======================//




//========================== Main Code =======================//
int main(){
    /*
    //ros init
    nh.initNode();
    nh.advertise(carstate);
    
    //thread
    thread1.start(ros_thread);
    */
   
    system_thread();

    
    return 0;
}
//========================== Main Code =======================//