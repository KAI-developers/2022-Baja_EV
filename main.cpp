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


#include "TorqueVectoringSystem.h"

#include "ros.h"

//#include "CarState.h"
#include "AutonomousMessage.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int8.h"
//#include <iostream>


// constant for ASMS switch (p11)
#define ASMS_MANUAL_MODE        1
#define ASMS_AUTONOMOUS_MODE    0


char global_autonomous_state = ASSI_MANUAL_MODE;
float global_accel_value = 0.0;
float global_velocity_ms = 0.;
char global_stop_trig = STOPTRIGGER_RUNNING;


void ROSPubSub();
void CarDriving();




int main(int argc, char **argv){

    DigitalIn b_estop(p12);
    DigitalIn b_ASMS(p11);


    Thread thread_ROS, thread_accel;

    thread_ROS.start(ROSPubSub);

    thread_accel.start(CarDriving);


    while(1)
    {
        if (b_ASMS == ASMS_MANUAL_MODE) {
            global_autonomous_state = ASSI_MANUAL_MODE;             // AS 눌려있음 (mbed에 high 인가)
            global_stop_trig = STOPTRIGGER_RUNNING;                // 자율주행 자동정지 변수를 running으로
        }     
            
        else                                                        // AS 풀려있음 (자율주행모드)
        {
            if (global_stop_trig == STOPTRIGGER_END) {              // 자율주행으로 정지된 상태를 subscribe하면
                global_autonomous_state == ASSI_AUTONOMOUS_END;
            }


            else if (global_stop_trig == STOPTRIGGER_RUNNING) 
            {
                if (b_estop == ESTOP_STOP)                                     // 릴레이가 mbed로 LOW 인가됨
                {
                    if (global_autonomous_state == ASSI_AUTONOMOUS_DRIVING) {       // 주행 중에 누름
                        global_autonomous_state = ASSI_AUTONOMOUS_EMERGENCY;        // 비상정지
                    }
                    else if (global_autonomous_state == ASSI_MANUAL_MODE)           // 수동주행 상태에서 자율로 넘어간 초기상황
                        global_autonomous_state = ASSI_AUTONOMOUS_READY;            // 준비 상태로 바뀜
                }

                else if (b_estop == ESTOP_RUN)                                      // 릴레이가 mbed로 HIGH 인가됨, 준비상태에서만 동작하는 코드임
                {
                    if (global_autonomous_state == ASSI_AUTONOMOUS_READY)           // 준비상태일 때 켜지면
                        global_autonomous_state = ASSI_AUTONOMOUS_DRIVING;          // 출발~
                }
            }
        }
    }

    return 0;
}




void throttleCallback(const std_msgs::Float32& msg)
{
    global_accel_value = msg.data;
    // nh.loginfo("throttle \r\n");
}

void brakeCallback(const std_msgs::Int8& msg)
{
    global_stop_trig = msg.data;
    // nh.loginfo("throttle \r\n");
}

void ROSPubSub(){

    /* for testing ros communication with arduino nano */


    KAI_msgs::AutonomousSignal auto_msg;
    ros::Publisher autonomous_message("AutonomousSignal", &auto_msg);

    ros::Subscriber<std_msgs::Float32> sub_throttle("throttle_control_command", &throttleCallback);
    ros::Subscriber<std_msgs::Int8> sub_brake_command("full_brake_sig", &brakeCallback);

    ros::NodeHandle nh;
    nh.initNode();
    nh.advertise(autonomous_message);
    nh.subscribe(sub_throttle);
    nh.subscribe(sub_brake_command);

    
    while(1) 
    {
        // publishing part
        auto_msg.c_autonomous_state = global_autonomous_state;
        auto_msg.f_car_speed = global_velocity_ms;

        autonomous_message.publish ( &auto_msg );
  
        nh.spinOnce();
        wait_ms(50);
    }
    

    /*
    // just for test
    AnalogIn resistor(p19);
    float resistor_value = 0.0;

    while(true){

        resistor_value = resistor.read();

        if (resistor_value >= 0.0 && resistor_value < 0.2)          auto_msg.c_autonomous_state = MANUAL_MODE;
        else if (resistor_value >= 0.2 && resistor_value < 0.4)     auto_msg.c_autonomous_state = AUTONOMOUS_READY;
        else if (resistor_value >= 0.4 && resistor_value < 0.6)     auto_msg.c_autonomous_state = AUTONOMOUS_DRIVING;
        else if (resistor_value >= 0.6 && resistor_value < 0.8)     auto_msg.c_autonomous_state = AUTONOMOUS_END;
        else if (resistor_value >= 0.8 && resistor_value < 1.1)     auto_msg.c_autonomous_state = AUTONOMOUS_EMERGENCY;
        
        autonomous_message.publish( &auto_msg );
        nh.spinOnce();
        wait_ms(50);
    } 
    //for testing ros communication with arduino nano
    */
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



//========================== Mbed to PC ROS Communication Thread =======================//


//========================== Torque Vectoring System Thread =======================//


void CarDriving() {

    // Serial pc_main(USBTX, USBRX, 115200);



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
        if (global_autonomous_state == ASSI_MANUAL_MODE) {
            TVS.process_accel();
            global_velocity_ms = TVS.f_vehicle_vel_ms;
        }
        else if (   global_autonomous_state == ASSI_AUTONOMOUS_READY ||
                    global_autonomous_state == ASSI_AUTONOMOUS_EMERGENCY ||
                    global_autonomous_state == ASSI_AUTONOMOUS_END) {
            TVS.process_accel(0.);
        }
        else if (global_autonomous_state == ASSI_AUTONOMOUS_DRIVING) {
            TVS.process_accel(global_accel_value);
            global_velocity_ms = TVS.f_vehicle_vel_ms;
        }
    }
}
//========================== Torque Vectoring System Thread =======================//


