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

//#include "CarState.h"
#include "AutonomousMessage.h"
#include <ros.h>





// accel값 subscribe하는 함수 넣어야 함
// 근데 thread를 publish하는 함수에 묶어서, 
// 한 스레드는 ros 관련 pub/sub, 다른 스레드는 구동 함수로 구성하면 좋을 것 같음
float global_subscribed_accel_value = 0.0;
char global_ASMS_state = 0;
char global_estop_trig = 0;
char global_auto_stop_trig = 0;
/*
void AccelValueSubscribe()
{
    //accel값 초기화
}
*/



void AssiStatePublish(){

    /* for testing ros communication with arduino nano */

    KAI_msgs::AutonomousSignal auto_msg;
    ros::Publisher autonomous_message("AutonomousSignal", &auto_msg);

    ros::NodeHandle nh;
    nh.initNode();
    nh.advertise(autonomous_message);
    


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
        wait_ms(1);
    } 
     for testing ros communication with arduino nano */



    // ASSI state publish
    while(1)
    {
        

        if(global_ASMS_state)
            auto_msg.c_autonomous_state = MANUAL_MODE;


        else
        {
            if (c.auto_stop_trig == STOPTRIGGER_END) {
                auto_msg.c_autonomous_state = AUTONOMOUS_END;
                global_auto_stop_trig = STOPTRIGGER_END;
            }

            else if (global_estop_trig == ESTOP_STOP) {
                auto_msg.c_autonomous_state = AUTONOMOUS_EMERGENCY;
            }
            else if (global_estop_trig == ESTOP_RUN) {
                auto_msg.c_autonomous_state = AUTONOMOUS_DRIVING;    
            }
        }

        autonomous_message.publish( &auto_msg );
        nh.spinOnce();
        wait_ms(1);
    }
    
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

    
    
    // mpu 시작했는지 안했는지 표시해주는 함수 작성해야되는데 귀찮음

    TorqueVectoringSystem TVS(
        TVS_SWITCH_PIN, FL_HALL_PIN, FR_HALL_PIN, RL_HALL_PIN, RR_HALL_PIN, 
        HANDLE_SENSOR_PIN, MPU_SDA, MPU_SCL, PEDAL_SENSOR_PIN,
        FL_CURRENT_SENSOR_PIN, FR_CURRENT_SENSOR_PIN, RL_CURRENT_SENSOR_PIN, RR_CURRENT_SENSOR_PIN,
        FL_OUTPUT_THROTTLE_PIN, FR_OUTPUT_THROTTLE_PIN, RL_OUTPUT_THROTTLE_PIN, RR_OUTPUT_THROTTLE_PIN
    );
    TVS.mpu.start();


    DigitalIn ASMS(p11);
    // 임시 estop 스위치
    DigitalIn estop_temporary(p30);
    

    while(1)
    {
        if (ASMS) {         // 수동주행
            global_ASMS_state = ASMS;
            TVS.process_accel();
        }

        else                // 자율주행
        {
            global_ASMS_state = ASMS;

            if (global_auto_stop_trig == STOPTRIGGER_RUNNING)
            {
                if (estop_temporary) {
                    TVS.process_accel(subscribed_accel_value);
                    global_estop_trig = ESTOP_RUN;
                }
                    
                else {
                    TVS.process_accel(0.0);
                    global_estop_trig = ESTOP_STOP;
                }
            }
            else if (global_auto_stop_trig == STOPTRIGGER_END)
            {
                TVS.process_accel(0.0);
                
            }
        }
    }

}
//========================== Torque Vectoring System Thread =======================//



int main(){

    // Thread thread_ROS;
    // thread_ROS.start(AssiStatePublish);

    CarDriving();

    return 0;
}