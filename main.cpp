#include "TorqueVectoringSystem.h"


//========================== Mbed to PC ROS Communication Thread =======================//
//#include "CarState.h"
#include "AutonomousMessage.h"
#include <ros.h>




Thread ros_thread;

void ros_thread(){

    ros::NodeHandle nh;
    KAI_msgs::AutonomousSignal auto_msg;
    ros::Publisher autonomous_signal("AutonomousSignal", &auto_msg);

    
    nh.initNode();
    nh.advertise(autonomous_signal);

    // just for test
    AnalogIn resistor(p19);
    float resistor_value = 0.0;

    while(true){

        resistor_value = resistor.read();
        
        if (resistor_value >= 0.0 && resistor_value < 0.2)          auto_msg.c_automotive_state = MANUAL_MODE;
        else if (resistor_value >= 0.2 && resistor_value < 0.4)     auto_msg.c_automotive_state = AUTONOMOUS_READY;
        else if (resistor_value >= 0.4 && resistor_value < 0.6)     auto_msg.c_automotive_state = AUTONOMOUS_DRIVING;
        else if (resistor_value >= 0.6 && resistor_value < 0.8)     auto_msg.c_automotive_state = AUTONOMOUS_END;
        else if (resistor_value >= 0.8 && resistor_value < 1.1)     auto_msg.c_automotive_state = AUTONOMOUS_EMERGENCY;
        
        autonomous_signal.publish( &auto_msg );
        nh.spinOnce();
        wait_ms(125);
    }
}


//========================== Mbed to PC ROS Communication Thread =======================//


//========================== Torque Vectoring System Thread =======================//


void system_thread() {

    Serial pc_main(USBTX, USBRX, 115200);



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

    
    // mpu.start();
    // pc_main.printf("mpu6050 started!\r\n");

    

    TorqueVectoringSystem TVS(
        TVS_SWITCH_PIN, FL_HALL_PIN, FR_HALL_PIN, RL_HALL_PIN, RR_HALL_PIN, 
        HANDLE_SENSOR_PIN, MPU_SDA, MPU_SCL, PEDAL_SENSOR_PIN,
        FL_CURRENT_SENSOR_PIN, FR_CURRENT_SENSOR_PIN, RL_CURRENT_SENSOR_PIN, RR_CURRENT_SENSOR_PIN,
        FL_OUTPUT_THROTTLE_PIN, FR_OUTPUT_THROTTLE_PIN, RL_OUTPUT_THROTTLE_PIN, RR_OUTPUT_THROTTLE_PIN
    );



    while(1) {
        TVS.process_accel();
  
    }
    
}
//========================== Torque Vectoring System Thread =======================//





int main(){

    ros_thread.start(ros_thread);

    system_thread();

    return 0;
}
