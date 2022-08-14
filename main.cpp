#include "mbed.h"

#include "TorqueVectoringSystem.h"
#include "HallSensor.h"
#include "MPU6050.h"

Serial pc1(USBTX, USBRX, 115200);


int main()
{
    pc1.printf("mbed reset!\r\n");
    

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

    TorqueVectoringSystem TVS;

    TVS.process_accel(
        FL_HALL_PIN, FR_HALL_PIN, RL_HALL_PIN, RR_HALL_PIN, 
        HANDLE_SENSOR_PIN, MPU_SDA, MPU_SCL, PEDAL_SENSOR_PIN,
        FL_CURRENT_SENSOR_PIN, FR_CURRENT_SENSOR_PIN, RL_CURRENT_SENSOR_PIN, RR_CURRENT_SENSOR_PIN,
        FL_OUTPUT_THROTTLE_PIN, FR_OUTPUT_THROTTLE_PIN, RL_OUTPUT_THROTTLE_PIN, RR_OUTPUT_THROTTLE_PIN);
        

    // mbed Publisher thread
    //
    //


    return 0;
}