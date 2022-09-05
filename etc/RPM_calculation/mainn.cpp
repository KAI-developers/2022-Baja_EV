#include "mbed.h"
#include "HallSensor.h"
#include "Plotting.h"

//////////////////////////////////
#define PEDAL_MIN_VALUE     0.213
#define PEDAL_MAX_VALUE     0.68

#define IDEAL_OPAMP_GAIN    1.515
#define REAL_OPAMP_GAIN     1.7

#define PERIOD_US           25



Plotting plot;
unsigned int uiFlag_50ms = 0;


void counter_1ms ()
{
    uiFlag_50ms++;
}


float map(float input, float in_min, float in_max, float out_min, float out_max)
{
    return (input - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


int main() {

    Serial pc(USBTX, USBRX, 115200);

    HallSensor Hall_RL(p13);
    HallSensor Hall_RR(p14);

    float RPM_RL;
    float RPM_RR;
    
    /*
    Ticker ticker_1ms;
    ticker_1ms.attach(&counter_1ms, 0.001);
    Timer time;
    time.start();
    */
    

    while(1) {
    
        RPM_RL = Hall_RL.getRPM();
        RPM_RR = Hall_RR.getRPM();

        pc.printf("RL RPM : %f\t\t RR RPM", RPM_RL, RPM_RR);

    }

}