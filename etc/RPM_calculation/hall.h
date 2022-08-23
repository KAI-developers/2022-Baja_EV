#ifndef MBED_HALLSENSOR_H
#define MBED_HALLSENSOR_H
 
#include "mbed.h"
 
#define PI 3.141592
 
 
class HallSensor
{
public:
    HallSensor(PinName a);
    
    int getPinState ();
    int edgestate;
    float getRPM();
 
protected:
    void risingCallback();
 
    InterruptIn m_hallSensor;
    Timer timer_sec;
    float period_sec;
    float prev_rising_edge_time;

};
 
#endif // MBED_HALLSENSOR_H