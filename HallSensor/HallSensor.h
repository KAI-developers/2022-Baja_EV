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
    Timer m_timer_sec;
    float m_period_sec;
    float m_prev_edge;
    float m_cur_edge;
};
 
#endif // MBED_HALLSENSOR_H