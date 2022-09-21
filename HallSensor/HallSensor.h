#ifndef MBED_HALLSENSOR_H
#define MBED_HALLSENSOR_H
 
#include "mbed.h"
 
 
class HallSensor
{
public:
    HallSensor(PinName a);
    
    int getPinState ();
    int edgestate;
    float getRPM();             // editted by NaYoung
 
protected:
    void risingCallback();      // editted by NaYoung
 
    InterruptIn m_hallSensor;
    Timer m_timer_sec;
    float m_period_sec;
    float m_prev_edge;
    float m_cur_edge;
};
 
#endif // MBED_HALLSENSOR_H