#include "HallSensor.h"
#define MIN         60.0
#define MOTOR_POLE  7

 
HallSensor::HallSensor (PinName a) : m_hallSensor (a,PullUp)
{
    float fDummy_sec = 1000.0;
    //edgestate = 0;
    m_hallSensor.rise (callback (this, &HallSensor::risingCallback));

    prev_rising_edge_time = 0.0;
    
    period_sec = fDummy_sec;
    timer_sec.start ();
}
 
float HallSensor::getRPM ()
{
    float fRpm;
    
    if (period_sec > 0.1) {
        fRpm = 0.0f;
    }
    else {
        fRpm = 60/(period_sec * MOTOR_POLE);
    }
    
    return fRpm;
}
 
void HallSensor::risingCallback()
{       
    static float rising_edge_time = 0.0;
    
    prev_rising_edge_time = rising_edge_time;

    rising_edge_time = timer_sec.read();
    
    period_sec = rising_edge_time - prev_rising_edge_time;


}
 
int HallSensor::getPinState ()
{
    return m_hallSensor;
}

