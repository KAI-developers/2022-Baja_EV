#include "HallSensor.h"
#define MIN         60.0
#define MOTOR_POLE  7

 
HallSensor::HallSensor (PinName a) : m_hallSensor (a,PullUp)
{
    float fDummy_sec = 1000.0;
    edgestate = 0;
    m_hallSensor.rise (callback (this, &HallSensor::risingCallback));
    m_period_sec = fDummy_sec;
    m_timer_sec.start ();
}
 
float HallSensor::getRPM ()
{
    float fRpm;
    
    if (m_period_sec > 0.1) {       //why 0.1
        fRpm = 0.0f;
    }
    else {
        fRpm = 60/(m_period_sec * MOTOR_POLE);
    }
    
    return fRpm;
}
 
void HallSensor::risingCallback()
{       
       
        edgestate=!edgestate;
        
        if(edgestate==1) {
            m_prev_edge = m_timer_sec.read();
        }
        if(edgestate==0) {
            m_cur_edge = m_timer_sec.read();
        }
        
        if((m_cur_edge) > (m_prev_edge)) {
            m_period_sec = (m_cur_edge)-(m_prev_edge);
        }
        else if((m_prev_edge) > (m_cur_edge)) {
            m_period_sec = (m_prev_edge)-(m_cur_edge);
        }
        // tktlf akadp dksemsmsep sjan wjdghkrgotj zldqkedma
        // wjswjsrltjf soatork sjan wkdnrgka.. dndnr...
}
 
int HallSensor::getPinState ()
{
    return m_hallSensor;
}