#include <ros.h>
#include "AutonomousMessage.h"


//create the ros node nh. The node will be used to publish to Arduino
ros::NodeHandle nh;
 

char state = 0;
void messageCb(const KAI_msgs::AutonomousSignal& msg)
{
    state = msg.c_autonomous_state;
}
 
ros::Subscriber<KAI_msgs::AutonomousSignal> sub("AutonomousSignal", &messageCb);
 
void setup()
{
    nh.initNode();
    nh.subscribe(sub);
    
    pinMode(2, OUTPUT);
    pinMode(3, OUTPUT);
    pinMode(4, OUTPUT);
    pinMode(5, OUTPUT);
}
 
void loop()
{
    if (state == ASSI_MANUAL_MODE)
    {
        digitalWrite(2, LOW);
        digitalWrite(3, LOW);
        digitalWrite(4, LOW);
        digitalWrite(5, LOW);
    } else if (state == ASSI_AUTONOMOUS_READY)
    {
        digitalWrite(2, HIGH);
        digitalWrite(3, LOW);
        digitalWrite(4, LOW);
        digitalWrite(5, LOW);
    } else if (state == ASSI_AUTONOMOUS_DRIVING)
    {
        digitalWrite(2, LOW);
        digitalWrite(3, HIGH);
        digitalWrite(4, LOW);
        digitalWrite(5, LOW);
    } else if (state == ASSI_AUTONOMOUS_END)
    {
        digitalWrite(2, LOW);
        digitalWrite(3, LOW);
        digitalWrite(4, HIGH);
        digitalWrite(5, LOW);
    } else if (state == ASSI_AUTONOMOUS_EMERGENCY)
    {
        digitalWrite(2, LOW);
        digitalWrite(3, LOW);
        digitalWrite(4, LOW);
        digitalWrite(5, HIGH);
    }

    
    nh.spinOnce();
    delay(10);
}