/*
* Team K.A.I. 2022 Baja_EV 개발파트가 2022 국제 대학생 창작자동차경진대회 자율전기차부문 참가를 위해 만듬
* 
* 구성 요소
 float32 f_accel_sig
 float32 f_brake_sig
 float32 f_steering_sig

 uint8 c_automotive_state
 uint8 c_estop_trig
 uint8 c_autonomous_stop_trig

*
*/

#ifndef ROS_MESSAGE_FOR_KAI_AUTONOMOUS
#define ROS_MESSAGE_FOR_KAI_AUTONOMOUS

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"


// for c_automotive_state
#define MANUAL_MODE             0
#define AUTONOMOUS_READY        1
#define AUTONOMOUS_DRIVING      2
#define AUTONOMOUS_END          3
#define AUTONOMOUS_EMERGENCY    4

// for c_estop_trig
#define ESTOP_STOP              0
#define ESTOP_RUN               1

// for c_autonomous_stop_trig
#define STOPTRIGGER_RUNNING     0
#define STOPTRIGGER_END         1


namespace KAI_msgs
{
    class AutonomousSignal : public ros::Msg
    {
    public:
        typedef float _float_data;
        typedef int8_t _char_data;

        _float_data f_accel_sig;
        _float_data f_brake_sig;
        _float_data f_steering_sig;

        _char_data c_automotive_state;

        _char_data c_estop_trig;

        _char_data c_autonomous_stop_trig;


        AutonomousSignal():
            f_accel_sig(0.0), 
            f_brake_sig(0.0), 
            f_steering_sig(0.0), 
        
            c_autonomotive_state(MANUAL_MODE),

            c_estop_trig(ESTOP_STOP),

            c_autonomous_stop_trig(STOPTRIGGER_RUNNING),
            {}



        virtual int serialize(unsigned char *outbuffer) const
        {
            int offset=0;

            union{
                float real;
                uint32_t base;
            } u_data;

            u_data.real=this->f_accel_sig;
            *(outbuffer + offset + 0) = (u_data.base >> (8 * 0)) & 0xFF;
            *(outbuffer + offset + 1) = (u_data.base >> (8 * 1)) & 0xFF;
            *(outbuffer + offset + 2) = (u_data.base >> (8 * 2)) & 0xFF;
            *(outbuffer + offset + 3) = (u_data.base >> (8 * 3)) & 0xFF;
            offset += sizeof(this->f_accel_sig);

            u_data.real=this->f_brake_sig;
            *(outbuffer + offset + 0) = (u_data.base >> (8 * 0)) & 0xFF;
            *(outbuffer + offset + 1) = (u_data.base >> (8 * 1)) & 0xFF;
            *(outbuffer + offset + 2) = (u_data.base >> (8 * 2)) & 0xFF;
            *(outbuffer + offset + 3) = (u_data.base >> (8 * 3)) & 0xFF;
            offset += sizeof(this->f_brake_sig);

            u_data.real=this->f_steering_sig;
            *(outbuffer + offset + 0) = (u_data.base >> (8 * 0)) & 0xFF;
            *(outbuffer + offset + 1) = (u_data.base >> (8 * 1)) & 0xFF;
            *(outbuffer + offset + 2) = (u_data.base >> (8 * 2)) & 0xFF;
            *(outbuffer + offset + 3) = (u_data.base >> (8 * 3)) & 0xFF;
            offset += sizeof(this->f_steering_sig);




            union{
                int8_t real;
                uint32_t base;
            } u_flag;

            u_flag.real=this->c_automotive_state;
            *(outbuffer + offset + 0) = (u_flag.base >> (8 * 0)) & 0xFF;
            offset += sizeof(this->c_automotive_state);

            u_flag.real=this->c_estop_trig;
            *(outbuffer + offset + 0) = (u_flag.base >> (8 * 0)) & 0xFF;
            offset += sizeof(this->c_estop_trig);

            u_flag.real=this->c_autonomous_stop_trig;
            *(outbuffer + offset + 0) = (u_flag.base >> (8 * 0)) & 0xFF;
            offset += sizeof(this->c_autonomous_stop_trig);


            return offset;
        }


        virtual int deserialize(unsigned char *inbuffer)
        {
            int offset=0;

            union{
                float real;
                uint32_t base;
            } u_data;

            u_data.base=0;
            u_data.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
            u_data.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
            u_data.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
            u_data.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
            this->f_accel_sig = u_data.real;
            offset += sizeof(this->f_accel_sig);

            u_data.base=0;
            u_data.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
            u_data.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
            u_data.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
            u_data.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
            this->f_brake_sig = u_data.real;
            offset += sizeof(this->f_brake_sig);

            u_data.base=0;
            u_data.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
            u_data.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
            u_data.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
            u_data.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
            this->f_steering_sig = u_data.real;
            offset += sizeof(this->f_steering_sig);



            union{
                int8_t real;
                uint32_t base;
            } u_flag;

            u_flag.base=0;
            u_flag.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
            this->c_automotive_state = u_flag.real;
            offset += sizeof(this->c_automotive_state);

            u_flag.base=0;
            u_flag.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
            this->c_estop_trig = u_flag.real;
            offset += sizeof(this->c_estop_trig);

            u_flag.base=0;
            u_flag.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
            this->c_automotive_stop_trig = u_flag.real;
            offset += sizeof(this->c_automotive_stop_trig);


            return offset;
        }

        const char * getType() { return "autonomous_message/AutonomousSignal"; };
        const char * getMD5() { return "9d781d5f9902017535bcd729642f2fe7"; };

    };
}
#endif      //ROS_MESSAGE_FOR_KAI_AUTONOMOUS
