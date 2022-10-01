/*
* Team K.A.I. 2022 Baja_EV 개발파트가 2022 국제 대학생 창작자동차경진대회 자율전기차부문 참가를 위해 만듬
* 
* 구성 요소
 float32 f_accel_sig
 float32 f_brake_sig
 float32 f_steering_sig

 float32 f_car_speed
 float32 f_filtered_yawrate


 uint8 c_autonomous_state
 # MANUAL_MODE             0  
 # AUTONOMOUS_READY        1
 # AUTONOMOUS_DRIVING      2
 # AUTONOMOUS_END          3
 # AUTONOMOUS_EMERGENCY    4

// 이제 안 쓸 것 같음
 uint8 c_estop_trig
 # ESTOP_STOP              0
 # ESTOP_RUN               1

 uint8 c_auto_stop_trig
 # STOPTRIGGER_RUNNING     0
 # STOPTRIGGER_END         1
*
*/

#ifndef _ROS_std_msgs_Byte_h
#define _ROS_std_msgs_Byte_h



#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"


// for c_autonomous_state
#define ASSI_MANUAL_MODE                0
#define ASSI_AUTONOMOUS_READY           1
#define ASSI_AUTONOMOUS_DRIVING         2
#define ASSI_AUTONOMOUS_END             3
#define ASSI_AUTONOMOUS_EMERGENCY       4

// for c_estop_trig
#define ESTOP_STOP                      0
#define ESTOP_RUN                       1

// for c_auto_stop_trig
#define STOPTRIGGER_RUNNING             0
#define STOPTRIGGER_END                 1               // 자율주행으로 인한 종료 상태


namespace KAI_msgs
{
    class AutonomousSignal: public ros::Msg
    {
    public:
        typedef float _float_data;
        _float_data f_accel_sig;
        _float_data f_brake_sig;
        _float_data f_steering_sig;
        _float_data f_car_speed;
        _float_data f_filtered_yawrate;

        typedef int8_t _char_data;
        _char_data c_autonomous_state;
        _char_data c_estop_trig;
        _char_data c_auto_stop_trig;


        AutonomousSignal():
            f_accel_sig(0), 
            f_brake_sig(0), 
            f_steering_sig(0), 
            c_autonomous_state(0),        // default MANUAL_MODE
            c_estop_trig(0),              // default STOP_MODE
            c_auto_stop_trig(0)           // default autonomous running
            { }



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

            u_data.real=this->f_car_speed;
            *(outbuffer + offset + 0) = (u_data.base >> (8 * 0)) & 0xFF;
            *(outbuffer + offset + 1) = (u_data.base >> (8 * 1)) & 0xFF;
            *(outbuffer + offset + 2) = (u_data.base >> (8 * 2)) & 0xFF;
            *(outbuffer + offset + 3) = (u_data.base >> (8 * 3)) & 0xFF;
            offset += sizeof(this->f_car_speed);

            u_data.real=this->f_filtered_yawrate;
            *(outbuffer + offset + 0) = (u_data.base >> (8 * 0)) & 0xFF;
            *(outbuffer + offset + 1) = (u_data.base >> (8 * 1)) & 0xFF;
            *(outbuffer + offset + 2) = (u_data.base >> (8 * 2)) & 0xFF;
            *(outbuffer + offset + 3) = (u_data.base >> (8 * 3)) & 0xFF;
            offset += sizeof(this->f_filtered_yawrate);



           union{
                int8_t real;
                uint32_t base;
            } u_data_i;

            u_data_i.real=this->c_autonomous_state;
            *(outbuffer + offset + 0) = (u_data_i.base >> (8 * 0)) & 0xFF;
            offset += sizeof(this->c_autonomous_state);

            u_data_i.real=this->c_estop_trig;
            *(outbuffer + offset + 0) = (u_data_i.base >> (8 * 0)) & 0xFF;
            offset += sizeof(this->c_estop_trig);

            u_data_i.real=this->c_auto_stop_trig;
            *(outbuffer + offset + 0) = (u_data_i.base >> (8 * 0)) & 0xFF;
            offset += sizeof(this->c_auto_stop_trig);


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

            
            u_data.base=0;
            u_data.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
            u_data.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
            u_data.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
            u_data.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
            this->f_car_speed = u_data.real;
            offset += sizeof(this->f_car_speed);

            
            u_data.base=0;
            u_data.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
            u_data.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
            u_data.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
            u_data.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
            this->f_filtered_yawrate = u_data.real;
            offset += sizeof(this->f_filtered_yawrate);



            union{
                int32_t real;
                uint32_t base;
            } u_data_i;

            u_data_i.base = 0;
            u_data_i.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
            this->c_autonomous_state = u_data_i.real;
            offset += sizeof(this->c_autonomous_state);


            u_data_i.base = 0;
            u_data_i.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
            this->c_estop_trig = u_data_i.real;
            offset += sizeof(this->c_estop_trig);


            u_data_i.base = 0;
            u_data_i.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
            this->c_auto_stop_trig = u_data_i.real;
            offset += sizeof(this->c_auto_stop_trig);


            return offset;
        }

        const char * getType() { return "autonomous_message/autonomous_message"; };
        const char * getMD5() { return "bb5506184944dfff7fe2a1f7b829572c"; };

    };
}
#endif 
