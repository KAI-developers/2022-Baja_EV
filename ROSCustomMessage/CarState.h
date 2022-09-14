#ifndef _ROS_std_msgs_Byte_h
#define _ROS_std_msgs_Byte_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"


namespace kai_msgs
{
    class CarState : public ros::Msg
    {
    public:
        typedef float _data_type_f;
        _data_type_f f_wheel_velocity_FL_ms;
        _data_type_f f_wheel_velocity_FR_ms;
        _data_type_f f_wheel_velocity_RL_ms;
        _data_type_f f_wheel_velocity_RR_ms;
        _data_type_f f_car_velocity_ms;
        _data_type_f f_motor_torque_FL_Nm;
        _data_type_f f_motor_torque_FR_Nm;
        _data_type_f f_motor_torque_RL_Nm;
        _data_type_f f_motor_torque_RR_Nm;
        typedef int32_t _data_type_i;
        _data_type_i i_throttle;
        typedef int8_t _flag_type;
        _flag_type c_torque_mode_flag;
        _flag_type c_motor_mode_flag[4];        
        enum {  ON  = 1    };
        enum {  OFF = 0    };

        CarState():
            f_wheel_velocity_FL_ms(0), 
            f_wheel_velocity_FR_ms(0), 
            f_wheel_velocity_RL_ms(0), 
            f_wheel_velocity_RR_ms(0),
            f_car_velocity_ms(0),
            f_motor_torque_FL_Nm(0), 
            f_motor_torque_FR_Nm(0), 
            f_motor_torque_RL_Nm(0), 
            f_motor_torque_RR_Nm(0),
            i_throttle(0),
            c_torque_mode_flag(0),
            c_motor_mode_flag() {}

        virtual int serialize(unsigned char *outbuffer) const
        {
            int offset=0;

            union{
                float real;
                uint32_t base;
            } u_data;

            u_data.real=this->f_wheel_velocity_FL_ms;
            *(outbuffer + offset + 0) = (u_data.base >> (8 * 0)) & 0xFF;
            *(outbuffer + offset + 1) = (u_data.base >> (8 * 1)) & 0xFF;
            *(outbuffer + offset + 2) = (u_data.base >> (8 * 2)) & 0xFF;
            *(outbuffer + offset + 3) = (u_data.base >> (8 * 3)) & 0xFF;
            offset += sizeof(this->f_wheel_velocity_FL_ms);

            u_data.real=this->f_wheel_velocity_FR_ms;
            *(outbuffer + offset + 0) = (u_data.base >> (8 * 0)) & 0xFF;
            *(outbuffer + offset + 1) = (u_data.base >> (8 * 1)) & 0xFF;
            *(outbuffer + offset + 2) = (u_data.base >> (8 * 2)) & 0xFF;
            *(outbuffer + offset + 3) = (u_data.base >> (8 * 3)) & 0xFF;
            offset += sizeof(this->f_wheel_velocity_FR_ms);

            u_data.real=this->f_wheel_velocity_RL_ms;
            *(outbuffer + offset + 0) = (u_data.base >> (8 * 0)) & 0xFF;
            *(outbuffer + offset + 1) = (u_data.base >> (8 * 1)) & 0xFF;
            *(outbuffer + offset + 2) = (u_data.base >> (8 * 2)) & 0xFF;
            *(outbuffer + offset + 3) = (u_data.base >> (8 * 3)) & 0xFF;
            offset += sizeof(this->f_wheel_velocity_RL_ms);

            u_data.real=this->f_wheel_velocity_RR_ms;
            *(outbuffer + offset + 0) = (u_data.base >> (8 * 0)) & 0xFF;
            *(outbuffer + offset + 1) = (u_data.base >> (8 * 1)) & 0xFF;
            *(outbuffer + offset + 2) = (u_data.base >> (8 * 2)) & 0xFF;
            *(outbuffer + offset + 3) = (u_data.base >> (8 * 3)) & 0xFF;
            offset += sizeof(this->f_wheel_velocity_RR_ms);

            u_data.real=this->f_car_velocity_ms;
            *(outbuffer + offset + 0) = (u_data.base >> (8 * 0)) & 0xFF;
            *(outbuffer + offset + 1) = (u_data.base >> (8 * 1)) & 0xFF;
            *(outbuffer + offset + 2) = (u_data.base >> (8 * 2)) & 0xFF;
            *(outbuffer + offset + 3) = (u_data.base >> (8 * 3)) & 0xFF;
            offset += sizeof(this->f_car_velocity_ms);

            u_data.real=this->f_motor_torque_FL_Nm;
            *(outbuffer + offset + 0) = (u_data.base >> (8 * 0)) & 0xFF;
            *(outbuffer + offset + 1) = (u_data.base >> (8 * 1)) & 0xFF;
            *(outbuffer + offset + 2) = (u_data.base >> (8 * 2)) & 0xFF;
            *(outbuffer + offset + 3) = (u_data.base >> (8 * 3)) & 0xFF;
            offset += sizeof(this->f_motor_torque_FL_Nm);

            u_data.real=this->f_motor_torque_FR_Nm;
            *(outbuffer + offset + 0) = (u_data.base >> (8 * 0)) & 0xFF;
            *(outbuffer + offset + 1) = (u_data.base >> (8 * 1)) & 0xFF;
            *(outbuffer + offset + 2) = (u_data.base >> (8 * 2)) & 0xFF;
            *(outbuffer + offset + 3) = (u_data.base >> (8 * 3)) & 0xFF;
            offset += sizeof(this->f_motor_torque_FR_Nm);

            u_data.real=this->f_motor_torque_RL_Nm;
            *(outbuffer + offset + 0) = (u_data.base >> (8 * 0)) & 0xFF;
            *(outbuffer + offset + 1) = (u_data.base >> (8 * 1)) & 0xFF;
            *(outbuffer + offset + 2) = (u_data.base >> (8 * 2)) & 0xFF;
            *(outbuffer + offset + 3) = (u_data.base >> (8 * 3)) & 0xFF;
            offset += sizeof(this->f_motor_torque_RL_Nm);

            u_data.real=this->f_motor_torque_RR_Nm;
            *(outbuffer + offset + 0) = (u_data.base >> (8 * 0)) & 0xFF;
            *(outbuffer + offset + 1) = (u_data.base >> (8 * 1)) & 0xFF;
            *(outbuffer + offset + 2) = (u_data.base >> (8 * 2)) & 0xFF;
            *(outbuffer + offset + 3) = (u_data.base >> (8 * 3)) & 0xFF;
            offset += sizeof(this->f_motor_torque_RR_Nm);


            union{
                int32_t real;
                uint32_t base;
            } u_data_i;

            u_data_i.real=this->i_throttle;
            *(outbuffer + offset + 0) = (u_data_i.base >> (8 * 0)) & 0xFF;
            *(outbuffer + offset + 1) = (u_data_i.base >> (8 * 1)) & 0xFF;
            *(outbuffer + offset + 2) = (u_data_i.base >> (8 * 2)) & 0xFF;
            *(outbuffer + offset + 3) = (u_data_i.base >> (8 * 3)) & 0xFF;
            offset += sizeof(this->i_throttle);

            union{
                int8_t real;
                uint32_t base;
            } u_flag;

            u_flag.real=this->c_torque_mode_flag;
            *(outbuffer + offset + 0) = (u_flag.base >> (8 * 0)) & 0xFF;
            offset += sizeof(this->c_torque_mode_flag);

            for( uint32_t i=0; i<4; i++){
                u_flag.real = this->c_motor_mode_flag[i];
                *(outbuffer + offset + 0) = (u_flag.base >> (8 * 0)) & 0xFF;\
                offset += sizeof(this->c_motor_mode_flag[i]);
            }

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
            this->f_wheel_velocity_FL_ms = u_data.real;
            offset += sizeof(this->f_wheel_velocity_FL_ms);

            u_data.base=0;
            u_data.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
            u_data.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
            u_data.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
            u_data.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
            this->f_wheel_velocity_FR_ms = u_data.real;
            offset += sizeof(this->f_wheel_velocity_FR_ms);

            u_data.base=0;
            u_data.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
            u_data.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
            u_data.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
            u_data.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
            this->f_wheel_velocity_RL_ms = u_data.real;
            offset += sizeof(this->f_wheel_velocity_RL_ms);

            u_data.base=0;
            u_data.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
            u_data.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
            u_data.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
            u_data.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
            this->f_wheel_velocity_RR_ms = u_data.real;
            offset += sizeof(this->f_wheel_velocity_RR_ms);

            u_data.base=0;
            u_data.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
            u_data.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
            u_data.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
            u_data.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
            this->f_car_velocity_ms = u_data.real;
            offset += sizeof(this->f_car_velocity_ms);

            u_data.base=0;
            u_data.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
            u_data.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
            u_data.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
            u_data.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
            this->f_motor_torque_FL_Nm = u_data.real;
            offset += sizeof(this->f_motor_torque_FL_Nm);

            u_data.base=0;
            u_data.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
            u_data.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
            u_data.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
            u_data.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
            this->f_motor_torque_FR_Nm = u_data.real;
            offset += sizeof(this->f_motor_torque_FR_Nm);

            u_data.base=0;
            u_data.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
            u_data.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
            u_data.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
            u_data.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
            this->f_motor_torque_RL_Nm = u_data.real;
            offset += sizeof(this->f_motor_torque_RL_Nm);

            u_data.base=0;
            u_data.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
            u_data.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
            u_data.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
            u_data.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
            this->f_motor_torque_RR_Nm = u_data.real;
            offset += sizeof(this->f_motor_torque_RR_Nm);

            union{
                int32_t real;
                uint32_t base;
            } u_data_i;

            u_data_i.base=0;
            u_data_i.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
            u_data_i.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
            u_data_i.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
            u_data_i.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
            this->i_throttle = u_data_i.real;
            offset += sizeof(this->i_throttle);

            union{
                int8_t real;
                uint32_t base;
            } u_flag;

            u_flag.base=0;
            u_flag.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
            this->c_torque_mode_flag = u_flag.real;
            offset += sizeof(this->c_torque_mode_flag);

            for( uint32_t i=0; i<4; i++){
                u_flag.base=0;
                u_flag.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
                this->c_motor_mode_flag[i] = u_flag.real;
                offset += sizeof(this->c_motor_mode_flag[i]);
            }

            return offset;
        }

        const char * getType() { return "driver_system/CarState"; };
        const char * getMD5() { return "9d781d5f9902017535bcd729642f2fe7"; };

    };
}
#endif
