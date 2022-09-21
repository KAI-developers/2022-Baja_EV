#ifndef _ROS_std_msgs_Byte_h
#define _ROS_std_msgs_Byte_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"


namespace kai_msgs
{
    class Practice : public ros::Msg
    {
    public:
        typedef float _data_type_f;
        _data_type_f f_speed_ms;

        typedef int32_t _data_type_i;
        _data_type_i i_throttle;

        typedef int8_t _flag_type;
        _flag_type c_State[4];        
        enum {  ON  = 1    };
        enum {  OFF = 0    };

        Practice():
            f_speed_ms(0),
            i_throttle(0),
            c_State()
        {
        }

        virtual int serialize(unsigned char *outbuffer) const
        {
            int offset=0;

            union{
                float real;
                uint32_t base;
            } u_data;

            u_data.real=this->f_speed_ms;
            *(outbuffer + offset + 0) = (u_data.base >> (8 * 0)) & 0xFF;
            *(outbuffer + offset + 1) = (u_data.base >> (8 * 1)) & 0xFF;
            *(outbuffer + offset + 2) = (u_data.base >> (8 * 2)) & 0xFF;
            *(outbuffer + offset + 3) = (u_data.base >> (8 * 3)) & 0xFF;
            offset += sizeof(this->f_speed_ms);



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


            for( uint32_t i=0; i<4; i++){
                u_flag.real = this->c_State[i];
                *(outbuffer + offset + 0) = (u_flag.base >> (8 * 0)) & 0xFF;
                offset += sizeof(this->c_State[i]);
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

            u_data.base = 0;
            u_data.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
            u_data.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
            u_data.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
            u_data.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
            this->f_speed_ms = u_data.real;
            offset += sizeof(this->f_speed_ms);




            union{
                int32_t real;
                uint32_t base;
            } u_data_i;

            u_data_i.base = 0;
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

            for( uint32_t i=0; i<4; i++){
                u_flag.base=0;
                u_flag.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
                this->c_State[i] = u_flag.real;
                offset += sizeof(this->c_State[i]);
            }

            return offset;
        }

        const char * getType() { return "Practice/Practice"; };
        const char * getMD5() { return "9c7e73cde7d2a1e786d24363b1b1b1c9"; };

    };
}
#endif
