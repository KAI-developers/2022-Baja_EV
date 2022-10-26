#include "actuator.h"


// Serial pc_header(USBTX, USBRX, 115200);


actuator::actuator(PinName BRAKE_PIN_PUL_PLUS, PinName BRAKE_PIN_DIR_PLUS, PinName BRAKE_PIN_ENA_PLUS, int brake_pulse_per_rev_)
    :   BRAKE_PUL_PLUS(BRAKE_PIN_PUL_PLUS),
        BRAKE_DIR_PLUS(BRAKE_PIN_DIR_PLUS),
        BRAKE_ENA_PLUS(BRAKE_PIN_ENA_PLUS),
        brake_pulse_per_rev(brake_pulse_per_rev_)        
{
    brake_count_ms = 0;
    BRAKE_PUL_PLUS.period_us(450);
    BRAKE_PUL_PLUS = 0.0;

    brake_degree_per_pulse = brakeDegreePerPulse(brake_pulse_per_rev);

    brake_time.start();
    brake_ticker_1ms.attach(callback (this, &actuator::brakeCount1ms), 0.001);

    // pc_header.printf("CLE34 object initialized \r\n");
}


void actuator::brakeCount1ms()
{
    brake_count_ms++;
    if (brake_count_ms > 4000000000)  brake_count_ms = 0;
}


/*
    사용자가 CLE34 객체를 초기화면서 Pulse/rev 입력값에 따라,
    한 pulse당 몇 도 돌아가는지 계산함
*/
float actuator::brakeDegreePerPulse(float pulse_per_rev)
{
    return 360 / pulse_per_rev;
}




/*
    dir CW or CCW
*/
void actuator::brakeSetDir(int dir)
{
    BRAKE_DIR_PLUS = dir;
}

/*
    state가
    MOTOR_OFF -> 모터 잠김
    MOTOR_ON -> 모터 풀림
*/
void actuator::brakeEnableOn(int state)
{
    BRAKE_ENA_PLUS = state;
}


void actuator::brakeSetRPS(float speed_rps)
{
    float pwm_frequency;
    float pwm_period_us;

    pwm_frequency = speed_rps * brake_pulse_per_rev;
    pwm_period_us = 1000000 / pwm_frequency;

    BRAKE_PUL_PLUS.period_us(pwm_period_us);

    // pc_header.printf("pwm period : %f us\r\n", pwm_period_us);
}


/*
    Pulse/rev 4000일 때로 설정 가정한 함수
    Pulse/rev 수정되었으면, 별도의 함수 작성 요망
*/
void actuator::brakeTurnAngle(float angle_deg, int dir, float speed_rps)
{
    
    float pwm_generate_time_ms;
    brakeSetRPS(speed_rps);
    brakeSetDir(dir);
    // pc_header.printf("dir = %d\r\n", dir);


    pwm_generate_time_ms = 1000 * angle_deg / (speed_rps * 360);
    // pc_header.printf("pwm generating time : %f ms\r\n", pwm_generate_time_ms);

    brake_count_ms = 0;
    
    while (1)
    {
        
        if (brake_count_ms < pwm_generate_time_ms)
        {
            BRAKE_PUL_PLUS = 0.5;
            // pc_header.printf("gen count_ms : %d\r\n", count_ms);
        }
            
        else
            break;
    
    }
}


void actuator::brake_stop_ms(float ms)
{
    brake_count_ms = 0;
    while(1)
    {
        if (brake_count_ms < ms)
        {
            BRAKE_PUL_PLUS = 0.0;
            // pc_header.printf("stop count_ms : %d\r\n", count_ms);
        }
        else
            break;
    }
}