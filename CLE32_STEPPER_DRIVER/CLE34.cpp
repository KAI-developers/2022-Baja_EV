#include "CLE34.h"


Serial pc_header(USBTX, USBRX, 115200);


CLE34::CLE34(PinName PIN_PUL_PLUS, PinName PIN_DIR_PLUS, PinName PIN_ENA_PLUS, int pulse_per_rev_)
    :   PUL_PLUS(PIN_PUL_PLUS),
        DIR_PLUS(PIN_DIR_PLUS),
        ENA_PLUS(PIN_ENA_PLUS),
        pulse_per_rev(pulse_per_rev_)        
{
    count_ms = 0;
    PUL_PLUS.period_us(450);
    PUL_PLUS = 0.0;

    degree_per_pulse = degreePerPulse(pulse_per_rev);

    time.start();
    ticker_1ms.attach(callback (this, &CLE34::count1ms), 0.001);

    pc_header.printf("CLE34 object initialized \r\n");
}

void CLE34::count1ms()
{
    count_ms++;
    if (count_ms > 4000000000)  count_ms = 0;
}


/*
    사용자가 CLE34 객체를 초기화면서 Pulse/rev 입력값에 따라,
    한 pulse당 몇 도 돌아가는지 계산함
*/
float CLE34::degreePerPulse(float pulse_per_rev)
{
    return 360 / pulse_per_rev;
}




/*
    dir CW or CCW
*/
void CLE34::setDir(int dir)
{
    DIR_PLUS = dir;
}

/*
    state가
    MOTOR_OFF -> 모터 잠김
    MOTOR_ON -> 모터 풀림
*/
void CLE34::enableOn(int state)
{
    ENA_PLUS = state;
}


void CLE34::setRPS(float speed_rps)
{
    float pwm_frequency;
    float pwm_period_us;

    pwm_frequency = speed_rps * pulse_per_rev;
    pwm_period_us = 1000000 / pwm_frequency;

    PUL_PLUS.period_us(pwm_period_us);

    pc_header.printf("pwm period : %f us\r\n", pwm_period_us);
}


/*
    Pulse/rev 4000일 때로 설정 가정한 함수
    Pulse/rev 수정되었으면, 별도의 함수 작성 요망
*/
void CLE34::turnAngle(float angle_deg, int dir, float speed_rps)
{
    
    float pwm_generate_time_ms;

    setDir(dir);
    pc_header.printf("dir = %d\r\n", dir);


    pwm_generate_time_ms = 1000 * angle_deg / (speed_rps * 360);
    pc_header.printf("pwm generating time : %f ms\r\n", pwm_generate_time_ms);

    count_ms = 0;
    
    while (1)
    {
        
        if (count_ms < pwm_generate_time_ms)
        {
            PUL_PLUS = 0.5;
            // pc_header.printf("gen count_ms : %d\r\n", count_ms);
        }
            
        else
            break;
    
    }
}

void CLE34::stop_ms(float ms)
{
    count_ms = 0;
    while(1)
    {
        if (count_ms < ms)
        {
            PUL_PLUS = 0.0;
            // pc_header.printf("stop count_ms : %d\r\n", count_ms);
        }
        else
            break;
    }
}