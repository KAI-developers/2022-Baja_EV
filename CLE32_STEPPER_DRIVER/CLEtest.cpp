#include "CLE34.h"


unsigned int count_ms = 0;

void count1ms()
{
    count_ms++;
    if (count_ms > 4000000000)  count_ms = 0;
}


int main()
{
    Serial pc(USBTX, USBRX, 115200);
    Timer time;
    Ticker ticker_1ms;

    time.start();
    ticker_1ms.attach(&count1ms, 0.001);



    float pwm_period_ms;

    PwmOut PUL_PLUS = p22;
    DigitalOut DIR_PLUS = p10;
    DigitalOut ENA_PLUS = p14;


    PUL_PLUS.period_us(467.29);
    PUL_PLUS = 0;
    ENA_PLUS = MOTOR_ON;

    while(1)
    {
        
        DIR_PLUS = CW;
        count_ms = 0;
        while(1)
        {
            if (count_ms < 300)
            {
                PUL_PLUS = 0.5;
                // pc.printf("count_ms : %d", count_ms);
            } else
                break;
        }
        
        count_ms = 0;
        while(1)
        {
            if (count_ms < 1000)
            { 
                PUL_PLUS = 0;
                // pc.printf("count_ms : %d", count_ms);
            } else
                break;
        }
        
        
        
        DIR_PLUS = CCW;
        count_ms = 0;
        while(1)
        {
            if (count_ms < 300)
            {
                PUL_PLUS = 0.5;
                // pc.printf("count_ms : %d", count_ms);
            } else
                break;
        }
        
        
        count_ms = 0;
        while(1)
        {
            if (count_ms < 1000)
            {
                PUL_PLUS = 0;
                // pc.printf("count_ms : %d", count_ms);
            } else
                break;
        } 
    }
}