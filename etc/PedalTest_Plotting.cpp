#include "mbed.h"
#include "Plotting.h"

#define PEDAL_MIN_VALUE     0.213
#define PEDAL_MAX_VALUE     0.68

#define IDEAL_OPAMP_GAIN    1.515
#define REAL_OPAMP_GAIN_FL  1.515
#define REAL_OPAMP_GAIN_FR  1.515
#define REAL_OPAMP_GAIN_RL  1.515
#define REAL_OPAMP_GAIN_RR  1.515


#define FL_THROTTLE_MAX     0.81
#define FR_THROTTLE_MAX     0.81
#define RL_THROTTLE_MAX     0.81
#define RR_THROTTLE_MAX     0.81


#define PERIOD_US           25

float map(float input, float in_min, float in_max, float out_min, float out_max)
{
    return (input - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


Plotting plot;
unsigned int uiFlag_50ms = 0;
void counter_1ms ()
{
    uiFlag_50ms++;
}


int main() {
    Serial pc(USBTX, USBRX, 115200);
    
    
    
    Ticker ticker_1ms;
    ticker_1ms.attach(&counter_1ms, 0.001);
    Timer time;
    time.start();
    
    
    AnalogIn steering(p19);
    AnalogIn pedal(p20);
    PwmOut throttle_out_FL(p21);
    PwmOut throttle_out_FR(p22);
    PwmOut throttle_out_RL(p23);
    PwmOut throttle_out_RR(p24);
    
    
    float pedal_read_value;
    float modified_pedal_value;
    float steering_value;

    throttle_out_FL.period_us(PERIOD_US);
    throttle_out_FR.period_us(PERIOD_US);
    throttle_out_RL.period_us(PERIOD_US);
    throttle_out_RR.period_us(PERIOD_US);

    while(1) {
        steering_value = steering.read();
        pedal_read_value = pedal.read();
//        pc.printf("pedal raw value (0.0 ~ 1.0) : %f \r\n", pedal_read_value);

        modified_pedal_value = map(pedal_read_value, PEDAL_MIN_VALUE, PEDAL_MAX_VALUE, 0.1, FL_THROTTLE_MAX);
        
        if (modified_pedal_value < 0.1)     modified_pedal_value = 0.1;
        if (modified_pedal_value >= 0.81)   modified_pedal_value = 0.81;

        
//        pc.printf("modified pedal value (0.0 ~ 1.0) : %f\r\n", modified_pedal_value);
        
        
        throttle_out_FL = modified_pedal_value * IDEAL_OPAMP_GAIN / REAL_OPAMP_GAIN_FL;
        throttle_out_FR = modified_pedal_value * IDEAL_OPAMP_GAIN / REAL_OPAMP_GAIN_FR;
        throttle_out_RL = modified_pedal_value * IDEAL_OPAMP_GAIN / REAL_OPAMP_GAIN_RL;
        throttle_out_RR = modified_pedal_value * IDEAL_OPAMP_GAIN / REAL_OPAMP_GAIN_RR;
        
        
        // 1V 출력하여 opamp 배율 확인
        /*
        throttle_out_FL = 1.0 / 3.3;
        throttle_out_FR = 1.0 / 3.3;
        throttle_out_RL = 1.0 / 3.3;
        throttle_out_RR = 1.0 / 3.3;
        */
        
                  
        //pc.printf("throttle output voltage (before amplified) : %f, %f, %f, %f \r\n\n\n",
//            throttle_out_FL.read() * 3.3, throttle_out_FR.read() * 3.3, throttle_out_RL.read() * 3.3, throttle_out_RR.read() * 3.3);
        
        
        if(uiFlag_50ms>=50) {
            uiFlag_50ms=0;

            // clear plotting buffer
            plot.reset();

            // put data to buffer
            // plot.put(pedal_read_value ,0);
            plot.put(steering_value, 0);
            // send buffer
            plot.send(&pc);
        }

    }
    
}
