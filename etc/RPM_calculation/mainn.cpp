#include "mbed.h"
#include "HallSensor.h"

//////////////////////////////////
#define PEDAL_MIN_VALUE     0.213
#define PEDAL_MAX_VALUE     0.68

#define IDEAL_OPAMP_GAIN    1.515
#define REAL_OPAMP_GAIN     1.7

#define PERIOD_US           25



Plotting plot;
unsigned int uiFlag_50ms = 0;

void counter_1ms ()
{
    uiFlag_50ms++;
}


float map(float input, float in_min, float in_max, float out_min, float out_max)
{
    return (input - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


int main() {

    Serial pc(USBTX, USBRX, 115200);
    HallSensor Hall_FL(p11);
  
    float RPM;
    
    AnalogIn pedal(p15);
    PwmOut throttle_out(p21);

    float pedal_read_value;
    float modified_pedal_value;

    throttle_out.period_us(PERIOD_US);




    Ticker ticker_1ms;
    ticker_1ms.attach(&counter_1ms, 0.001);
    Timer time;
    time.start();

    

    while(1) {
        pedal_read_value = pedal.read();
        pc.printf("pedal raw value (0.0 ~ 1.0) : %f \r\n", pedal_read_value);

        modified_pedal_value = map(pedal_read_value, PEDAL_MIN_VALUE, PEDAL_MAX_VALUE, 0.1, 0.8);
        
        if (modified_pedal_value < 0.0)
            modified_pedal_value = 0;
            
        pc.printf("modified pedal value (0.0 ~ 1.0) : %f\r\n", modified_pedal_value);

        throttle_out = modified_pedal_value * IDEAL_OPAMP_GAIN / REAL_OPAMP_GAIN;

        pc.printf("throttle output voltage (before amplified) : %f\r\n\n\n", throttle_out.read() * 3.3);



        RPM = Hall_FL.getRPM();
        pc.printf("rpm: %f\r\n", RPM);

        /*
        if(uiFlag_50ms>=50) {
            uiFlag_50ms=0;

            // clear plotting buffer
            plot.reset();

            // put data to buffer
            plot.put(OUTPUT ,0);

            // send buffer
            plot.send(&pc);
        }*/
    }

}