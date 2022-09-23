#include "mbed.h"
#include "MD200.h"

// need to edit MD200.h

int main()
{
    Serial pc(USBTX, USBRX);
    
    MD200 driver(p11, p12, p13, p14);

    driver.setINT_SPEED(EXTERNAL_SPEED);
    driver.enableBrake(BRAKE_ON);

    AnalogIn run_motor(p15);
    AnalogIn set_direction(p16);
    
    float p15value;
    float p16value;
    
    
    while(1)
    {
        p15value = run_motor.read();
        p16value = set_direction.read();
        
        if(p15value > 0.5)      // run motor by potentiometer
        {
            if(p16value > 0.5)
            {
                driver.runMotor(CW, RUN);
                pc.printf("CW, RUN \r\n");
            }
            else
            {
                driver.runMotor(CCW, RUN);
                pc.printf("CCW, RUN \r\n");
            }
        }

        else
        {
            pc.printf("STOP \r\n");
            driver.runMotor(CW, STOP);
        }
    }

}