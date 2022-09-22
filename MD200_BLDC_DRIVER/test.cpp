#include "mbed.h"
#include "MD200.h"

int main()
{
    MD200 driver(p11, p12, p13, p14);

    AnalogIn run_motor(p15);
    AnalogIn set_direction(p16);

    driver.setINT_SPEED(EXTERNAL_SPEED);
    driver.enableBrake(BRAKE_ON);


    while(1)
    {
        if(run_motor.read() > 0.5)      // run motor by potentiometer
        {
            if(set_direction.read() > 0.5)
                driver.runMotor(CW, RUN);
            else
                driver.runMotor(CCW, RUN);
        }

        else
        {
            driver.runMotor(CW, STOP);
        }
    }

}