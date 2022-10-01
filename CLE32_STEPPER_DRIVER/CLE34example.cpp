#include "CLE34.h"


/*
 * break actuator
 * 요구 속도 : 32.1 rpm (0.535 rps)
 * 요구 각도 : 57.78 degree
 *  
*/ 

int main()
{
    // Serial pc(USBTX, USBRX, 115200);
    // pc.printf("pc init\r\n");

    CLE34 stepdriver(p22, p10, p14, 4000);
    // pc.printf("CLE34 init\r\n");

    stepdriver.enableOn(MOTOR_ON);

    

    wait(1);


    while(1)
    {
        stepdriver.turnAngle(360, CW, 0.535);
        // pc.printf("stop 1sec\r\n");
        stepdriver.stop_ms(1000);
        // pc.printf("process 1 end \r\n\n");
        
        stepdriver.turnAngle(90, CCW, 0.535);
        // pc.printf("stop 1sec\r\n");
        stepdriver.stop_ms(1000);
        // pc.printf("process 2 end \r\n\n");

        stepdriver.turnAngle(90, CCW, 0.535);
        // pc.printf("stop 1sec\r\n");
        stepdriver.stop_ms(1000);
        // pc.printf("process 3 end \r\n\n");

        stepdriver.turnAngle(90, CCW, 0.535);
        // pc.printf("stop 1sec\r\n");
        stepdriver.stop_ms(1000);
        // pc.printf("process 4 end \r\n\n");

        stepdriver.turnAngle(90, CCW, 0.535);
        // pc.printf("stop 1sec\r\n");
        stepdriver.stop_ms(1000);
        // pc.printf("process 5 end \r\n\n");
    }
}