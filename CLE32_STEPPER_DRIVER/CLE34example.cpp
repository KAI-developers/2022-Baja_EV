#include "CLE34.h"


int main()
{
    Serial pc(USBTX, USBRX, 115200);
    pc.printf("pc init\r\n");

    CLE34 stepdriver(p22, p10, p14, 4000);
    pc.printf("CLE34 init\r\n");

    stepdriver.enableOn(MOTOR_ON);

    stepdriver.setRPS(1.111);

    wait(1);


    while(1)
    {
        stepdriver.turnAngle(180, CW, 1.111);
        stepdriver.stop_ms(1000);
        pc.printf("process 1 end \r\n");
        
        stepdriver.turnAngle(90, CCW, 1.111);
        stepdriver.stop_ms(1000);
        pc.printf("process 2 end \r\n");

        stepdriver.turnAngle(90, CW, 1.111);
        stepdriver.stop_ms(1000);
        pc.printf("process 3 end \r\n");

        stepdriver.turnAngle(180, CCW, 1.111);
        stepdriver.stop_ms(1000);
        pc.printf("process 4 end \r\n");

    }
}