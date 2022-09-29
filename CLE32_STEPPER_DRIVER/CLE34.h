


/*
         _____                              _   __     ___      _____    
        |_   _|                            | | / /    / _ \    |_   _|   
          | |    ___   __ _  _ __ ___      | |/ /    / /_\ \     | |     
          | |   / _ \ / _` || '_ ` _ \     |    \    |  _  |     | |     
          | |  |  __/| (_| || | | | | |    | |\  \ _ | | | | _  _| |_  _ 
          \_/   \___| \__,_||_| |_| |_|    \_| \_/(_)\_| |_/(_) \___/ (_)

 _____  _____  _____  _____  ______           _                 _____  _   _  ______                     _                                
/ __  \|  _  |/ __  \/ __  \ | ___ \         (_)               |  ___|| | | | |  _  \                   | |                               
`' / /'| |/' |`' / /'`' / /' | |_/ /  __ _    _   __ _  ______ | |__  | | | | | | | |  ___ __   __  ___ | |  ___   _ __    ___  _ __  ___ 
  / /  |  /| |  / /    / /   | ___ \ / _` |  | | / _` ||______||  __| | | | | | | | | / _ \\ \ / / / _ \| | / _ \ | '_ \  / _ \| '__|/ __|
./ /___\ |_/ /./ /___./ /___ | |_/ /| (_| |  | || (_| |        | |___ \ \_/ / | |/ / |  __/ \ V / |  __/| || (_) || |_) ||  __/| |   \__ \
\_____/ \___/ \_____/\_____/ \____/  \__,_|  | | \__,_|        \____/  \___/  |___/   \___|  \_/   \___||_| \___/ | .__/  \___||_|   |___/
                                            _/ |                                                                  | |                     
                                           |__/                                                                   |_|                     

    박성훈 designed from Konkuk Univ, Team K.A.I. 2022 Baja-EV team Develop Part

    CLE34 stepper motor driver code, using as brake actuator

    mbed LPC1768
 
    수업시간 프로그래밍 갬성으로
*/

#ifndef CLE34_H
#define CLE34_H

#include "mbed.h"
// #include "Plotting.h"

#define CW      0
#define CCW     1



class CLE34 {
private:
    DigitalOut PUL_PLUS;
    DigitalOut DIR_PLUS;
    DigitalOut ENA_PLUS;

protected:
    Timer time;
    Ticker ticker_1us;
    float m_period_sec;

    long count_us = 0.;
    void counter_us();

public:
    CLE34(PinName PIN_PUL_PLUS, PinName PIN_DIR_PLUS, PinName PIN_ENA_PLUS);

    void setDIR(int dir);
    void turnAngle(float angle, int dir, float speed);

};







#endif      // CLE34_H