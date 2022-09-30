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


#define CW              0
#define CCW             1

#define MOTOR_ON        0
#define MOTOR_OFF       1

class CLE34 {
private:
    PwmOut PUL_PLUS;
    DigitalOut DIR_PLUS;
    DigitalOut ENA_PLUS;

protected:
    Timer time;
    Ticker ticker_1ms;

    unsigned int count_ms;

    float pulse_per_rev;
    float degree_per_pulse;

    void count1ms();
    float degreePerPulse(float pulse_per_rev);

public:
    CLE34(PinName PIN_PUL_PLUS, PinName PIN_DIR_PLUS, PinName PIN_ENA_PLUS, int pulse_per_rev_);

    void setDir(int dir);
    void enableOn(int state);
    void turnAngle(float angle_deg, int dir, float speed_radps);

};


#endif      // CLE34_H