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


#ifndef ACTUATOR_H
#define ACTUATOR_H

#include "mbed.h"


#define BRAKE_CW              0
#define BRAKE_CCW             1

#define BRAKE_MOTOR_ON        0
#define BRAKE_MOTOR_OFF       1




class actuator {
public:

    PwmOut BRAKE_PUL_PLUS;
    DigitalOut BRAKE_DIR_PLUS;
    DigitalOut BRAKE_ENA_PLUS;

    Timer brake_time;
    Ticker brake_ticker_1ms;

    unsigned int brake_count_ms;

    float brake_pulse_per_rev;
    float brake_degree_per_pulse;

    void brakeCount1ms();
    float brakeDegreePerPulse(float pulse_per_rev);

    actuator(PinName PIN_PUL_PLUS, PinName PIN_DIR_PLUS, PinName PIN_ENA_PLUS, int pulse_per_rev_);

    void brakeSetDir(int dir);
    void brakeEnableOn(int state);
    void brakeSetRPS(float speed_rps);
    void brakeTurnAngle(float angle_deg, int dir, float speed_rps);
    void brake_stop_ms(float ms);


};


#endif // ACTUATOR_H