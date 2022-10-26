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



#define STEERING_PPR                         16384

#define STEERING_EXTERNAL_SPEED     1
#define STEERING_INTERNAL_SPEED     0

#define STEERING_CCW                1
#define STEERING_CW                 0

#define STEERING_BRAKE_OFF          1
#define STEERING_BRAKE_ON           0

#define STEERING_STOP               1
#define STEERING_RUN                0

#define STEERING_MAX_RPM            3300.0  // MD200 최대 5000RPM으로 설정 시 mbed가 인가하는 최대RPM신호 근사값


class actuator {
private:
    PwmOut BRAKE_PUL_PLUS;
    DigitalOut BRAKE_DIR_PLUS;
    DigitalOut BRAKE_ENA_PLUS;

    Timer brake_time;
    Ticker brake_ticker_1ms;

    unsigned int brake_count_ms;

    float brake_pulse_per_rev;
    float brake_degree_per_pulse;
//////////////////////////////////////////////
//////////////////////////////////////////////
    DigitalOut STEERING_INT_SPEED;
    DigitalOut STEERING_DIR;
    DigitalOut STEERING_START_STOP;
    DigitalOut STEERING_RUN_BRAKE;
    PwmOut STEERING_SPEED;     // need to erase when using constant velocity



public:
    void brakeCount1ms();
    float brakeDegreePerPulse(float pulse_per_rev);

    actuator(PinName PIN_PUL_PLUS, PinName PIN_DIR_PLUS, PinName PIN_ENA_PLUS, int pulse_per_rev_);

    void brakeSetDir(int dir);
    void brakeEnableOn(int state);
    void brakeSetRPS(float speed_rps);
    void brakeTurnAngle(float angle_deg, int dir, float speed_rps);
    void brake_stop_ms(float ms);
////////////////////////////////////////////////
////////////////////////////////////////////////
    void steeringSetINT_SPEED(int mode);                          // EXTERNAL_SPEED or INTERNAL_SPEED
    
    void steeringEnableBrake(int action);                         // BRAKE_OFF or BRAKE_ON (START/STOP 핀)
    void steeringRunMotor(int dir, int action);                   // CW or CCW, RUN or STOP (BRK&PULSE_IN 핀), for INTERNAL_SPEED setting
    void steeringRunMotor(int dir, int action, float speed_RPM);  // CW/CCW, RUN/STOP, input speed (SPEED 핀)
    
    // 안 쓸 예정
    void steeringSetDIR(int dir);
};


#endif // ACTUATOR_H