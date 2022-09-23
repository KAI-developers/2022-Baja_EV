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

    making control signal for MD200 BLDC motor driver and precise position check(by encoder attached),
    to control steering actuator motor(BL9N from MD robot)

    using MEGA2560, driving motor in external speed setting in driver
    with CHG switch OFF

    수업시간 프로그래밍 갬성으로
*/

// #ifndef MD200_H
// #define MD200_H

// #include "Arduino.h"


// #define PPR                 16384

// #define EXTERNAL_SPEED      HIGH
// #define INTERNAL_SPEED      LOW

// #define CCW                 HIGH
// #define CW                  LOW

// #define BRAKE_OFF           HIGH
// #define BRAKE_ON            LOW

// #define STOP                HIGH
// #define RUN                 LOW


// for mbed mega2560
// class MD200 {
// private:
//     int pin_INT_SPEED_;
//     int pin_DIR_;
//     int pin_START_STOP_;
//     int pin_RUN_BRAKE_;

// public:
//     MD200(int pin_INT_SPEED, int pin_DIR, int pin_START_STOP, int pin_RUN_BRAKE);

//     void setINT_SPEED(int mode);
    
//     void enableBrake(int action);
//     void runMotor(int dir, int action);


//     // 안 쓸 예정
//     void setDIR(int dir);
// };


#include "mbed.h"


#define PPR                 16384

#define EXTERNAL_SPEED      1
#define INTERNAL_SPEED      0

#define CCW                 1
#define CW                  0

#define BRAKE_OFF           1
#define BRAKE_ON            0

#define STOP                1
#define RUN                 0

#define MAX_RPM             3300.0  // MD200 최대 5000RPM으로 설정 시의 값

class MD200 {
private:
    DigitalOut INT_SPEED;
    DigitalOut DIR;
    DigitalOut START_STOP;
    DigitalOut RUN_BRAKE;
    PwmOut SPEED;

public:
    MD200(PinName PIN_INT_SPEED, PinName PIN_DIR, PinName PIN_RUN_BRAKE, PinName PIN_START_STOP, PinName PIN_SPEED);

    void setINT_SPEED(int mode);                      // EXTERNAL_SPEED or INTERNAL_SPEED
    
    void enableBrake(int action);                     // BRAKE_OFF or BRAKE_ON
    void runMotor(int dir, int action);               // CW or CCW, RUN or STOP
    void runMotor(int dir, int action, float speed_RPM);  // CW/CCW, RUN/STOP, input speed


    // 안 쓸 예정
    void setDIR(int dir);
};


// #endif  // MD200_H33 