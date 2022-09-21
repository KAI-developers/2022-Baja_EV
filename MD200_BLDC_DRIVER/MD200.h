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

    making control signal for MD200 BLCD motor driver and precise position check(by encoder attached),
    to control steering actuator motor(BL9N from MD robot)

    using MEGA2560, driving motor in constant speed(internal speed setting in driver)
    with CHG switch OFF

    수업시간 프로그래밍 갬성으로
*/

#include "Arduino.h"


#define PPR                 16384

#define EXTERNAL_SPEED      HIGH
#define INTERNAL_SPEED      LOW

#define CCW                 HIGH
#define CW                  LOW

#define BRAKE_OFF           HIGH
#define BRAKE_ON            LOW

#define STOP                HIGH
#define RUN                 LOW


class MD200 {
private:
    int pin_INT_SPEED_;
    int pin_DIR_;
    int pin_START_STOP_;
    int pin_RUN_BRAKE_;

public:
    MD200(int pin_INT_SPEED, int pin_DIR, int pin_START_STOP, int pin_RUN_BRAKE);

    void setINT_SPEED(int mode);
    
    void enableBrake(int action);
    void runMotor(int dir, int action);


    // 안 쓸 예정
    void setDIR(int dir);
};

