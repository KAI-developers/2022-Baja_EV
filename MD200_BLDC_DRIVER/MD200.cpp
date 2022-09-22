#include "MD200.h"


// for arduino mega2560

// MD200::MD200(int pin_INT_SPEED, int pin_DIR, int pin_START_STOP, int pin_RUN_BRAKE)
// {
//     pin_INT_SPEED_ = pin_INT_SPEED;
//     pin_DIR_ = pin_DIR;
//     pin_START_STOP_ = pin_START_STOP;
//     pin_RUN_BRAKE_ = pin_RUN_BRAKE;


//     pinMode(pin_INT_SPEED, OUTPUT);
//     pinMode(pin_DIR, OUTPUT);
//     pinMode(pin_START_STOP, OUTPUT);
//     pinMode(pin_RUN_BRAKE, OUTPUT);
// }



// /*
// 드라이버 내부 가변저항 설정값으로 속도를 정할 것인지 정함.
// LOW(GND)면 내부볼륨을 사용

// mode 
//     - EXTERNAL_SPEED : 외부 가변저항으로 속도 조정
//     - INTERNAL_SPEED : 내부 가변저항(볼륨)으로 속도 정해짐
// */
// void MD200::setINT_SPEED(int mode)
// {
//     digitalWrite(pin_INT_SPEED_, mode);
// }



// /*
// 모터 회전 방향 결정
// LOW(GND)면 모터 축을 바라보는 기준으로 CW, 그 외 CCW

// dir
//     - CW (clockwise)
//     - CCW (counter clockwise)
// */
// void MD200::setDIR(int dir)
// {
//     digitalWrite(pin_DIR_, dir)
// }


// /*
// 스티어링 액츄에이터 고정을 위해 브레이크정지가 발생되는 구동 상태를 정의

// action
//     - BRAKE_ON
//     - BRAKE_OFF

// 자세한 내용은 MD200 사용설명서 참조, 아마 다르게 굴릴려면 수정해야함
// */
// void MD200::enableBrake(int action)
// {
//     digitalWrite(pin_START_STOP_, action);
// }


// /*
// dir
//     - CW (clockwise)
//     - CCW (counter clockwise)

// RUN ON일 때
//     - START/STOP ON이면 구동
//     - START/STOP OFF면 관성에 의한 정지
// RUN OFF일 때
//     - START/STOP ON이면 구동 안함
//     - START/STOP OFF면 브레이크 정지
// */
// void MD200::runMotor(int dir, int action)
// {
//     digitalWrite(pin_DIR_, dir)
//     digitalWrite(pin_RUN_BRAKE_, action);
// }




// for mbed lpc1768

// for controlling velocity
MD200::MD200(PinName PIN_INT_SPEED, PinName PIN_DIR, PinName PIN_START_STOP, PinName PIN_RUN_BRAKE, PinName PIN_SPEED)
    :   INT_SPEED(PIN_INT_SPEED), DIR(PIN_DIR), RUN_BRAKE(PIN_RUN_BRAKE), START_STOP(PIN_START_STOP), SPEED(PIN_SPEED)
{
    INT_SPEED = INTERNAL_SPEED;
    DIR = CW;
    START_STOP = BRAKE_ON;
    RUN_BRAKE = STOP;

    SPEED.period_us(25);
    SPEED = 0.0;
}


/*
// for using constant velocity
MD200::MD200(PinName PIN_INT_SPEED, PinName PIN_DIR, PinName PIN_START_STOP, PinName PIN_RUN_BRAKE, PinName PIN_SPEED)
    :   INT_SPEED(PIN_INT_SPEED), DIR(PIN_DIR), RUN_BRAKE(PIN_RUN_BRAKE), START_STOP(PIN_START_STOP)
{
    INT_SPEED = INTERNAL_SPEED;
    DIR = CW;
    START_STOP = BRAKE_ON;
    RUN_BRAKE = STOP;
}*/


/*
드라이버 내부 가변저항 설정값으로 속도를 정할 것인지 정함.
LOW(GND)면 내부볼륨을 사용

mode 
    - EXTERNAL_SPEED : 외부 가변저항으로 속도 조정
    - INTERNAL_SPEED : 내부 가변저항(볼륨)으로 속도 정해짐
*/
void MD200::setINT_SPEED(int mode)
{
    //digitalWrite(pin_INT_SPEED_, mode);
    INT_SPEED = mode;
}



/*
모터 회전 방향 결정
LOW(GND)면 모터 축을 바라보는 기준으로 CW, 그 외 CCW

dir
    - CW (clockwise)
    - CCW (counter clockwise)
*/
void MD200::setDIR(int dir)
{
    DIR = dir;
}


/*
스티어링 액츄에이터 고정을 위해 브레이크정지가 발생되는 구동 상태를 정의

action
    - BRAKE_ON
    - BRAKE_OFF

자세한 내용은 MD200 사용설명서 참조, 아마 다르게 굴릴려면 수정해야함
*/
void MD200::enableBrake(int action)
{
    START_STOP = action;
}


/*
for INTERNAL_SPEED mode or using external volume assembled
dir
    - CW (clockwise)
    - CCW (counter clockwise)

RUN ON일 때
    - START/STOP ON이면 구동
    - START/STOP OFF면 관성에 의한 정지
RUN OFF일 때
    - START/STOP ON이면 구동 안함
    - START/STOP OFF면 브레이크 정지
*/
void MD200::runMotor(int dir, int action)
{
    // digitalWrite(pin_DIR_, dir)
    // digitalWrite(pin_RUN_BRAKE_, action);

    DIR = dir;
    RUN_BRAKE = action;
}


/*
모터 최대 RPM 5000으로 설정한다고 가정함.
mbed 구동전압 3V3의 한계로 최대 3300RPM 출력 가능

map함수 이용
long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
in_min : 0
in_max : 3300RPM
out_min : 0
out_max : 1.0
*/
void MD200::runMotor(int dir, int action, float speed_RPM)
{
    // digitalWrite(pin_DIR_, dir)
    // digitalWrite(pin_RUN_BRAKE_, action);
    float SPEED_pwm = 0.0;

    DIR = dir;
    RUN_BRAKE = action;
    
    // 목표 RPM만큼 SPEED핀에 인가함
    SPEED_pwm = (speed_RPM - 0.0) * (1.0 - 0.0) / (MAX_RPM - 0.0) + 0.0;

    if (SPEED_pwm < 0.0)                            SPEED = 0.0;
    else if (SPEED_pwm >= 0.0 && SPEED_pwm < 1.0)   SPEED = SPEED_pwm;
    else                                            SPEED = 1.0;
}

