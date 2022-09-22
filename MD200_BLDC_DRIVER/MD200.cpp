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
MD200::MD200(PinName PIN_INT_SPEED, PinName PIN_DIR, PinName PIN_START_STOP, PinName PIN_RUN_BRAKE)
    : INT_SPEED(PIN_INT_SPEED), DIR(PIN_DIR), START_STOP(PIN_START_STOP), RUN_BRAKE(PIN_RUN_BRAKE)
{
    INT_SPEED = INTERNAL_SPEED;
    DIR = CW;
    START_STOP = BRAKE_ON;
    RUN_BRAKE = STOP;
}



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

