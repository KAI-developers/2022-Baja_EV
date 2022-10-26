#include "actuator.h"


// Serial pc_header(USBTX, USBRX, 115200);


actuator::actuator(PinName BRAKE_PIN_PUL_PLUS, PinName BRAKE_PIN_DIR_PLUS, PinName BRAKE_PIN_ENA_PLUS, int brake_pulse_per_rev_,
                    PinName STEERING_PIN_INT_SPEED, PinName STEERING_PIN_DIR, PinName STEERING_PIN_START_STOP, PinName STEERING_PIN_RUN_BRAKE, PinName STEERING_PIN_SPEED)
    :   BRAKE_PUL_PLUS(BRAKE_PIN_PUL_PLUS),
        BRAKE_DIR_PLUS(BRAKE_PIN_DIR_PLUS),
        BRAKE_ENA_PLUS(BRAKE_PIN_ENA_PLUS),
        brake_pulse_per_rev(brake_pulse_per_rev_),

        STEERING_INT_SPEED(STEERING_PIN_INT_SPEED),
        STEERING_DIR(STEERING_PIN_DIR),
        STEERING_START_STOP(STEERING_PIN_START_STOP),
        STEERING_RUN_BRAKE(STEERING_PIN_RUN_BRAKE),
        STEERING_SPEED(STEERING_PIN_SPEED)    
{
    brake_count_ms = 0;
    BRAKE_PUL_PLUS.period_us(450);
    BRAKE_PUL_PLUS = 0.0;

    brake_degree_per_pulse = brakeDegreePerPulse(brake_pulse_per_rev);

    brake_time.start();
    brake_ticker_1ms.attach(callback (this, &actuator::brakeCount1ms), 0.001);

    // pc_header.printf("CLE34 object initialized \r\n");
    ///////////////////////////////////////////////////

    STEERING_INT_SPEED = STEERING_INTERNAL_SPEED;
    STEERING_DIR = STEERING_CW;
    STEERING_START_STOP = STEERING_BRAKE_ON;
    STEERING_RUN_BRAKE = STEERING_STOP;

    STEERING_SPEED.period_us(25);
    STEERING_SPEED = 0.0;
}


void actuator::brakeCount1ms()
{
    brake_count_ms++;
    if (brake_count_ms > 4000000000)  brake_count_ms = 0;
}


/*
    사용자가 CLE34 객체를 초기화면서 Pulse/rev 입력값에 따라,
    한 pulse당 몇 도 돌아가는지 계산함
*/
float actuator::brakeDegreePerPulse(float pulse_per_rev)
{
    return 360 / pulse_per_rev;
}




/*
    dir CW or CCW
*/
void actuator::brakeSetDir(int dir)
{
    BRAKE_DIR_PLUS = dir;
}

/*
    state가
    MOTOR_OFF -> 모터 잠김
    MOTOR_ON -> 모터 풀림
*/
void actuator::brakeEnableOn(int state)
{
    BRAKE_ENA_PLUS = state;
}


void actuator::brakeSetRPS(float speed_rps)
{
    float pwm_frequency;
    float pwm_period_us;

    pwm_frequency = speed_rps * brake_pulse_per_rev;
    pwm_period_us = 1000000 / pwm_frequency;

    BRAKE_PUL_PLUS.period_us(pwm_period_us);

    // pc_header.printf("pwm period : %f us\r\n", pwm_period_us);
}


/*
    Pulse/rev 4000일 때로 설정 가정한 함수
    Pulse/rev 수정되었으면, 별도의 함수 작성 요망
*/
void actuator::brakeTurnAngle(float angle_deg, int dir, float speed_rps)
{
    
    float pwm_generate_time_ms;
    brakeSetRPS(speed_rps);
    brakeSetDir(dir);
    // pc_header.printf("dir = %d\r\n", dir);


    pwm_generate_time_ms = 1000 * angle_deg / (speed_rps * 360);
    // pc_header.printf("pwm generating time : %f ms\r\n", pwm_generate_time_ms);

    brake_count_ms = 0;
    
    while (1)
    {
        
        if (brake_count_ms < pwm_generate_time_ms)
        {
            BRAKE_PUL_PLUS = 0.5;
            // pc_header.printf("gen count_ms : %d\r\n", count_ms);
        }
            
        else
            break;
    
    }
}


void actuator::brake_stop_ms(float ms)
{
    brake_count_ms = 0;
    while(1)
    {
        if (brake_count_ms < ms)
        {
            BRAKE_PUL_PLUS = 0.0;
            // pc_header.printf("stop count_ms : %d\r\n", count_ms);
        }
        else
            break;
    }
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
void actuator::steeringSetINT_SPEED(int mode)
{
    //digitalWrite(pin_INT_SPEED_, mode);
    STEERING_INT_SPEED = mode;
}



/*
모터 회전 방향 결정
LOW(GND)면 모터 축을 바라보는 기준으로 CW, 그 외 CCW

dir
    - CW (clockwise)
    - CCW (counter clockwise)
*/
void actuator::steeringSetDIR(int dir)
{
    STEERING_DIR = dir;
}


/*
스티어링 액츄에이터 고정을 위해 브레이크정지가 발생되는 구동 상태를 정의

action
    - BRAKE_ON
    - BRAKE_OFF

자세한 내용은 MD200 사용설명서 참조, 아마 다르게 굴릴려면 수정해야함
*/
void actuator::steeringEnableBrake(int action)
{
    STEERING_START_STOP = action;
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
void actuator::steeringRunMotor(int dir, int action)
{
    // digitalWrite(pin_DIR_, dir)
    // digitalWrite(pin_RUN_BRAKE_, action);

    STEERING_DIR = dir;
    STEERING_RUN_BRAKE = action;
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
float actuator::steeringRunMotor(int dir, int action, float speed_RPM)
{
    // digitalWrite(pin_DIR_, dir)
    // digitalWrite(pin_RUN_BRAKE_, action);
    float SPEED_pwm = 0.0;
    float retvalue;

    STEERING_DIR = dir;
    STEERING_RUN_BRAKE = action;
    
    // 목표 RPM만큼 SPEED핀에 인가함
    SPEED_pwm = (speed_RPM - 0.0) * (1.0 - 0.0) / (MAX_RPM - 0.0) + 0.0;

    if (SPEED_pwm < 0.0)                            retvalue = 0.0f;
    else if (SPEED_pwm >= 0.0 && SPEED_pwm < 1.0)   retvalue = SPEED_pwm;
    else                                            retvalue = 1.0f;

    return retvalue;
}