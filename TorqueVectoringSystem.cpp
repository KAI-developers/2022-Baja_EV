#include "TorqueVectoringSystem.h"


TorqueVectoringSystem::TorqueVectoringSystem(
    PinName TVS_SWITCH_PIN, PinName FL_HALL_PIN, PinName FR_HALL_PIN, PinName RL_HALL_PIN, PinName RR_HALL_PIN, 
    PinName HANDLE_SENSOR_PIN, PinName MPU_SDA, PinName MPU_SCL, PinName PEDAL_SENSOR_PIN,
    PinName FL_CURRENT_SENSOR_PIN, PinName FR_CURRENT_SENSOR_PIN, PinName RL_CURRENT_SENSOR_PIN, PinName RR_CURRENT_SENSOR_PIN,
    PinName FL_OUTPUT_THROTTLE_PIN, PinName FR_OUTPUT_THROTTLE_PIN, PinName RL_OUTPUT_THROTTLE_PIN, PinName RR_OUTPUT_THROTTLE_PIN
    )
    :   RL_Hall_A(RL_HALL_PIN), RR_Hall_A(RR_HALL_PIN), mpu(MPU_SDA, MPU_SCL),
        Handle_Sensor(HANDLE_SENSOR_PIN), FL_Current_OUT(FL_CURRENT_SENSOR_PIN), FR_Current_OUT(FR_CURRENT_SENSOR_PIN), RL_Current_OUT(RL_CURRENT_SENSOR_PIN),
        RR_Current_OUT(RR_CURRENT_SENSOR_PIN), Pedal_Sensor(PEDAL_SENSOR_PIN),
        FL_Throttle_PWM(FL_OUTPUT_THROTTLE_PIN), FR_Throttle_PWM(FR_OUTPUT_THROTTLE_PIN), RL_Throttle_PWM(RL_OUTPUT_THROTTLE_PIN), RR_Throttle_PWM(RR_OUTPUT_THROTTLE_PIN)
{


    FL_Throttle_PWM.period_us(PWM_PERIOD_US);
    FR_Throttle_PWM.period_us(PWM_PERIOD_US);
    RL_Throttle_PWM.period_us(PWM_PERIOD_US);
    RR_Throttle_PWM.period_us(PWM_PERIOD_US);
    /*
    FL_Throttle_PWM.period_ms(PWM_PERIOD_MS);
    FR_Throttle_PWM.period_ms(PWM_PERIOD_MS);
    RL_Throttle_PWM.period_ms(PWM_PERIOD_MS);
    RR_Throttle_PWM.period_ms(PWM_PERIOD_MS);
    */


    f_motor_current_FL_A = 0.0;
    f_motor_current_FR_A = 0.0;
    f_motor_current_RL_A = 0.0;
    f_motor_current_RR_A = 0.0;


    f_motor_RPM_FL = 0.0;
    f_motor_RPM_FR = 0.0;
    f_motor_RPM_RL = 0.0;
    f_motor_RPM_RR = 0.0;

    //pedal box
    f_pedal_sensor_value = 0.0;
    //handle
    f_pedal_modified_sensor_value = 0.0;
    i_PWR_percentage = 0;

    f_steering_sensor_value = 0.0;


    f_yaw_rate_meas_filtered_degs = 0.0;
    //temp value
    f_wheel_angle_deg = 0.0;
    
    f_vel_FL_ms = 0.0;
    f_vel_FR_ms = 0.0;
    f_vel_RL_ms = 0.0;
    f_vel_RR_ms = 0.0;

    f_vehicle_vel_ms = 0.0;

    f_yawrate_input_deg = 0.0;
    f_wheel_torque_FL_Nm = 0.0;
    f_wheel_torque_FR_Nm = 0.0;
    f_wheel_torque_RL_Nm = 0.0;
    f_wheel_torque_RR_Nm = 0.0;
    
    f_PID_yaw_rate2torque_FL_Nm = 0.0;
    f_PID_yaw_rate2torque_FR_Nm = 0.0;
    f_PID_yaw_rate2torque_RL_Nm = 0.0;
    f_PID_yaw_rate2torque_RR_Nm = 0.0;

    f_measured_torque_FL_Nm = 0.0;
    f_measured_torque_FR_Nm = 0.0;
    f_measured_torque_RL_Nm = 0.0;
    f_measured_torque_RR_Nm = 0.0;

    f_torque_FL_Nm = 0.0;               // targetted torque
    f_torque_FR_Nm = 0.0;
    f_torque_RL_Nm = 0.0;
    f_torque_RR_Nm = 0.0;

    f_output_throttle_FL = 0.0;
    f_output_throttle_FR = 0.0;
    f_output_throttle_RL = 0.0;
    f_output_throttle_RR = 0.0;

    f_PID_throttle_FL = 0.0;
    f_PID_throttle_FR = 0.0;
    f_PID_throttle_RL = 0.0;
    f_PID_throttle_RR = 0.0;

    f_PWM_input_FL = 0.0;
    f_PWM_input_FR = 0.0;
    f_PWM_input_RL = 0.0;
    f_PWM_input_RR = 0.0;

    IMU_gx = 0.0, IMU_gy = 0.0, IMU_gz = 0.0, IMU_ax = 0.0, IMU_ay = 0.0, IMU_az = 0.0;
    f_yawrate_meas_degs = 0.0;

    FL_Throttle_PWM = 0.0;
    FR_Throttle_PWM = 0.0;
    RL_Throttle_PWM = 0.0;
    RR_Throttle_PWM = 0.0;

}


Serial pc(USBTX, USBRX, 115200);

/*
- RPM 구하는 함수
- rising edge 시간 차를 이용해 계산
- input
    ??
- output
    f_motor_RPM
- configuration
    MOTOR_POLE = 7
float TorqueVectoringSystem::CalRPM(HallSensor hall)
{
    pc.printf("rpm: %f\n", hall.getRPM());        // for debuging
    float rpm = hall.getRPM();
    return rpm;
}
*/

/*
- 단순한 map 함수 구현(아두이노 함수 참조)
- float형 return
*/
float TorqueVectoringSystem::map_f(float input, float in_min, float in_max, float out_min, float out_max)
{
    return (input - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/*
- motor RPM --> m/s 변환하는 함수.
- input
    f_motor_RPM: float, motor(RPM)
- output
    f_vel_ms: float, velocity(m/s)
- configuration
    WHEEL_RADIUS = 0.15m
    GEAR_RATIO = 5.27
*/
float TorqueVectoringSystem::CvtRPM2Vel(float f_motor_RPM)
{
    float f_vel_ms = 2. * PI * WHEEL_RADIUS * (f_motor_RPM / GEAR_RATIO) / 60;
    return f_vel_ms;
}


/*
- 평균속도 구하는 함수.
- input
    f_vel_RR_ms: float, velocity(m/s), RR
    f_vel_RL_ms: float, velocity(m/s), RL
- output
    f_avg_vel_ms: float, velocity(m/s), average
- configuration: X
*/


float TorqueVectoringSystem::CalAvgVel(float f_velocity1_ms, float f_velocity2_ms)
{
    float f_avg_vel_ms = (f_velocity1_ms + f_velocity2_ms) / 2;
    return f_avg_vel_ms;
}



/*
- 가변 저항 센서 값을 조향 각으로 변환하는 함수.
- configuration
*/
float TorqueVectoringSystem::CalHandlingVolt2WheelSteeringAngle(float f_handling_sensor_value)
{
    // float resistor_angle = (f_handling_sensor_value - DEFAULT_VOLTAGE_INPUT) * MAX_RESISTOR_ANGLE;
    // float handle_angle = resistor_angle * (MAX_HANDLE_ANGLE / MAX_RESISTOR_LIMITED_ANGLE);
    // return handle_angle * (MAX_STEERING_ANGLE / MAX_HANDLE_ANGLE);

    float resistor_angle;
    float handle_angle;
    
    resistor_angle = (f_handling_sensor_value - DEFAULT_VOLTAGE_INPUT) * MAX_RESISTOR_ANGLE;
    handle_angle = resistor_angle * (MAX_HANDLE_ANGLE / MAX_RESISTOR_LIMITED_ANGLE);
    handle_angle = handle_angle * (MAX_STEERING_ANGLE / MAX_HANDLE_ANGLE);

    return handle_angle;
}



/*
- 차량 평균 속도와 바퀴 회전각을 이용하여 yawrate를 반환하는 함수.
- input
    f_wheel_steering_angle_deg: float, degree
    f_avg_vel_ms: float, velocity(m/s), average
- output
    f_input_yaw_rate_radps: float, yawrate(rad/s)
- configuration
    WHEEL_BASE = 1.390m
*/
float TorqueVectoringSystem::CalInputYawRate(float f_avg_vel_ms, float f_wheel_steering_angle_deg)
{
    float f_input_yaw_rate_radps = f_avg_vel_ms * sin(f_wheel_steering_angle_deg) / WHEEL_BASE;
    return f_input_yaw_rate_radps;
}



/*
- imu로 측정한 yawrate를 지수감쇠필터를 이용해 노이즈를 제거한 yawrate를 반환하는 함수.
- input
    f_IMU_yaw_rate_radps: from imu, yawrate(rad/s)
- output
    f_filtered_yaw_rate_radps: filtered, yawrate(rad/s)
- configuration
    ALPHA = 0.85
*/
float TorqueVectoringSystem::IMUFilter(float i_IMU_yaw_rate_radps)
{
    static float f_prev_yaw_rate_radps = 0.0;
    float f_filtered_yaw_rate_radps = ((1.0 - ALPHA) * f_prev_yaw_rate_radps) + (ALPHA * i_IMU_yaw_rate_radps);
    f_prev_yaw_rate_radps = f_filtered_yaw_rate_radps;
    return f_filtered_yaw_rate_radps;
}




/*
- 회전 반경, phi, 조향각을 이용하여 팔 길이를 구한 후, 팔 길이의 비율로 토크를 분배하는 함수.
- 팔 길이 계산
    fl = R * sin(phi - f_wheel_steering_angle_deg);
    fr = R * sin(phi + f_wheel_steering_angle_deg);
    rl = (-1) * R * sin(phi);
    rr = R * sin(phi)
- weight = 팔 길이 * 조향각 * throttle + throttle
- sum = weight(4방향)
- torque(1방향) = throttle / sum * weight(1방향)
- input
    f_wheel_steering_angle_deg: float, degree
    f_pedal_sensor_value : float, voltage(V)
- output
    f_wheel_torque_(4방향)_Nm: 4개 출력, float, torque(N*m)
- configuration
    WHEEL_BASE = 1.390m
    TRACK = 1.300m
-가장 큰 주의사항
    이 함수는 페달신호 최대 5V를 입력받는다고 가정하며, 이 것을 5V 스로틀신호 입력을 받는
    컨트롤러에 4개의 적절한 값 분배(스로틀신호)를 이룬 후에, 이에 대한 목표 토크를 출력한다.
*/
bool TorqueVectoringSystem::WheelSteeringAngle2Torque(float f_wheel_steering_angle_deg, float f_pedal_sensor_value,
    float& f_wheel_torque_FL_Nm, float& f_wheel_torque_FR_Nm, float& f_wheel_torque_RL_Nm, float& f_wheel_torque_RR_Nm)

{
    int dir;
    // normalize값 중 최대를 계산
    float max_weight;
    float f_wheel_torque_Nm[4];
    float f_steering_angle_rad;
    float pedal_throttle_voltage;
    float R;                        // 팔길이 계산에 필요한 조향각 0도일 때 차량 중심과 바퀴축 거리
    float phi;

    float FL_arm_m;
    float FR_arm_m;
    float RL_arm_m;
    float RR_arm_m;

    float weight[4] = {0.0, 0.0, 0.0, 0.0};
    float normalized_weight[4] = {0.0, 0.0, 0.0, 0.0};

    float sum = 0.;



    f_steering_angle_rad = f_wheel_steering_angle_deg * PI / 180;

    pedal_throttle_voltage = f_pedal_sensor_value * CONTROLLER_INPUT_VOLT_RANGE;      //  need to set by configuration
    
    if (pedal_throttle_voltage == 0.000000)
    {
        f_wheel_torque_FL_Nm = 0.0;
        f_wheel_torque_FR_Nm = 0.0;
        f_wheel_torque_RL_Nm = 0.0;
        f_wheel_torque_RR_Nm = 0.0;

        return -1;
    }

    
    R = sqrt(pow(WHEEL_BASE / 2.0, 2.0) + pow(TRACK / 2.0, 2.0));

    phi = atan((TRACK / 2.) / (WHEEL_BASE / 2.));

    // 팔길이 계산
    FL_arm_m = (-1) * R * sin(phi - f_steering_angle_rad);
    FR_arm_m = R * sin(phi + f_steering_angle_rad);
    RL_arm_m = (-1) * R * sin(phi);
    RR_arm_m = R * sin(phi);

    /*
    // only positive value of arm length used
    if(FL_arm_m < 0.0)  FL_arm_m = 0.0;
    if(FR_arm_m < 0.0)  FR_arm_m = 0.0;
    if(RL_arm_m < 0.0)  RL_arm_m = 0.0;
    if(RR_arm_m < 0.0)  RR_arm_m = 0.0;
    */

    // need to erase this
    pc.printf("\tfirst feed forward func \r\n");
    pc.printf("\tarm length : \r\n\tFL : %f, FR : %f, RL : %f, RR : %f\r\n", FL_arm_m, FR_arm_m, RL_arm_m, RR_arm_m);


    
    // 프로파일 함수에 대한 결과 계산 (rad 입력으로 수정)
    weight[FL] = FL_arm_m * f_steering_angle_rad * pedal_throttle_voltage + pedal_throttle_voltage;
    weight[FR] = FR_arm_m * f_steering_angle_rad * pedal_throttle_voltage + pedal_throttle_voltage;
    weight[RL] = RL_arm_m * f_steering_angle_rad * pedal_throttle_voltage + pedal_throttle_voltage;
    weight[RR] = RR_arm_m * f_steering_angle_rad * pedal_throttle_voltage + pedal_throttle_voltage;

    pc.printf("\tweight (profile func output) \r\n");
    pc.printf("\tFL : %f, FR : %f, RL : %f, RR : %f\r\n", weight[FL], weight[FR], weight[RL], weight[RR]);


    sum = weight[FL] + weight[FR] + weight[RL] + weight[RR];
    pc.printf("\tsum : %f\r\n", sum);

    //normalize
    for(dir = 0; dir < 4; dir++)
        normalized_weight[dir] = 4 * (pedal_throttle_voltage / sum) * weight[dir];

    pc.printf("\tnormalized weight\r\n");
    pc.printf("\tFL : %f, FR : %f, RL : %f, RR : %f\r\n", 
            normalized_weight[FL], normalized_weight[FR], normalized_weight[RL], normalized_weight[RR]);

    // find maximum normalized value
    max_weight = normalized_weight[FL];
    for(dir = 0; dir < 4; dir++)
    {
        if(max_weight < normalized_weight[dir]){
            max_weight = normalized_weight[dir];
        }
    }

    pc.printf("\tmax weight : %f\r\n", max_weight);


    // 0~max_weight 범위의 normalize된 값을 0~페달스로틀입력 으로 mapping
    for (dir = 0; dir < 4; dir++)
    {
        f_wheel_torque_Nm[dir] = map_f(normalized_weight[dir], TORQUE_VECTORING_RATE, max_weight, 0.0, pedal_throttle_voltage)
            * (ACTUAL_MAX_TORQUE_NY / CONTROLLER_INPUT_VOLT_RANGE);
            // 홀전류센서 장착 이후 실제 MAX_TORQUE 수정 요망
    }

    pc.printf("\tnormalized torque \r\n");
    pc.printf("\tFL : %f, FR : %f, RL : %f, RR : %f\r\n",
            f_wheel_torque_Nm[FL], f_wheel_torque_Nm[FR], f_wheel_torque_Nm[RL], f_wheel_torque_Nm[RR]);

    // 안전장치
    for (dir = 0; dir < 4; dir++)
    {
        if (f_wheel_torque_Nm[dir] < 0.0)   f_wheel_torque_Nm[dir] = 0;
    }
    
    f_wheel_torque_FL_Nm = f_wheel_torque_Nm[FL];
    f_wheel_torque_FR_Nm = f_wheel_torque_Nm[FR];
    f_wheel_torque_RL_Nm = f_wheel_torque_Nm[RL];
    f_wheel_torque_RR_Nm = f_wheel_torque_Nm[RR];

    return 0;
}


/*
- 차량 평균 속도와 바퀴 회전각을 이용해 계산한 yawrate와 imu로 측정하여 필터링된 yawrate를 입력으로 받아 PID 제어기로 torque 계산하는 함수.
- PID 제어기를 위한 error = 입력값(f_input_yaw_rate_radps) - 측정값(f_filtered_yaw_rate_radps)
- f_PID_yaw_rate2torque_Nm = KP * error (각각 4개)
- input
    f_input_yaw_rate_radps: float, yawrate(rad/s), 차량 평균 속도와 바퀴 회전각으로 계산.
    f_filtered_yaw_rate_radps: float, yawrate(rad/s), imu로 측정한 값을 필터링함.
- output
    f_PID_yaw_rate2torque_Nm (4개 각각 출력): float, torque(N*m)
- configuration
    KP_FOR_TORQUE (4개)
*/
void TorqueVectoringSystem::PIDYawRate2Torque(float f_input_yaw_rate_radps, float f_filtered_yaw_rate_radps,
    float& f_PID_yaw_rate2torque_FL_Nm, float& f_PID_yaw_rate2torque_FR_Nm,
    float& f_PID_yaw_rate2torque_RL_Nm, float& f_PID_yaw_rate2torque_RR_Nm)
{
    float f_yaw_rate_error_radps = f_input_yaw_rate_radps - f_filtered_yaw_rate_radps;


    if (f_yaw_rate_error_radps < 0.0)
    {
        f_PID_yaw_rate2torque_FL_Nm = KP_FOR_TORQUE_FL * f_yaw_rate_error_radps;
        f_PID_yaw_rate2torque_RL_Nm = KP_FOR_TORQUE_RL * f_yaw_rate_error_radps;

        f_PID_yaw_rate2torque_FR_Nm = 0;
        f_PID_yaw_rate2torque_RR_Nm = 0;
    }


    else
    {
        f_PID_yaw_rate2torque_FL_Nm = 0;
        f_PID_yaw_rate2torque_RL_Nm = 0;

        f_PID_yaw_rate2torque_FR_Nm = KP_FOR_TORQUE_FR * f_yaw_rate_error_radps;
        f_PID_yaw_rate2torque_RR_Nm = KP_FOR_TORQUE_RR * f_yaw_rate_error_radps;
    }

}

/*
- opamp 이용 전압 측정
- 션트 저항 양단 전압 증폭값을 읽음
- 옴의 법칙 이용 전류로 환산
- configuration
    amp rate(200)
    shunt resistance(50u)float
*/
float TorqueVectoringSystem::OpAmp2Current(float f_opamp_ADC)
{
    float opamp_voltage = f_opamp_ADC * ANALOG_RANGE;
    float shunt_voltage = opamp_voltage / AMP_RATE_MOTOR;
    float motor_current = shunt_voltage / SHUNT_R;
    return motor_current;
}

/*
- hall current transducer 이용 전류 측정
- 3.3V 전원 공급한 transducer의 출력 전압값을 전류로 환산
- map 함수 이용
- configuration
    입력 : ANALOG_RANGE (3.3V)
    출력 : CURRENT_SENSOR_VALUE (200A)
*/
float TorqueVectoringSystem::ReadCurrentSensor(float current_sensor_value)
{
    float current_sensor_output_V = current_sensor_value * ANALOG_RANGE;

    // map 함수 형식 이용, (-50A일 때의 센서값 ~ 100A일 때의 센서값) 을 (-50A ~ 100A) 로
    // long map(long x, long in_min, long in_max, long out_min, long out_max) {
    //     return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min; }

    // 사용한 전류센서는 wcs1500

    return (current_sensor_output_V - CURRENT_SENSOR_VALUE_m50A) * (100. * (-50.))
            / (CURRENT_SENSOR_VALUE_100A - CURRENT_SENSOR_VALUE_m50A) + (-50.);
}

/*
- 캔 통신 인터페이스 모듈(MCP2515)로 측정한 전류를 플레밍의 왼손 법칙을 이용하여 토크로 계산하는 함수.
- input
    f_motor_current_A(4개): float, current(A)
- output
    f_measured_torque_Nm(4개): float, torque(N*m)
- configuration
    KT(모터 토크 상수) = 17 / 148
*/
float TorqueVectoringSystem::CvtCurrent2Torque(float f_motor_current_A)
{
    return KT * f_motor_current_A;

}

/*
- PID 제어기로 계산한 torque를 이용해 output_throttle 생성하는 함수.
- input
    f_torque_(4방향)_Nm: 각각 4개, float, torque(N*m)
- output
    f_output_throttle_(4방향): 각각 4개, float, voltage(V)
- configuration
    MAX_TORQUE = 17Nm
    ACTUAL_MAX_TORQUE 는 추후 측정 예정.
*/
float TorqueVectoringSystem::Torque2Throttle(float f_torque_Nm)
{
    float f_output_throttle = f_torque_Nm * (ANALOG_RANGE / ACTUAL_MAX_TORQUE_NY);
    return f_output_throttle;
}

/*
- PID 제어기로 계산한 torque와 캔 통신으로 측정한 전류를 이용해 계산한 torque를 입력으로 받아 PID 제어기로 throttle을 계산하는 함수.
- PID 제어기를 위한 error = 입력값(f_torque_(4방향)_Nm) - 측정값(f_output_throttle_(4방향))
- f_PID_throttle_(4방향) = KP_FOR_THROTTLE * error (각각 4개)
- input
    f_torque_(4방향)_Nm: 각각 4개, float, torque(N*m), PID 제어기로 계산한 토크.
    f_output_throttle_(4방향): 각각 4개, float, torque(N*m), 캔 통신으로 측정한 전류를 이용해 계산한 토크.
- output
    f_PID_throttle_(4방향): 각각 4개, float, voltage(V)
- configuration
    KP_FOR_THROTTLE
*/
float TorqueVectoringSystem::PIDforThrottle(float f_torque_Nm, float f_measured_torque_Nm, int direction)
{
    float error = f_torque_Nm - f_measured_torque_Nm;
    float f_PID_throttle;

    if (direction == FL)        f_PID_throttle = KP_FOR_THROTTLE_FL * error * (ANALOG_RANGE / ACTUAL_MAX_TORQUE_NY);
    else if (direction == FR)   f_PID_throttle = KP_FOR_THROTTLE_FR * error * (ANALOG_RANGE / ACTUAL_MAX_TORQUE_NY);
    else if (direction == RL)   f_PID_throttle = KP_FOR_THROTTLE_RL * error * (ANALOG_RANGE / ACTUAL_MAX_TORQUE_NY);
    else if (direction == RR)   f_PID_throttle = KP_FOR_THROTTLE_RR * error * (ANALOG_RANGE / ACTUAL_MAX_TORQUE_NY); 

    return f_PID_throttle;
}

/*
- PWM 함수의 입력 범위가 올바르도록 Saturation하는 함수.
- input
    f_output_throttle_(4방향): 각각 4개, float, torque(N*m)
    f_PID_throttle_(4방향): 각각 4개, float, voltage(V)
- output
    f_PWM_input: 각각 4개, float, PWM 출력
- configuration
    LOWER_BOUND = 0
    UPPER_BOUND = 1
*/
float TorqueVectoringSystem::SumFFandPID(float f_output_throttle, float f_PID_throttle)
{
    float PWM_throttle_mbed_value = (f_output_throttle + f_PID_throttle) / ANALOG_RANGE;

    if (PWM_throttle_mbed_value >= UPPER_BOUND)
        return UPPER_BOUND;
    else if (PWM_throttle_mbed_value <= LOWER_BOUND)
        return LOWER_BOUND;
    else
        return PWM_throttle_mbed_value;
}

/*
페달의 최대각, 최소각에 따른 실제 입력값(mbed의 경우 0.0~1.0)
의 값을 실제로 측정 후, 이를 0.0~1.0의 값으로 mapping하는 함수
*/
float TorqueVectoringSystem::ModifyPedalThrottle(float input, float in_min, float in_max, float out_min, float out_max)
{
    return (input - in_min)*(out_max - out_min) /  (in_max - in_min) + out_min;
}


void TorqueVectoringSystem::process_accel()
{


    // DigitalIn  TVS_SWITCH(TVS_SWITCH_PIN);

    float trimmed_throttle_FL;      // 0.5V에서 4.1V로 출력 변경을 위한 변수
    float trimmed_throttle_FR;
    float trimmed_throttle_RL;
    float trimmed_throttle_RR;



    pc.printf("entered WHILE : \r\n");

    //f_motor_RPM_FL = FL_Hall_A.getRPM();
    //f_motor_RPM_FR = FR_Hall_A.getRPM();
    f_motor_RPM_RL = RL_Hall_A.getRPM();
    f_motor_RPM_RR = RR_Hall_A.getRPM();
    
    pc.printf("FL RPM : %f, FR RPM : %f, RL RPM : %f, RR RPM : %f\r\n", f_motor_RPM_FL, f_motor_RPM_FR, f_motor_RPM_RL, f_motor_RPM_RR);



    //f_vel_FL_ms = CvtRPM2Vel(f_motor_RPM_FL);
    //f_vel_FR_ms = CvtRPM2Vel(f_motor_RPM_FR);
    f_vel_RL_ms = CvtRPM2Vel(f_motor_RPM_RL);
    f_vel_RR_ms = CvtRPM2Vel(f_motor_RPM_RR);
    
    pc.printf("FL vel : %f, FR vel : %f, RL vel : %f, RR vel : %f\r\n", f_vel_FL_ms, f_vel_FR_ms, f_vel_RL_ms, f_vel_RR_ms);



    f_vehicle_vel_ms = CalAvgVel(f_vel_RR_ms, f_vel_RL_ms);
    

    pc.printf("Car velocity : %f \r\n", f_vehicle_vel_ms);


    //f_steering_sensor_value 받기!
    pc.printf("Handle sensor value : %f\r\n", Handle_Sensor.read());


    f_wheel_angle_deg = CalHandlingVolt2WheelSteeringAngle(Handle_Sensor.read());

    pc.printf("wheel angle : %f\r\n",f_wheel_angle_deg);
    


    f_yawrate_input_deg = CalInputYawRate(f_vehicle_vel_ms, f_wheel_angle_deg);

    pc.printf("target yaw rate : %f \t\t", f_yawrate_input_deg);

    

    mpu.read(&IMU_gx, &IMU_gy, &IMU_gz, &IMU_ax, &IMU_ay, &IMU_az);

    f_yawrate_meas_degs = IMU_gy;               // 김치박스가 위로 세워짐!
    ////////////////////////////////////////////////////////////////// 

    f_yaw_rate_meas_filtered_degs = IMUFilter(f_yawrate_meas_degs);

    pc.printf("measured yaw rate : %f \r\n", f_yaw_rate_meas_filtered_degs);


    //f_pedal_sensor_value 받기!
    f_pedal_sensor_value = Pedal_Sensor.read();
    if (f_pedal_sensor_value <= PEDAL_MIN_VALUE)    f_pedal_sensor_value = PEDAL_MIN_VALUE;     // 안전장치
    if (f_pedal_sensor_value > PEDAL_MAX_VALUE)     f_pedal_sensor_value = PEDAL_MAX_VALUE;

    pc.printf("pedal raw value (0.0~1.0 value) : %f\r\n", f_pedal_sensor_value);

    //Modify pedal sensor vlaue range(true sensor value min~max) ----> (0.0 ~ 1.0)
    f_pedal_modified_sensor_value = ModifyPedalThrottle(f_pedal_sensor_value, PEDAL_MIN_VALUE, PEDAL_MAX_VALUE, 0.0, 1.0);
    if (f_pedal_modified_sensor_value < 0.0)    f_pedal_modified_sensor_value = 0.0;
    if (f_pedal_modified_sensor_value > 1.0)    f_pedal_modified_sensor_value = 1.0;
    
    pc.printf("modified pedal value(0.0~1.0 value) : %f\r\n", f_pedal_modified_sensor_value);
    

    // for MMS PWR
    i_PWR_percentage = (int)(f_pedal_modified_sensor_value * 100);
    pc.printf("PWR percentage : %d\r\n", i_PWR_percentage);
    


    WheelSteeringAngle2Torque(f_wheel_angle_deg, f_pedal_modified_sensor_value,
        f_wheel_torque_FL_Nm, f_wheel_torque_FR_Nm,
        f_wheel_torque_RL_Nm, f_wheel_torque_RR_Nm);


    pc.printf("feedforward torque : \r\n");
    pc.printf("FL : %f, FR : %f, RL : %f, RR : %f\r\n", f_wheel_torque_FL_Nm, f_wheel_torque_FR_Nm, f_wheel_torque_RL_Nm, f_wheel_torque_RR_Nm);
    


    PIDYawRate2Torque(f_yawrate_input_deg, f_yaw_rate_meas_filtered_degs,
        f_PID_yaw_rate2torque_FL_Nm, f_PID_yaw_rate2torque_FR_Nm,
        f_PID_yaw_rate2torque_RL_Nm, f_PID_yaw_rate2torque_RR_Nm);
        
    pc.printf("P controlled torque output \r\n");
    pc.printf("FL : %f, FR : %f, RL : %f, RR : %f\r\n", f_PID_yaw_rate2torque_FL_Nm, f_PID_yaw_rate2torque_FR_Nm, f_PID_yaw_rate2torque_RL_Nm, f_PID_yaw_rate2torque_RR_Nm);


    f_torque_FL_Nm = f_wheel_torque_FL_Nm + f_PID_yaw_rate2torque_FL_Nm;
    f_torque_FR_Nm = f_wheel_torque_FR_Nm + f_PID_yaw_rate2torque_FR_Nm;
    f_torque_RL_Nm = f_wheel_torque_RL_Nm + f_PID_yaw_rate2torque_RL_Nm;
    f_torque_RR_Nm = f_wheel_torque_RR_Nm + f_PID_yaw_rate2torque_RR_Nm;

    pc.printf("actual generating torque\r\n");
    pc.printf("FL : %f, FR : %f, RL : %f, RR : %f\r\n", f_torque_FL_Nm, f_torque_FR_Nm, f_torque_RL_Nm, f_torque_RR_Nm);


    f_output_throttle_FL = Torque2Throttle(f_torque_FL_Nm);
    f_output_throttle_FR = Torque2Throttle(f_torque_FR_Nm);
    f_output_throttle_RL = Torque2Throttle(f_torque_RL_Nm);
    f_output_throttle_RR = Torque2Throttle(f_torque_RR_Nm);

    pc.printf("feedforward output throttle signal(voltage)\r\n");
    pc.printf("FL : %f, FR : %f, RL : %f, RR : %f\r\n", f_output_throttle_FL, f_output_throttle_FR, f_output_throttle_RL, f_output_throttle_RR);



    /* 나중에 썼으면 좋겠는 것들....
    //f_motor_current 받기!
    f_motor_current_FL_A = ReadCurrentSensor(FL_Current_OUT.read());
    f_motor_current_FR_A = ReadCurrentSensor(FR_Current_OUT.read());
    f_motor_current_RL_A = ReadCurrentSensor(RL_Current_OUT.read());
    f_motor_current_RR_A = ReadCurrentSensor(RR_Current_OUT.read());
    pc.printf("FL : %f, FR : %f, RL : %f, RR : %f\r\n", f_motor_current_FL_A, f_motor_current_FR_A, f_motor_current_RL_A, f_motor_current_RR_A);
    
    
    f_measured_torque_FL_Nm = CvtCurrent2Torque(f_motor_current_FL_A);
    f_measured_torque_FR_Nm = CvtCurrent2Torque(f_motor_current_FR_A);
    f_measured_torque_RL_Nm = CvtCurrent2Torque(f_motor_current_RL_A);
    f_measured_torque_RR_Nm = CvtCurrent2Torque(f_motor_current_RR_A);
    pc.printf("measured torque \r\n");
    pc.printf("FL : %f, FR : %f, RL : %f, RR : %f\r\n", f_measured_torque_FL_Nm, f_measured_torque_FR_Nm, f_measured_torque_RL_Nm, f_measured_torque_RR_Nm);
    */



    f_PID_throttle_FL = PIDforThrottle(f_torque_FL_Nm, f_measured_torque_FL_Nm, FL);
    f_PID_throttle_FR = PIDforThrottle(f_torque_FR_Nm, f_measured_torque_FR_Nm, FR);
    f_PID_throttle_RL = PIDforThrottle(f_torque_RL_Nm, f_measured_torque_RL_Nm, RL);
    f_PID_throttle_RR = PIDforThrottle(f_torque_RR_Nm, f_measured_torque_RR_Nm, RR);

    pc.printf("feedback output throttle signal(voltage)\r\n");
    pc.printf("FL : %f, FR : %f, RL : %f, RR : %f\r\n", f_PID_throttle_FL, f_PID_throttle_FR, f_PID_throttle_RL, f_PID_throttle_RR);



    f_PWM_input_FL = SumFFandPID(f_output_throttle_FL, f_PID_throttle_FL);
    f_PWM_input_FR = SumFFandPID(f_output_throttle_FR, f_PID_throttle_FR);
    f_PWM_input_RL = SumFFandPID(f_output_throttle_RL, f_PID_throttle_RL);
    f_PWM_input_RR = SumFFandPID(f_output_throttle_RR, f_PID_throttle_RR);


    pc.printf("raw throttle signal(PWM)\r\n");
    pc.printf("FL : %f, FR : %f, RL : %f, RR : %f\r\n", f_PWM_input_FL, f_PWM_input_FR, f_PWM_input_RL, f_PWM_input_RR);



    // 0.0 ~ 1.0의 값으로 설정된 PWM신호를, 컨트롤러 특성에 맞게 map함수 구현
    trimmed_throttle_FL = map_f(f_PWM_input_FL, 0.0, 1.0, CONTROLLER_IN_MIN, CONTROLLER_IN_MAX);
    trimmed_throttle_FR = map_f(f_PWM_input_FR, 0.0, 1.0, CONTROLLER_IN_MIN, CONTROLLER_IN_MAX);
    trimmed_throttle_RL = map_f(f_PWM_input_RL, 0.0, 1.0, CONTROLLER_IN_MIN, CONTROLLER_IN_MAX);
    trimmed_throttle_RR = map_f(f_PWM_input_RR, 0.0, 1.0, CONTROLLER_IN_MIN, CONTROLLER_IN_MAX);

    pc.printf("modified PWM value : \r\n");
    pc.printf("FL : %f, FR : %f, RL : %f, RR : %f\r\n", 
                trimmed_throttle_FL, trimmed_throttle_FR, trimmed_throttle_RL, trimmed_throttle_RR);


    
    FL_Throttle_PWM = trimmed_throttle_FL * IDEAL_OPAMP_GAIN / FL_OPAMP_GAIN;            // noninverting amp outworld(ideal gain 1.515)
    FR_Throttle_PWM = trimmed_throttle_FR * IDEAL_OPAMP_GAIN / FR_OPAMP_GAIN; 
    RL_Throttle_PWM = trimmed_throttle_RL * IDEAL_OPAMP_GAIN / RL_OPAMP_GAIN; 
    RR_Throttle_PWM = trimmed_throttle_RR * IDEAL_OPAMP_GAIN / RR_OPAMP_GAIN; 
    
    
    pc.printf("actual throttle signal(voltage)\r\n");
    pc.printf("FL : %f, FR : %f, RL : %f, RR : %f\r\n", 
            FL_Throttle_PWM.read() * 3.3, FR_Throttle_PWM.read() * 3.3, RL_Throttle_PWM.read() * 3.3, RR_Throttle_PWM.read() * 3.3);

        

    /* for TVS on, off mode
    if(TVS_SWITCH==TVS_OFF) {
        f_pedal_sensor_value = Pedal_Sensor.read();
        pc.printf("pedal sensor value : %f\r\n", f_pedal_sensor_value);
        //Modify pedal sensor vlaue range(0.4~1.4) ----> (0~3.3)
        f_pedal_modified_sensor_value = ModifyPedalThrottle(f_pedal_sensor_value, PEDAL_MIN_VALUE, PEDAL_MAX_VALUE, THROTTLE_MAX, THROTTLE_MIN);
    
        FL_Throttle_PWM = f_PWM_input_FL * IDEAL_OPAMP_GAIN / FL_OPAMP_GAIN;            // OUTPUT from mbed to opamp gain modify(5V), input from controller
        FR_Throttle_PWM = f_PWM_input_FR * IDEAL_OPAMP_GAIN / FR_OPAMP_GAIN; 
        RL_Throttle_PWM = f_PWM_input_RL * IDEAL_OPAMP_GAIN / RL_OPAMP_GAIN; 
        RR_Throttle_PWM = f_PWM_input_RR * IDEAL_OPAMP_GAIN / RR_OPAMP_GAIN; 
    }
    */

    pc.printf("\r\n\n\n\n\n");


}


// for autonomous driving
void TorqueVectoringSystem::process_accel(float accel_value)        // accel value 0.0 ~ 1.0
{
    // DigitalIn  TVS_SWITCH(TVS_SWITCH_PIN);

    float trimmed_throttle_FL;      // 0.5V에서 4.1V로 출력 변경을 위한 변수
    float trimmed_throttle_FR;
    float trimmed_throttle_RL;
    float trimmed_throttle_RR;



    pc.printf("entered WHILE : \r\n");

    //f_motor_RPM_FL = FL_Hall_A.getRPM();
    //f_motor_RPM_FR = FR_Hall_A.getRPM();
    f_motor_RPM_RL = RL_Hall_A.getRPM();
    f_motor_RPM_RR = RR_Hall_A.getRPM();
    
    pc.printf("FL RPM : %f, FR RPM : %f, RL RPM : %f, RR RPM : %f\r\n", f_motor_RPM_FL, f_motor_RPM_FR, f_motor_RPM_RL, f_motor_RPM_RR);



    f_vel_FL_ms = CvtRPM2Vel(f_motor_RPM_FL);
    f_vel_FR_ms = CvtRPM2Vel(f_motor_RPM_FR);
    f_vel_RL_ms = CvtRPM2Vel(f_motor_RPM_RL);
    f_vel_RR_ms = CvtRPM2Vel(f_motor_RPM_RR);
    
    pc.printf("FL vel : %f, FR vel : %f, RL vel : %f, RR vel : %f\r\n", f_vel_FL_ms, f_vel_FR_ms, f_vel_RL_ms, f_vel_RR_ms);



    f_vehicle_vel_ms = CalAvgVel(f_vel_RR_ms, f_vel_RL_ms);

    pc.printf("Car velocity : %f \r\n", f_vehicle_vel_ms);


    //f_steering_sensor_value 받기!
    pc.printf("Handle sensor value : %f\r\n", Handle_Sensor.read());


    f_wheel_angle_deg = CalHandlingVolt2WheelSteeringAngle(Handle_Sensor.read());

    pc.printf("wheel angle : %f\r\n",f_wheel_angle_deg);
    


    f_yawrate_input_deg = CalInputYawRate(f_vehicle_vel_ms, f_wheel_angle_deg);

    pc.printf("target yaw rate : %f \t\t", f_yawrate_input_deg);

    

    mpu.read(&IMU_gx, &IMU_gy, &IMU_gz, &IMU_ax, &IMU_ay, &IMU_az);

    f_yawrate_meas_degs = IMU_gy;               // 김치박스가 위로 세워짐!
    ////////////////////////////////////////////////////////////////// 

    f_yaw_rate_meas_filtered_degs = IMUFilter(f_yawrate_meas_degs);

    pc.printf("measured yaw rate : %f \r\n", f_yaw_rate_meas_filtered_degs);


    /*
    // 수동 주행 페달 신호 가공 제외
    //f_pedal_sensor_value 받기!
    f_pedal_sensor_value = Pedal_Sensor.read();
    if (f_pedal_sensor_value <= PEDAL_MIN_VALUE)    f_pedal_sensor_value = PEDAL_MIN_VALUE;     // 안전장치
    if (f_pedal_sensor_value > PEDAL_MAX_VALUE)     f_pedal_sensor_value = PEDAL_MAX_VALUE;

    pc.printf("pedal raw value (0.0~1.0 value) : %f\r\n", f_pedal_sensor_value);

    //Modify pedal sensor vlaue range(true sensor value min~max) ----> (0.0 ~ 1.0)
    f_pedal_modified_sensor_value = ModifyPedalThrottle(f_pedal_sensor_value, PEDAL_MIN_VALUE, PEDAL_MAX_VALUE, 0.0, 1.0);
    if (f_pedal_modified_sensor_value < 0.0)    f_pedal_modified_sensor_value = 0.0;
    if (f_pedal_modified_sensor_value > 1.0)    f_pedal_modified_sensor_value = 1.0;
    
    pc.printf("modified pedal value(0.0~1.0 value) : %f\r\n", f_pedal_modified_sensor_value);
    */


    // publish한 accel값을 구동에 이용
    f_pedal_modified_sensor_value = accel_value;
    if (f_pedal_modified_sensor_value < 0.0)    f_pedal_modified_sensor_value = 0.0;

    // for MMS PWR
    i_PWR_percentage = (int)(f_pedal_modified_sensor_value * 100);
    pc.printf("PWR percentage : %d\r\n", i_PWR_percentage);
    


    WheelSteeringAngle2Torque(f_wheel_angle_deg, f_pedal_modified_sensor_value,
        f_wheel_torque_FL_Nm, f_wheel_torque_FR_Nm,
        f_wheel_torque_RL_Nm, f_wheel_torque_RR_Nm);


    pc.printf("feedforward torque : \r\n");
    pc.printf("FL : %f, FR : %f, RL : %f, RR : %f\r\n", f_wheel_torque_FL_Nm, f_wheel_torque_FR_Nm, f_wheel_torque_RL_Nm, f_wheel_torque_RR_Nm);
    


    PIDYawRate2Torque(f_yawrate_input_deg, f_yaw_rate_meas_filtered_degs,
        f_PID_yaw_rate2torque_FL_Nm, f_PID_yaw_rate2torque_FR_Nm,
        f_PID_yaw_rate2torque_RL_Nm, f_PID_yaw_rate2torque_RR_Nm);
        
    pc.printf("P controlled torque output \r\n");
    pc.printf("FL : %f, FR : %f, RL : %f, RR : %f\r\n", f_PID_yaw_rate2torque_FL_Nm, f_PID_yaw_rate2torque_FR_Nm, f_PID_yaw_rate2torque_RL_Nm, f_PID_yaw_rate2torque_RR_Nm);


    f_torque_FL_Nm = f_wheel_torque_FL_Nm + f_PID_yaw_rate2torque_FL_Nm;
    f_torque_FR_Nm = f_wheel_torque_FR_Nm + f_PID_yaw_rate2torque_FR_Nm;
    f_torque_RL_Nm = f_wheel_torque_RL_Nm + f_PID_yaw_rate2torque_RL_Nm;
    f_torque_RR_Nm = f_wheel_torque_RR_Nm + f_PID_yaw_rate2torque_RR_Nm;

    pc.printf("actual generating torque\r\n");
    pc.printf("FL : %f, FR : %f, RL : %f, RR : %f\r\n", f_torque_FL_Nm, f_torque_FR_Nm, f_torque_RL_Nm, f_torque_RR_Nm);


    f_output_throttle_FL = Torque2Throttle(f_torque_FL_Nm);
    f_output_throttle_FR = Torque2Throttle(f_torque_FR_Nm);
    f_output_throttle_RL = Torque2Throttle(f_torque_RL_Nm);
    f_output_throttle_RR = Torque2Throttle(f_torque_RR_Nm);

    pc.printf("feedforward output throttle signal(voltage)\r\n");
    pc.printf("FL : %f, FR : %f, RL : %f, RR : %f\r\n", f_output_throttle_FL, f_output_throttle_FR, f_output_throttle_RL, f_output_throttle_RR);



    /* 나중에 썼으면 좋겠는 것들....
    //f_motor_current 받기!
    f_motor_current_FL_A = ReadCurrentSensor(FL_Current_OUT.read());
    f_motor_current_FR_A = ReadCurrentSensor(FR_Current_OUT.read());
    f_motor_current_RL_A = ReadCurrentSensor(RL_Current_OUT.read());
    f_motor_current_RR_A = ReadCurrentSensor(RR_Current_OUT.read());
    pc.printf("FL : %f, FR : %f, RL : %f, RR : %f\r\n", f_motor_current_FL_A, f_motor_current_FR_A, f_motor_current_RL_A, f_motor_current_RR_A);
    
    
    f_measured_torque_FL_Nm = CvtCurrent2Torque(f_motor_current_FL_A);
    f_measured_torque_FR_Nm = CvtCurrent2Torque(f_motor_current_FR_A);
    f_measured_torque_RL_Nm = CvtCurrent2Torque(f_motor_current_RL_A);
    f_measured_torque_RR_Nm = CvtCurrent2Torque(f_motor_current_RR_A);
    pc.printf("measured torque \r\n");
    pc.printf("FL : %f, FR : %f, RL : %f, RR : %f\r\n", f_measured_torque_FL_Nm, f_measured_torque_FR_Nm, f_measured_torque_RL_Nm, f_measured_torque_RR_Nm);
    */



    f_PID_throttle_FL = PIDforThrottle(f_torque_FL_Nm, f_measured_torque_FL_Nm, FL);
    f_PID_throttle_FR = PIDforThrottle(f_torque_FR_Nm, f_measured_torque_FR_Nm, FR);
    f_PID_throttle_RL = PIDforThrottle(f_torque_RL_Nm, f_measured_torque_RL_Nm, RL);
    f_PID_throttle_RR = PIDforThrottle(f_torque_RR_Nm, f_measured_torque_RR_Nm, RR);

    pc.printf("feedback output throttle signal(voltage)\r\n");
    pc.printf("FL : %f, FR : %f, RL : %f, RR : %f\r\n", f_PID_throttle_FL, f_PID_throttle_FR, f_PID_throttle_RL, f_PID_throttle_RR);



    f_PWM_input_FL = SumFFandPID(f_output_throttle_FL, f_PID_throttle_FL);
    f_PWM_input_FR = SumFFandPID(f_output_throttle_FR, f_PID_throttle_FR);
    f_PWM_input_RL = SumFFandPID(f_output_throttle_RL, f_PID_throttle_RL);
    f_PWM_input_RR = SumFFandPID(f_output_throttle_RR, f_PID_throttle_RR);


    pc.printf("raw throttle signal(PWM)\r\n");
    pc.printf("FL : %f, FR : %f, RL : %f, RR : %f\r\n", f_PWM_input_FL, f_PWM_input_FR, f_PWM_input_RL, f_PWM_input_RR);



    // 0.0 ~ 1.0의 값으로 설정된 PWM신호를, 컨트롤러 특성에 맞게 map함수 구현
    trimmed_throttle_FL = map_f(f_PWM_input_FL, 0.0, 1.0, CONTROLLER_IN_MIN, CONTROLLER_IN_MAX);
    trimmed_throttle_FR = map_f(f_PWM_input_FR, 0.0, 1.0, CONTROLLER_IN_MIN, CONTROLLER_IN_MAX);
    trimmed_throttle_RL = map_f(f_PWM_input_RL, 0.0, 1.0, CONTROLLER_IN_MIN, CONTROLLER_IN_MAX);
    trimmed_throttle_RR = map_f(f_PWM_input_RR, 0.0, 1.0, CONTROLLER_IN_MIN, CONTROLLER_IN_MAX);

    pc.printf("modified PWM value : \r\n");
    pc.printf("FL : %f, FR : %f, RL : %f, RR : %f\r\n", 
                trimmed_throttle_FL, trimmed_throttle_FR, trimmed_throttle_RL, trimmed_throttle_RR);


    
    FL_Throttle_PWM = trimmed_throttle_FL * IDEAL_OPAMP_GAIN / FL_OPAMP_GAIN;            // noninverting amp outworld(ideal gain 1.515)
    FR_Throttle_PWM = trimmed_throttle_FR * IDEAL_OPAMP_GAIN / FR_OPAMP_GAIN; 
    RL_Throttle_PWM = trimmed_throttle_RL * IDEAL_OPAMP_GAIN / RL_OPAMP_GAIN; 
    RR_Throttle_PWM = trimmed_throttle_RR * IDEAL_OPAMP_GAIN / RR_OPAMP_GAIN; 
    
    
    pc.printf("actual throttle signal(voltage)\r\n");
    pc.printf("FL : %f, FR : %f, RL : %f, RR : %f\r\n", 
            FL_Throttle_PWM.read() * 3.3, FR_Throttle_PWM.read() * 3.3, RL_Throttle_PWM.read() * 3.3, RR_Throttle_PWM.read() * 3.3);

        

    /* for TVS on, off mode
    if(TVS_SWITCH==TVS_OFF) {
        f_pedal_sensor_value = Pedal_Sensor.read();
        pc.printf("pedal sensor value : %f\r\n", f_pedal_sensor_value);
        //Modify pedal sensor vlaue range(0.4~1.4) ----> (0~3.3)
        f_pedal_modified_sensor_value = ModifyPedalThrottle(f_pedal_sensor_value, PEDAL_MIN_VALUE, PEDAL_MAX_VALUE, THROTTLE_MAX, THROTTLE_MIN);
    
        FL_Throttle_PWM = f_PWM_input_FL * IDEAL_OPAMP_GAIN / FL_OPAMP_GAIN;            // OUTPUT from mbed to opamp gain modify(5V), input from controller
        FR_Throttle_PWM = f_PWM_input_FR * IDEAL_OPAMP_GAIN / FR_OPAMP_GAIN; 
        RL_Throttle_PWM = f_PWM_input_RL * IDEAL_OPAMP_GAIN / RL_OPAMP_GAIN; 
        RR_Throttle_PWM = f_PWM_input_RR * IDEAL_OPAMP_GAIN / RR_OPAMP_GAIN; 
    }
    */

    pc.printf("\r\n\n\n\n\n");
}