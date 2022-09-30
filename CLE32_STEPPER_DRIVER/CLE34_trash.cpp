#include "CLE34.h"


Serial pc_header(USBTX, USBRX, 115200);


CLE34::CLE34(PinName PIN_PUL_PLUS, PinName PIN_DIR_PLUS, PinName PIN_ENA_PLUS, int pulse_per_rev_)
    :   PUL_PLUS(PIN_PUL_PLUS),
        DIR_PLUS(PIN_DIR_PLUS),
        ENA_PLUS(PIN_ENA_PLUS),
        pulse_per_rev(pulse_per_rev_)        
{
    count_100us = 0;
    pulse_count = 0;


    degree_per_pulse = degreePerPulse(pulse_per_rev);

    time.start();
    ticker_1us.attach_us(callback (this, &CLE34::count100us), 100);

    pc_header.printf("CLE34 object initialized \r\n");
}

/*
    사용자가 CLE34 객체를 초기화면서 Pulse/rev 입력값에 따라,
    한 pulse당 몇 도 돌아가는지 계산함
*/
float CLE34::degreePerPulse(float pulse_per_rev)
{
    return 360 / pulse_per_rev;
}

void CLE34::count100us()
{
    count_100us++;
    if (count_100us == 4000000000)     // 4천*100초까지 저장..?
        count_100us = 0;
}


/*
    dir CW or CCW
*/
void CLE34::setDir(int dir)
{
    DIR_PLUS = dir;
}

/*
    state가
    MOTOR_OFF -> 모터 잠김
    MOTOR_ON -> 모터 풀림
*/
void CLE34::enableOn(int state)
{
    ENA_PLUS = state;
}


/*
    Pulse/rev 4000일 때로 설정 가정한 함수
    Pulse/rev 수정되었으면, 별도의 함수 작성 요망
*/
void CLE34::turnAngle(float angle_deg, int dir, float speed_radps)
{
    float f_num_of_pulse;
    int i_num_of_pulse;
    float half_pulse_time_us;

    setDir(dir);
    pc_header.printf("dir set \r\n");

    f_num_of_pulse = angle_deg / degree_per_pulse;  // pulse/rev 4000일 때 한 펄스 당 0.09도 돌아감, 반펄스 시간 계산용 float
    i_num_of_pulse = (int)f_num_of_pulse;
    pc_header.printf("number of pulse : %d \r\n", i_num_of_pulse);

    half_pulse_time_us = 1000000 * (1 / (speed_radps)) / (i_num_of_pulse * 2);
    pc_header.printf("half pulse time : %f us \r\n", half_pulse_time_us);

    PUL_PLUS = 0;                                   // 일단 low로 초기화
    count_100us = 0;
    pulse_count = 0;
    pc_header.printf("variables initialized \r\n");

    while (pulse_count != i_num_of_pulse)           // 발생 펄스 갯수 만족할 때 까지
    {
        if (count_100us < half_pulse_time_us * 100)          // 반펄스시간 지날 때 까지
        {
            PUL_PLUS = !PUL_PLUS;
            count_100us = 0;
            pulse_count++;
            pc_header.printf("loop spined once \r\n");
        }
    }
    PUL_PLUS = 0;
}