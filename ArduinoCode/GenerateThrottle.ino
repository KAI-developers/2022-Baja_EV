// code for Arduino NANO, read mapping analog data 0~3.3V to 0~5V, 4 channel

int FL_READ_PIN = A1;
int FR_READ_PIN = A2;
int RL_READ_PIN = A3;
int RR_READ_PIN = A4;

int FL_PWM_PIN = 3;
int FR_PWM_PIN = 9;
int RL_PWM_PIN = 10;
int RR_PWM_PIN = 11;


//float FL_V = 0;
//float FR_V = 0;
//float RL_V = 0;
//float RR_V = 0;



void setup() {
    TCCR2B = TCCR2B & B11111000 | B00000001; // for PWM frequency of 31372.55 Hz, D3 & D11
    TCCR1B = TCCR1B & B11111000 | B00000001; // for PWM frequency of 31372.55 Hz, D9 & D10


    //  Serial.begin(115200);
    pinMode(FL_READ_PIN, INPUT);
    pinMode(FR_READ_PIN, INPUT);
    pinMode(RL_READ_PIN, INPUT);
    pinMode(RR_READ_PIN, INPUT);

    pinMode(FL_PWM_PIN, OUTPUT);
    pinMode(FR_PWM_PIN, OUTPUT);
    pinMode(RL_PWM_PIN, OUTPUT);
    pinMode(RR_PWM_PIN, OUTPUT);
}



void loop() {

  
    float f_PWM_FL = 0;
    float f_PWM_FR = 0;
    float f_PWM_RL = 0;
    float f_PWM_RR = 0;

    int i_PWM_FL = 0;
    int i_PWM_FR = 0;
    int i_PWM_RL = 0;
    int i_PWM_RR = 0;



    /*
    FL_V = (float)analogRead(FL_READ_PIN) * 5.0 / 1023.0;
    FR_V = (float)analogRead(FR_READ_PIN) * 5.0 / 1023.0;
    RL_V = (float)analogRead(RL_READ_PIN) * 5.0 / 1023.0;
    RR_V = (float)analogRead(RR_READ_PIN) * 5.0 / 1023.0;


    Serial.print(FL_V);
    Serial.print(',');

    Serial.print(FR_V);
    Serial.print(',');

    Serial.print(RL_V);
    Serial.print(',');

    Serial.println(RR_V);
    */


    f_PWM_FL = (float)analogRead(FL_READ_PIN) * 255 / 1023;
    f_PWM_FR = (float)analogRead(FR_READ_PIN) * 255 / 1023;
    f_PWM_RL = (float)analogRead(RL_READ_PIN) * 255 / 1023;
    f_PWM_RR = (float)analogRead(RR_READ_PIN) * 255 / 1023;

    // int형으로 바꿔주면서, 3.3V -> 5V 출력에 맞게 곱셈 함
    i_PWM_FL = (int)f_PWM_FL * 5 / 3.3;
    i_PWM_FR = (int)f_PWM_FR * 5 / 3.3;
    i_PWM_RL = (int)f_PWM_RL * 5 / 3.3;
    i_PWM_RR = (int)f_PWM_RR * 5 / 3.3;

    if(i_PWM_FL >= 209)   i_PWM_FL = 209;
    if(i_PWM_FR >= 209)   i_PWM_FR = 209;

    if(i_PWM_FL <= 25)    i_PWM_FL = 25;
    if(i_PWM_FR <= 25)    i_PWM_FR = 25;


    //  Serial.print(i_PWM_FL);
    //  Serial.print(',');
    //  
    //  Serial.print(i_PWM_FR);
    //  Serial.println(',');

    //  Serial.print(i_PWM_RL);
    //  Serial.print(',');
    //  
    //  Serial.println(i_PWM_RR);


    analogWrite(FL_PWM_PIN, i_PWM_FL);
    analogWrite(FR_PWM_PIN, i_PWM_FR);
    analogWrite(RL_PWM_PIN, i_PWM_RL);
    analogWrite(RR_PWM_PIN, i_PWM_RR);

}