#include "mbed.h"
#include "Adafruit_ADS1015.h"

#include "MPU6050.h"




int main(){

    PinName MPU_SDA = p28;
    PinName MPU_SCL = p27;

    PinName ADS_SDA = p9;
    PinName ADS_SCL = p10;

    Serial pc(USBTX, USBRX, 115200);

    I2C I2C_ADS(ADS_SDA, ADS_SCL);
    Adafruit_ADS1115 ads(&I2C_ADS);
    MPU6050 mpu(MPU_SDA, MPU_SCL);                  // (sda, scl)

    ads.setGain(GAIN_TWOTHIRDS); // set range to +/-0.256V-->6.144V

    int reading;
    float IMU_gx, IMU_gy, IMU_gz, IMU_ax, IMU_ay, IMU_az;

    mpu.start();

    while (1) {

        mpu.read(&IMU_gx, &IMU_gy, &IMU_gz, &IMU_ax, &IMU_ay, &IMU_az);
        pc.printf("yaw rate : %f\t\t", IMU_gx);

        reading = ads.readADC_SingleEnded(0); // read channel 0
        //pc.printf("reading: %d\r\n", reading); // print reading
        pc.printf("voltage: %f\r\n",reading*0.1875/1000);
        //wait(2); // loop 2 sek
        
        //reading = ads.readADC_Differential_2_3(); // differential channel 2-3
        //pc.printf("diff a2, a3: %d\r\n", reading); // print diff
    }
}