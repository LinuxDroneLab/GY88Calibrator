//============================================================================
// Name        : GY88Calibrator.cpp
// Author      : Andrea Lambruschini
// Version     :
// Copyright   : Apache 2.0
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include <drivers/MPU6050.h>

using namespace std;

int16_t accel[3] = { 0 };
int16_t gyro[3] = { 0 };
int16_t temperature = 0;
int32_t sumGyro[3] = { 0 };
int16_t minGyro[3] = { 32767, 32767, 32767 };
int16_t maxGyro[3] = { 0 };
int16_t offsetGyro[3] = { 0 };
MPU6050 mpu;

void calibrateGyro()
{
    for (uint16_t counter = 0; counter < 4096; counter++)
    {
        while (!mpu.getIntDataReadyStatus())
        {
        };
        mpu.getMotion6(accel, accel + 1, accel + 2, gyro, gyro + 1, gyro + 2);
        for (int i = 0; i < 3; i++)
        {
            sumGyro[i] += gyro[i];
            minGyro[i] = min(minGyro[i], gyro[i]);
            maxGyro[i] = max(maxGyro[i], gyro[i]);
        }
    }
    for (int i = 0; i < 3; i++)
    {
        offsetGyro[i] = (sumGyro[i] >> 12);
    }
    cout << "S(" << sumGyro[0] << "," << sumGyro[1] << "," << sumGyro[2]
         << "), " << "M(" << offsetGyro[0] << "," << offsetGyro[1] << ","
         << offsetGyro[2] << "), " << "Min(" << minGyro[0] << "," << minGyro[1]
         << "," << minGyro[2] << "), " << "Max(" << maxGyro[0] << ","
         << maxGyro[1] << "," << maxGyro[2] << "), " << endl;
}

int main()
{
    cout << "GY88 Calibrator Started" << endl; // prints pru-i2c-arm started
    mpu.initialize();
    if (mpu.testConnection())
    {
        calibrateGyro();
        while(1) {
            if (mpu.getIntDataReadyStatus())
            {
                mpu.getMotion6(accel, accel + 1, accel + 2, gyro, gyro + 1,
                               gyro + 2);
                temperature = mpu.getTemperature();
                for (int i = 0; i < 3; i++)
                {
                    gyro[i] -= offsetGyro[i];
                    if (gyro[i] <= 3 && gyro[i] >= -3)
                    {
                        gyro[i] = 0;
                    }
                }
                cout << "A(" << (accel[0]>> 3) << "," << (accel[1]>> 3) << ","
                     << (accel[2]>> 3) << "); " << "G(" << gyro[0] << ","
                     << gyro[1] << "," << gyro[2] << "); " << "T("
                     << float(temperature)/340.0f + 36.53f << ")" << endl;
            }
        }
    }
    else
    {
        cout << "Sorry!! MPU6050 not connected" << endl;
    }
    return 0;
}
