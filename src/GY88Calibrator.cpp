//============================================================================
// Name        : GY88Calibrator.cpp
// Author      : Andrea Lambruschini
// Version     :
// Copyright   : Apache 2.0
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include <drivers/MPU6050.h>
#include <cmath>

using namespace std;

int16_t accel[3] = { 0 };
int32_t sumAccel[3] = { 0 };
int16_t minAccel[3] = { 32767, 32767, 32767 };
int16_t maxAccel[3] = { -32767, -32767, -32767 };
int16_t meanAccel[3] = { 0 };
float axisFactorAccel[3] = { 1.0f, 1.0f, 1.0f };
float k[3] = {0.0f};
int16_t prevAccel[3] = {0};

int16_t gyro[3] = { 0 };
int16_t temperature = 0;
int32_t sumGyro[3] = { 0 };
int16_t minGyro[3] = { 32767, 32767, 32767 };
int16_t maxGyro[3] = { -32767, -32767, -32767 };
int16_t offsetGyro[3] = { 0 };

float rollDegAcc = 0.0f;
float pitchDegAcc = 0.0f;
float rollDeg = 0.0f;
float pitchDeg = 0.0f;

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
void calibrateAccel() {

    // determino fattore k per smorzare le oscillazioni
    for (uint16_t counter = 0; counter < 4096; counter++)
    {
        while (!mpu.getIntDataReadyStatus())
        {
        };
        mpu.getMotion6(accel, accel + 1, accel + 2, gyro, gyro + 1, gyro + 2);
        for (int i = 0; i < 3; i++)
        {
            sumAccel[i] += accel[i];
            minAccel[i] = min(minAccel[i], accel[i]);
            maxAccel[i] = max(maxAccel[i], accel[i]);
        }

    }
    for (int i = 0; i < 3; i++)
    {
        meanAccel[i] = (sumAccel[i] >> 12);
    }
    cout <<
            "Accel: Mean(" << meanAccel[0] <<
            ", " << meanAccel[1] <<
            ", " << meanAccel[2] <<
            "), " <<
            "Min(" << minAccel[0] <<
            ", " << minAccel[1] <<
            ", " << minAccel[2] <<
            "), " <<
            "Max(" << maxAccel[0] <<
            ", " << maxAccel[1] <<
            ", " << maxAccel[2] <<
            ")" << endl;

    for(int i = 0; i < 3; i++) {
        float ampiezza = (float(maxAccel[i]) - float(minAccel[i]));
        if(ampiezza != 0) {
            k[i] = 1.0f/ampiezza;
        } else {
            k[i] = 1.0f;
        }
        maxAccel[i] = -32767;
        minAccel[i] = 32767;
        prevAccel[i] = accel[i];
    }
    cout << "K=(" << k[0] << ", " << k[1] << ". " << k[2] << ")" << endl;
    bool done = false;
    bool doneMin[3] = {false};
    bool doneMax[3] = {false};

    // determino ampiezza assi X,Y,Z sui campioni smorzati con il fattore k
    while (!done)
    {
        while (!mpu.getIntDataReadyStatus())
        {
        };
        mpu.getMotion6(accel, accel + 1, accel + 2, gyro, gyro + 1, gyro + 2);
        for(int i = 0; i < 3; i++) {
            float accelTmp = float(prevAccel[i])*(1.0f-k[i]) + float(accel[i]) * k[i];
            accel[i]  = int16_t(accelTmp);
            prevAccel[i] = accel[i];
        }
        if(abs(accel[0]) < 100 && abs(accel[1]) < 100) {
            if(accel[2] < 0) {
                minAccel[2] = min(minAccel[2], accel[2]);
                doneMin[2] = true;
            } else {
                maxAccel[2] = max(maxAccel[2], accel[2]);
                doneMax[2] = true;
            }
        } else if(abs(accel[0]) < 100 && abs(accel[2]) < 100) {
            if(accel[1] < 0) {
                minAccel[1] = min(minAccel[1], accel[1]);
                doneMin[1] = true;
            } else {
                maxAccel[1] = max(maxAccel[1], accel[1]);
                doneMax[1] = true;
            }
        } if(abs(accel[1]) < 100 && abs(accel[2]) < 100) {
            if(accel[0] < 0) {
                minAccel[0] = min(minAccel[0], accel[0]);
                doneMin[0] = true;
            } else {
                maxAccel[0] = max(maxAccel[0], accel[0]);
                doneMax[0] = true;
            }
        }

        done = true;
        for(int i = 0; i < 3; i++) {
            done &=doneMin[i] & doneMax[i];
        }
        cout << "XYZ(" << accel[0] << ", " <<  accel[1] << ", " << accel[2] << "), " <<
                "X(" << minAccel[0] << ", " << maxAccel[0] << "), " <<
                "Y(" << minAccel[1] << ", " << maxAccel[1] << "), " <<
                "Z(" << minAccel[2] << ", " << maxAccel[2] << ")" << endl;
    }

    for(int i = 0; i < 3; i++) {
        axisFactorAccel[i] = 8192.0f * 2.0f/(maxAccel[i] - minAccel[i]);
    }
    cout << "AxisFactors(" << axisFactorAccel[0] << ", "  << axisFactorAccel[1] << ", "  << axisFactorAccel[2] << ")" << endl;
}


void calcRollPitchAccel() {
    float accX = float(accel[0])/8192.0f;
    float accY = float(accel[1])/8192.0f;
    float accZ = float(accel[2])/8192.0f;

    float accModule = sqrt(accX*accX + accY*accY + accZ*accZ);
    if(abs(accX) < accModule) {
        rollDegAcc = asin((float)accY/accModule)* 57.296;
    }
    if(abs(accY) < accModule) {
        pitchDegAcc = asin((float)accX/accModule)* 57.296;
    }
}

/*
 * from http://www.brokking.net
 */
void calcRollPitch() {
    //Gyro angle calculations
    //0.0000611 = 1 / (250Hz / 65.5)
    pitchDeg -= float(gyro[1]) * 0.0000611f;
    rollDeg += float(gyro[0]) * 0.0000611f;

    // cout << "GyroVal: " << gyro[0] << " - " << gyro[1] << " - @" << float(gyro[1]) * 0.0000611f << endl;
    pitchDeg -= rollDeg * sin(gyro[2] * 0.000001066); // 0.0000611 * (pi)/360
    rollDeg += pitchDeg * sin(gyro[2] * 0.000001066);

    pitchDeg = pitchDeg * 0.9991 + pitchDegAcc * 0.0009;
    rollDeg = rollDeg * 0.9991 + rollDegAcc * 0.0009;

}

void calibrateAll() {
    calibrateGyro();
    calibrateAccel();
    cout << "Accel AxisFactors(" << axisFactorAccel[0] << ", "  << axisFactorAccel[1] << ", "  << axisFactorAccel[2] << ")" << endl;
    cout << "Accel K=(" << k[0] << ", " << k[1] << ". " << k[2] << ")" << endl;
    cout << "Gyro Offsets(" << offsetGyro[0] << ", " << offsetGyro[1] << ", "<< offsetGyro[2] << ")"  << endl;
}

int main()
{
    cout << "GY88 Calibrator Started" << endl; // prints pru-i2c-arm started
    mpu.initialize();
    if (mpu.testConnection())
    {
        /*
         * Accel AxisFactors(1.00245, 1.00968, 1.00337)
         * Accel K=(0.00819672, 0.0070922. 0.00454545)
         * Gyro Offsets(-123, -7, 18)
         *
         */
        axisFactorAccel[0] = 1.00245f;
        axisFactorAccel[1] = 1.00968f;
        axisFactorAccel[2] = 1.00337f;
        k[0] = 0.00819672f;
        k[1] = 0.0070922f;
        k[2] = 0.00454545f;
        offsetGyro[0] = -123;
        offsetGyro[1] = -7;
        offsetGyro[2] = 18;

        // TODO: richiedere in input le calibrazioni da eseguire
        //calibrateAll();
        while(!mpu.getIntDataReadyStatus());
        mpu.getMotion6(accel, accel + 1, accel + 2, gyro, gyro + 1,
                       gyro + 2);

        for(int i = 0; i < 3; i++) {
            prevAccel[i] = float(accel[i])*axisFactorAccel[i];
        }
        // calc initial roll & pitch (fro accel)
        calcRollPitchAccel();
        pitchDeg = pitchDegAcc;
        rollDeg = rollDegAcc;

        uint8_t counter = 0;
        while(1) {
            if (mpu.getIntDataReadyStatus())
            {
                mpu.getMotion6(accel, accel + 1, accel + 2, gyro, gyro + 1,
                               gyro + 2);
                temperature = mpu.getTemperature();
                // aggiusto gyro e accel
                for (int i = 0; i < 3; i++)
                {
                    gyro[i] -= offsetGyro[i];
                    if (gyro[i] <= 3 && gyro[i] >= -3)
                    {
                        gyro[i] = 0;
                    }
                    float accelTmp = float(prevAccel[i])*(1.0f-k[i]) + float(accel[i])*axisFactorAccel[i] * k[i];
                    accel[i]  = int16_t(accelTmp);
                    prevAccel[i] = accel[i];
                }
                // calcolo roll e pitch
                // per lo yaw vedo dopo ...
                calcRollPitchAccel();
                calcRollPitch();

                if(counter == 0) {
                    cout << "RPAcc(" << rollDegAcc << ", " << pitchDegAcc << "), RP(" << rollDeg << ", " << pitchDeg << "), " << " Temperature: " << float(temperature)/340.0f + 36.53f << endl;
                }
                counter++;
                counter %= 250;
            }
        }
    }
    else
    {
        cout << "Sorry!! MPU6050 not connected" << endl;
    }
    return 0;
}
