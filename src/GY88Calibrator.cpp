//============================================================================
// Name        : GY88Calibrator.cpp
// Author      : Andrea Lambruschini
// Version     :
// Copyright   : Apache 2.0
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <stdlib.h>
#include <iostream>
#include <drivers/MPU6050.h>
#include <cmath>
#include <string.h>
#include <SerialPort.h>
#include <SerialStream.h>
#include <drivers/TinyGPS++.h>
#include <drivers/MS5611.h>
#include <KalmanFilter.h>
using namespace std;

int16_t accel[3] = { 0 };
int32_t sumAccel[3] = { 0 };
int16_t minAccel[3] = { 32767, 32767, 32767 };
int16_t maxAccel[3] = { -32767, -32767, -32767 };
int16_t meanAccel[3] = { 0 };
float axisFactorAccel[3] = { 1.0f, 1.0f, 1.0f };
float axisOffsetAccel[3] = { 0.0f, 0.0f, 0.0f };
float k[3] = { 0.0f };
int16_t prevAccel[3] = { 0 };
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
float yawDeg = 0.0f;
Quaternion q;

MPU6050 mpu;

void calibrateGyro()
{
    if (mpu.testConnection())
    {

        for (uint16_t counter = 0; counter < 4096; counter++)
        {
            while (!mpu.getIntDataReadyStatus())
            {
            };
            mpu.getMotion6(accel, accel + 1, accel + 2, gyro, gyro + 1,
                           gyro + 2);
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
             << offsetGyro[2] << "), " << "Min(" << minGyro[0] << ","
             << minGyro[1] << "," << minGyro[2] << "), " << "Max(" << maxGyro[0]
             << "," << maxGyro[1] << "," << maxGyro[2] << "), " << endl;
    }
}
void calibrateAccel()
{

    if (mpu.testConnection())
    {
        // determino fattore k per smorzare le oscillazioni
        // il fattore K lo definisco come 1/(max-min) ... sembra plausibile
        // TODO: provare con un filtro di Kalman determinando P,R e K ottimi
        for (uint16_t counter = 0; counter < 4096; counter++)
        {
            while (!mpu.getIntDataReadyStatus())
            {
            };
            mpu.getMotion6(accel, accel + 1, accel + 2, gyro, gyro + 1,
                           gyro + 2);
            for (int i = 0; i < 3; i++)
            {
                sumAccel[i] += accel[i];
                minAccel[i] = min(minAccel[i], accel[i]);
                maxAccel[i] = max(maxAccel[i], accel[i]);
            }

        }
        // calcolo la media solo per info ...
        for (int i = 0; i < 3; i++)
        {
            meanAccel[i] = (sumAccel[i] >> 12);
        }
        cout << "Accel: Mean(" << meanAccel[0] << ", " << meanAccel[1] << ", "
             << meanAccel[2] << "), " << "Min(" << minAccel[0] << ", "
             << minAccel[1] << ", " << minAccel[2] << "), " << "Max("
             << maxAccel[0] << ", " << maxAccel[1] << ", " << maxAccel[2] << ")"
             << endl;

        for (int i = 0; i < 3; i++)
        {
            float ampiezza = (float(maxAccel[i]) - float(minAccel[i]));
            if (ampiezza != 0)
            {
                k[i] = 1.0f / ampiezza;
            }
            else
            {
                k[i] = 1.0f;
            }
            maxAccel[i] = -32767;
            minAccel[i] = 32767;
            prevAccel[i] = accel[i];
        }
        cout << "K=(" << k[0] << ", " << k[1] << ". " << k[2] << ")" << endl;
        bool done = false;
        bool doneMin[3] = { false };
        bool doneMax[3] = { false };

        // determino ampiezza assi X,Y,Z sui campioni smorzati con il fattore k
        while (!done)
        {
            while (!mpu.getIntDataReadyStatus())
            {
            };
            mpu.getMotion6(accel, accel + 1, accel + 2, gyro, gyro + 1,
                           gyro + 2);
            for (int i = 0; i < 3; i++)
            {
                float accelTmp = float(prevAccel[i]) * (1.0f - k[i])
                        + float(accel[i]) * k[i];
                accel[i] = int16_t(accelTmp);
                prevAccel[i] = accel[i];
            }
            if (abs(accel[0]) < 100 && abs(accel[1]) < 100)
            {
                if (accel[2] < 0)
                {
                    minAccel[2] = min(minAccel[2], accel[2]);
                    doneMin[2] = true;
                }
                else
                {
                    maxAccel[2] = max(maxAccel[2], accel[2]);
                    doneMax[2] = true;
                }
            }
            else if (abs(accel[0]) < 100 && abs(accel[2]) < 100)
            {
                if (accel[1] < 0)
                {
                    minAccel[1] = min(minAccel[1], accel[1]);
                    doneMin[1] = true;
                }
                else
                {
                    maxAccel[1] = max(maxAccel[1], accel[1]);
                    doneMax[1] = true;
                }
            }
            if (abs(accel[1]) < 100 && abs(accel[2]) < 100)
            {
                if (accel[0] < 0)
                {
                    minAccel[0] = min(minAccel[0], accel[0]);
                    doneMin[0] = true;
                }
                else
                {
                    maxAccel[0] = max(maxAccel[0], accel[0]);
                    doneMax[0] = true;
                }
            }

            done = true;
            for (int i = 0; i < 3; i++)
            {
                done &= doneMin[i] & doneMax[i];
            }
            cout << "XYZ(" << accel[0] << ", " << accel[1] << ", " << accel[2]
                 << "), " << "X(" << minAccel[0] << ", " << maxAccel[0] << "), "
                 << "Y(" << minAccel[1] << ", " << maxAccel[1] << "), " << "Z("
                 << minAccel[2] << ", " << maxAccel[2] << ")" << endl;
        }

        for (int i = 0; i < 3; i++)
        {
            axisFactorAccel[i] = 8192.0f * 2.0f / (maxAccel[i] - minAccel[i]);
            axisOffsetAccel[i] = ((maxAccel[i] > -minAccel[i]) ? -1.0f : 1.0f)
                    * ((maxAccel[i] - minAccel[i]) * axisFactorAccel[i] / 2.0f
                            - 8192.0f);
        }
        cout << "AxisFactors(" << axisFactorAccel[0] << ", "
             << axisFactorAccel[1] << ", " << axisFactorAccel[2] << ") \n"
             << "AxisOffset(" << axisOffsetAccel[0] << ", "
             << axisOffsetAccel[1] << ", " << axisOffsetAccel[2] << ")" << endl;
    }
}

void calcRollPitchAccel()
{
    float accX = float(accel[0]) / 8192.0f;
    float accY = float(accel[1]) / 8192.0f;
    float accZ = float(accel[2]) / 8192.0f;

    float accModule = sqrt(accX * accX + accY * accY + accZ * accZ);
    if (abs(accY) < accModule)
    {
        rollDegAcc = asin((float) accY / accModule) * 57.296;
    }
    if (abs(accX) < accModule)
    {
        pitchDegAcc = asin((float) accX / accModule) * 57.296;
    }
}

/*
 * from http://www.brokking.net
 */
void calcRollPitch()
{
    //Gyro angle calculations
    //0.0000611 = 1 / (250Hz / 65.5)
    pitchDeg -= float(gyro[1]) * 0.0000611f;
    rollDeg += float(gyro[0]) * 0.0000611f;
    yawDeg += float(gyro[2]) * 0.0000611f;

    // cout << "GyroVal: " << gyro[0] << " - " << gyro[1] << " - @" << float(gyro[1]) * 0.0000611f << endl;
    pitchDeg += rollDeg * sin(gyro[2] * 0.000001066); // 0.0000611 * (2pi)/360
    rollDeg -= pitchDeg * sin(gyro[2] * 0.000001066);

    pitchDeg = pitchDeg * 0.9991 + pitchDegAcc * 0.0009;
    rollDeg = rollDeg * 0.9991 + rollDegAcc * 0.0009;

    {
        // Abbreviations for the various angular functions
        double cy = cos(yawDeg * 0.017453293f * 0.5);
        double sy = sin(yawDeg * 0.017453293f * 0.5);
        double cr = cos(rollDeg * 0.017453293f * 0.5);
        double sr = sin(rollDeg * 0.017453293f * 0.5);
        double cp = cos(pitchDeg * 0.017453293f * 0.5);
        double sp = sin(pitchDeg * 0.017453293f * 0.5);

        q.w = cy * cr * cp + sy * sr * sp;
        q.x = cy * sr * cp - sy * cr * sp;
        q.y = cy * cr * sp + sy * sr * cp;
        q.z = sy * cr * cp - cy * sr * sp;
    }
}

void calibrateAll()
{
    calibrateGyro();
    calibrateAccel();
    cout << "Accel AxisFactors(" << axisFactorAccel[0] << ", "
         << axisFactorAccel[1] << ", " << axisFactorAccel[2] << ")" << endl;
    cout << "Accel K=(" << k[0] << ", " << k[1] << ". " << k[2] << ")" << endl;
    cout << "Gyro Offsets(" << offsetGyro[0] << ", " << offsetGyro[1] << ", "
         << offsetGyro[2] << ")" << endl;
}

// IMU GY88:
//X(-7982, 8310), Y(-8114, 8036), Z(-8255, 7467)
//AxisFactors(1.00565, 1.01449, 1.04211)
// Accel: Mean(-2, -128, -8280), Min(-69, -181, -8378), Max(53, -63, -8192)
// K=(0.00819672, 0.00847458. 0.00537634)

void trackAll()
{
    if (mpu.testConnection())
    {
        /*
         * Accel AxisFactors(1.00868, 1.00288, 0.98794)
         * Accel: X(-7812, 8431), Y(-8297, 8040), Z(-7919, 8665)
         *
         * Accel: Mean(157, -108, 8763), Min(82, -172, 8689), Max(242, -47, 8849)
         * Accel: K=(0.00625, 0.008. 0.00625)
         * Gyro Offsets(-394,-59,12)
         *
         */
        axisOffsetAccel[0] = -309.5f;
        axisOffsetAccel[1] = 128.5f;
        axisOffsetAccel[2] = -459.0f;
        axisFactorAccel[0] = 1.00868f;
        axisFactorAccel[1] = 1.00288f;
        axisFactorAccel[2] = 0.98794f;
        k[0] = 0.00625f;
        k[1] = 0.008f;
        k[2] = 0.00625f;
        offsetGyro[0] = -394;
        offsetGyro[1] = -59;
        offsetGyro[2] = 12;

        // TODO: richiedere in input le calibrazioni da eseguire
        while (!mpu.getIntDataReadyStatus())
            ;
        mpu.getMotion6(accel, accel + 1, accel + 2, gyro, gyro + 1, gyro + 2);

        for (int i = 0; i < 3; i++)
        {
            prevAccel[i] = float(accel[i]) * axisFactorAccel[i]
                    + axisOffsetAccel[i];
        }
        // calc initial roll & pitch (fro accel)
        calcRollPitchAccel();
        pitchDeg = pitchDegAcc;
        rollDeg = rollDegAcc;
        yawDeg = 0;

        uint8_t counter = 0;
        while (1)
        {
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
                    float accelTmp = float(prevAccel[i]) * (1.0f - k[i])
                            + float((accel[i]) * axisFactorAccel[i]
                                    + axisOffsetAccel[i]) * k[i];
                    accel[i] = int16_t(accelTmp);
                    prevAccel[i] = accel[i];
                }
                // calcolo roll e pitch
                // per lo yaw vedo dopo ...
                calcRollPitchAccel();
                calcRollPitch();

                if (counter == 0)
                {
//                    cout << "Acc(" << accel[0] << "," << accel[1] << "," << accel[2] << "), " << "Gyro(" << gyro[0] << "," << gyro[1] << "," << gyro[2] << ")"<< endl;
                    cout << "RPAcc("
                         << rollDegAcc
                         << ", "
                         << pitchDegAcc
                         << "), RPY("
                         << rollDeg
                         << ", "
                         << pitchDeg
                         << ", "
                         << yawDeg
                         << "), "
                         << " Temp("
                         << float(temperature) / 340.0f + 36.53f
                         << "), "
                         << "Thrust["
                         << int16_t(1500)
                                 * (0.85f
                                         + 0.15f
                                                 / (cos(rollDeg * 0.017453293f)
                                                         * cos(pitchDeg
                                                                 * 0.017453293f)))
                         << "]" << endl;
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
}

// muovere in verticale senza rotazioni
// si assume roll = pitch = 0;
void trackAltitude()
{

    if (mpu.testConnection())
    {
        // Estraggo 2000 campioni per stabilizzare il filtro di Kalman
        KalmanFilter kf[3] = { KalmanFilter(0.0000005, 0.005, 1.0, 0.0, 0.0),
                               KalmanFilter(0.0000005, 0.005, 1.0, 0.0, 0.0),
                               KalmanFilter(0.0000005, 0.005, 1.0, 0.0, 0.0) };
        double speed[3] = { 0, 0, 0 }; // cm/sec
        double position[3] = { 0, 0, 0 }; // cm

        /*
         * Accel: Mean(-7, -97, -8316), Min(-102, -194, -8508), Max(100, -22, -8228)
         * K=(0.00495049, 0.00581395. 0.00357143)
         * XYZ(-15, -8094, 99), X(-7940, 8259), Y(-8094, 8018), Z(-8306, 8077)
         * AxisFactors(1.01142, 1.01688, 1.00006)
         * Gyro Offsets(-123, -7, 18)
         *
         */
        axisOffsetAccel[0] = -309.5f;
        axisOffsetAccel[1] = 128.5f;
        axisOffsetAccel[2] = -459.0f;
        axisFactorAccel[0] = 1.00868f;
        axisFactorAccel[1] = 1.00288f;
        axisFactorAccel[2] = 0.98794f;
        k[0] = 0.00625f;
        k[1] = 0.008f;
        k[2] = 0.00625f;
        offsetGyro[0] = -394;
        offsetGyro[1] = -59;
        offsetGyro[2] = 12;

        // stabilizzazione iniziale
        for (uint16_t i = 0; i < 2000; i++)
        {
            while (!mpu.getIntDataReadyStatus())
                ;
            mpu.getMotion6(accel, accel + 1, accel + 2, gyro, gyro + 1,
                           gyro + 2);

            for (int i = 0; i < 3; i++)
            {
                kf[i].compute(
                        float(accel[i]) * axisFactorAccel[i]
                                + axisOffsetAccel[i]);
            }
        }

        // inizializzazione per il ciclo di lettura
        while (!mpu.getIntDataReadyStatus())
            ;

        mpu.getMotion6(accel, accel + 1, accel + 2, gyro, gyro + 1, gyro + 2);

        for (int i = 0; i < 3; i++)
        {
            gyro[i] -= offsetGyro[i];
            if (gyro[i] <= 3 && gyro[i] >= -3)
            {
                gyro[i] = 0;
            }
            prevAccel[i] = kf[i].compute(
                    float(accel[i]) * axisFactorAccel[i] + axisOffsetAccel[i]);
        }
        calcRollPitchAccel();
        pitchDeg = pitchDegAcc;
        rollDeg = rollDegAcc;

        uint8_t counter = 0;
        while (1)
        {
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
                    prevAccel[i] = accel[i];
                    accel[i] = kf[i].compute(
                            float(accel[i]) * axisFactorAccel[i]
                                    + axisOffsetAccel[i]);
                    calcRollPitchAccel();
                    calcRollPitch();
                }

                VectorInt16 accelInWorld(accel[0], accel[1], accel[2]);
                accelInWorld.rotate(&q);
                if(counter == 0) {
                    cout << "WX[" << accelInWorld.x << "], WY[" << accelInWorld.y << "], WZ[" << accelInWorld.z << "]" << endl;
                }
                double acc = (double(accelInWorld.z) / 8192.0f - 1.0f) * 9.7803f;
                if (counter == 0)
                {
                    cout << accel[2] << ",  " << acc << ", Roll=[" << rollDeg
                            << "], Pitch=[" << pitchDeg << "], gyroZ=["
                            << gyro[2] << "]" << endl;
                }
                float v = acc * 0.004; // 0.004 = 1/250Hz
                speed[2] += v;
                position[2] += speed[2] * 0.4; // in cm
                if (acc == 0)
                {
                    speed[2] = speed[2] * 0.8f;
                    if (abs<double>(speed[2]) < 0.02f)
                    {
                        speed[2] = 0.0f;
                    }
                }
                if (counter == 0)
                {
                    cout << "alt=[" << position[2] << "], speed=[" << speed[2]
                            << "], acc=[" << accel[2] << "]" << endl;
                }
                counter++;
                counter %= 100;
            }
        }
    }
    else
    {
        cout << "Sorry!! MPU6050 not connected" << endl;
    }
}
void readMS5611()
{
    MS5611 ms5611;
    while (true)
    {
        if (ms5611.pulse())
        {
            cout << "P["
                 << (ms5611.getData().pressure)
                 << ", "
                 << (uint32_t) (ms5611.getData().rawPressure)
                 << "]"
                 << "T["
                 << ms5611.getData().temperature
                 << ", "
                 << (uint32_t) (ms5611.getData().rawTemperature)
                 << "]"
                 << "A["
                 << ms5611.getAltitude(ms5611.getData().pressure,
                                       double(MS5611_SEALEVEL_PRESSURE))
                 << "]" << endl;
        }
    }

}
void readGps()
{
    TinyGPSPlus gps;
    using namespace LibSerial;
    SerialPort my_serial_port("/dev/ttyS1");
    my_serial_port.Open();
    my_serial_port.SetBaudRate(SerialPort::BAUD_9600);
    my_serial_port.SetCharSize(SerialPort::CHAR_SIZE_8);
    my_serial_port.SetParity(SerialPort::PARITY_NONE);
    int timeout_ms = 2500; // timeout value in milliseconds
//    cout << "FullExample" << endl;
//    cout << "An extensive example of many interesting TinyGPS++ features" << endl;
//    cout << "Testing TinyGPS++ library v. " << endl;
//    cout << TinyGPSPlus::libraryVersion() << endl;
//    cout << "by Mikal Hart" << endl;
//    cout << endl;
//    cout << "Sats HDOP  Latitude   Longitude   Fix  Date       Time     Date Alt    Course Speed Card  Distance Course Card  Chars Sentences Checksum" << endl;
//    cout << "           (deg)      (deg)       Age                      Age  (m)    --- from GPS ----  ---- to London  ----  RX    RX        Fail" << endl;
//    cout << "----------------------------------------------------------------------------------------------------------------------------------------" << endl;

    while (true)
    {
        for (int i = 0; i < 1000; i++)
        {
            char next_char = my_serial_port.ReadByte(timeout_ms);
            gps.encode(next_char);
            cout << next_char;
        }
//        cout << "#Sat(" << gps.satellites.value() << ", " << gps.satellites.isValid() << ")" << endl;
//        cout << "HDOP(" << gps.hdop.hdop() << ", " << gps.hdop.isValid() << ")" << endl;
//        cout << "LAT(" << gps.location.lat()<< ", " << gps.location.isValid() << ")" << endl;
//        cout << "LNG(" << gps.location.lng()<< ", " <<  gps.location.isValid()<< ")" << endl;
//        cout << "AGE(" << gps.location.age()<< ", " << gps.location.isValid() << ")" << endl;
//        cout << "DATE(" << gps.date.year() << "/"<< uint16_t(gps.date.month())<< "/"<< uint16_t(gps.date.day())<< " " << uint16_t(gps.time.hour()) << ":" << uint16_t(gps.time.minute()) << ":" << uint16_t(gps.time.second()) << ":" << uint16_t(gps.time.centisecond()) <<")" << endl;
//        cout << "ALT(" << gps.altitude.meters() << ", "<< gps.altitude.isValid() << ")" << endl;
//        cout << "DEG(" << gps.course.deg() << ", " << gps.course.isValid() << ")" << endl;
//        cout << "KM/H(" << gps.speed.kmph() << ", " << gps.speed.isValid() << ")" << endl;
//        cout << "CARD(" << (gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.deg()) : "*** ") << ")" << endl;
    }
}

int main(int argc, char* argv[])
{
    cout << "GY88 Calibrator Started" << endl; // prints pru-i2c-arm started
    if (argc <= 1)
    {
        mpu.initialize();
        trackAll();
    }
    else if (strcmp(argv[1], "-gyro") == 0)
    {
        mpu.initialize();
        calibrateGyro();
    }
    else if (strcmp(argv[1], "-accel") == 0)
    {
        mpu.initialize();
        calibrateAccel();
    }
    else if (strcmp(argv[1], "-all") == 0)
    {
        mpu.initialize();
        calibrateAll();
        trackAll();
    }
    else if (strcmp(argv[1], "-gps") == 0)
    {
        readGps();
    }
    else if (strcmp(argv[1], "-ms5611") == 0)
    {
        readMS5611();
    }
    else if (strcmp(argv[1], "-altitude") == 0)
    {
        mpu.initialize();
        trackAltitude();
    }
    return 0;
}
