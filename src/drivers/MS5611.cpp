/*
MS5611.cpp - Class file for the MS5611 Barometric Pressure & Temperature Sensor Arduino Library.

Version: 1.0.0
(c) 2014 Korneliusz Jarzebski
www.jarzebski.pl

This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <math.h>

#include <drivers/MS5611.h>
#include <drivers/I2Cdev.h>

bool MS5611::begin(ms5611_osr_t osr)
{
    reset();

    setOversampling(osr);

    data.altitude = 0.0f;
    data.dtimeMillis = 0;
    data.pressure = 0.0f;
    data.rawPressure = 0;
    data.rawTemperature = 0;
    data.seaLevelPressure = MS5611_SEALEVEL_PRESSURE;
    data.temperature = 0.0f;
    data.timestamp = boost::posix_time::microsec_clock::local_time();

    status = MS5611_none;
    this->discardSamples = 3000;
    this->pressureCycles = 0;
    this->pressureValueFast = 0.0f;
    this->pressureValueSlow = 0.0f;
    this->pressureBufferPosition = 0.0f;
    for(uint8_t i = 0; i < MS5611_MAX_PRESSURE_CYCLES; i++) {
        pressureBuffer[i] = 0;
    }

    this->initialized = true;
    return initialized;
}


// Set oversampling value
void MS5611::setOversampling(ms5611_osr_t osr)
{
    switch (osr)
    {
	case MS5611_ULTRA_LOW_POWER:
	    ct = 1;
	    break;
	case MS5611_LOW_POWER:
	    ct = 2;
	    break;
	case MS5611_STANDARD:
	    ct = 3;
	    break;
	case MS5611_HIGH_RES:
	    ct = 5;
	    break;
	case MS5611_ULTRA_HIGH_RES:
	    ct = 10;
	    break;
    }

    uosr = osr;
}

// Get oversampling value
ms5611_osr_t MS5611::getOversampling(void)
{
    return (ms5611_osr_t)uosr;
}

void MS5611::reset(void)
{
    writeCmd(MS5611_CMD_RESET);
}

void MS5611::readPROM(void)
{
    std::cout << "Reading PROM" << std::endl;
    for (uint8_t offset = 0; offset < 6; offset++)
    {
	fc[offset] = read16(MS5611_CMD_READ_PROM + (offset * 2));
	std::cout << "fc[" << uint16_t(offset) << "]=" << fc[uint16_t(offset)] << " ";
    }
    std::cout << std::endl;
}

uint8_t MS5611::getConversionTime() {
    uint8_t waitMillis = 10;
    switch(this->uosr) {
    case MS5611_ULTRA_LOW_POWER: {
        waitMillis = 1;
        break;
    }
    case MS5611_LOW_POWER: {
        waitMillis = 2;
        break;
    }
    case MS5611_STANDARD: {
        waitMillis = 3;
        break;
    }
    case MS5611_HIGH_RES: {
        waitMillis = 5;
        break;
    }
    case MS5611_ULTRA_HIGH_RES: {
        waitMillis = 10;
        break;
    }
    }
    return waitMillis;
}
// state machine based on ticks
bool MS5611::pulse() {
    bool result = false;
    if(initialized || status == MS5611_none) { // initialized only after begin() on none
        switch (status) {
        case MS5611_none: {
            begin(MS5611_ULTRA_HIGH_RES);
            changeStatus(MS5611_waitStartup);
            break;
        }
        case MS5611_waitStartup: {
            if (calcMillisFrom(statusAtTime) >= 5) { // 5millis for reset on begin
                this->readPROM();
                startCycle();
                changeStatus(MS5611_requireTemperature);
            }
            break;
        }
        case MS5611_requireTemperature: {
            sendRawTemperatureCmd();
            changeStatus(MS5611_waitTemperature);
            break;
        }
        case MS5611_waitTemperature: {
            if (calcMillisFrom(statusAtTime) >= this->getConversionTime()) {
                data.rawTemperature = readRawTemperature();
                data.temperature = calcTemperature(data.rawTemperature, true);
                changeStatus(MS5611_requirePressure);
            }
            break;
        }
        case MS5611_requirePressure: {
            sendRawPressureCmd();
            changeStatus(MS5611_waitPressure);
            break;
        }
        case MS5611_waitPressure: {
            if (calcMillisFrom(statusAtTime) >= this->getConversionTime()) {
                boost::posix_time::ptime prev = data.timestamp;
                data.timestamp = boost::posix_time::microsec_clock::local_time();
                data.dtimeMillis = data.timestamp.time_of_day().total_milliseconds() - prev.time_of_day().total_milliseconds();

                data.rawPressure = readRawPressure();
                data.pressure = calcPressure(data.rawPressure, data.rawTemperature, true);
                data.altitude = getAltitude(data.pressure);
                if(this->discardSamples == 0) {
                    result = true;
                } else {
                    this->discardSamples--;
                }
                this->pressureCycles++;
                this->pressureCycles %= MS5611_MAX_PRESSURE_CYCLES;
                if(this->pressureCycles == 0) {
                    changeStatus(MS5611_requireTemperature);
                } else {
                    changeStatus(MS5611_requirePressure);
                }
            }
            break;
        }
        }
    }
    return result; // true if update data (cycle complete)
}

MS5611::SensorData const & MS5611::getData() const {
    return data;
}
uint64_t MS5611::calcMillisFrom(
        std::chrono::time_point<std::chrono::system_clock> since) {
    auto duration = std::chrono::system_clock::now() - since;
    auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(
            duration).count();
    return millis;
}
void MS5611::startCycle() {
    cycleAtTime = std::chrono::system_clock::now();
}
void MS5611::changeStatus(MS5611_SensorStatus s) {
    statusAtTime = std::chrono::system_clock::now();
    status = s;
}

uint32_t MS5611::readRawTemperature(void)
{
    return read24(MS5611_CMD_ADC_READ);
}

uint32_t MS5611::readRawPressure(void)
{
    uint32_t result =  read24(MS5611_CMD_ADC_READ);
    return result;
}

double MS5611::calcPressure(uint32_t rawPress, uint32_t rawTemp, bool compensation) {
    int32_t dT = rawTemp - (uint32_t)fc[4] * 256;

    int64_t OFF = (int64_t)fc[1] * 65536 + (int64_t)fc[3] * dT / 128;
    int64_t SENS = (int64_t)fc[0] * 32768 + (int64_t)fc[2] * dT / 256;

    if (compensation)
    {
    int32_t TEMP = 2000 + ((int64_t) dT * fc[5]) / 8388608;

    OFF2 = 0;
    SENS2 = 0;

    if (TEMP < 2000)
    {
        OFF2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 2;
        SENS2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 4;
    }

    if (TEMP < -1500)
    {
        OFF2 = OFF2 + 7 * ((TEMP + 1500) * (TEMP + 1500));
        SENS2 = SENS2 + 11 * ((TEMP + 1500) * (TEMP + 1500)) / 2;
    }

    OFF = OFF - OFF2;
    SENS = SENS - SENS2;
    }

    uint32_t P = (rawPress * SENS / 2097152 - OFF) / 32768;
    this->pushPressure(P);
    this->calcPressureFast();
    this->calcPressureSlow();

    return this->pressureValueSlow;
}
double MS5611::calcTemperature(uint32_t rawTemp, bool compensation) {
    int32_t dT = rawTemp - (uint32_t)fc[4] * 256;
    int32_t TEMP = 2000 + ((int64_t) dT * fc[5]) / 8388608;

    TEMP2 = 0;

    if (compensation)
    {
    if (TEMP < 2000)
    {
        TEMP2 = (dT * dT) / (2 << 30);
    }
    }
    TEMP = TEMP - TEMP2;
    return ((double)TEMP/100);
}
uint32_t MS5611::readPressure(bool compensation)
{
    uint32_t D1 = readRawPressure();
    uint32_t D2 = data.rawTemperature;
    return calcPressure(D1, D2, compensation);
}
double MS5611::readTemperature(bool compensation)
{
    uint32_t D2 = readRawTemperature();
    return calcTemperature(D2, compensation);
}

// Calculate altitude from Pressure & Sea level pressure
double MS5611::getAltitude(double pressure, double seaLevelPressure)
{
    double newAltitude = (44330.0f * (1.0f - pow((double)pressure / (double)seaLevelPressure, 0.1902949f)));
    data.altitude = newAltitude;
    return data.altitude;
}

// Calculate sea level from Pressure given on specific altitude
double MS5611::getSeaLevel(double pressure, double altitude)
{
    return ((double)pressure / pow(1.0f - ((double)altitude / 44330.0f), 5.255f));
}

void MS5611::sendRawTemperatureCmd(void) {
    writeCmd(MS5611_CMD_CONV_D2 + this->uosr);
}

void MS5611::sendRawPressureCmd(void) {
    writeCmd(MS5611_CMD_CONV_D1 + this->uosr);
}

void MS5611::writemem(uint8_t _addr, uint8_t _val) {
    I2Cdev::writeByte(MS5611_ADDRESS, _addr, _val);
}

void MS5611::readmem(uint8_t _addr, uint8_t _nbytes, uint8_t __buff[]) {
    I2Cdev::readBytes(MS5611_ADDRESS, _addr, _nbytes, __buff);
}
uint8_t MS5611::read8(uint8_t a) {
    uint8_t ret;
    readmem(a, 1, &ret);
    return ret;
}
void MS5611::write8(uint8_t a, uint8_t d) {
    writemem(a, d);
}
void MS5611::writeCmd(uint8_t a) {
    I2Cdev::writeCmd(MS5611_ADDRESS, a);
}
uint16_t MS5611::read16(uint8_t a) {
    uint16_t ret;
    uint8_t __buff[2] = { 0, 0 };
    readmem(a, 2, __buff);
    ret = ((uint16_t) __buff[0]) << 8 | ((uint16_t) __buff[1]);
    return ret;
}
uint32_t MS5611::read24(uint8_t a) {
    uint32_t ret;
    uint8_t __buff[3] = { 0, 0, 0 };
    readmem(a, 3, __buff);
    ret = ((uint32_t) __buff[0]) << 16 | ((uint32_t) __buff[1]) << 8 | ((uint32_t) __buff[2]);
    return ret;
}
void MS5611::pushPressure(uint32_t pressure) {
    this->pressureBuffer[this->pressureBufferPosition] = pressure;
    this->pressureBufferPosition++;
    this->pressureBufferPosition %= MS5611_MAX_PRESSURE_CYCLES;
}
void MS5611::calcPressureSlow() {
    this->pressureValueSlow = this->pressureValueSlow * 0.975 + this->pressureValueFast * 0.025;

//    double pressureDiff = std::max<double>(-8.0, std::min<double>(8.0, this->pressureValueSlow - this->pressureValueFast));
//    if(pressureDiff < -1 || pressureDiff > 1) {
//        this->pressureValueSlow -= pressureDiff / 6.0f;
//    }

}
void MS5611::calcPressureFast() {
    uint8_t samples = 0;
    uint32_t sum = 0;
    uint32_t sample = 0;
    for(uint8_t i = 0; i < MS5611_MAX_PRESSURE_CYCLES; i++) {
        sample=this->pressureBuffer[(this->pressureBufferPosition + i) % MS5611_MAX_PRESSURE_CYCLES];
        if(sample == 0) {
            break;
        }
        samples++;
        sum += sample;
    }
    if(samples != 0) {
        this->pressureValueFast = sum/float(samples);
    } else {
        this->pressureValueFast = 0.0f;
    }
    if(this->pressureValueSlow == 0.0f) {
        this->pressureValueSlow = this->pressureValueFast;
    }
}

