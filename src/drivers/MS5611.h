/*
MS5611.h - Header file for the MS5611 Barometric Pressure & Temperature Sensor Arduino Library.

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

#ifndef MS5611_h
#define MS5611_h

#include <stdint.h>
#include <chrono>
#include <boost/date_time/local_time/local_time.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#define MS5611_ADDRESS                (0x77)

#define MS5611_CMD_ADC_READ           (0x00)
#define MS5611_CMD_RESET              (0x1E)
#define MS5611_CMD_CONV_D1            (0x40)
#define MS5611_CMD_CONV_D2            (0x50)
#define MS5611_CMD_READ_PROM          (0xA2)

#define MS5611_MAX_PRESSURE_CYCLES 20

typedef enum
{
    MS5611_ULTRA_HIGH_RES   = 0x08,
    MS5611_HIGH_RES         = 0x06,
    MS5611_STANDARD         = 0x04,
    MS5611_LOW_POWER        = 0x02,
    MS5611_ULTRA_LOW_POWER  = 0x00
} ms5611_osr_t;

class MS5611
{
    public:
    typedef struct {
        uint32_t rawTemperature;
        uint32_t rawPressure;
        double temperature; // gradi centigradi
        int32_t pressure; // Pa
        int32_t seaLevelPressure; // Pa
        double altitude; // meters
        boost::posix_time::ptime timestamp;
        uint16_t dtimeMillis;
    } SensorData;

	bool begin(ms5611_osr_t osr = MS5611_ULTRA_HIGH_RES);
    uint8_t getConversionTime();
	uint32_t readRawTemperature(void);
	uint32_t readRawPressure(void);
    void sendRawTemperatureCmd(void);
    void sendRawPressureCmd(void);
	double readTemperature(bool compensation = false);
	int32_t readPressure(bool compensation = false);
    double calcTemperature(uint32_t rawTemp, bool compensation = false);
    int32_t calcPressure(uint32_t rawPress, uint32_t rawTemp, bool compensation = false);
	double getAltitude(double pressure, double seaLevelPressure = 101325);
	double getSeaLevel(double pressure, double altitude);
	void setOversampling(ms5611_osr_t osr);
	ms5611_osr_t getOversampling(void);
	void writeCmd(uint8_t a);
    bool pulse();
    SensorData const & getData() const;

    private:

	uint16_t fc[6];
	uint8_t ct;
	uint8_t uosr;
	int32_t TEMP2;
	int64_t OFF2, SENS2;

	void reset(void);
	void readPROM(void);

    uint8_t read8(uint8_t addr);
    uint16_t read16(uint8_t addr);
    uint16_t read24(uint8_t addr);
    void write8(uint8_t addr, uint8_t data);
    void readmem(uint8_t _addr, uint8_t _nbytes, uint8_t __buff[]);
    void writemem(uint8_t _addr, uint8_t _val);

    typedef enum {MS5611_none, MS5611_waitStartup, MS5611_requireTemperature, MS5611_waitTemperature, MS5611_requirePressure, MS5611_waitPressure} MS5611_SensorStatus;

    bool initialized = false;
    SensorData data;
    MS5611_SensorStatus status = MS5611_none;
    std::chrono::time_point<std::chrono::system_clock>  statusAtTime;
    std::chrono::time_point<std::chrono::system_clock>  cycleAtTime;
    boost::posix_time::ptime pulseAtTime;
    uint16_t discardSamples;
    uint8_t pressureCycles;

    uint64_t calcMillisFrom(std::chrono::time_point<std::chrono::system_clock> since);
    void startCycle();
    void changeStatus(MS5611_SensorStatus s);
};

#endif
