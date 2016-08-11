//////////////////////////////////////////////////////////////////////////////////////
// The MIT License (MIT)
//
// Submit Date: 08/10/2016
// Author: Juan Jose Chong <juanjchong@gmail.com>
// Copyright (c) 2016 Juan Jose Chong
//
//////////////////////////////////////////////////////////////////////////////////////
// adis16448.hpp
//////////////////////////////////////////////////////////////////////////////////////
//
// This library is built for the Intel Edison and uses mraa framework to acquire data
// from an ADIS16448. This library can be used as a base for other iSensor products.
//
// This software has been tested using an ADIS16448 conneted to an Intel Edison using
// an ADG3308 for voltage translation. Power is provided to the ADIS16448 by the
// Intel Edison's internal voltage regulator. Pin assignments are shown below:
//
// MISO - GP114
// MOSI - GP115
// !CS - GP111
// !RST - GP44
// SCLK - GP109
// DIO1 - GP45
// DIO3 - GP46
// DIO4 - GP47
//
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to
// the following conditions:
//
// The above copyright notice and this permission notice shall be
// included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
// NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
// LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
// OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
// WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
//////////////////////////////////////////////////////////////////////////////////////

#include <mraa/spi.h>
#include <mraa/gpio.h>

// User Register Memory Map from Table 6 of the Datasheet
#define FLASH_CNT 0x00 	// Flash memory write count
#define DRCNTR 0x02	// DataReady Counter (Not for Release)
#define XGYRO_OUT 0x04 	// X-axis gyroscope output
#define YGYRO_OUT 0x06 	// Y-axis gyroscope output
#define ZGYRO_OUT 0x08 	// Z-axis gyroscope output
#define XACCL_OUT 0x0A 	// X-axis accelerometer output
#define YACCL_OUT 0x0C 	// Y-axis accelerometer output
#define ZACCL_OUT 0x0E 	// Z-axis accelerometer output
#define XMAGN_OUT 0X10 	// X-axis magnetometer output
#define YMAGN_OUT 0x12 	// Y-axis magnetometer output
#define ZMAGN_OUT 0x14 	// Z-axis magnetometer output
#define BARO_OUT 0x16 	// Barometer pressure measurement, high word
#define TEMP_OUT 0x18 	// Temperature output
#define XGYRO_OFF 0x1A 	// X-axis gyroscope bias offset factor
#define YGYRO_OFF 0x1C 	// Y-axis gyroscope bias offset factor
#define ZGYRO_OFF 0x1E 	// Z-axis gyroscope bias offset factor
#define XACCL_OFF 0x20 	// X-axis acceleration bias offset factor
#define YACCL_OFF 0x22 	// Y-axis acceleration bias offset factor
#define ZACCL_OFF 0x24 	// Z-axis acceleration bias offset factor
#define XMAGN_HIC 0x26 	// X-axis magnetometer, hard iron factor
#define YMAGN_HIC 0x28 	// Y-axis magnetometer, hard iron factor
#define ZMAGN_HIC 0x2A 	// Z-axis magnetometer, hard iron factor
#define XMAGN_SIC 0x2C 	// X-axis magnetometer, soft iron factor
#define YMAGN_SIC 0x2E 	// Y-axis magnetometer, soft iron factor
#define ZMAGN_SIC 0x30 	// Z-axis magnetometer, soft iron factor
#define GPIO_CTRL 0x32 	// GPIO control
#define MSC_CTRL 0x34 	// MISC control
#define SMPL_PRD 0x36 	// Sample clock/Decimation filter control
#define SENS_AVG 0x38 	// Digital filter control
#define SEQ_CNT 0x3A 	// MAGN_OUT and BARO_OUT counter
#define DIAG_STAT 0x3C 	// System status
#define GLOB_CMD 0x3E 	// System command
#define ALM_MAG1 0x40 	// Alarm 1 amplitude threshold
#define ALM_MAG2 0x42 	// Alarm 2 amplitude threshold
#define ALM_SMPL1 0x44 	// Alarm 1 sample size
#define ALM_SMPL2 0x46 	// Alarm 2 sample size
#define ALM_CTRL 0x48 	// Alarm control
#define LOT_ID1 0x52 	// Lot identification number
#define LOT_ID2 0x54 	// Lot identification number
#define PROD_ID 0x56 	// Product identifier
#define SERIAL_NUM 0x58 // Lot-specific serial number

namespace upm {

	// ADIS16448 Class Definition
	class ADIS16448 {

		public:

		// ADIS16448 Constructor (SPI Bus, RESET Pin);
		ADIS16448(int bus, int rst);

		// Destructor
		~ADIS16448();

		// Performs hardware reset. Delay in microseconds, returns 1 when complete.
		int resetDUT(uint8_t delay);

		// Sets SPI bit order, clock divider, and data mode. Returns 1 when complete.
		int configSPI();

		// Sets SPI bit order, clock divider, and data mode for BURST MODE. Returns 1 when complete.
		int configBurstSPI();

		// Read register (two bytes)
		int16_t regRead(uint8_t regAddr);

		// Write register (two bytes)
		int regWrite(unsigned char regAddr, int regData);

		// Reads gyro, accel, mag sensors and returns array pointer
		int16_t * readSensors();

		// Reads DIAG_STAT,XGYRO,YGYRO,ZGYRO,XACCEL,YACCEL,ZACCEL,XMAG,YMAG,ZMAG,BARO,and TEMP
		//	using the burst read technique. Sensor MUST be configured for burst read using configBurstSPI().
		int16_t * burstReadSensors();

		// Scale accelerator data
		float accelScale(int16_t sensorData);

		// Scale gyro data
		float gyroScale(int16_t sensorData);

		// Scale temperature data
		float tempScale(int16_t sensorData);

		// Scale barometer data
		float pressureScale(int16_t sensorData);

		// Scale magnetometer data
		float magnetometerScale(int16_t sensorData);

		// Variables exclusive to the library
		private:
		mraa_spi_context _spi;
		mraa_gpio_context _rst;

	};

}

