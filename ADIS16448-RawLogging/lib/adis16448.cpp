//////////////////////////////////////////////////////////////////////////////////////
// The MIT License (MIT)
//
// Submit Date: 08/10/2016
// Author: Juan Jose Chong <juanjchong@gmail.com>
// Copyright (c) 2016 Juan Jose Chong
//
//////////////////////////////////////////////////////////////////////////////////////
// adis16448.cxx
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

#include <iostream>
#include <fstream>
#include <unistd.h>

#include "adis16448.hpp"

using namespace upm;
using namespace std;

////////////////////////////////////////////////////////////////////////////
// Constructor
////////////////////////////////////////////////////////////////////////////
// bus - SPI bus designator (default = 0)
// rst - Hardware reset pin
////////////////////////////////////////////////////////////////////////////
ADIS16448::ADIS16448(int bus, int rst)
{
// Configure I/O
	_rst = mraa_gpio_init(rst); // Initialize RST pin
	mraa_gpio_dir(_rst, MRAA_GPIO_IN); // Set direction as INPUT

// Configure SPI
	_spi = mraa_spi_init(bus);
}

////////////////////////////////////////////////////////////////////////////
// Destructor - Stops SPI and Closes all GPIO used. Reports an error if
// unable to close either properly.
////////////////////////////////////////////////////////////////////////////
ADIS16448::~ADIS16448()
{
// Close SPI bus
	mraa_spi_stop(_spi);
	mraa_result_t error;
	error = mraa_spi_stop(_spi);
	if(error != MRAA_SUCCESS)
	{
		mraa_result_print(error);
	}
	error = mraa_gpio_close(_rst);
	if(error != MRAA_SUCCESS)
	{
		mraa_result_print(error);
	}
}

////////////////////////////////////////////////////////////////////////////
// Performs hardware reset by setting _RST pin low for delay microseconds
////////////////////////////////////////////////////////////////////////////
int ADIS16448::resetDUT(uint8_t delay) {
	mraa_gpio_write(_rst, 0);
	usleep(100000); // Sleep for 100ms
	mraa_gpio_write(_rst, 1);
	usleep(delay); // Sleep for delay us
	return (1);
}

////////////////////////////////////////////////////////////////////////////
// Sets SPI bit order, clock divider, and data mode. This function is useful
// when there are multiple SPI devices using different settings.
// Returns 1 when complete.
////////////////////////////////////////////////////////////////////////////
int ADIS16448::configSPI() {
	mraa_spi_frequency(_spi, 2000000); // Set SPI frequency to 1MHz
	mraa_spi_mode(_spi, MRAA_SPI_MODE3); // Set SPI mode/polarity
	mraa_spi_bit_per_word(_spi, 16); // Set # of bits per word
	return (1);
}

////////////////////////////////////////////////////////////////////////////
// Sets SPI bit order, clock divider, and data mode. This function is useful
// when there are multiple SPI devices using different settings. Burst-mode
// maximum clock is limited to 1MHz.
// Returns 1 when complete.
////////////////////////////////////////////////////////////////////////////
int ADIS16448::configBurstSPI() {
	mraa_spi_frequency(_spi, 1000000); // Set SPI frequency to 1MHz
	mraa_spi_mode(_spi, MRAA_SPI_MODE3); // Set SPI mode/polarity
	mraa_spi_bit_per_word(_spi, 16); // Set # of bits per word
	return (1);
}

////////////////////////////////////////////////////////////////////////////////////////////
// Reads two bytes (one word) in two sequential registers over SPI
////////////////////////////////////////////////////////////////////////////////////////////
// regAddr - register address from the lookup table in ADIS16448.h
// return - (int) signed 16 bit 2's complement number
////////////////////////////////////////////////////////////////////////////////////////////
int16_t ADIS16448::regRead(uint8_t regAddr) {
	mraa_spi_write_word(_spi,(regAddr << 8));
	usleep(20);
	int16_t dataOut = mraa_spi_write_word(_spi,0x00);
	return(dataOut);
}
////////////////////////////////////////////////////////////////////////////
// Writes one byte of data to the specified register over SPI
////////////////////////////////////////////////////////////////////////////
// regAddr - address of register to be written
// regData - data to be written to the register
////////////////////////////////////////////////////////////////////////////
int ADIS16448::regWrite(unsigned char regAddr, int regData) {
	// Write register address and data
	unsigned short addr = (((regAddr & 0x7F) | 0x80) << 8); // Toggle sign bit, and check that the address is 8 bits
	unsigned short lowWord = (addr | (regData & 0xFF)); // OR Register address (A) with data(D) (AADD)
	unsigned short highWord = ((addr | 0x100) | ((regData >> 8) & 0xFF)); // OR Register address with data and increment address

	//std::cout << "Low:" << hex << (int) lowWord <<"; High: " << hex << (int) highWord << std::endl;

	mraa_spi_write_word(_spi, highWord); // Write the buffer to the SPI port
	usleep(20);
	mraa_spi_write_word(_spi, lowWord);
	return 1;
}

////////////////////////////////////////////////////////////////////////////////////////////
// Reads one word from each sensor output and places the data in an array.
// Duplex SPI communication is used. Optimized for single application.
////////////////////////////////////////////////////////////////////////////////////////////
// return - (pointer) pointer to signed 16 bit 2's complement number array
////////////////////////////////////////////////////////////////////////////////////////////
int16_t * ADIS16448::readSensors() {
	static int16_t data[10];
	mraa_spi_write_word(_spi, (XGYRO_OUT << 8));
	data[0] = mraa_spi_write_word(_spi,(YGYRO_OUT << 8));
	data[1] = mraa_spi_write_word(_spi,(ZGYRO_OUT << 8));
	data[2] = mraa_spi_write_word(_spi,(XACCL_OUT << 8));
	data[3] = mraa_spi_write_word(_spi,(YACCL_OUT << 8));
	data[4] = mraa_spi_write_word(_spi,(ZACCL_OUT << 8));
	data[5] = mraa_spi_write_word(_spi,(XMAGN_OUT << 8));
	data[6] = mraa_spi_write_word(_spi,(YMAGN_OUT << 8));
	data[7] = mraa_spi_write_word(_spi,(ZMAGN_OUT << 8));
	data[8] = mraa_spi_write_word(_spi,(DRCNTR << 8));
	data[9] = mraa_spi_write_word(_spi,(FLASH_CNT << 8));
	return data;
}

int16_t * ADIS16448::burstReadSensors() {
	static uint8_t trigger[] = {0,0x3E,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	static uint8_t burstdata[26];
	static int16_t burstwords[12];
	mraa_spi_transfer_buf(_spi,trigger,burstdata,26);
	// Data Order: burstdata[0],burstdata[1] = UPPER,LOWER
	// To form 16-bit words, ((burstdata[1] << 8) | burstdata[0])

	#ifdef DEBUG
	std::cout << (int)burstdata[0] << ',' << (int)burstdata[1] << ',' << (int)burstdata[2] << ',' << (int)burstdata[3] << ',' << (int)burstdata[4] << ',' << (int)burstdata[5] << ',' << (int)burstdata[6] << ',' << (int)burstdata[7] << ',' << (int)burstdata[8] << ',' << (int)burstdata[9] << ',' << (int)burstdata[10] << ',' << (int)burstdata[11] << ',' << (int)burstdata[12] << ',' << (int)burstdata[13] << ',' << (int)burstdata[14] << ',' << (int)burstdata[15] << ',' << (int)burstdata[16] << ',' << (int)burstdata[17] << ',' << (int)burstdata[18] << ',' << (int)burstdata[19] << ',' << (int)burstdata[20] << ',' << (int)burstdata[21] << ',' << (int)burstdata[22] << ',' << (int)burstdata[23] << ',' << (int)burstdata[24] << ',' << (int)burstdata[25] << std::endl;
	#endif

	// Ignore burstdata[0] and burstdata[1] - Junk data from previous frame
	burstwords[0] = ((burstdata[3] << 8) | burstdata[2]); //DIAG_STAT
	burstwords[1] = ((burstdata[5] << 8) | burstdata[4]); //XGYRO
	burstwords[2] = ((burstdata[7] << 8) | burstdata[6]); //YGYRO
	burstwords[3] = ((burstdata[9] << 8) | burstdata[8]); //ZGYRO
	burstwords[4] = ((burstdata[11] << 8) | burstdata[10]); //XACCEL
	burstwords[5] = ((burstdata[13] << 8) | burstdata[12]); //YACCEL
	burstwords[6] = ((burstdata[15] << 8) | burstdata[14]); //ZACCEL
	burstwords[7] = ((burstdata[17] << 8) | burstdata[16]); //XMAG
	burstwords[8] = ((burstdata[19] << 8) | burstdata[18]); //YMAG
	burstwords[9] = ((burstdata[21] << 8) | burstdata[20]); //ZMAG
	burstwords[10] = ((burstdata[23] << 8) | burstdata[22]); //BARO
	burstwords[11] = ((burstdata[25] << 8) | burstdata[24]); //TEMP

	#ifdef DEBUG
	std::cout << (int)burstwords[0] << ',' << (int)burstwords[1] << ',' <<(int)burstwords[2] << ',' << (int)burstwords[3] << ',' << (int)burstwords[4] << ',' << (int)burstwords[5] << ',' << (int)burstwords[6] << ',' << (int)burstwords[7] << ',' << (int)burstwords[8] << ',' << (int)burstwords[9] << ',' << (int)burstwords[10] << ',' << (int)burstwords[11] <<std::endl;
	#endif

	return burstwords;
}

/////////////////////////////////////////////////////////////////////////////////////////
// Converts accelerometer data output from the regRead() function and returns
// acceleration in g's
/////////////////////////////////////////////////////////////////////////////////////////
// sensorData - data output from regRead()
// return - (float) signed/scaled accelerometer in g's
/////////////////////////////////////////////////////////////////////////////////////////
float ADIS16448::accelScale(int16_t sensorData) {
	float finalData = sensorData * 0.000833; // Multiply by accel sensitivity (1200 LSB/g)
	return finalData;
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Converts gyro data output from the regRead() function and returns gyro rate in deg/sec
/////////////////////////////////////////////////////////////////////////////////////////////
// sensorData - data output from regRead()
// return - (float) signed/scaled gyro in degrees/sec
/////////////////////////////////////////////////////////////////////////////////////////
float ADIS16448::gyroScale(int16_t sensorData) {
	float finalData = sensorData * 0.04; // Multiply by gyro sensitivity (25LSB/dps)
	return finalData;
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Converts temperature data output from the regRead() function and returns temperature
// in degrees Celcius
/////////////////////////////////////////////////////////////////////////////////////////////
// sensorData - data output from regRead()
// return - (float) signed/scaled temperature in degrees Celcius
/////////////////////////////////////////////////////////////////////////////////////////
float ADIS16448::tempScale(int16_t sensorData) {
	float finalData = (sensorData * 0.07386) + 31; // Multiply by temperature scale and add 31 to equal 0x0000
	return finalData;
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Converts barometer data output from regRead() function and returns pressure in bar
/////////////////////////////////////////////////////////////////////////////////////////////
// sensorData - data output from regRead()
// return - (float) signed/scaled pressure in mBar
/////////////////////////////////////////////////////////////////////////////////////////
float ADIS16448::pressureScale(int16_t sensorData) {
	float finalData = (sensorData * 0.02); // Multiply by barometer sensitivity (0.02 mBar/LSB)
	return finalData;
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Converts magnetometer output from regRead() function and returns magnetic field
// reading in Gauss
/////////////////////////////////////////////////////////////////////////////////////////////
// sensorData - data output from regRead()
// return - (float) signed/scaled magnetometer data in mgauss
/////////////////////////////////////////////////////////////////////////////////////////
float ADIS16448::magnetometerScale(int16_t sensorData) {
	float finalData = (sensorData * 0.0001429); // Multiply by sensor resolution (142.9 uGa/LSB)
	return finalData;
}
