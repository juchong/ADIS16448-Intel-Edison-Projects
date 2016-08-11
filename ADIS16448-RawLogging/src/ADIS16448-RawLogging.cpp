////////////////////////////////////////////////////////////////////////////////////////////////////////
// August 2016
// By: Juan Jose Chong
////////////////////////////////////////////////////////////////////////////////////////////////////////
// ADIS16448 Interrupt Data Logging Example Program
////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// This project interfaces the ADIS16448 with the Intel Edison using mraa SPI libraries.
//
// SPI Assignment = 0
// DR (DIO1) Interrupt Assignment = 45
//
////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <signal.h>
#include <mraa.hpp>

#include "adis16448.hpp"

using namespace upm;
using namespace std;

// Uncomment to enable printing to terminal
//#define DEBUG

// Instantiate Variables
static float gyroData[3];
static float accelData[3];
static float magnetometerData[3];
static float DRCOUNTER;
const char* separator = " ";

// Initialize Sensor
ADIS16448* imu = new ADIS16448(0,3);

// Initialize Output File
ofstream outfile;

void interrupt(void* args){

	int16_t *p;

	// Read data from sensors and return array pointer where data is located
	p = imu->burstReadSensors();
	// Scale each type of data and put into array
	for(int i = 0; i < 3; i++)
		gyroData[i] = ((*(p + (i+1))*.04)*3.14159)*0.0055555;

	for(int i = 0; i < 3; i++)
		accelData[i] = ((*(p + (i+4))*0.000833)*9.80665);

	for(int i = 0; i < 3; i++)
		magnetometerData[i] = (*(p + (i+7))*0.0001429);

	DRCOUNTER = *(p + 11); // Currently outputting TEMP_OUT **

	// Concatenate the IMU data and insert spaces between each point
	outfile << DRCOUNTER << separator << accelData[0] << separator
			<< accelData[1] << separator << accelData[2] << separator
			<< gyroData[0] << separator << gyroData[1] << separator
			<< gyroData[2] << separator << magnetometerData[0]
			<< separator << magnetometerData[1] << separator << magnetometerData[2] << "\n";

}

int main(){
	//// Initialize IMU ////
	imu->configBurstSPI();

	//// Initialize GPIO as Interrupt ////

	// Initialize interrupt
	mraa::Gpio* drgpio = new mraa::Gpio(45);

	// Set GPIO direction
	drgpio->dir(mraa::DIR_IN);

	// Set interrupt trigger edge & attach interrupt to pin
	drgpio->isr(mraa::EDGE_RISING, &interrupt, NULL);

	// Open a .csv file for data collection
	outfile.open ("/home/root/Datalog.csv");

	// Write file header
	outfile << "DRCOUNT" << separator << "XACCEL" << separator << "YACCEL" << separator << "ZACCEL" << separator << "XGYRO" << separator << "YGYRO" << separator << "ZGYRO" << separator << "XMAG" << separator << "YMAG" << separator << "ZMAG" << "\n";

	for(;;) {

#ifdef DEBUG
		// Write variables to the console
		std::cout << "XGYRO_OUT: " << imu->gyroScale(SPIdata[0]) << std::endl;
		std::cout << "YGYRO_OUT: " << imu->gyroScale(SPIdata[1]) << std::endl;
		std::cout << "ZGYRO_OUT: " << imu->gyroScale(SPIdata[2]) << std::endl;
		std::cout << " " << std::endl;
		std::cout << "XACCL_OUT: " << imu->accelScale(SPIdata[3]) << std::endl;
		std::cout << "YACCL_OUT: " << imu->accelScale(SPIdata[4]) << std::endl;
		std::cout << "ZACCL_OUT: " << imu->accelScale(SPIdata[5]) << std::endl;
		std::cout << " " << std::endl;
		std::cout << "XMAG_OUT: " << imu->magnetometerScale(SPIdata[6]) << std::endl;
		std::cout << "YMAG_OUT: " << imu->magnetometerScale(SPIdata[7]) << std::endl;
		std::cout << "ZMAG_OUT: " << imu->magnetometerScale(SPIdata[8]) << std::endl;
		std::cout << " " << std::endl;
		std::cout << "TEMP_OUT: " << imu->tempScale(SPIdata[9]) << std::endl;
		usleep(90000);
		printf("\33[H\33[2J");

#else
		sleep(1);
#endif

	}

	outfile.close();

	return (MRAA_SUCCESS);
}
