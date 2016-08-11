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
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <cstring>
#include <cstdio>
#include <cerrno>
#include <sstream>
#include <cassert>
#include <csignal>

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

ssize_t total_bytes_written = 0;
int sockfd, portno;
struct sockaddr_in serv_addr;
struct hostent *server;

volatile sig_atomic_t flag = 0;
int killprog = 0;
int cpuAffinity = 0;

string strdata;

void quit(int sig){ // can be called asynchronously
  flag = 1; // set flag
}

void error (const char *msg) {
	perror(msg);
	exit(0);
}

void interrupt(void* args){

	// Instantiate variables
	ostringstream data;
	total_bytes_written = 0;
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
	data << DRCOUNTER << separator << accelData[0] << separator
		 << accelData[1] << separator << accelData[2] << separator
		 << gyroData[0] << separator << gyroData[1] << separator << gyroData[2]
		 << separator  << magnetometerData[0] << separator << magnetometerData[1]
		 << separator << magnetometerData[2] << "\n";

	// Convert OutputStringStream data to char data
		const char * c  = data.str().c_str();

	// Write data to TCP socket while making sure every byte is written
	while (total_bytes_written != strlen(c)) {
		assert(total_bytes_written < strlen(c));
		ssize_t bytes_written = write(sockfd, c, (strlen(c) - total_bytes_written));
		if (bytes_written == -1) {
			std::cout << "Write Error Occurred" << std::endl;
			exit(0);
		}
	total_bytes_written += bytes_written;

	// Empty the float arrays and string buffer
		std::fill_n(gyroData, 3, 0);
		std::fill_n(accelData, 3, 0);
		std::fill_n(magnetometerData, 3, 0);
	}
}

int main(int argc, char *argv[]){

	// Initialize IMU
	imu->configBurstSPI();

	// Configure CPU Affinity
	if (cpuAffinity > -1) {
		cpu_set_t mask;
		int status;

		CPU_ZERO(&mask);
		CPU_SET(cpuAffinity, &mask);
		status = sched_setaffinity(0, sizeof(mask), &mask);
		if (status != 0) {
			perror("sched_setaffinity");
		}
	}

	signal(SIGINT, quit);

	// Initialize Client
	if (argc < 5) {
		fprintf(stderr, "ERROR Missing Arguments. Usage: %s <IP/HOSTNAME> <PORT> <DECIMATE(Decimal)> <AVERAGE(Decimal)>\n", argv[0]);
		exit(0);
	}

	portno = atoi(argv[2]);
	sockfd = socket(AF_INET, SOCK_STREAM, 0);

	if (sockfd < 0)
		error("ERROR opening socket");

	server = gethostbyname(argv[1]);

	if (server == NULL) {
		fprintf(stderr, "ERROR no such host\n");
		exit(0);
	}

	bzero((char *) &serv_addr, sizeof(serv_addr));
	serv_addr.sin_family = AF_INET;
	bcopy((char *)server->h_addr, (char *)&serv_addr.sin_addr.s_addr, server->h_length);
	serv_addr.sin_port = htons(portno);
	if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0){
		error("ERROR connecting");
		exit(0);
	}

	// Write IMU Configuration Registers
	imu->regWrite(MSC_CTRL, 0x06); // Enable DataReady on DIO1
	imu->regWrite(SMPL_PRD, (atoi(argv[3]))); // Set Decimation Rate to user specified value.
	imu->regWrite(SENS_AVG, (atoi(argv[4]))); // Set Averaging

	// Initialize interrupt
	mraa::Gpio* drgpio = new mraa::Gpio(45);

	// Set GPIO direction
	drgpio->dir(mraa::DIR_IN);

	// Set interrupt trigger edge & attach interrupt to pin
	drgpio->isr(mraa::EDGE_RISING, &interrupt, NULL);

	printf("Streaming Data...\n");

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
	close(sockfd);
	printf("Program Terminated.");
	printf("\n");
	return (MRAA_SUCCESS);
}
