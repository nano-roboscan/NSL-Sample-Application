/*
 * Copyright (c) 2013, NANOSYSTEMS CORPORATION. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */


#ifdef _WINDOWS

#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include <Winsock2.h>
#include <WS2tcpip.h>

#include "videoSource.h"

#pragma comment(lib, "Ws2_32.lib")



struct WINSOCK2_INITIALIZE {
	WINSOCK2_INITIALIZE()
	{
		WSADATA wsaData;
		WSAStartup(MAKEWORD(2, 2), &wsaData);
	}
	~WINSOCK2_INITIALIZE() { WSACleanup(); }
};

int signal_recieved = 0;

#else // linux
#include <stdio.h>
#include <signal.h>

#include "videoSource.h"

int signal_recieved = 0;


void sig_handler(int signo)
{
	if( signo == SIGINT )
	{
		printf("received SIGINT\n");
		signal_recieved = 1;
	}
}

#endif

#if 1

int main(int argc, char** argv)
{
	CaptureOptions camOpt;

#ifdef _WINDOWS 
	WINSOCK2_INITIALIZE _startup;
#else
	if( signal(SIGINT, sig_handler) == SIG_ERR )
		printf("can't catch SIGINT\n");
#endif

	videoSource* lidarSrc = videoSource::initAppCfg(argc, argv, &camOpt);

#ifdef SUPPORT_DEEPLEARNING
	lidarSrc->initDeepLearning(&camOpt);
#endif

	lidarSrc->setLidarOption(SSD_TYPE, &camOpt);

	while ( !signal_recieved )
	{
		if (!lidarSrc->captureLidar(3000, &camOpt)) {
			printf("capture : failed...\n");
			break;
		}

		cv::Mat grayMat = *(cv::Mat*)camOpt.frameMat;
		cv::Mat distMat = *(cv::Mat*)camOpt.distMat;

#ifdef SUPPORT_DEEPLEARNING
		lidarSrc->deepLearning(grayMat);
#endif
		lidarSrc->drawCaption(grayMat, distMat, &camOpt);

		if (lidarSrc->prockey(&camOpt) == 27) // ESC
			break;
	}

	lidarSrc->stopLidar();
	delete lidarSrc;
	lidarSrc = NULL;

	printf("end sample_detector\n");

	return 0;
}


#else

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <termios.h>


#define SERIAL_START_MARK				0xFFFFAA55 										///<Start marker for the data (camera to host)
#define SERIAL_END_MARK					0xFFFF55AA										   ///<End marker if no CRC is used
#define SERIAL_BUFFER_SIZE 				1024

int setSerialBaudrate(void)
{
	int fileID;

	char path[100];
	sprintf(path, "%s", "/dev/ttyLidar");
	fileID = open(path, O_RDWR | O_NOCTTY | O_SYNC);
	if( fileID < 0 ) return 0;
	tcflush(fileID, TCIOFLUSH);

	struct termios tty;
	memset (&tty, 0, sizeof tty);

	tcgetattr (fileID, &tty); //TODO...

	cfsetospeed (&tty, B4000000);
	cfsetispeed (&tty, B4000000);

	// no canonical processing
	// disable IGNBRK for mismatched speed tests; otherwise receive break as \000 chars

	tty.c_oflag = 0;				// no remapping, no delays
	tty.c_oflag &= ~(ONLCR | OCRNL); //TODO...

	tty.c_lflag = 0;				// no signaling chars, no echo,
	tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN); //TODO...

	tty.c_iflag &= ~IGNBRK; 		// disable break processing
	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
	tty.c_iflag &= ~(INLCR | IGNCR | ICRNL); //TODO...

	tty.c_cc[VMIN]	= 0;			// non-blocking read
	tty.c_cc[VTIME] = 5;			// 0.5 second read timeout

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; 	// 8-bit chars				  
	tty.c_cflag &= ~(PARENB | PARODD);	// shut off parity
	tty.c_cflag &= ~CSTOPB;    //one stop bit
	tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls   
	tty.c_cflag |= CRTSCTS;   //data DTR hardware control do not use it

	tcflush(fileID, TCIOFLUSH);

	if (tcsetattr (fileID, TCSANOW, &tty) != 0){
		printf("error %d from tcsetattr\n", errno);
		return 0;
	}

	printf("opened USB-Serial : %d\n", fileID);

	return fileID;
}


void reqSingleFrame(int fd)
{
	uint8_t data[3] = {0x00, 0x02, VALUE_AUTO_REPEAT_MEASUREMENT};		// USB 20 fps
//	uint8_t data[3] = {0x00, 0x02, VALUE_SINGLE_MEASUREMENT};			// USB 10 fps
	uint32_t data_len = 3;	
	int cmdType = 2;	

	data[1] = cmdType;
	
	static uint8_t serialData[SERIAL_BUFFER_SIZE];
	int pktLen = 0;

	uint32_t pyalodLen = data_len;
	
	serialData[0] = (uint8_t)((SERIAL_START_MARK >> 24) & 0xFF);
	serialData[1] = (uint8_t)((SERIAL_START_MARK >> 16) & 0xFF);
	serialData[2] = (uint8_t)((SERIAL_START_MARK >> 8) & 0xFF);
	serialData[3] = (uint8_t)((SERIAL_START_MARK >> 0) & 0xFF);
	
	serialData[4] = (uint8_t)((pyalodLen >> 24) & 0xFF);
	serialData[5] = (uint8_t)((pyalodLen >> 16) & 0xFF);
	serialData[6] = (uint8_t)((pyalodLen >> 8) & 0xFF);
	serialData[7] = (uint8_t)((pyalodLen >> 0) & 0xFF);
	
	memcpy(&serialData[8], data, data_len);
	
	serialData[SERIAL_BUFFER_SIZE-4] = (uint8_t)((SERIAL_END_MARK >> 24) & 0xFF);
	serialData[SERIAL_BUFFER_SIZE-3] = (uint8_t)((SERIAL_END_MARK >> 16) & 0xFF);
	serialData[SERIAL_BUFFER_SIZE-2] = (uint8_t)((SERIAL_END_MARK >> 8) & 0xFF);
	serialData[SERIAL_BUFFER_SIZE-1] = (uint8_t)((SERIAL_END_MARK >> 0) & 0xFF);

	int ret = write(fd, (const void *)serialData, SERIAL_BUFFER_SIZE);
}


int rxSerial(int fd) 
{
	static uint8_t socketbuff[308000];
    uint8_t buf[4096];
	int n = 0;
	int buffLen = 307238;	// 4 * 320 * 240 + 13 + 25;

//	const auto& time_cap0 = std::chrono::steady_clock::now();
	reqSingleFrame(fd);

	for(int i=0; i< buffLen; i+=n)
	{
		unsigned long int buf_size = buffLen;
		if(buf_size > sizeof(buf))
			buf_size = sizeof(buf);

		n = read(fd, buf, buf_size);

		if(n > 0){						  
			memcpy(socketbuff + i, buf, n);
		}else if(n == -1){
			printf("Error on  SerialConnection::readRxData= -1\n");
			return -1;
		}else if(n == 0 && i < buffLen-1){
			printf("serialConnection->readRxData %d bytes from %d received\n", i, buffLen);
			return -2;
		}

	}

	printf("rx data len = %d\n", buffLen);
//	const auto& time_cap1 = std::chrono::steady_clock::now();
//	double time_cam = (time_cap1 - time_cap0).count() / 1000000.0;
//	printf("  serial-Rx:		   %9.3lf [msec] len = %d\n", time_cam, buffLen);


	return buffLen;
}


int flushRx(int fd)
{
	uint8_t buf[5000];
	int n = 0;
	int readflushData = 0;

	reqSingleFrame(fd);

	while(true)
	{
		n = read(fd, buf, 5000);

		if(n > 0){
			readflushData += n;
		}else if( n == -1 ){
			printf("flush Error on  SerialConnection::readRxData= -1\n");
			break;
		}else if( n == 0 ){
			printf("flush readData %d bytes\n", readflushData);
			break;
		}

	}

	return 0;
}


/*
	single frame tx/rx test
*/
int main(int argc, char** argv)
{
	if( signal(SIGINT, sig_handler) == SIG_ERR )
		printf("can't catch SIGINT\n");

	int fd = setSerialBaudrate();
	while(signal_recieved == 0){
		if( rxSerial(fd) < 0 ){
			flushRx(fd);
		}
	}

	close(fd);
	printf("end main\n");
	
	return 0;
}

#endif

