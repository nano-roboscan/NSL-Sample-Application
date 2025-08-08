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

int main(int argc, char** argv)
{
	CaptureOptions camOpt;

#ifdef _WINDOWS 
	WINSOCK2_INITIALIZE _startup;
#else
	if( signal(SIGINT, sig_handler) == SIG_ERR )
		printf("can't catch SIGINT\n");
#endif

	videoSource* lidarSrc = createApp(argc, argv, &camOpt);

#ifdef SUPPORT_DEEPLEARNING
	lidarSrc->initDeepLearning(&camOpt);
#endif

	lidarSrc->setLidarOption(&camOpt);

	while ( !signal_recieved )
	{
		if (!lidarSrc->captureLidar(3000, &camOpt)) {
			printf("capture : failed...\n");
			break;
		}

#ifdef SUPPORT_DEEPLEARNING
		lidarSrc->deepLearning(camOpt.frameMat);
#endif
		lidarSrc->drawCaption(camOpt.frameMat, camOpt.distMat, &camOpt);

		if (lidarSrc->prockey(&camOpt) == 27) // ESC
			break;
	}

	lidarSrc->stopLidar();
	delete lidarSrc;
	lidarSrc = NULL;

	printf("end sample_detector\n");

	return 0;
}



