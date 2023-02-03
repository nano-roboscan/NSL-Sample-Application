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

	videoSource* lidarSrc = videoSource::initAppCfg(argc, argv, &camOpt);

	lidarSrc->setLidarOption(YOLO_TYPE, &camOpt);

	while ( !signal_recieved )
	{
		if (!lidarSrc->captureLidar(3000, &camOpt)) {
			printf("capture : failed...\n");
			break;
		}

		cv::Mat grayMat = *(cv::Mat*)camOpt.frameMat;
		cv::Mat distMat = *(cv::Mat*)camOpt.distMat;
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

// 프로그램 실행: <Ctrl+F5> 또는 [디버그] > [디버깅하지 않고 시작] 메뉴
// 프로그램 디버그: <F5> 키 또는 [디버그] > [디버깅 시작] 메뉴

// 시작을 위한 팁: 
//   1. [솔루션 탐색기] 창을 사용하여 파일을 추가/관리합니다.
//   2. [팀 탐색기] 창을 사용하여 소스 제어에 연결합니다.
//   3. [출력] 창을 사용하여 빌드 출력 및 기타 메시지를 확인합니다.
//   4. [오류 목록] 창을 사용하여 오류를 봅니다.
//   5. [프로젝트] > [새 항목 추가]로 이동하여 새 코드 파일을 만들거나, [프로젝트] > [기존 항목 추가]로 이동하여 기존 코드 파일을 프로젝트에 추가합니다.
//   6. 나중에 이 프로젝트를 다시 열려면 [파일] > [열기] > [프로젝트]로 이동하고 .sln 파일을 선택합니다.
