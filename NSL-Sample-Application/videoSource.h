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

#ifndef __VIDEO_SOURCE_H__
#define __VIDEO_SOURCE_H__

#include <opencv2/opencv.hpp>
#ifdef SUPPORT_DEEPLEARNING
#include <opencv2/dnn.hpp>
#endif
#include <opencv2/highgui.hpp>
#include <opencv2/core/ocl.hpp>

#include <atomic>
#include <cmath>
#include <string>
#include <vector>
#include <chrono>

#include "nanolib.h"

#define SUPPORT_DEEPLEARNING


typedef struct CaptureOptions_{
	// lidar parameter	
	int captureType;
	int integrationTime;
	int grayIntegrationTime;
	int minAmplitude;
	int edgeThresHold;	
	int medianFilterSize;
	int medianFilterIterations;	
	int gaussIteration;

	int medianFilterEnable;
	int averageFilterEnable;
	int	temporalFilterFactorActual;
	int	temporalFilterThreshold;
	int	interferenceUseLashValueEnable;
	int	interferenceLimit;

	cv::Mat	frameMat;
	cv::Mat	distMat;

	// deep learning parameter
	int inputSize;
	float detectThreshold;
	float minConfidence;
	float maxConfidence;
	float curConfidence;


	// display & running parameter	
	int	fpsCount;
	int displayFps;
	int maxDistance;	
	
	NslConfig	nslDevConfig;
}CaptureOptions;


class videoSource
{
private:	

	std::string distStringCap;
	std::string nonDistStringCap;
	std::string cntStringCap, timeStringCap;

	std::chrono::steady_clock::time_point timeDelay;
	std::chrono::steady_clock::time_point frameTime;
	std::chrono::steady_clock::time_point fpsTime;
	clock_t beginTime, endTime ;

	std::atomic<int> x_start, y_start;
#ifdef SUPPORT_DEEPLEARNING
	cv::dnn::Net dnnNet;
	float conf_threshold;
#endif
	NslPCD pcdData;


public:
	void getMouseEvent( int *mouse_xpos, int *mouse_ypos );
	void mouse_click_func(int event, int x, int y);
	void setMatrixColor(cv::Mat image, int x, int y, NslOption::NslVec3b color);
	void del_arg(int argc, char **argv, int index);
	int find_arg(int argc, char* argv[], char *arg);
	int find_int_arg(int argc, char **argv, const char *arg, int def);
	float find_float_arg(int argc, char **argv, const char *arg, float def);
	const char *find_char_arg(int argc, char **argv, const char *arg, const char *def);

	videoSource();
	~videoSource();
	void setLidarOption(void *pCapOpt);
	bool captureLidar( int timeout, CaptureOptions *pAppCfg );
	int prockey(CaptureOptions *appCfg);
	void stopLidar();
	void drawCaption(cv::Mat grayMat, cv::Mat distMat, CaptureOptions *appCfg);
	void initDeepLearning( CaptureOptions *pAppCfg );
	void deepLearning( cv::Mat imageLidar );

	///////////////////// virtual interface ////////////////////////////////////////////////////////////////
	/**
	 * Create videoSource interface from a videoOptions struct that's already been filled out.
	 * It's expected that the supplied videoOptions already contain a valid resource URI.
	 */
	std::string getDistanceString(int distance );
	int getVideoWidth();
	int getVideoHeight();	
	int getWidthDiv();
	int getHeightDiv();
	int getWidth();
	int getHeight();
	std::string getLeftViewName();

	int handle;

};

videoSource * createApp(int argc, char **argv, CaptureOptions *pAppCfg);

#endif // __VIDEO_SOURCE_H__
