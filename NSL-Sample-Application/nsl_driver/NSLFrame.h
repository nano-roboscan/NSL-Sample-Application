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

#ifndef __NSL_FRAME_H__
#define __NSL_FRAME_H__

//#define HAVE_CV_CUDA

#define HDR_NONE_MODE 		0
#define HDR_SPATIAL_MODE 	1
#define HDR_TEMPORAL_MODE 	2

#define VALUE_SINGLE_MEASUREMENT		0			   ///<The sensor makes one acquisition on command
#define VALUE_STREAMING_MEASUREMENT		1			   ///<The sensor starts to make acquisitions until it gets a stop command
#define VALUE_AUTO_REPEAT_MEASUREMENT	2			   ///<The sensor makes an acquisition, and during sending the data, the next acquisition is started already

#ifndef _WINDOWS
typedef int	SOCKET;
typedef pthread_mutex_t CRITICAL_SECTION;

#define EnterCriticalSection	pthread_mutex_lock	
#define LeaveCriticalSection	pthread_mutex_unlock
#define closesocket	close
#define Sleep(N)	usleep(N*1000)
#endif

enum networkModelType_{
	NONEMODEL_TYPE,
	SSD_TYPE,
	MAX_TYPE
};

enum tofcamMode_{
	AMPLITEDE_DISTANCE_MODE,
	AMPLITEDE_DISTANCE_EX_MODE,
	DISTANCE_GRAYSCALE_MODE,
	DISTANCE_MODE,
	GRAYSCALE_MODE,
	MAX_TOFCAM_MODE
};


typedef struct ImageFrame_{
	void	*frameMat;
	void	*distMat;
	int	   *pDistanceTable;	

	int 	localFileTest;
	int 	localFileTotalCnt;
	char 	localFileName[300];
}ImageFrame;

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

	int 	*pDistanceTable;
	void	*frameMat;
	void	*distMat;

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
	int lidarType;
	

}CaptureOptions;

#endif
