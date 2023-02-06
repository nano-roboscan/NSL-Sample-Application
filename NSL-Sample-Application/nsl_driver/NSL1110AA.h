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

#ifndef __NSL1110AA_CAMERA_H__
#define __NSL1110AA_CAMERA_H__

#ifdef _WINDOWS
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/vlp_grabber.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#endif

#include "videoSource.h"

#define DME660_ETH_BUFF_SIZE		3
#define DME660_BUFF_SIZE   			308000	// 320 x 240 x 4

#define MAX_PHASE  					30000
#define DISTANCE_OFFSET				0
#define AMPLITUDE_THRESHOLD			20
#define DISTANCE_THRESHOLD			72
#define DATA_SIZE 		  			153600 // 320 X 240 X 2

#define DME660_IMAGE_WIDTH 			320
#define DME660_IMAGE_HEIGHT			240
#define DME660_NUM_COLORS     		30000
#define DME660_MARKER_SIZE 			4
#define DME660_PORT 				50660
#define DME660_HOST 				"192.168.7.2"

#define DME660_WAIT_FRAME_COUNT		0

//Special codes for pixels without valid data
#define DME660_LOW_AMPLITUDE		65300
#define DME660_ADC_OVERFLOW			65500
#define DME660_SATURATION			65400
#define DME660_PIXEL_BADASYM		65450


typedef struct Dme660Info_{

	int GrayscaleGainType;
	int nCorrectAmbientLight;
	int GrayscaleOffsetType;
	int saturationType;
	int overflowType;
	int printBasicInfo;
	int drnuType;
	int client_socket;
	int tofcamEvent_key;
	int rotate_90;
	int roiXMin;
	int roiYMin;
	int roiXMax;
	int roiYMax;

	int	integrationTimeGrayscale;
	int	integrationTime3D;
	int	minAmplitude;

	
	int hdr_mode;
	int tofcamModeType;
	int frameTotalLength;
	int wait_frame_cnt;
	int led_control;
	int captureNetType;

	bool bUsedMedianFilter;
	int medianFilterSize;
	int medianFilterIterations;
	
	bool bUsedGaussFilter;
	int  gaussIteration;

	bool bUsedEdgeFilter;	
	int  edgeThresHold;

	double meanAvg;

	int width;
	int height;

	int usedPointCloud;
}DME660_INFO, *PDME660_INFO;


typedef struct DME660_DATABUF_
{
	CRITICAL_SECTION lock;

	int			overflow;
	int			head_idx;
	int 		tail_idx;
	int		 	bufLen[DME660_ETH_BUFF_SIZE];
	uint8_t 	tofcamBuf[DME660_ETH_BUFF_SIZE][DME660_BUFF_SIZE];

	DME660_DATABUF_() {
#ifdef _WINDOWS		
		InitializeCriticalSection(&lock);
#else
		pthread_mutex_init(&lock, NULL);
#endif
	}
	
	~DME660_DATABUF_() {
#ifdef _WINDOWS		
		DeleteCriticalSection(&lock);
#else
		pthread_mutex_destroy(&lock);
#endif
	}
}DME660_DATABUF, *PDME660_DATABUF;



/**
 * MIPI CSI and V4L2 camera capture using GStreamer and `nvarguscamerasrc` or `v4l2src` elements.
 * gstCamera supports both MIPI CSI cameras and V4L2-compliant devices like USB webcams.
 *
 * Examples of MIPI CSI cameras that work out of the box are the OV5693 module from the
 * Jetson TX1/TX2 devkits, and the IMX219 sensor from the Raspberry Pi Camera Module v2.
 *
 * For MIPI CSI cameras, the GStreamer element `nvarguscamerasrc` will be used for capture.
 * For V4L2 devices, the GStreamer element `v4l2src` will be used for camera capture.
 *
 * gstCamera uses CUDA underneath for any necessary colorspace conversion, and provides
 * the captured image frames in CUDA device memory, or zero-copy shared CPU/GPU memory.
 *
 * @note gstCamera now implements the videoSource interface and is intended to be used through 
 * that as opposed to directly. videoSource implements additional command-line parsing of 
 * videoOptions to construct instances. Some legacy APIs of gstCamera are now marked deprecated.
 *
 * @see videoSource
 * @ingroup camera
 */
class NSL1110AA : public videoSource
{
public:
	/**
	 * Create a DME660 lidar device.
	 */
	static NSL1110AA* Create( std::string ipaddr );
	~NSL1110AA();

	virtual void closeLidar();
	virtual bool Capture( void** image, int timeout=3000 );
	virtual void startCaptureCommand( int netType, void *pCapOpt) ;
	virtual void setKey(int cmdKey);
	virtual std::string getDistanceString(int distance );
	virtual int getVideoWidth();
	virtual int getVideoHeight();
	virtual bool isRotate90();
	virtual int getWidth();
	virtual int getHeight();
	virtual std::string getLeftViewName();
	virtual void drawPointCloud(void);

	/**
	 * Default camera width, unless otherwise specified during Create()
 	 */
	static const uint32_t DefaultWidth  = 320;//224;//300;

	/**
	 * Default camera height, unless otherwise specified during Create()
 	 */
	static const uint32_t DefaultHeight = 240;//224;//300;

	
private:
	NSL1110AA( std::string ipaddr );
//	image makeImage(int w, int h, int c);
//	image matToImage(cv::Mat mat);

	void reqStartVideo(void);
	void reqStopVideo(void);
	void initializeDME660(void);
	void reqStartVideo(uint8_t *pTotalBuff);
	void reqStopVideo(uint8_t *pTotalBuff);
	void reqCommandStr(const char *pData, uint8_t *pTotalBuff);
	void reqCommandArg(char *pStrCmd, int flag, uint8_t *pTotalBuff);
	void reqGrayScaleIntegrationTime0(uint8_t *pTotalBuff);
	void reqIntegrationTime3D(uint8_t *pTotalBuff);
	int reqDistanceAmplitudeFrame(SOCKET control_sock);
	int reqSingleFrame(int modeType, uint8_t *pTotalBuff);

	void asymBadPixelReplacer(uint16_t *pMemDistance, uint16_t *pMemAmplitude, int x, int y, int targetAmp_LSB);
	std::string getModeString(int modeType );
	int recvFromTcp(SOCKET sock, uint8_t *total_buf, int expectedRcvLen=0);
	double getPhase(int distanceCM);
	void setGrayScaledColor(cv::Mat &imageLidar, int x, int y, int value, double end_range );
	int setDistanceColor(cv::Mat &imageLidar, int x, int y, int value );
	void setAmplitudeColor(cv::Mat &imageLidar, int x, int y, int value );
	void reqMinAmplitude(uint8_t *pTotalBuff);
	void getComparedHDRAmplitudeImage(uint16_t *pMemDistance, uint16_t *pMemAmplitude, int x, int y);	
	void edgeFilterPixels(uint16_t *pMemDistance, uint16_t *pMemAmplitude);
	void medianFilter(uint16_t *pMemDistance, uint16_t *pMemAmplitude);
	void gaussFilter(uint16_t *pMemDistance, uint16_t *pMemAmplitude);
	int getDistanceAmplitude(cv::Mat &imageDistance, cv::Mat &imageAmplitude, bool bUsedPointCloud);
	int getCodeLen(uint8_t *codeData, int maxLen);
	double interpolate( double x, double x0, double y0, double x1, double y1);
	void createColorMapPixel(int numSteps, int indx, unsigned char &red, unsigned char &green, unsigned char &blue);
	void adjustGrayscaled(void);	
	void keyProc();
	void *rxDme660(void *arg);
	void waitClosingThread();
#ifdef _WINDOWS
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcbVis();
	pcl::visualization::PCLVisualizer::Ptr rgbVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
	static UINT WINAPI rxWrapper(void* thisPtr) {
    	((NSL1110AA*) thisPtr)->rxDme660(NULL);
		_endthreadex( 0 );
	    return 0;
	}

	HANDLE hThread;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr;
	pcl::visualization::PCLVisualizer::Ptr viewer;
	LensTransform  lensTransform;
#else
	static void* rxWrapper(void* thisPtr) {
		((NSL1110AA*) thisPtr)->rxDme660(NULL);
		return NULL;
	}

	pthread_t threadID;
#endif

	std::string  mIpaddrStr;
	
	ImageFrame tofcamImage;
	
	DME660_INFO		tofcamInfo;
	DME660_DATABUF	tofcamBuff;
	int					exit_thtread;

	int 				distanceTable[DME660_IMAGE_WIDTH*DME660_IMAGE_HEIGHT];
	uint8_t 			procBuff[2][DME660_BUFF_SIZE];
	uint8_t 			response[2][DME660_BUFF_SIZE];
	std::vector<cv::Vec3b> colorVector;
	double	sin_angle;
	double	cos_angle;

};

#endif
