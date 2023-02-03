#ifndef __VIDEO_SOURCE_H__
#define __VIDEO_SOURCE_H__

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/ocl.hpp>

#include <atomic>
#include <cmath>
#include <string>
#include <vector>

#include "NSLFrame.h"
#include "lens_transform.h"

class videoSource
{
protected:
	videoSource();

private:

	static void del_arg(int argc, char **argv, int index);
	static int find_arg(int argc, char* argv[], char *arg);
	static int find_int_arg(int argc, char **argv, char *arg, int def);
	static float find_float_arg(int argc, char **argv, char *arg, float def);
	static char *find_char_arg(int argc, char **argv, char *arg, char *def);
	static void callback_mouse_click(int event, int x, int y, int flags, void* user_data);
	static videoSource* Create( int type, char *ipaddr );	

	
	void getMouseEvent( int *mouse_xpos, int *mouse_ypos );
	void mouse_click_func(int event, int x, int y);

	std::string distStringCap;
	std::string nonDistStringCap;
	std::string cntStringCap, timeStringCap;

	std::chrono::steady_clock::time_point timeDelay;
	std::chrono::steady_clock::time_point fpsTime;
	clock_t beginTime, endTime ;
	double second_time;

	std::atomic<int> x_start, y_start;


public:
	static videoSource * initAppCfg(int argc, char **argv, CaptureOptions *pAppCfg);
	void setLidarOption(int netType, void *pCapOpt);
	bool captureLidar( int timeout, CaptureOptions *pAppCfg );
	int prockey(CaptureOptions *appCfg);
	void stopLidar();
	void drawCaption(cv::Mat grayMat, cv::Mat distMat, CaptureOptions *appCfg);

	///////////////////// virtual interface ////////////////////////////////////////////////////////////////
	/**
	 * Create videoSource interface from a videoOptions struct that's already been filled out.
	 * It's expected that the supplied videoOptions already contain a valid resource URI.
	 */
	virtual ~videoSource();
	virtual bool Capture( void** image, int timeout=3000 ) = 0;
	virtual void startCaptureCommand( int netType, void *pCapOpt) = 0;
	virtual void setKey(int cmdKey) = 0;
	virtual std::string getDistanceString(int distance ) = 0;
	virtual int getVideoWidth() = 0;
	virtual int getVideoHeight() = 0;	
	virtual int getWidth() = 0;
	virtual int getHeight() = 0;
	virtual bool isRotate90() = 0;	
	virtual void closeLidar() = 0;
	virtual std::string getLeftViewName() = 0;
	virtual void drawPointCloud() = 0;
};

#endif // __VIDEO_SOURCE_H__
