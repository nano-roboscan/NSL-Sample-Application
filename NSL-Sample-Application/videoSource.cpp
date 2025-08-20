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

#include <memory>
#include <thread>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "videoSource.h"

#ifdef HAVE_CV_CUDA
#include <opencv2/cudawarping.hpp>
#endif

using namespace cv;
using namespace std;
using namespace NslOption;


#define WIN_NAME			"nanosystems"

#define NSL1110_TYPE			0
#define NSL3130_TYPE			1

#define VIEW_INFO_UPPER_SIZE	50
#define VIEW_INFO_Y_SIZE		160
#define VIEW_INFO_LOWER_SIZE	30

#define MINIBOX_WIDTH		160
#define MINIBOX_HEIGHT		120

#define DETECT_DELAY_TIME	2000
#define CHECK_LONG_TIME		(3600 * 24)	// 24 hour
#define MOTION_MAX_CNT		(120 * 20)	//20 min
#define MOTION_SENSITIVITY	40

void callback_mouse_click(int event, int x, int y, int flags, void* user_data)
{
	videoSource* vidSrc = reinterpret_cast<videoSource*>(user_data);
	vidSrc->mouse_click_func(event, x, y);
}


void print_help()
{
	printf("-captureType : 1 : DISTANCE_MODE, 2 : GRAYSCALE_MODE, 3 : DISTANCE_AMPLITUDE_MODE, 4 : DISTANCE_GRAYSCALE_MODE\n");
	printf("-maxDistance : 0 ~ 12500\n");
	printf("-intTime : 0 ~ 4000\n");
	printf("-grayintTime : 0 ~ 50000\n");
	printf("-amplitudeMin : 30 ~ 1000\n");
	printf("-edgeThresHold : 0=disable, 1~5000\n");
	printf("-medianFilterEnable : 0=disable, 1=enable\n");
	printf("-medianFilter : 0=disable, 1~99\n");
	printf("-medianIter : 0=disable, 1~10000\n");
	printf("-gaussIter : 0=disable, 1~10000\n");
	printf("-averageFilterEnable : 0=disable, 1=enable\n");
	printf("-temporalFactor : 0=disable, 1~1000\n");
	printf("-temporalThresHold : 0=disable, 1~1000\n");
	printf("-interferenceEnable : 0=disable, 1=enable\n");
	printf("-interferenceLimit : 0=disable, 1~1000\n");
}

/*
# -captureType : 1 ~ 8
	enum class OPERATION_MODE_OPTIONS
	{
		NONE_MODE = 0,
		DISTANCE_MODE = 1,
		GRAYSCALE_MODE = 2,
		DISTANCE_AMPLITUDE_MODE = 3,
		DISTANCE_GRAYSCALE_MODE = 4,
		RGB_MODE = 5,
		RGB_DISTANCE_MODE = 6,
		RGB_DISTANCE_AMPLITUDE_MODE = 7,
		RGB_DISTANCE_GRAYSCALE_MODE = 8
	};

# -maxDistance : 0 ~ 12500
# -intTime : 50 ~ 4000
# -grayintTime : 50 ~ 50000
# -amplitudeMin : 30 ~ 1000
# -edgeThresHold : 0=disable, 1~5000
# -medianFilterEnable :  0=disable, 1=enable
# -medianFilter : 0=disable, 1~99
# -medianIter : 0=disable, 1~10000
# -gaussIter :  0=disable, 1~10000
# -averageFilterEnable :  0=disable, 1=enable
# -temporalFactor :  0=disable, 1~1000
# -temporalThresHold :  0=disable, 1~1000
# -interferenceEnable :  0=disable, 1=enable
# -interferenceLimit :  0=disable, 1~1000
*/
videoSource* createApp(int argc, char **argv, CaptureOptions *pAppCfg)
{
	memset(pAppCfg, 0, sizeof(CaptureOptions));
	
	pAppCfg->minConfidence= 100.0;
	pAppCfg->maxConfidence = 0;

	// HELP ...
	for(int i = 0; i < argc; ++i){
		if(!argv[i]) continue;
		if(0==strcmp(argv[i], "-help")){
			print_help();
			exit(0);
		}
	}

	
	videoSource *vidSrc = new videoSource();
	
	cv::namedWindow(WIN_NAME, cv::WINDOW_NORMAL);
	cv::setWindowProperty(WIN_NAME, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);	
	cv::setMouseCallback(WIN_NAME, callback_mouse_click, vidSrc);
		

	const char *ipAddr = vidSrc->find_char_arg(argc, argv, "ipaddr", "192.168.0.220");
//	const char *ipAddr = vidSrc->find_char_arg(argc, argv, "ipaddr", "/dev/ttyLidar");

	pAppCfg->inputSize = vidSrc->find_int_arg(argc, argv, "-inputSize", 0); // 0 : 320, 1 : 416
	pAppCfg->detectThreshold = vidSrc->find_float_arg(argc, argv, "-thresh", .4f);

	pAppCfg->captureType = vidSrc->find_int_arg(argc, argv, "-captureType", 3);// 1 ~ 8
	pAppCfg->integrationTime = vidSrc->find_int_arg(argc, argv, "-intTime", 800);//800;
	pAppCfg->grayIntegrationTime = vidSrc->find_int_arg(argc, argv, "-grayintTime", 100);//100;
	pAppCfg->maxDistance = vidSrc->find_int_arg(argc, argv, "-maxDistance", 12500);
	pAppCfg->minAmplitude = vidSrc->find_int_arg(argc, argv, "-amplitudeMin", 50);

	pAppCfg->edgeThresHold = vidSrc->find_int_arg(argc, argv, "-edgeThresHold", 0);
	pAppCfg->medianFilterSize = vidSrc->find_int_arg(argc, argv, "-medianFilterSize", 0);
	pAppCfg->medianFilterIterations = vidSrc->find_int_arg(argc, argv, "-medianIter", 0);
	pAppCfg->gaussIteration = vidSrc->find_int_arg(argc, argv, "-gaussIter", 0);

	pAppCfg->medianFilterEnable = vidSrc->find_int_arg(argc, argv, "-medianFilterEnable", 0);
	pAppCfg->averageFilterEnable = vidSrc->find_int_arg(argc, argv, "-averageFilterEnable", 0);
	pAppCfg->temporalFilterFactorActual = vidSrc->find_int_arg(argc, argv, "-temporalFactor", 0);
	pAppCfg->temporalFilterThreshold = vidSrc->find_int_arg(argc, argv, "-temporalThresHold", 0);
	pAppCfg->interferenceUseLashValueEnable = vidSrc->find_int_arg(argc, argv, "-interferenceEnable", 0);
	pAppCfg->interferenceLimit = vidSrc->find_int_arg(argc, argv, "-interferenceLimit", 0);


	if( pAppCfg->inputSize == 0 ) pAppCfg->inputSize = 320;
	else if( pAppCfg->inputSize == 1 ) pAppCfg->inputSize = 416;

	if( pAppCfg->captureType <= 0 || pAppCfg->captureType > 8 ) pAppCfg->captureType = 3;
	if( pAppCfg->maxDistance == 0 ) pAppCfg->maxDistance = 12500;
	if( pAppCfg->integrationTime == 0 ) pAppCfg->integrationTime = 800;
	if( pAppCfg->grayIntegrationTime == 0 ) pAppCfg->grayIntegrationTime = 100;
	if( pAppCfg->minAmplitude == 0 ) pAppCfg->minAmplitude = 50;
	if( pAppCfg->medianFilterSize < 0 || pAppCfg->medianFilterSize > 99 ) pAppCfg->medianFilterSize = 0;
	if( pAppCfg->medianFilterIterations < 0 || pAppCfg->medianFilterIterations > 10000 ) pAppCfg->medianFilterIterations = 0;
	if( pAppCfg->gaussIteration < 0 || pAppCfg->gaussIteration > 10000 ) pAppCfg->gaussIteration = 0;
	if( pAppCfg->edgeThresHold < 0 || pAppCfg->edgeThresHold > 10000 ) pAppCfg->edgeThresHold = 0;
	
	pAppCfg->nslDevConfig.lidarAngle = 0;
	pAppCfg->nslDevConfig.lensType = NslOption::LENS_TYPE::LENS_SF;

	vidSrc->handle = nsl_open(ipAddr, &pAppCfg->nslDevConfig, FUNCTION_OPTIONS::FUNC_ON);
	
	return vidSrc;
}



// constructor
videoSource::videoSource()
{
	printf("new videoSource()\n");

	fpsTime = std::chrono::steady_clock::now();
}

videoSource::~videoSource()
{
	printf("~videoSource()\n");
	nsl_close();
}


/////////////////////////////////////// private function ///////////////////////////////////////////////////////////
std::string videoSource::getDistanceString(int distance )
{
	std::string distStr;

	if( distance == NSL_LOW_AMPLITUDE )
		distStr = "LOW_AMPLITUDE";
	else if( distance == NSL_ADC_OVERFLOW )
		distStr = "ADC_OVERFLOW";
	else if( distance == NSL_SATURATION )
		distStr = "SATURATION";
	else if( distance == NSL_INTERFERENCE )
		distStr = "INTERFERENCE";
	else if( distance == NSL_EDGE_DETECTED )
		distStr = "EDGE_DETECTED";
	else if( distance == NSL_BAD_PIXEL )
		distStr = "BAD_FIXEL";
	else
		distStr = format("%d mm", distance);

	return distStr;
}

int videoSource::getWidthDiv()
{
	return getWidth()/NSL_LIDAR_TYPE_A_WIDTH;
}


int videoSource::getHeightDiv()
{
	return getHeight()/NSL_LIDAR_TYPE_A_HEIGHT;
}


int videoSource::getWidth()
{
	return 640;
}


int videoSource::getHeight()
{
	return 480;
}



std::string videoSource::getLeftViewName()
{
	std::string nameStr;
	if( pcdData.operationMode == OPERATION_MODE_OPTIONS::DISTANCE_AMPLITUDE_MODE )
		nameStr = "AMPL&Gray";
	else //if( tofcamInfo.tofcamModeType == DISTANCE_GRAYSCALE_MODE )
		nameStr = "GRAYSCALE";	

	return nameStr;

}

/////////////////////////////////////// public function ///////////////////////////////////////////////////////////
void videoSource::getMouseEvent( int *mouse_xpos, int *mouse_ypos )
{
	*mouse_xpos = x_start;
	*mouse_ypos = y_start;
}


void videoSource::del_arg(int argc, char **argv, int index)
{
    int i;
    for(i = index; i < argc-1; ++i) argv[i] = argv[i+1];
    argv[i] = 0;
}

int videoSource::find_arg(int argc, char* argv[], char *arg)
{
    int i;
    for(i = 0; i < argc; ++i) {
        if(!argv[i]) continue;
        if(0==strcmp(argv[i], arg)) {
            del_arg(argc, argv, i);
            return 1;
        }
    }
    return 0;
}

int videoSource::find_int_arg(int argc, char **argv, const char *arg, int def)
{
    int i;

    for(i = 0; i < argc-1; ++i){
        if(!argv[i]) continue;
        if(0==strcmp(argv[i], arg)){
            def = atoi(argv[i+1]);
            del_arg(argc, argv, i);
            del_arg(argc, argv, i);
            break;
        }
    }
    return def;
}

float videoSource::find_float_arg(int argc, char **argv, const char *arg, float def)
{
    int i;
    for(i = 0; i < argc-1; ++i){
        if(!argv[i]) continue;
        if(0==strcmp(argv[i], arg)){
            def = (float)atof(argv[i+1]);
            del_arg(argc, argv, i);
            del_arg(argc, argv, i);
            break;
        }
    }
    return def;
}

const char *videoSource::find_char_arg(int argc, char **argv, const char *arg, const char *def)
{
    int i;
    for(i = 0; i < argc-1; ++i){
        if(!argv[i]) continue;
        if(0==strcmp(argv[i], arg)){
            def = argv[i+1];
            del_arg(argc, argv, i);
            del_arg(argc, argv, i);
            break;
        }
    }
    return def;
}


void videoSource::mouse_click_func(int event, int x, int y)
{
    if (event == cv::EVENT_LBUTTONDOWN)
    {
        x_start = x;
        y_start = y;
    }
    else if (event == cv::EVENT_LBUTTONUP)
    {
    }
    else if (event == cv::EVENT_MOUSEMOVE)
    {
    }
}




void videoSource::initDeepLearning( CaptureOptions *pAppCfg )
{
#ifdef SUPPORT_DEEPLEARNING
#ifdef _WINDOWS
	cv::String ssd_model = "..\\..\\..\\deepLearning\\frozen_inference_graph.pb";
	cv::String ssd_config = "..\\..\\..\\deepLearning\\ssd_mobilenet_v2.pbtxt";
#else
	cv::String ssd_model = "../../deepLearning/frozen_inference_graph.pb";
	cv::String ssd_config = "../../deepLearning/ssd_mobilenet_v2.pbtxt";
#endif

	struct stat buffer;   
    if(stat (ssd_model.c_str(), &buffer) != 0 || stat (ssd_config.c_str(), &buffer) != 0 ){
		printf("mismatch deepLearning weight files !!!\n");
		return;
    }

	dnnNet = cv::dnn::readNetFromTensorflow(ssd_model, ssd_config);
#ifdef HAVE_CV_CUDA
	dnnNet.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
	dnnNet.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
#else
	dnnNet.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
	dnnNet.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
#endif

	conf_threshold = pAppCfg->detectThreshold;
	printf("THRESHOLD::conf_threshold = %.1f\n", conf_threshold);
#endif	
}


void videoSource::deepLearning( cv::Mat imageLidar )
{
#ifdef SUPPORT_DEEPLEARNING
	if( dnnNet.empty() ) return;
	
	cv::Mat blob = cv::dnn::blobFromImage(imageLidar, 1.0, cv::Size(300, 300), true, false);
	dnnNet.setInput(blob);

	cv::Mat res = dnnNet.forward();
	cv::Mat detect(res.size[2], res.size[3], CV_32FC1, res.ptr<float>());

	for (int i = 0; i < detect.rows; i++) {
		float confidence = detect.at<float>(i, 2);

		if (confidence >= conf_threshold && detect.at<float>(i, 1) == 1.0 ){
			int x1_90d = cvRound(detect.at<float>(i, 3) * imageLidar.cols);
			int y1_90d = cvRound(detect.at<float>(i, 4) * imageLidar.rows);
			int x2_90d = cvRound(detect.at<float>(i, 5) * imageLidar.cols);
			int y2_90d = cvRound(detect.at<float>(i, 6) * imageLidar.rows);

			cv::rectangle(imageLidar, cv::Rect(cv::Point(x1_90d, y1_90d), cv::Point(x2_90d, y2_90d)), cv::Scalar(0, 255, 0), 2);
			std::string label = cv::format("%2.0f:%4.3f", detect.at<float>(i, 1), confidence);
			cv::putText(imageLidar, label, cv::Point(x1_90d, y1_90d - 1), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0));
		}
	}
#endif
}




void videoSource::drawCaption(cv::Mat grayMat, cv::Mat distMat, CaptureOptions *appCfg)
{
	int mouse_xpos, mouse_ypos;
	getMouseEvent(&mouse_xpos, &mouse_ypos);

//	printf("grayMat.row = %d col = %d\n", grayMat.rows, grayMat.cols);

#if 1
	cv::Mat drawMat;
#ifdef HAVE_CV_CUDA
	cv::cuda::GpuMat gpuGrayImage(grayMat), gpuDistImage(distMat), gpuHconcat (grayMat.rows, grayMat.cols * 2, grayMat.type());
	
	gpuGrayImage.copyTo(gpuHconcat(cv::Rect(0,0,gpuGrayImage.cols, gpuGrayImage.rows)));
	gpuDistImage.copyTo(gpuHconcat(cv::Rect(gpuGrayImage.cols, 0,gpuDistImage.cols, gpuDistImage.rows)));

	gpuHconcat.download(drawMat);
#else
	cv::hconcat(grayMat, distMat, drawMat);
#endif

	cv::Mat viewInfoUpper(VIEW_INFO_UPPER_SIZE, grayMat.cols * 2, CV_8UC3, cv::Scalar(0,0,0));
	cv::Mat viewInfoLower(VIEW_INFO_LOWER_SIZE, grayMat.cols * 2, CV_8UC3, cv::Scalar(255,255,255));

	cv::Mat viewInfo(VIEW_INFO_Y_SIZE, grayMat.cols * 2, CV_8UC3, cv::Scalar(0,0,0));
	std::string dist_caption;
	std::string defaultInfoTitle;

	int width_div = getWidthDiv();
	int height_div = getHeightDiv();

	defaultInfoTitle = cv::format("NANOSYSTEMS NSL-3130AA Viewer");
	
	std::string defaultInfoLower = cv::format("Nanosystems. co.,Ltd.\u00402022");

	cv::line(drawMat, cv::Point(0, 0), cv::Point(0,10), cv::Scalar(0, 255, 255), 1);
	cv::line(drawMat, cv::Point(0, 0), cv::Point(10,0), cv::Scalar(0, 255, 255), 1);
	
	cv::line(drawMat, cv::Point(grayMat.cols, 0), cv::Point(grayMat.cols,10), cv::Scalar(0, 255, 255), 1);
	cv::line(drawMat, cv::Point(grayMat.cols, 0), cv::Point(grayMat.cols+10,0), cv::Scalar(0, 255, 255), 1);


	if( mouse_xpos >= 0 && mouse_ypos >= VIEW_INFO_UPPER_SIZE && mouse_ypos < grayMat.rows + VIEW_INFO_UPPER_SIZE )
	{
		int tofcam_XPos;
		int tofcam_YPos;

		mouse_ypos -= VIEW_INFO_UPPER_SIZE;

		if( mouse_xpos < getWidth() )
		{
			int x_limit_left = mouse_xpos >= 10 ? 10 : mouse_xpos;
			int x_limit_right = mouse_xpos <= (getWidth()-10) ? 10 : getWidth()-mouse_xpos;
			
			int y_limit_left = mouse_ypos >= 10 ? 10 : mouse_ypos;
			int y_limit_right = mouse_ypos <= (getHeight()-10) ? 10 : getHeight()-mouse_ypos;

//				printf("x = %d, %d :: y = %d, %d\n", x_limit_left, x_limit_right, y_limit_left, y_limit_right);
			
			cv::line(drawMat, cv::Point(mouse_xpos-x_limit_left, mouse_ypos), cv::Point(mouse_xpos+x_limit_right, mouse_ypos), cv::Scalar(255, 255, 0), 2);
			cv::line(drawMat, cv::Point(mouse_xpos, mouse_ypos-y_limit_left), cv::Point(mouse_xpos, mouse_ypos+y_limit_right), cv::Scalar(255, 255, 0), 2);
			
			tofcam_XPos = mouse_xpos/width_div;				
		}
		else{

			int x_limit_left = mouse_xpos >= getWidth()+10 ? 10 : mouse_xpos-getWidth();
			int x_limit_right = mouse_xpos <= getWidth()+(getWidth()-10) ? 10 : (getWidth()*2)-mouse_xpos;
			
			int y_limit_left = mouse_ypos >= 10 ? 10 : mouse_ypos;
			int y_limit_right = mouse_ypos <= (getHeight()-10) ? 10 : getHeight()-mouse_ypos;
			
//				printf("x = %d, %d :: y = %d, %d\n", x_limit_left, x_limit_right, y_limit_left, y_limit_right);

			cv::line(drawMat, cv::Point(mouse_xpos-x_limit_left, mouse_ypos), cv::Point(mouse_xpos+x_limit_right, mouse_ypos), cv::Scalar(255, 255, 0), 2);
			cv::line(drawMat, cv::Point(mouse_xpos, mouse_ypos-y_limit_left), cv::Point(mouse_xpos, mouse_ypos+y_limit_right), cv::Scalar(255, 255, 0), 2);

			tofcam_XPos = (mouse_xpos-getWidth())/width_div;
		}

		tofcam_YPos = mouse_ypos/height_div;

//		int dist_pos = tofcam_YPos*NSL_LIDAR_TYPE_A_WIDTH + tofcam_XPos;
		dist_caption = cv::format("X:%d, Y:%d, %s", tofcam_XPos, tofcam_YPos, getDistanceString(pcdData.distance2D[tofcam_YPos][tofcam_XPos]).c_str());
	}
	else if( distStringCap.length() > 0 ){
		dist_caption = distStringCap;
	}
	else if( nonDistStringCap.length() > 0 ){
		dist_caption = nonDistStringCap;
	}

	std::string defaultInfoCap2;
	std::string defaultInfoCap3;
	std::string defaultInfoCap1 = cv::format("<%s>                           <Distance>", getLeftViewName().c_str());

	
	defaultInfoCap2 = cv::format("position       :     %s", dist_caption.c_str());
	defaultInfoCap3 = cv::format("frame rate    :     %d fps", appCfg->displayFps);

	putText(viewInfoUpper, defaultInfoTitle.c_str(), cv::Point(340, 35), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 255, 255));
	putText(viewInfoLower, defaultInfoLower.c_str(), cv::Point(780, 26), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 0));

	putText(viewInfo, defaultInfoCap1.c_str(), cv::Point(245, 35), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 255, 255));
	putText(viewInfo, defaultInfoCap2.c_str(), cv::Point(90, 90), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 255, 255));
	putText(viewInfo, defaultInfoCap3.c_str(), cv::Point(90, 125), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 255, 255));
	
#ifdef HAVE_CV_CUDA
	cv::cuda::GpuMat gpuUpper(viewInfoUpper), gpuView(viewInfo), gpuLower(viewInfoLower), gpuImage(drawMat), gpuVconcat(drawMat.rows+viewInfoUpper.rows+viewInfoLower.rows+viewInfo.rows, drawMat.cols, drawMat.type());

	gpuUpper.copyTo(gpuVconcat(cv::Rect(0, 0, gpuUpper.cols, gpuUpper.rows)));
	gpuImage.copyTo(gpuVconcat(cv::Rect(0, gpuUpper.rows,gpuImage.cols, gpuImage.rows)));
	gpuView.copyTo(gpuVconcat(cv::Rect(0, gpuUpper.rows+gpuImage.rows, gpuView.cols, gpuView.rows)));
	gpuLower.copyTo(gpuVconcat(cv::Rect(0, gpuUpper.rows+gpuImage.rows+gpuView.rows, gpuLower.cols, gpuLower.rows)));
	gpuVconcat.download(drawMat);
#else
	cv::vconcat(viewInfoUpper, drawMat, drawMat);
	cv::vconcat(drawMat, viewInfo, drawMat);
	cv::vconcat(drawMat, viewInfoLower, drawMat);
#endif

    cv::imshow(WIN_NAME, drawMat);
#endif

	std::chrono::steady_clock::time_point curTime = std::chrono::steady_clock::now();
	double frame_time = (curTime - frameTime).count() / 1000000.0;
	double fps_time = (curTime - fpsTime).count() / 1000000.0;

	appCfg->fpsCount++;

	if( fps_time >= 1000.0f ){
		appCfg->displayFps = appCfg->fpsCount;

		printf("sample %d fps time = %.3f/%.3f\n", appCfg->displayFps, frame_time, fps_time);
		appCfg->fpsCount = 0;
		fpsTime = curTime;
	}

	return;
}



void videoSource::stopLidar()
{
	printf("stop Lidar\n");
	nsl_streamingOff(handle);
}

void videoSource::setMatrixColor(Mat image, int x, int y, NslVec3b color)
{
	image.at<Vec3b>(y,x)[0] = color.b;
	image.at<Vec3b>(y,x)[1] = color.g;
	image.at<Vec3b>(y,x)[2] = color.r;
}

bool videoSource::captureLidar( int timeout, CaptureOptions *pAppCfg )
{
	frameTime = std::chrono::steady_clock::now();
	bool ret = false;
	
	if( nsl_getPointCloudData(handle, &pcdData, timeout) == NSL_ERROR_TYPE::NSL_SUCCESS )
	{
		ret = true;

		if( pcdData.includeLidar )
		{
			int width = pcdData.width;
			int height = pcdData.height;
			int xMin = pcdData.roiXMin;
			int yMin = pcdData.roiYMin;
			int lidarWidth = NSL_LIDAR_TYPE_A_WIDTH;
			int lidarHeight = NSL_LIDAR_TYPE_A_HEIGHT;
		
			Mat distanceMat = Mat(lidarHeight, lidarWidth, CV_8UC3, Scalar(255,255,255));
			Mat amplitudeMat = Mat(lidarHeight, lidarWidth, CV_8UC3, Scalar(255,255,255));
		
			for(int y = 0, index = 0; y < height; y++)
			{
				for(int x = 0; x < width; x++, index++)
				{
					setMatrixColor(distanceMat, x+xMin, y+yMin, nsl_getDistanceColor(pcdData.distance2D[y+yMin][x+xMin]));
					setMatrixColor(amplitudeMat, x+xMin, y+yMin, nsl_getAmplitudeColor(pcdData.amplitude[y+yMin][x+xMin]));
				}
			}
					
			cv::resize( distanceMat, pAppCfg->distMat, cv::Size( 640, 480 ) );
			cv::resize( amplitudeMat, pAppCfg->frameMat, cv::Size( 640, 480 ));
		}
	}

	return ret;
}


void videoSource::setLidarOption(void *pCapOption)
{
	CaptureOptions *pCapOpt = (CaptureOptions *)pCapOption;

	beginTime = std::clock();
	
	int maxDistanceValue = (pCapOpt->maxDistance <= 0 || pCapOpt->maxDistance > MAX_DISTANCE_12MHZ) ? MAX_DISTANCE_12MHZ : pCapOpt->maxDistance;

	pCapOpt->maxDistance = maxDistanceValue;

	nsl_setIntegrationTime(handle, pCapOpt->integrationTime, 200, 0, pCapOpt->grayIntegrationTime);
	nsl_setMinAmplitude(handle, pCapOpt->minAmplitude);

	nsl_setFilter(handle, pCapOpt->medianFilterEnable ? FUNCTION_OPTIONS::FUNC_ON : FUNCTION_OPTIONS::FUNC_OFF
						, pCapOpt->averageFilterEnable ? FUNCTION_OPTIONS::FUNC_ON: FUNCTION_OPTIONS::FUNC_OFF
						, pCapOpt->temporalFilterFactorActual, pCapOpt->temporalFilterThreshold
						, pCapOpt->edgeThresHold
						, pCapOpt->interferenceLimit, pCapOpt->interferenceUseLashValueEnable ? FUNCTION_OPTIONS::FUNC_ON : FUNCTION_OPTIONS::FUNC_OFF);


	if( pCapOpt->captureType == (int)OPERATION_MODE_OPTIONS::DISTANCE_GRAYSCALE_MODE ){
		nsl_setGrayscaleillumination(handle, FUNCTION_OPTIONS::FUNC_ON);
	}
	else{
		nsl_setGrayscaleillumination(handle, FUNCTION_OPTIONS::FUNC_OFF);
	}

	nsl_setColorRange(pCapOpt->maxDistance, MAX_GRAYSCALE_VALUE, NslOption::FUNCTION_OPTIONS::FUNC_ON);
	nsl_streamingOn(handle, static_cast<OPERATION_MODE_OPTIONS>(pCapOpt->captureType));
}


int videoSource::prockey(CaptureOptions *appCfg)
{
	int key = cv::waitKey(1);

//	printf("prockey key = %d\n", key);
	switch(key)
	{
		case 'b':
		case 'B':
			// black & white(grayscle) mode
			nsl_setColorRange(appCfg->maxDistance, MAX_GRAYSCALE_VALUE, NslOption::FUNCTION_OPTIONS::FUNC_ON);
			if( appCfg->captureType != (int)OPERATION_MODE_OPTIONS::GRAYSCALE_MODE ){
				appCfg->captureType = (int)OPERATION_MODE_OPTIONS::GRAYSCALE_MODE;
				nsl_streamingOn(handle, static_cast<OPERATION_MODE_OPTIONS>(appCfg->captureType));
			}
			
			break;
		case '0':
			// HDR Off
			nsl_setHdrMode(handle, HDR_OPTIONS::HDR_NONE_MODE);
			break;
		case '1':
			// HDR spatial
			nsl_setHdrMode(handle, HDR_OPTIONS::HDR_SPATIAL_MODE);
			break;
		case '2':
			// HDR temporal
			nsl_setHdrMode(handle, HDR_OPTIONS::HDR_TEMPORAL_MODE);
			break;
		case 'd':
		case 'D':
			//graysca & distance mode
			nsl_setColorRange(appCfg->maxDistance, MAX_GRAYSCALE_VALUE, NslOption::FUNCTION_OPTIONS::FUNC_OFF);
			if( appCfg->captureType != (int)OPERATION_MODE_OPTIONS::DISTANCE_GRAYSCALE_MODE ){
				appCfg->captureType = (int)OPERATION_MODE_OPTIONS::DISTANCE_GRAYSCALE_MODE;
				nsl_streamingOn(handle, static_cast<OPERATION_MODE_OPTIONS>(appCfg->captureType));
			}
			break;
		case 'a':
		case 'A':
			// amplitude & distance mode
			nsl_setColorRange(appCfg->maxDistance, MAX_GRAYSCALE_VALUE, NslOption::FUNCTION_OPTIONS::FUNC_OFF);

			if( appCfg->captureType != (int)OPERATION_MODE_OPTIONS::DISTANCE_AMPLITUDE_MODE ){
				appCfg->captureType = (int)OPERATION_MODE_OPTIONS::DISTANCE_AMPLITUDE_MODE;
				nsl_streamingOn(handle, static_cast<OPERATION_MODE_OPTIONS>(appCfg->captureType));
			}
			break;
		case 'e':
		case 'E':
			// amplitude(Gray) & distance mode
			nsl_setColorRange(appCfg->maxDistance, MAX_GRAYSCALE_VALUE, NslOption::FUNCTION_OPTIONS::FUNC_ON);
			break;
		case 't':
		case 'T':
		{
			nsl_getGrayscaleillumination(handle, &appCfg->nslDevConfig.grayscaleIlluminationOpt);
			// grayscale LED On / Off
			if( appCfg->nslDevConfig.grayscaleIlluminationOpt != NslOption::FUNCTION_OPTIONS::FUNC_ON )
				nsl_setGrayscaleillumination(handle, NslOption::FUNCTION_OPTIONS::FUNC_ON);
			else
				nsl_setGrayscaleillumination(handle, NslOption::FUNCTION_OPTIONS::FUNC_OFF);
			break;
		}
		case 's':
		case 'S':
			//saturation enable/disable
			nsl_getAdcOverflowSaturation(handle, &appCfg->nslDevConfig.overflowOpt, &appCfg->nslDevConfig.saturationOpt);

			if( appCfg->nslDevConfig.saturationOpt != NslOption::FUNCTION_OPTIONS::FUNC_ON )
				nsl_setAdcOverflowSaturation(handle, appCfg->nslDevConfig.overflowOpt, NslOption::FUNCTION_OPTIONS::FUNC_ON);
			else
				nsl_setAdcOverflowSaturation(handle, appCfg->nslDevConfig.overflowOpt, NslOption::FUNCTION_OPTIONS::FUNC_OFF);
				
			break;
		case 'f':
		case 'F':
			//overflow enable/disable
			nsl_getAdcOverflowSaturation(handle, &appCfg->nslDevConfig.overflowOpt, &appCfg->nslDevConfig.saturationOpt);

			if( appCfg->nslDevConfig.overflowOpt != NslOption::FUNCTION_OPTIONS::FUNC_ON )
				nsl_setAdcOverflowSaturation(handle, NslOption::FUNCTION_OPTIONS::FUNC_ON, appCfg->nslDevConfig.saturationOpt);
			else
				nsl_setAdcOverflowSaturation(handle, NslOption::FUNCTION_OPTIONS::FUNC_OFF, appCfg->nslDevConfig.saturationOpt);
			break;
		case 'h':
		case 'H':
			//Help
			printf("-----------------------------------------------\n");
			printf("b key : change GRAYSCALE mode\n");
			printf("d key : change DISTANCE & Grayscale mode\n");
			printf("a key : change AMPLITUDE & DISTANCE mode\n");
			printf("e key : change AMPLITUDE(log) & DISTANCE mode\n");
			printf("t key : change Grayscale LED\n");
			printf("s key : change saturation on/off\n");
			printf("f key : change overflow on/off\n");
			printf("0 key : change HDR off\n");
			printf("1 key : change HDR Spatial\n");
			printf("2 key : change HDR Temporal\n");
			printf("-----------------------------------------------\n");
			break;			
	}

	return key;
}


