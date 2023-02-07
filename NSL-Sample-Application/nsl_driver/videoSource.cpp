
#include <memory>
#include <thread>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

//#include "class_detector.h"
#include "videoSource.h"
#include "NSL3130AA.h"
#include "NSL1110AA.h"
//#include "jetsonGpio.h"

#ifdef HAVE_CV_CUDA
#include <opencv2/cudawarping.hpp>
#endif

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


// constructor
videoSource::videoSource()
{
	printf("new videoSource()\n");

	fpsTime = std::chrono::steady_clock::now();
}

videoSource::~videoSource()
{
	printf("~videoSource()\n");
}

/////////////////////////////////////// static function ///////////////////////////////////////////////////////////

/*
# DEVICE_TYPE : 0 : NSL-1110AA, 1 : NSL-3130AA
# ETH_TYPE : 0 : ethernet , 1 : USB
# NET_TYPE : 0 : SSD-MobilenetV2, 1 : YoloV3, 2 : YoloV4, 3 : YoloV5-TRT
# INPUT_SIZE : 0 : 320, 1 : 416

# maxDetectingCnt : 1, 2 ... 10
# showGuide : 0=disable, 1=enable (detector/discard_area.txt)
# showConvertColor : 0=disable, 1=enable (detector/convert_area.txt)
# showMini : 0=disable, 1=enable (show detected mini box)
# resetTime : 3600 x 10 : 10 hours
# captureType : 0 ~ 2
#  0	AMPLITEDE_DISTANCE_MODE
#  1	AMPLITEDE_DISTANCE_EX_MODE
#  2	DISTANCE_GRAYSCALE_MODE
# detectDistance : 0 ~ 12500
# maxDistance : 0 ~ 12500
# intTime : 50 ~ 4000
# grayintTime : 50 ~ 40000
# threshold : 0.1 ~ 0.9
# overlay : 0=disable, 1=enable
# amplitudeMin : 30 ~ 1000
# edgeThresHold : 0=disable, 1~10000
# medianFilter : 0=disable, 1~99
# medianIter : 0=disable, 1~10000
# gaussIter :  0=disable, 1~10000
*/

videoSource *videoSource::initAppCfg(int argc, char **argv, CaptureOptions *pAppCfg)
{
	memset(pAppCfg, 0, sizeof(CaptureOptions));
	
	pAppCfg->minConfidence= 100.0;
	pAppCfg->maxConfidence = 0;
	
	char *ipAddr = find_char_arg(argc, argv, "ipaddr", "192.168.0.220");	

	pAppCfg->lidarType = find_int_arg(argc, argv, "-nslType", NSL3130_TYPE); // 0 : NSL1110AA, 1 : NSL3130AA
	pAppCfg->inputSize = find_int_arg(argc, argv, "-inputSize", 0); // 0 : 320, 1 : 416
	pAppCfg->detectThreshold = find_float_arg(argc, argv, "-thresh", .4);

	pAppCfg->captureType = find_int_arg(argc, argv, "-captureType", 1);//1;
	pAppCfg->integrationTime = find_int_arg(argc, argv, "-intTime", 800);//800;
	pAppCfg->grayIntegrationTime = find_int_arg(argc, argv, "-grayintTime", 100);//800;
	pAppCfg->maxDistance = find_int_arg(argc, argv, "-maxDistance", 12500);
	pAppCfg->minAmplitude = find_int_arg(argc, argv, "-amplitudeMin", 50);

	pAppCfg->edgeThresHold = find_int_arg(argc, argv, "-edgeThresHold", 0);
	pAppCfg->medianFilterSize = find_int_arg(argc, argv, "-medianFilterSize", 0);
	pAppCfg->medianFilterIterations = find_int_arg(argc, argv, "-medianIter", 0);
	pAppCfg->gaussIteration = find_int_arg(argc, argv, "-gaussIter", 0);

	pAppCfg->medianFilterEnable = find_int_arg(argc, argv, "-medianFilterEnable", 0);
	pAppCfg->averageFilterEnable = find_int_arg(argc, argv, "-averageFilterEnable", 0);
	pAppCfg->temporalFilterFactorActual = find_int_arg(argc, argv, "-temporalFactor", 0);
	pAppCfg->temporalFilterThreshold = find_int_arg(argc, argv, "-temporalThresHold", 0);
	pAppCfg->interferenceUseLashValueEnable = find_int_arg(argc, argv, "-interferenceEnable", 0);
	pAppCfg->interferenceLimit = find_int_arg(argc, argv, "-interferenceLimit", 0);

	if( pAppCfg->inputSize == 0 ) pAppCfg->inputSize = 320;
	else if( pAppCfg->inputSize == 1 ) pAppCfg->inputSize = 416;

	if( pAppCfg->captureType < 0 || pAppCfg->captureType > 2 ) pAppCfg->captureType = 1;
	if( pAppCfg->maxDistance == 0 ) pAppCfg->maxDistance = 12500;
	if( pAppCfg->integrationTime == 0 ) pAppCfg->integrationTime = 800;
	if( pAppCfg->grayIntegrationTime == 0 ) pAppCfg->grayIntegrationTime = 100;
	if( pAppCfg->minAmplitude == 0 ) pAppCfg->minAmplitude = 50;
	if( pAppCfg->medianFilterSize < 0 || pAppCfg->medianFilterSize > 99 ) pAppCfg->medianFilterSize = 0;
	if( pAppCfg->medianFilterIterations < 0 || pAppCfg->medianFilterIterations > 10000 ) pAppCfg->medianFilterIterations = 0;
	if( pAppCfg->gaussIteration < 0 || pAppCfg->gaussIteration > 10000 ) pAppCfg->gaussIteration = 0;
	if( pAppCfg->edgeThresHold < 0 || pAppCfg->edgeThresHold > 10000 ) pAppCfg->edgeThresHold = 0;
	
	return videoSource::Create(pAppCfg->lidarType, ipAddr);
}

videoSource* videoSource::Create( int type, char *ipaddr )
{
	videoSource* src = NULL;

	if( type == 0 ){
		src = NSL1110AA::Create(ipaddr);
	}
	else if( type == 1 ){
		src = NSL3130AA::Create(ipaddr);
	}

	return src;
}


void videoSource::callback_mouse_click(int event, int x, int y, int flags, void* user_data)
{
	videoSource* vidSrc = reinterpret_cast<videoSource*>(user_data);
	vidSrc->mouse_click_func(event, x, y);
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

int videoSource::find_int_arg(int argc, char **argv, char *arg, int def)
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

float videoSource::find_float_arg(int argc, char **argv, char *arg, float def)
{
    int i;
    for(i = 0; i < argc-1; ++i){
        if(!argv[i]) continue;
        if(0==strcmp(argv[i], arg)){
            def = atof(argv[i+1]);
            del_arg(argc, argv, i);
            del_arg(argc, argv, i);
            break;
        }
    }
    return def;
}

char *videoSource::find_char_arg(int argc, char **argv, char *arg, char *def)
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



/////////////////////////////////////// private function ///////////////////////////////////////////////////////////


void videoSource::getMouseEvent( int *mouse_xpos, int *mouse_ypos )
{
	*mouse_xpos = x_start;
	*mouse_ypos = y_start;
}

/////////////////////////////////////// public function ///////////////////////////////////////////////////////////
void videoSource::initDeepLearning( void )
{
#ifdef _WINDOWS
	cv::String ssd_model = "..\\deepLearning\\frozen_inference_graph.pb";
	cv::String ssd_config = "..\\deepLearning\\ssd_mobilenet_v2.pbtxt";
#else
	cv::String ssd_model = "../../deepLearning/frozen_inference_graph.pb";
	cv::String ssd_config = "../../deepLearning/ssd_mobilenet_v2.pbtxt";
#endif

	struct stat buffer;   
    if(stat (ssd_model.c_str(), &buffer) != 0 || stat (ssd_config.c_str(), &buffer) != 0 ){
		printf("not support ssd mobilenetV2\n");
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

	conf_threshold = 0.4f;
}


void videoSource::deepLearning( cv::Mat imageLidar )
{
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
}




void videoSource::drawCaption(cv::Mat grayMat, cv::Mat distMat, CaptureOptions *appCfg)
{
	int mouse_xpos, mouse_ypos;
	getMouseEvent(&mouse_xpos, &mouse_ypos);

//	printf("grayMat.row = %d col = %d\n", grayMat.rows, grayMat.cols);


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

	if( appCfg->lidarType == NSL3130_TYPE ){
		defaultInfoTitle = cv::format("NANOSYSTEMS NSL-3130AA Viewer");
	}
	else{
		defaultInfoTitle = cv::format("NANOSYSTEMS NSL-1110AA Viewer");
	}
	
	std::string defaultInfoLower = cv::format("Nanosystems. co.,Ltd.\u00402022");

	if( isRotate90() == false ){
		cv::line(drawMat, cv::Point(0, 0), cv::Point(0,10), cv::Scalar(0, 255, 255), 1);
		cv::line(drawMat, cv::Point(0, 0), cv::Point(10,0), cv::Scalar(0, 255, 255), 1);
		
		cv::line(drawMat, cv::Point(grayMat.cols, 0), cv::Point(grayMat.cols,10), cv::Scalar(0, 255, 255), 1);
		cv::line(drawMat, cv::Point(grayMat.cols, 0), cv::Point(grayMat.cols+10,0), cv::Scalar(0, 255, 255), 1);
	}
	else{
		cv::line(drawMat, cv::Point(grayMat.cols-1, 0), cv::Point(grayMat.cols-1,10), cv::Scalar(0, 255, 255), 1);
		cv::line(drawMat, cv::Point(grayMat.cols-1, 0), cv::Point(grayMat.cols-1-10, 0), cv::Scalar(0, 255, 255), 1);
		
		cv::line(drawMat, cv::Point(grayMat.cols*2-1, 0), cv::Point(grayMat.cols*2-1,10), cv::Scalar(0, 255, 255), 1);
		cv::line(drawMat, cv::Point(grayMat.cols*2-1, 0), cv::Point(grayMat.cols*2-1-10,0), cv::Scalar(0, 255, 255), 1);

	}


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
			
			if( isRotate90() == false )
				tofcam_XPos = mouse_xpos/width_div;
			else
				tofcam_YPos = (getWidth()-mouse_xpos)/width_div;
				
		}
		else{

			int x_limit_left = mouse_xpos >= getWidth()+10 ? 10 : mouse_xpos-getWidth();
			int x_limit_right = mouse_xpos <= getWidth()+(getWidth()-10) ? 10 : (getWidth()*2)-mouse_xpos;
			
			int y_limit_left = mouse_ypos >= 10 ? 10 : mouse_ypos;
			int y_limit_right = mouse_ypos <= (getHeight()-10) ? 10 : getHeight()-mouse_ypos;
			
//				printf("x = %d, %d :: y = %d, %d\n", x_limit_left, x_limit_right, y_limit_left, y_limit_right);

			cv::line(drawMat, cv::Point(mouse_xpos-x_limit_left, mouse_ypos), cv::Point(mouse_xpos+x_limit_right, mouse_ypos), cv::Scalar(255, 255, 0), 2);
			cv::line(drawMat, cv::Point(mouse_xpos, mouse_ypos-y_limit_left), cv::Point(mouse_xpos, mouse_ypos+y_limit_right), cv::Scalar(255, 255, 0), 2);

			if( isRotate90() == false )
				tofcam_XPos = (mouse_xpos-getWidth())/width_div;
			else
				tofcam_YPos = (getWidth()-(mouse_xpos-getWidth()))/width_div;
		}

		if( isRotate90() == false )
			tofcam_YPos = mouse_ypos/height_div;
		else
			tofcam_XPos = mouse_ypos/height_div;

		int dist_pos;
		
		if( isRotate90() == false ) dist_pos = tofcam_YPos*getVideoWidth() + tofcam_XPos;
		else dist_pos = tofcam_YPos*getVideoHeight() + tofcam_XPos;

		dist_caption = cv::format("X:%d, Y:%d, %s", tofcam_XPos, tofcam_YPos, getDistanceString(appCfg->pDistanceTable[dist_pos]).c_str());
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

	
	if( dist_caption.length() > 0 ){
		std::string detecTimeCap = cntStringCap.length() > 0 ? cntStringCap : timeStringCap;
		defaultInfoCap2 = cv::format("position       :     %s %s", detecTimeCap.c_str(), dist_caption.c_str());
		defaultInfoCap3 = cv::format("frame rate    :      %d fps", appCfg->displayFps);
	}
	else{
		defaultInfoCap2 = cv::format("position       :     %s", timeStringCap.c_str());
		defaultInfoCap3 = cv::format("frame rate    :      %d fps", appCfg->displayFps);
	}

	if( width_div == 2 ){
		putText(viewInfoUpper, defaultInfoTitle.c_str(), cv::Point(340, 35), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 255, 255));
		putText(viewInfoLower, defaultInfoLower.c_str(), cv::Point(780, 26), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 0));

		putText(viewInfo, defaultInfoCap1.c_str(), cv::Point(245, 35), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 255, 255));
		putText(viewInfo, defaultInfoCap2.c_str(), cv::Point(90, 90), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 255, 255));
		putText(viewInfo, defaultInfoCap3.c_str(), cv::Point(90, 125), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 255, 255));
	}
	else{
		defaultInfoCap1 = cv::format("<%s>             <Distance>", getLeftViewName().c_str());
		
		putText(viewInfoUpper, defaultInfoTitle.c_str(), cv::Point(170, 35), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(255, 255, 255));
		putText(viewInfoLower, defaultInfoLower.c_str(), cv::Point(390, 20), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0, 0, 0));

		putText(viewInfo, defaultInfoCap1.c_str(), cv::Point(100, 15), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(255, 255, 255));
		putText(viewInfo, defaultInfoCap2.c_str(), cv::Point(45, 90), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(255, 255, 255));
		putText(viewInfo, defaultInfoCap3.c_str(), cv::Point(45, 125), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(255, 255, 255));
	}
	
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
	drawPointCloud();

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
	closeLidar();
}


bool videoSource::captureLidar( int timeout, CaptureOptions *pAppCfg )
{
	ImageFrame *camImage = NULL;	
	frameTime = std::chrono::steady_clock::now();
	bool ret = Capture((void **)&camImage, timeout);

	if( ret ){
		pAppCfg->frameMat = camImage->frameMat;
		pAppCfg->distMat = camImage->distMat;
		pAppCfg->pDistanceTable = camImage->pDistanceTable;
	}


	return ret;
}


void videoSource::setLidarOption(int netType, void *pCapOpt)
{
	beginTime = std::clock();

	cv::namedWindow(WIN_NAME, cv::WINDOW_NORMAL);
//	cv::setWindowProperty(WIN_NAME, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);	
	cv::setMouseCallback(WIN_NAME, callback_mouse_click, this);

	startCaptureCommand(netType, pCapOpt);
}


int videoSource::prockey(CaptureOptions *appCfg)
{
	int key = cv::waitKey(10);

//	printf("prockey key = %d\n", key);
	switch(key)
	{
		case 'b':
		case 'B':
			// black & white(grayscle) mode
			setKey('b');
			break;
		case '0':
			// HDR Off
			setKey('0');
			break;
		case '1':
			// HDR spatial
			setKey('1');
			break;
		case '2':
			// HDR temporal
			setKey('2');
			break;
		case 'd':
		case 'D':
			//graysca & distance mode
			setKey('d');
			break;
		case 'a':
		case 'A':
			// amplitude & distance mode
			setKey('a');
			break;
		case 'e':
		case 'E':
			// amplitude(Gray) & distance mode
			setKey('e');
			break;
		case 't':
		case 'T':
			// grayscale LED On / Off
			setKey('t');
			break;
		case 'g':
		case 'G':
			//grayscale gain corrected
			setKey('g');
			break;
		case 'o':
		case 'O':
			//grayscale offset corrected
			setKey('o');
			break;
		case 'l':
		case 'L':
			//ambient light enable/disable
			setKey('l');
			break;
		case 's':
		case 'S':
			//saturation enable/disable
			setKey('s');
			break;
		case 'f':
		case 'F':
			//overflow enable/disable
			setKey('f');
			break;
		case 'h':
		case 'H':
			//Help
			setKey('h');
			break;
		case 'u':
		case 'U':
			//DRNU enable / disable
			setKey('u');
			break;
		case 'r':
		case 'R':
			//rotate 90
			setKey('r');
			break;
		case 'p':
		case 'P':
			// point-cloud
			setKey('p');
			break;
			
	}

	return key;
}


