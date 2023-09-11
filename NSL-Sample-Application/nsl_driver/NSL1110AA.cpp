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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <io.h>
#include <process.h>
#else
#include <sstream> 
#include <unistd.h>
#include <string.h>
#include <sys/socket.h>
#include <linux/socket.h>
#include <netinet/tcp.h>
	 
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <arpa/inet.h>
#include <sys/time.h> 
#endif

#include "NSL1110AA.h"
#include "NSLFrame.h"


using namespace cv;

#ifdef HAVE_CV_CUDA
#include <opencv2/cudawarping.hpp>
#endif

//#define MAINTAIN_SOCKET_STREAMING
//#define DME660_ROTATE_IMAGE_90
//#define ROTATE_IMAGE_ADJUST_ROI
//#define MODULATION_24MHZ


#define RESIZE_IMAGE_MAT

#define ENABLE_DRNU_TEMPERATURE

#define DEFAULT_HDR_MODE	HDR_NONE_MODE
//#define DEFAULT_HDR_MODE	HDR_SPATIAL_MODE
//#define DEFAULT_HDR_MODE	HDR_TEMPORAL_MODE

#define WAIT_FRAME_COUNT		1

#define ADD_TOFCAM_BUFF(TOFCAMBUFF, MAX_BUFF_CNT)		do{\
															if( TOFCAMBUFF.overflow != 0){\
																if(TOFCAMBUFF.tail_idx + 1 == MAX_BUFF_CNT)\
																	TOFCAMBUFF.tail_idx = 0;\
																else\
																	TOFCAMBUFF.tail_idx++;\
															}\
															if(TOFCAMBUFF.head_idx + 1 == MAX_BUFF_CNT)\
																TOFCAMBUFF.head_idx = 0;\
															else\
																TOFCAMBUFF.head_idx ++;\
															if( TOFCAMBUFF.tail_idx == TOFCAMBUFF.head_idx )\
																TOFCAMBUFF.overflow = 1;\
															else\
																TOFCAMBUFF.overflow = 0;\
														}while(0)

#define	POP_TOFCAM_BUFF(TOFCAMBUFF, MAX_BUFF_CNT)	do{\
														if(TOFCAMBUFF.tail_idx + 1 == MAX_BUFF_CNT)\
															TOFCAMBUFF.tail_idx = 0;\
														else\
															TOFCAMBUFF.tail_idx ++;\
														TOFCAMBUFF.overflow = 0;\
													}while(0)


#define GET_BUFF_CNT(TOFCAMBUFF, MAX_BUFF_CNT)		(( TOFCAMBUFF.head_idx > TOFCAMBUFF.tail_idx) ? (TOFCAMBUFF.head_idx - TOFCAMBUFF.tail_idx) : (TOFCAMBUFF.head_idx < TOFCAMBUFF.tail_idx) ? (MAX_BUFF_CNT + TOFCAMBUFF.head_idx - TOFCAMBUFF.tail_idx) : (TOFCAMBUFF.overflow != 0) ? MAX_BUFF_CNT : 0)


#define DEFAULT_ROI_XMIN	4 // 4
#define DEFAULT_ROI_XMAX	323 //323
#define DEFAULT_ROI_YMIN	6 //6
#define DEFAULT_ROI_YMAX	125 // 125	// Don't modify YMAX, fixed value

#define ADJUST_ROI_XMIN		4
#define ADJUST_ROI_XMAX		323
#define ADJUST_ROI_YMIN		68
#define ADJUST_ROI_YMAX		125


#ifdef MODULATION_24MHZ
static int maxAmplitudeValue = 2895;
static int maxDistanceValue = 6250;
#define MAX_DISTANCEVALUE 6250
#define MAX_DISTANCEVALUE_CM 625
#else
static int maxAmplitudeValue = 4095;
static int maxDistanceValue = 12500;
#define MAX_DISTANCEVALUE 12500
#define MAX_DISTANCEVALUE_CM 1250
#endif

static const double PI = 3.14159265358979323846264338328;

int maxPhase = MAX_PHASE;

int pixel_buff[2][320][240];
const char *initialcmd[]=
{
	"stopVideo\n",
	"version\n",
	"getIcVersion\n",
	"getModulationFrequencies\n",
	"getCalibrationTypeForFreqIdx 0\n",
	"getCalibrationTypeForFreqIdx 1\n",
	"getCalibrationTypeForFreqIdx 2\n",
	"getCalibrationTypeForFreqIdx 3\n",
	"getCalibrationTypeForFreqIdx 4\n",
	"getCalibrationTypeForFreqIdx 5\n",
	"getCalibrationTypeForFreqIdx 6\n",
	"correctAsymDCS 1\n",
	"correctAsymDCS 0\n",
	"nloopFilter 6\n",
	"nloopFilter 10\n",
	"isDRNUAvailable\n",
	"isGrayscaleCorrectionAvailable\n",
	"isDRNUAvailable\n",
	"correctDRNU 0\n",
	"correctTemperature 0\n",
	"correctAmbientLight 0",
	"setKalmanK 0.1000\n",
	"setKalmanK 0.1000\n",
	"setKalmanQ 2.0000\n",
	"setKalmanKdiff 0.10\n",
	"setKalmanThreshold 600\n",
	"setKalmanNumCheck 2\n",
#ifdef MODULATION_24MHZ
	"setModulationFrequency 0\n",
	"setKalmanThreshold 1200\n",
	"setKalmanThreshold2 1200\n",
#else			
	"setModulationFrequency 1\n",
	"setKalmanThreshold 600\n",
	"setKalmanThreshold2 600\n",
#endif	
	"setABS 0\n",
//	"enableDefaultOffset 0\n",
//	"setOffset 0\n",
//	"enableDefaultOffset 1\n",
	"selectMode -1\n",
	"selectMode 0\n",
#ifdef ENABLE_DRNU_TEMPERATURE	
	"setABS 1\n",
	"correctDRNU 2\n",
	"correctTemperature 2\n",
#endif	
//	"correctAmbientLight 1",
//	"correctGrayscaleGain 1\n",
//	"correctGrayscaleOffset 1\n",
	"isDRNUAvailable\n",
	"enableDefaultOffset 0\n",
//	"setOffset 0\n",
//	"enableDefaultOffset 1\n",
//	"enablePiDelay 0\n",
	"enablePiDelay 1\n",
	"enableDualMGX 0\n",	// enableDualMGX 1 <-> enableHDR 0
	
#if DEFAULT_HDR_MODE == HDR_SPATIAL_MODE
	"enableHDR 1\n",
#elif DEFAULT_HDR_MODE == HDR_TEMPORAL_MODE
	"enableHDR 2\n",
#else
	"enableHDR 0\n",
#endif
	"loadConfig 1\n",
	"setHysteresis 10\n",
//	"setMinAmplitude 50\n",
	"enableDefaultOffset 1\n",
	"getOffset\n",
	// integration time
//	"setIntegrationTime2D 30\n",
//	"setIntegrationTime2D 100\n",	//grayscale : integrationTimeGrayscale
//	"setIntegrationTime3D 800\n",	// distance & amplitude
//	"setIntegrationTime3D 1000\n",	// distance & amplitude
	"setIntegrationTime3DHDR 100\n",
//	"setIntegrationTime3DHDR2 3000\n",

	"setKalmanK 0.6000\n",
//	"enableSaturation 1\n",
//	"enableSaturation 0\n",
//	"enableAdcOverflow 1\n",
//	"enableAdcOverflow 0\n",
//	"setROI 4 323 6 125\n",
	"enableVerticalBinning 1\n",
	"setRowReduction 1\n",
	"enableVerticalBinning 0\n",
	"setRowReduction 0\n",
	"enableHorizontalBinning 1\n",
	"enableHorizontalBinning 0\n",
	"setRowReduction 1\n",
	"setRowReduction 0\n",
	"selectPolynomial 1 0\n",
	"selectPolynomial 4 0\n",
	"selectPolynomial 4 1\n",
	"selectPolynomial 4 0\n",
//	"setROI 4 323 6 125\n",
	"getAmbientLightFactorOrg 34\n",
};

int NSL1110AA::recvFromTcp(SOCKET sock, uint8_t *total_buf, int expectedRcvLen)
{
	int read_total_size = 0;
	int nbyte = 1;

	struct timeval timeout;    
	fd_set readfds;

	
	while( nbyte > 0 )
	{
		FD_ZERO(&readfds);
		FD_SET(sock, &readfds);
		timeout.tv_sec = 3;  
		timeout.tv_usec = 0;  
#ifdef _WINDOWS		
		int state = select(0, &readfds, NULL, NULL, &timeout);
#else
		int state = select(sock+1, &readfds, NULL, NULL, &timeout);
#endif
		if( state == 0 || state == -1 ){  //timeout , error
			printf("NSL1110AA no response state = %d\n", state);
			return 0;
		}

		nbyte = recv( sock, (char *)&total_buf[read_total_size], DME660_BUFF_SIZE, 0 );
		if( nbyte > 0 ){
			read_total_size += nbyte;
			if( expectedRcvLen > 0 && expectedRcvLen == read_total_size ) break;
		}
	}

//	printf("recv ret = %d/%d\n", read_total_size, nbyte);

	return read_total_size;
}


double NSL1110AA::getPhase(int distanceCM) 
{
    return  MAX_PHASE / (double)MAX_DISTANCEVALUE_CM * distanceCM;
}

void NSL1110AA::setGrayScaledColor(cv::Mat &imageLidar, int x, int y, int value, double end_range )
{
	if( value == DME660_LOW_AMPLITUDE )
	{
		imageLidar.at<Vec3b>(y, x)[0] = 0;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 0; 
	}
	else if (value == DME660_SATURATION)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 255;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 128; 
	}
	else if (value == DME660_ADC_OVERFLOW)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 169;
		imageLidar.at<Vec3b>(y, x)[1] = 14;
		imageLidar.at<Vec3b>(y, x)[2] = 255; 
	}
	else if(value == DME660_PIXEL_BADASYM)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 0;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 0; 
	}
	else if (value < 0)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 0;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 0; 
	}
	else if (value > end_range)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 255;
		imageLidar.at<Vec3b>(y, x)[1] = 255;
		imageLidar.at<Vec3b>(y, x)[2] = 255; 
	}
	else{
		int color = (int)(value * (255.0 /end_range));
		imageLidar.at<Vec3b>(y, x)[0] = color;
		imageLidar.at<Vec3b>(y, x)[1] = color;
		imageLidar.at<Vec3b>(y, x)[2] = color; 
	}

}


int NSL1110AA::setDistanceColor(cv::Mat &imageLidar, int x, int y, int value )
{
	if( value == DME660_LOW_AMPLITUDE )
	{
		imageLidar.at<Vec3b>(y, x)[0] = 0;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 0; 
	}
	else if (value == DME660_SATURATION)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 128;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 255; 
	}
	else if (value == DME660_ADC_OVERFLOW)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 255;
		imageLidar.at<Vec3b>(y, x)[1] = 14;
		imageLidar.at<Vec3b>(y, x)[2] = 169; 
	}
	else if(value == DME660_PIXEL_BADASYM)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 0;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 0; 
	}
	else if(value <= 0)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 0;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 0; 
//		imageLidar.at<Vec3b>(y, x) = colorVector.at(colorVector.size()-1);
	}
	else if (value > maxPhase)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 0;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 0; 
//		imageLidar.at<Vec3b>(y, x) = colorVector.at(0);
	}
	else{
		int index = (int)colorVector.size() - (value*(DME660_NUM_COLORS / maxPhase));
		if( index < 0 ){
			printf("error index = %d\n", index);
			index = (int)colorVector.size()-1;
		}
		else if( index >= (int)colorVector.size() ){
			index = 0;
		}
		
		imageLidar.at<Vec3b>(y, x) = colorVector.at(index);
	}

	return value;
}

void NSL1110AA::setAmplitudeColor(cv::Mat &imageLidar, int x, int y, int value )
{
	if( value == DME660_LOW_AMPLITUDE )
	{
		imageLidar.at<Vec3b>(y, x)[0] = 0;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 0; 
	}
	else if (value == DME660_SATURATION)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 255;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 128; 
	}
	else if (value == DME660_ADC_OVERFLOW)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 169;
		imageLidar.at<Vec3b>(y, x)[1] = 14;
		imageLidar.at<Vec3b>(y, x)[2] = 255; 
	}
	else if(value == DME660_PIXEL_BADASYM)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 0;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 0; 
	}
	else if(value == 0)
	{
		imageLidar.at<Vec3b>(y, x) = colorVector.at(0);
	}
	else if (value < 0)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 0;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 0; 
	}
	else if (value > maxAmplitudeValue)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 0;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 0; 
	}
	else{
		int index = value * (DME660_NUM_COLORS / maxAmplitudeValue);
		if( index < 0 ){
			printf("error index = %d\n", index);
			index = 0;
		}
		else if( index >= (int)colorVector.size() ){
			index = (int)colorVector.size()-1;
		}
		
		imageLidar.at<Vec3b>(y, x) = colorVector.at(index);
	}

}

void NSL1110AA::asymBadPixelReplacer(uint16_t *pMemDistance, uint16_t *pMemAmplitude, int x, int y, int targetAmp_LSB)
{
    int val_replacePixel_amp=0,val_pixelRight_amp=0, val_pixelDown_amp=0, val_pixelUp_amp=0,val_pixelLeft_amp=0;
    int val_replacePixel_dist=0,val_pixelRight_dist=0, val_pixelDown_dist=0, val_pixelUp_dist=0,val_pixelLeft_dist=0;
    int diffToCalibAmp_LSB;
    int diff_LSB=0;
    int width = tofcamInfo.roiXMax-tofcamInfo.roiXMin+1;//DME660_IMAGE_WIDTH;

	if( pMemAmplitude[y*width+x] == DME660_PIXEL_BADASYM )
	{
		val_pixelRight_amp	= pMemAmplitude[y*width+x+1];
		val_pixelDown_amp = pMemAmplitude[(y+1)*width+x];
		val_pixelUp_amp  = pMemAmplitude[(y-1)*width+x];
		val_pixelLeft_amp = pMemAmplitude[y*width+x-1];
		
		val_pixelRight_dist  = pMemDistance[y*width+x+1];
		val_pixelDown_dist = pMemDistance[(y+1)*width+x];
		val_pixelUp_dist  = pMemDistance[(y-1)*width+x];
		val_pixelLeft_dist = pMemDistance[y*width+x-1];
		
		diffToCalibAmp_LSB=maxAmplitudeValue;		//MAX_AMPLITUDE=2895
		diff_LSB=abs(val_pixelRight_amp-targetAmp_LSB);
		if(diff_LSB<diffToCalibAmp_LSB){
			diffToCalibAmp_LSB=diff_LSB;
			val_replacePixel_amp=val_pixelRight_amp;
			val_replacePixel_dist=val_pixelRight_dist;
		}
		diff_LSB=abs(val_pixelDown_amp-targetAmp_LSB);
		if(diff_LSB<diffToCalibAmp_LSB){
			diffToCalibAmp_LSB=diff_LSB;
			val_replacePixel_amp=val_pixelDown_amp;
			val_replacePixel_dist=val_pixelDown_dist;
		}
		diff_LSB=abs(val_pixelUp_amp-targetAmp_LSB);
		if(diff_LSB<diffToCalibAmp_LSB){
			diffToCalibAmp_LSB=diff_LSB;
			val_replacePixel_amp=val_pixelUp_amp;
			val_replacePixel_dist=val_pixelUp_dist;
		}
		diff_LSB=abs(val_pixelLeft_amp-targetAmp_LSB);
		if(diff_LSB<diffToCalibAmp_LSB){
			diffToCalibAmp_LSB=diff_LSB;
			val_replacePixel_amp=val_pixelLeft_amp;
			val_replacePixel_dist=val_pixelLeft_dist;
		}
		
		if(val_replacePixel_amp!=maxAmplitudeValue){
			pMemAmplitude[y*width+x] = val_replacePixel_amp;
			pMemDistance[y*width+x] = val_replacePixel_dist;
		}
	}
}

void NSL1110AA::getComparedHDRAmplitudeImage(uint16_t *pMemDistance, uint16_t *pMemAmplitude, int xpos, int ypos)
{
    int width = tofcamInfo.roiXMax-tofcamInfo.roiXMin+1;//DME660_IMAGE_WIDTH;
    int height = DME660_IMAGE_HEIGHT;
    int highAmpThreshold = 2000;

	for(int x = 0 ; x < width ; x++){
		for( int y = 0; y < height; y+=2) {

		    int amp1 = pMemAmplitude[y*width+x]; //amp[i][j]
		    int amp2 = pMemAmplitude[(y+1)*width+x]; //amp[i][j+1];
		    int ampCase = 1;
			
		    if(amp1 < highAmpThreshold && amp2 < highAmpThreshold){ // both amplitudes in range
		        if(amp2 > amp1) ampCase = 2;
		        else            ampCase = 1;
		    }else if(amp1 >= highAmpThreshold && amp2 <= highAmpThreshold){ //one amplitude in range or both amplitudes LOW_AMPLITUDE
		        ampCase = 2;
		    }else if(amp1 <= highAmpThreshold && amp2 >= highAmpThreshold){ //one amplitude in range or both amplitudes LOW_AMPLITUDE
		        ampCase = 1;
		    }else{ //both amplitudes out of range
		        if(amp2 < amp1) ampCase = 1;
		        else            ampCase = 2;
		    }

//		    if(/*ampCase == 1 &&*/ y >= 120){
			if(ampCase == 1 ){
				pMemDistance[(y+1)*width+x] = pMemDistance[y*width+x]; // dist[i][j+1] = dist[i][j];
				pMemAmplitude[(y+1)*width+x] = pMemAmplitude[y*width+x]; // amp[i][j+1] = amp[i][j];
		    }else{
				pMemDistance[y*width+x] = pMemDistance[(y+1)*width+x]; // dist[i][j] = dist[i][j+1];
				pMemAmplitude[y*width+x] = pMemAmplitude[(y+1)*width+x]; // amp[i][j] = amp[i][j+1];
		    }
		}
	}
}

void NSL1110AA::edgeFilterPixels(uint16_t *pMemDistance, uint16_t *pMemAmplitude)
{
    if(tofcamInfo.bUsedEdgeFilter)
	{
		int width = tofcamInfo.roiXMax-tofcamInfo.roiXMin+1;//DME660_IMAGE_WIDTH;
		int height = (tofcamInfo.roiYMax-tofcamInfo.roiYMin+1)*2;//DME660_IMAGE_HEIGHT;
		int maxEdgeAmpDiff = (int)(getPhase(tofcamInfo.edgeThresHold)/10+0.5);
        int minPhase = DME660_LOW_AMPLITUDE;
        int diffX = 0;
        int diffY = 0;
        int edgeXdetect = 0;
        int edgeYdetect = 0;

		printf("edgeThresHold = %d maxEdgeAmpDiff = %d\n", tofcamInfo.edgeThresHold, maxEdgeAmpDiff);

        maxEdgeAmpDiff = (int)(maxEdgeAmpDiff  * ((double)(12000000.0) /(double)24000000.0));

        for(int i= 0; i<width; i++){
            for(int j= 0; j<height; j++){
				diffY = abs(pMemDistance[j*width+i]-pMemDistance[(j+1)*width+i]);
                if(maxEdgeAmpDiff < diffY){
                   edgeYdetect++;
                }else{
                    edgeYdetect=0;
                }

                if((maxEdgeAmpDiff < diffY) && edgeYdetect!=1){
					pMemDistance[j*width+i] = minPhase;
                }
            }
        }

        for(int j= 0; j<height; j++){
            for(int i= 0; i<width; i++){
				diffX = abs(pMemDistance[j*width+i]-pMemDistance[j*width+i+1]);

                if(maxEdgeAmpDiff < diffX){
					edgeXdetect++;
                }else{
					edgeXdetect=0;
                }

                if((maxEdgeAmpDiff < diffX) && edgeXdetect!=1){
					pMemDistance[j*width+i] = minPhase;
                }
            }
        }
    }
}



void NSL1110AA::medianFilter(uint16_t *pMemDistance, uint16_t *pMemAmplitude)
{
	int i, j, k, x, y, val;
	std::vector<int> arr;
	int data [320][240];

	int sz = tofcamInfo.medianFilterSize;	// getmedianFilterSize
	int n = sz/2;
	int pos = (sz*sz)/2;
	int width = tofcamInfo.roiXMax-tofcamInfo.roiXMin+1;//DME660_IMAGE_WIDTH;
	int height = (tofcamInfo.roiYMax-tofcamInfo.roiYMin+1)*2;//DME660_IMAGE_HEIGHT;

	if(tofcamInfo.bUsedMedianFilter)
	{
		printf("set medianfilter::\n");
		for(k=0; k < tofcamInfo.medianFilterIterations; k++)	// getmedianFilterIterations
		{
			for(y=n; y<DME660_IMAGE_HEIGHT-n; y++)
			{
				for(x=n; x<DME660_IMAGE_WIDTH-n; x++)
				{
					arr.clear();

					for(j=y-n; j<=y+n; j++)
					{
						for(i=x-n; i<= x+n; i++)
						{
//							val = pixelField.getPixelData(i,j);
							val = pMemDistance[j*width+i];
	 						arr.push_back(val);
						}
					}
					std::sort(arr.begin(), arr.end());
					data[x][y] = arr.at(pos);
				}
			}

			for(x= 0; x<width; x++){
				for(y=0; y<height; y++){
					//pixelField.setPixelData(x, y, data[x][y]);
					pMemDistance[y*width+x] = data[x][y];
				}
			}
		}
 	}

}


void NSL1110AA::gaussFilter(uint16_t *pMemDistance, uint16_t *pMemAmplitude)
{
    int i, x, y;
    int data [328][252];
    int val, val2, val3, val4, val5, val6, val7, val8, val9;
	int minPhase = 30;
	int width = tofcamInfo.roiXMax-tofcamInfo.roiXMin+1;//DME660_IMAGE_WIDTH;
	int height = (tofcamInfo.roiYMax-tofcamInfo.roiYMin+1)*2;//DME660_IMAGE_HEIGHT;

    if(tofcamInfo.bUsedGaussFilter){
		printf("set gaussFilter::\n");

        for(i=0; i < tofcamInfo.gaussIteration; i++){
            for(y=1; y<height; y++){
                for(x=1; x<width; x++){

                    val  = pMemDistance[(y-1)*width+(x-1)];//(x-1, y-1);
                    val2 = pMemDistance[(y-1)*width+(x)];//x,   y-1);
                    val3 = pMemDistance[(y-1)*width+(x+1)];//(x+1, y-1);

                    val4 = pMemDistance[(y)*width+(x-1)];//(x-1, y);
                    val5 = pMemDistance[(y)*width+(x)];//(x,   y);
                    val6 = pMemDistance[(y)*width+(x+1)];//x+1, y);

                    val7 = pMemDistance[(y+1)*width+(x-1)];//(x-1, y+1);
                    val8 = pMemDistance[(y+1)*width+(x)];//(x,   y+1);
                    val9 = pMemDistance[(y+1)*width+(x+1)];//(x+1, y+1);

                    if((val  < DME660_LOW_AMPLITUDE && (val>minPhase)) &&
                       (val2  < DME660_LOW_AMPLITUDE && (val>minPhase)) &&
                       (val3  < DME660_LOW_AMPLITUDE && (val>minPhase)) &&
                       (val4  < DME660_LOW_AMPLITUDE && (val>minPhase)) &&
                       (val5  < DME660_LOW_AMPLITUDE && (val>minPhase)) &&
                       (val6  < DME660_LOW_AMPLITUDE && (val>minPhase)) &&
                       (val7  < DME660_LOW_AMPLITUDE && (val>minPhase)) &&
                       (val8  < DME660_LOW_AMPLITUDE && (val>minPhase)) &&
                       (val9  < DME660_LOW_AMPLITUDE && (val>minPhase))){

                        val += val2 << 1;  // * 2;
                        val += val3;
                        val += val4 << 1; // * 2;
                        val += val5 << 2; // * 4;
                        val += val6 << 1; // * 2;
                        val += val7;
                        val += val8 << 1; // * 2;
                        val += val9;
                        val = val>>4;     // /16;
                        data[x][y] = val;

                    }else{
                        data[x][y] = val5;
                    }
                }
            }

            for(x=1; x<width; x++){
                for(y=1; y<height; y++){
                    pMemDistance[y*width+x]=data[x][y];//(x, y, data[x][y]);
                }
            }

        }
    }

}


int NSL1110AA::getDistanceAmplitude(cv::Mat &imageDistance, cv::Mat &imageAmplitude, bool bUsedPointCloud)
{
    int width = tofcamInfo.roiXMax-tofcamInfo.roiXMin+1;//DME660_IMAGE_WIDTH;
    int height = (tofcamInfo.roiYMax-tofcamInfo.roiYMin+1)*2;//DME660_IMAGE_HEIGHT;
    int stepY = 1;
	
#if 1
	uint16_t *pMemDistance = (uint16_t *)procBuff[0];
	uint16_t *pMemAmplitude = (uint16_t *)procBuff[1];
#else
	updateData(DATA_SIZE);
#endif
//	printf("maxAmplitudeValue = %d\n", maxAmplitudeValue);

	maxPhase = (int)getPhase(maxDistanceValue/10);

	if( tofcamInfo.bUsedMedianFilter ){
		medianFilter(pMemDistance, pMemAmplitude);
	}

	if( tofcamInfo.bUsedGaussFilter ){
		gaussFilter(pMemDistance, pMemAmplitude);
	} 

	if( tofcamInfo.bUsedEdgeFilter ){
		edgeFilterPixels(pMemDistance, pMemAmplitude);
	}

	if( tofcamInfo.hdr_mode == HDR_SPATIAL_MODE )
	{
		getComparedHDRAmplitudeImage(pMemDistance, pMemAmplitude, 0, 0);
	}

	memset(distanceTable, 0, sizeof(distanceTable));

#ifdef _WINDOWS
	point_cloud_ptr->clear();
#endif

    for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			int pixelDistance = pMemDistance[y*width+x];
			int pixelAmplitude = pMemAmplitude[y*width+x];

			if( tofcamInfo.tofcamModeType == GRAYSCALE_MODE && pixelAmplitude < 8000 ) 
				pixelAmplitude -= 2048;

			setDistanceColor(imageDistance, x, y, pixelDistance);
			
			if( tofcamInfo.tofcamModeType == AMPLITEDE_DISTANCE_EX_MODE
				|| tofcamInfo.tofcamModeType == GRAYSCALE_MODE
				|| tofcamInfo.tofcamModeType == DISTANCE_GRAYSCALE_MODE ) 
			{
				if((pixelAmplitude < 0 || pixelAmplitude > maxAmplitudeValue) 
					&& (pixelAmplitude < DME660_LOW_AMPLITUDE) )
				{
					setGrayScaledColor(imageAmplitude, x, y, DME660_LOW_AMPLITUDE, maxAmplitudeValue);
				}
				else
				{
					setGrayScaledColor(imageAmplitude, x, y, pixelAmplitude, maxAmplitudeValue);
				}
			}
			else if( tofcamInfo.tofcamModeType == AMPLITEDE_DISTANCE_MODE )
			{
				setAmplitudeColor(imageAmplitude, x, y, pixelAmplitude);
			}


			if( tofcamInfo.tofcamModeType != GRAYSCALE_MODE ){
				if( pixelDistance < DME660_LOW_AMPLITUDE ){

					int Distance = (MAX_DISTANCEVALUE * pixelDistance / MAX_PHASE);
					distanceTable[y*DME660_IMAGE_WIDTH+x] = Distance;
					
#ifdef _WINDOWS
					if( bUsedPointCloud ){
						pcl::PointXYZRGB point;
						pcl::PointXYZRGB basePoint;

						double outX, outY, outZ;
						lensTransform.transformPixel((tofcamInfo.roiXMin-DEFAULT_ROI_XMIN)+x, (tofcamInfo.roiYMin-DEFAULT_ROI_YMIN)+y, Distance, outX, outY, outZ, sin_angle, cos_angle);

						point.x = (double)(outX/1000);
						point.y = (double)(outY/1000);
						point.z = (double)(outZ/1000);

						point.b = imageDistance.at<Vec3b>(y, x)[0];
						point.g = imageDistance.at<Vec3b>(y, x)[1];
						point.r = imageDistance.at<Vec3b>(y, x)[2];

						if(y == 120 || x == 160)
						{ 
							basePoint.x = (double)(outX/1000);
							basePoint.y = (double)(outY/1000);
							basePoint.z = (double)(outZ/1000);

							basePoint.b = 255;
							basePoint.g = 255;
							basePoint.r = 255;
						}

						point_cloud_ptr->points.push_back(point);
						point_cloud_ptr->points.push_back(basePoint);
					}
#endif				
				}
				else{
					distanceTable[y*DME660_IMAGE_WIDTH+x] = pixelDistance;
				}
			}

		}
	}

	memset(procBuff, 0, sizeof(procBuff));
	return 0;
}



void NSL1110AA::reqCommandStr(const char *pData, uint8_t *pTotalBuff)
{
	SOCKET client_socket;
	struct sockaddr_in	 server_addr;
	
	memset(&server_addr, 0, sizeof(server_addr));
	server_addr.sin_family = AF_INET;
	server_addr.sin_port = htons(DME660_PORT);
	server_addr.sin_addr.s_addr = inet_addr(mIpaddrStr.c_str());
	
	client_socket = socket(PF_INET, SOCK_STREAM, 0);

	if(-1 == client_socket)
	{
		printf("Can not open socket\n");
		waitClosingThread();
		exit(1);
	}

	if(-1 == connect(client_socket, (struct sockaddr*)&server_addr, sizeof(server_addr)))
	{
		printf("[%d]Socket not connect\n", __LINE__);
		waitClosingThread();
		exit(1);
	}

	send(client_socket, pData, (int)strlen(pData), 0);	
	recvFromTcp(client_socket, pTotalBuff);

	int16_t retData = 0;
	memcpy(&retData, pTotalBuff, 2);
	printf("command : %s=%d \n", pData, retData);

	closesocket(client_socket);
#if 0
#ifndef ENABLE_DRNU_TEMPERATURE	
	if( memcmp(pData, "getOffset", 	9) == 0 ){
		short nOffset = (short)retData;
		char buff[40];
		sprintf(buff,"setOffset %d\n", nOffset);
		printf("setOffset value = %d\n", nOffset);
		reqCommandStr(buff, pTotalBuff);
	}
#endif	
#endif
}


void NSL1110AA::reqCommandArg(char *pStrCmd, int flag, uint8_t *pTotalBuff)
{
	SOCKET client_socket;
	struct sockaddr_in	 server_addr;
	char cmd_buff[100];

	memset(&server_addr, 0, sizeof(server_addr));
	server_addr.sin_family = AF_INET;
	server_addr.sin_port = htons(DME660_PORT);
	server_addr.sin_addr.s_addr = inet_addr(mIpaddrStr.c_str());

	client_socket = socket(PF_INET, SOCK_STREAM, 0);

	if(-1 == client_socket)
	{
		printf("Can not open socket\n");
		waitClosingThread();
		exit(1);
	}

	sprintf(cmd_buff,"%s %d\n", pStrCmd, flag);

	if(-1 == connect(client_socket, (struct sockaddr*)&server_addr, sizeof(server_addr)))
	{
		printf("[%d]Socket not connect\n", __LINE__);
		waitClosingThread();
		exit(1);
	}

	send(client_socket, cmd_buff, (int)strlen(cmd_buff), 0);
	recvFromTcp(client_socket, pTotalBuff);

	closesocket(client_socket);

	printf("send cmd::%s\n", cmd_buff);
}
	



void NSL1110AA::reqStartVideo(uint8_t *pTotalBuff)
{
	SOCKET client_socket;
	struct sockaddr_in	 server_addr;
	char cmd_buff[100];
	
	memset(&server_addr, 0, sizeof(server_addr));
	server_addr.sin_family = AF_INET;
	server_addr.sin_port = htons(DME660_PORT);
	server_addr.sin_addr.s_addr = inet_addr(mIpaddrStr.c_str());
	
	client_socket = socket(PF_INET, SOCK_STREAM, 0);
	if(-1 == client_socket)
	{
		printf("Can not open socket\n");
		waitClosingThread();
		exit(1);
	}

#ifdef MAINTAIN_SOCKET_STREAMING

	int buff_size = 160000;
	setsockopt( client_socket, SOL_SOCKET, SO_RCVBUF, &buff_size, sizeof(buff_size));
	
//	struct linger ling;
//	ling.l_onoff = 1;
//	ling.l_linger = 0;
//	setsockopt( client_socket, SOL_SOCKET, SO_LINGER, &ling, sizeof(ling));
	
//	int nValue = 1;
//	setsockopt( client_socket, IPPROTO_TCP, TCP_NODELAY, &nValue, sizeof(int) );

	sprintf(cmd_buff,"getDistanceAndAmplitudeSortedMaintainSock\n");
#else
	sprintf(cmd_buff,"startVideo\n");

	printf("========>> Not support startVideo\n");
	closesocket(client_socket);
	return;
#endif

	if(-1 == connect(client_socket, (struct sockaddr*)&server_addr, sizeof(server_addr)))
	{
		printf("[%d]Socket not connect\n", __LINE__);
		waitClosingThread();
		exit(1);
	}
	
	send(client_socket, cmd_buff, (int)strlen(cmd_buff), 0);	

#ifdef MAINTAIN_SOCKET_STREAMING
	tofcamInfo.client_socket = client_socket;
#else
	recvFromTcp(client_socket, pTotalBuff);
	closesocket(client_socket);
#endif
}


void NSL1110AA::reqStopVideo(uint8_t *pTotalBuff)
{
	SOCKET client_socket;
	struct sockaddr_in	 server_addr;
	char cmd_buff[100];

	memset(&server_addr, 0, sizeof(server_addr));
	server_addr.sin_family = AF_INET;
	server_addr.sin_port = htons(DME660_PORT);
	server_addr.sin_addr.s_addr = inet_addr(mIpaddrStr.c_str());

	client_socket = socket(PF_INET, SOCK_STREAM, 0);

	if(-1 == client_socket)
	{
		printf("Can not open socket\n");
		waitClosingThread();
		exit(1);
	}

	sprintf(cmd_buff,"stopVideo\n");
	
	if(-1 == connect(client_socket, (struct sockaddr*)&server_addr, sizeof(server_addr)))
	{
		printf("[%d]Socket not connect\n", __LINE__);
		waitClosingThread();
		exit(1);
	}

	send(client_socket, "stopVideo", (int)strlen("stopVideo")+1, 0);
	recvFromTcp(client_socket, pTotalBuff);

	closesocket(client_socket);

}

int NSL1110AA::reqSingleFrame(int modeType, uint8_t *pTotalBuff)
{
	SOCKET client_socket = 0;
	struct sockaddr_in   server_addr;
//	uint8_t rx_buff[2000];
	int ret = 0;
#ifdef MAINTAIN_SOCKET_STREAMING
	int expectedRcvLen = 307200;
	ret = recvFromTcp(tofcamInfo.client_socket, pTotalBuff, expectedRcvLen);
#else
	memset(&server_addr, 0, sizeof(server_addr));
	server_addr.sin_family = AF_INET;
	server_addr.sin_port = htons(DME660_PORT);
	server_addr.sin_addr.s_addr = inet_addr(mIpaddrStr.c_str());
	
	client_socket = socket(PF_INET, SOCK_STREAM, 0);
	
	if(-1 == client_socket)
	{
		printf("Can not open socket\n");
		waitClosingThread();
		exit(1);
	}
	
#if 0	
	int buff_size = 160000;
	setsockopt( client_socket, SOL_SOCKET, SO_RCVBUF, &buff_size, sizeof(buff_size));
	
	struct linger ling;
	ling.l_onoff = 1;
	ling.l_linger = 0;
	setsockopt( client_socket, SOL_SOCKET, SO_LINGER, &ling, sizeof(ling));
	
	int nValue = 1;
	setsockopt( client_socket, IPPROTO_TCP, TCP_NODELAY, &nValue, sizeof(int) );
#endif	
	if(-1 == connect(client_socket, (struct sockaddr*)&server_addr, sizeof(server_addr)))
	{
		printf("[%d]Socket not connect\n", __LINE__);
		return 0;
	}

	int expectedRcvLen = 307200;
	if( tofcamInfo.tofcamModeType == AMPLITEDE_DISTANCE_MODE
		|| tofcamInfo.tofcamModeType == AMPLITEDE_DISTANCE_EX_MODE ){
		send(client_socket, "getDistanceAndAmplitudeSorted\n", (int)strlen("getDistanceAndAmplitudeSorted\n"), 0);
	}
	else if( tofcamInfo.tofcamModeType == DISTANCE_GRAYSCALE_MODE ){
		send(client_socket, "getDistanceBWSorted\n", (int)strlen("getDistanceBWSorted\n"), 0);
	}
	else if( tofcamInfo.tofcamModeType == GRAYSCALE_MODE ){
		send(client_socket, "getBWSorted\n", (int)strlen("getBWSorted\n"), 0);
		expectedRcvLen = 153600;
	}
	
	ret = recvFromTcp(client_socket, pTotalBuff, expectedRcvLen);

	closesocket(client_socket);
#endif
	return ret;
}

double NSL1110AA::interpolate( double x, double x0, double y0, double x1, double y1){

    if( x1 == x0 ){
        return y0;
    } else {
        return ((x-x0)*(y1-y0)/(x1-x0) + y0);
    }

}


void NSL1110AA::createColorMapPixel(int numSteps, int indx, unsigned char &red, unsigned char &green, unsigned char &blue){
    double k = 1;
    double B0 = -0.125 * k - 0.25;
    double B1 = B0 + 0.25 * k;
    double B2 = B1 + 0.25 * k;
    double B3 = B2 + 0.25 * k;

    double G0 = B1;
    double G1 = G0 + 0.25 * k;
    double G2 = G1 + 0.25 * k;
    double G3 = G2 + 0.25 * k + 0.125;

    double R0 = B2;
    double R1 = R0 + 0.25 * k;
    double R2 = R1 + 0.25 * k;
    double R3 = R2 + 0.25 * k + 0.25;
    double i = (double)indx/(double)numSteps - 0.25 * k;

    if( i>= R0 && i < R1 ){
        red = (unsigned char)interpolate(i, R0, 0, R1, 255);
    } else if((i >= R1) && (i < R2)){
        red = 255;
    } else if((i >= R2) && (i < R3)) {
        red = (unsigned char)interpolate(i, R2, 255, R3, 0);
    } else {
        red = 0;
    }

    if( i>= G0 && i < G1 ){
        green = (unsigned char)interpolate(i, G0, 0, G1, 255);
    } else if((i>=G1)&&(i<G2)){
        green = 255;
    } else if((i >= G2)&&(i < G3)){
        green = (unsigned char)interpolate(i, G2, 255, G3, 0);
    } else {
        green = 0;
    }


    if( i>= B0 && i < B1 ){
        blue = (unsigned char)interpolate(i, B0, 0, B1, 255);
    } else if((i >= B1)&&(i < B2)){
        blue = 255;
    } else if((i >= B2)&&(i < B3)) {
        blue = (unsigned char)interpolate(i, B2, 255, B3, 0);
    } else{
        blue = 0;
    }
}


void NSL1110AA::reqGrayScaleIntegrationTime0(uint8_t *pTotalBuff)
{
	SOCKET client_socket;
	struct sockaddr_in   server_addr;
	char cmd_buff[100];
	
	memset(&server_addr, 0, sizeof(server_addr));
	server_addr.sin_family = AF_INET;
	server_addr.sin_port = htons(DME660_PORT);
	server_addr.sin_addr.s_addr = inet_addr(mIpaddrStr.c_str());

	client_socket = socket(PF_INET, SOCK_STREAM, 0);

	if(-1 == client_socket)
	{
		printf("Can not open socket\n");
		waitClosingThread();
		exit(1);
	}

	sprintf(cmd_buff,"setIntegrationTime2D %d\n", tofcamInfo.integrationTimeGrayscale);
	
	if(-1 == connect(client_socket, (struct sockaddr*)&server_addr, sizeof(server_addr)))
	{
		printf("[%d]Socket not connect\n", __LINE__);
		waitClosingThread();
		exit(1);
	}


	send(client_socket, cmd_buff, (int)strlen(cmd_buff), 0);
	recvFromTcp(client_socket, pTotalBuff);

	closesocket(client_socket);
	printf("Grayscaled:setIntegrationTime0 : %d\n", tofcamInfo.integrationTimeGrayscale);

	return;
}

void NSL1110AA::reqIntegrationTime3D(uint8_t *pTotalBuff)
{
	SOCKET client_socket;
	struct sockaddr_in   server_addr;
	char cmd_buff[100];
	
	memset(&server_addr, 0, sizeof(server_addr));
	server_addr.sin_family = AF_INET;
	server_addr.sin_port = htons(DME660_PORT);
	server_addr.sin_addr.s_addr = inet_addr(mIpaddrStr.c_str());

	client_socket = socket(PF_INET, SOCK_STREAM, 0);

	if(-1 == client_socket)
	{
		printf("Can not open socket\n");
		waitClosingThread();
		exit(1);
	}

	sprintf(cmd_buff,"setIntegrationTime3D %d\n", tofcamInfo.integrationTime3D);
	
	if(-1 == connect(client_socket, (struct sockaddr*)&server_addr, sizeof(server_addr)))
	{
		printf("[%d]Socket not connect\n", __LINE__);
		waitClosingThread();
		exit(1);
	}


	send(client_socket, cmd_buff, (int)strlen(cmd_buff), 0);
	recvFromTcp(client_socket, pTotalBuff);

	closesocket(client_socket);
	printf("setIntegrationTime3D : %d\n", tofcamInfo.integrationTime3D);

	return;
}

void NSL1110AA::reqMinAmplitude(uint8_t *pTotalBuff)
{
	SOCKET client_socket;
	struct sockaddr_in	 server_addr;
	char cmd_buff[100];
	
	memset(&server_addr, 0, sizeof(server_addr));
	server_addr.sin_family = AF_INET;
	server_addr.sin_port = htons(DME660_PORT);
	server_addr.sin_addr.s_addr = inet_addr(mIpaddrStr.c_str());
	
	client_socket = socket(PF_INET, SOCK_STREAM, 0);
	
	if(-1 == client_socket)
	{
		printf("Can not open socket\n");
		waitClosingThread();
		exit(1);
	}
	
	sprintf(cmd_buff,"setMinAmplitude %d\n", tofcamInfo.minAmplitude);
	
	if(-1 == connect(client_socket, (struct sockaddr*)&server_addr, sizeof(server_addr)))
	{
		printf("[%d]Socket not connect\n", __LINE__);
		waitClosingThread();
		exit(1);
	}
	
	
	send(client_socket, cmd_buff, (int)strlen(cmd_buff), 0);
	recvFromTcp(client_socket, pTotalBuff);
	
	closesocket(client_socket);
	printf("setMinAmplitude : %d\n", tofcamInfo.minAmplitude);
	
	return;

}

void NSL1110AA::adjustGrayscaled(void)
{
	int integrationTimeGrayscale = tofcamInfo.integrationTimeGrayscale;
	printf("adjustGrayscaled : %f/%d\n", tofcamInfo.meanAvg, integrationTimeGrayscale);
#if 0	
	if( tofcamInfo.tofcamModeType == GRAYSCALE_MODE
		|| tofcamInfo.tofcamModeType == DISTANCE_GRAYSCALE_MODE )
	{
		if( tofcamInfo.meanAvg < 70 && tofcamInfo.wait_frame_cnt == 0 ){
			if( integrationTimeGrayscale + 1000 <= 50000 ){
				integrationTimeGrayscale += 1000;
				reqGrayScaleIntegrationTime0(response[1]);
				tofcamInfo.wait_frame_cnt = WAIT_FRAME_COUNT;
			}
		}
		else if( tofcamInfo.meanAvg > 80 && tofcamInfo.wait_frame_cnt == 0 ){
			if( integrationTimeGrayscale - 1000 > 0 ){
				integrationTimeGrayscale -= 1000;
				reqGrayScaleIntegrationTime0(response[1]);
				tofcamInfo.wait_frame_cnt = WAIT_FRAME_COUNT;
			}
			else if( integrationTimeGrayscale - 1000 <= 0
				&& integrationTimeGrayscale - 100 > 0)
			{
				integrationTimeGrayscale -= 100;
				reqGrayScaleIntegrationTime0(response[1]);
				tofcamInfo.wait_frame_cnt = WAIT_FRAME_COUNT;
			}
			else if( integrationTimeGrayscale - 100 <= 0
				&& integrationTimeGrayscale - 10 > 0)
			{
				integrationTimeGrayscale -= 10;
				reqGrayScaleIntegrationTime0(response[1]);
				tofcamInfo.wait_frame_cnt = WAIT_FRAME_COUNT;
			}
		}
		else if( tofcamInfo.wait_frame_cnt > 0 ){
			tofcamInfo.wait_frame_cnt--;
		}
	}
#endif	
}



void NSL1110AA::initializeDME660(void)
{
	int numSteps = DME660_NUM_COLORS;
	unsigned char red, green, blue;
	char cmd[100];

	for(int i=0;  i< numSteps; i++)
	{
	  createColorMapPixel(numSteps, i, red, green, blue);
	  colorVector.push_back(Vec3b(blue, green, red));
	}
#if 1
	int size = sizeof(initialcmd)/sizeof(char*);

	for(int i = 0; i < size ; i++){
		reqCommandStr(initialcmd[i], response[1]);
		Sleep(30);
	}
#endif
	
	reqIntegrationTime3D(response[1]);
	Sleep(20);
	reqGrayScaleIntegrationTime0(response[1]);
	Sleep(20);
	reqCommandArg("enableSaturation", tofcamInfo.saturationType, response[1]);
	Sleep(20);
	reqCommandArg("enableAdcOverflow", tofcamInfo.overflowType, response[1]);
	Sleep(20);
	reqCommandArg("correctGrayscaleGain", tofcamInfo.GrayscaleGainType, response[1]);
	Sleep(20);
	reqCommandArg("correctGrayscaleOffset", tofcamInfo.GrayscaleOffsetType, response[1]);
	Sleep(20);
	sprintf(cmd, "setROI %d %d %d %d\n", tofcamInfo.roiXMin, tofcamInfo.roiXMax, tofcamInfo.roiYMin, tofcamInfo.roiYMax);
	reqCommandStr(cmd, response[1]);
	Sleep(30);

	if( tofcamInfo.tofcamModeType == GRAYSCALE_MODE ){
		reqCommandStr("loadConfig 0\n", response[1]);
	}
	else{
		reqCommandStr("loadConfig 1\n", response[1]);
	}

	Sleep(30);
//	printf("End initializeDME660() SIZE = %d\n", size);
}

void NSL1110AA::keyProc()
{
	if( tofcamInfo.tofcamEvent_key != 0 )
	{	
//		LogDebug("DME660 -- CHANGE :: %d - mode :: %d\n", tofcamEvent_key, tofcamInfo.tofcamModeType);
	
		// black & white (grayscale) mode
		if( tofcamInfo.tofcamEvent_key == 'b' && tofcamInfo.tofcamModeType != GRAYSCALE_MODE )
		{
			tofcamInfo.tofcamEvent_key =0;
			if( tofcamInfo.tofcamModeType != GRAYSCALE_MODE )
			{
				tofcamInfo.tofcamModeType = GRAYSCALE_MODE;
				maxAmplitudeValue = 2047;
				reqCommandStr("loadConfig 0\n", response[1]);
			}
		}
		// distance (+grayscale) mode
		else if( tofcamInfo.tofcamEvent_key == 'd')
		{
			tofcamInfo.tofcamEvent_key = 0;
			if( tofcamInfo.tofcamModeType != DISTANCE_GRAYSCALE_MODE ){
				tofcamInfo.tofcamModeType = DISTANCE_GRAYSCALE_MODE;
				maxAmplitudeValue = 2047;
				reqCommandStr("loadConfig 1\n", response[1]);
			}
		}
		// amplitude & distance mode
		else if( tofcamInfo.tofcamEvent_key == 'e' )
		{
			tofcamInfo.tofcamEvent_key = 0;
			if( tofcamInfo.tofcamModeType != AMPLITEDE_DISTANCE_EX_MODE ){
				tofcamInfo.tofcamModeType = AMPLITEDE_DISTANCE_EX_MODE;
				maxAmplitudeValue = 4095;
				reqCommandStr("loadConfig 1\n", response[1]);
			}
		}
		// amplitude & distance mode
		else if( tofcamInfo.tofcamEvent_key == 'a' )
		{
			tofcamInfo.tofcamEvent_key = 0;
			if( tofcamInfo.tofcamModeType != AMPLITEDE_DISTANCE_MODE ){
				tofcamInfo.tofcamModeType = AMPLITEDE_DISTANCE_MODE;
				maxAmplitudeValue = 4095;
				reqCommandStr("loadConfig 1\n", response[1]);
			}
		}
		// grayscale gain corrected
		else if( tofcamInfo.tofcamEvent_key == 'g' )
		{
			tofcamInfo.tofcamEvent_key = 0;
			tofcamInfo.GrayscaleGainType = tofcamInfo.GrayscaleGainType ? 0 : 1;
			reqCommandArg("correctGrayscaleGain", tofcamInfo.GrayscaleGainType, response[0]);
		}
		// grayscale offset corrected
		else if( tofcamInfo.tofcamEvent_key == 'o' )
		{
			tofcamInfo.tofcamEvent_key = 0;
			tofcamInfo.GrayscaleOffsetType = tofcamInfo.GrayscaleOffsetType ? 0 : 1;
			reqCommandArg("correctGrayscaleOffset", tofcamInfo.GrayscaleOffsetType, response[0]);
		}			
		// ambient light enable/disable
		else if( tofcamInfo.tofcamEvent_key == 'l' )
		{
			tofcamInfo.tofcamEvent_key = 0;
			tofcamInfo.nCorrectAmbientLight = tofcamInfo.nCorrectAmbientLight ? 0 : 1;
			reqCommandArg("correctAmbientLight", tofcamInfo.nCorrectAmbientLight, response[0]);
		}
		// saturation enable/disable
		else if( tofcamInfo.tofcamEvent_key == 's' )
		{
			tofcamInfo.tofcamEvent_key = 0;
			tofcamInfo.saturationType = tofcamInfo.saturationType ? 0 : 1;
			reqCommandArg("enableSaturation", tofcamInfo.saturationType, response[0]);
		}
		// overflow enable/disable
		else if( tofcamInfo.tofcamEvent_key == 'f' )
		{
			tofcamInfo.tofcamEvent_key = 0;
			tofcamInfo.overflowType = tofcamInfo.overflowType ? 0 : 1;
			reqCommandArg("enableAdcOverflow", tofcamInfo.overflowType, response[0]);
		}
		else if( tofcamInfo.tofcamEvent_key == 'r' )
		{
			tofcamInfo.tofcamEvent_key = 0;
			tofcamInfo.rotate_90 = tofcamInfo.rotate_90 ? 0 : 1;
#ifdef ROTATE_IMAGE_ADJUST_ROI
			if( tofcamInfo.rotate_90 != 0 ){
				tofcamInfo.roiXMin = ADJUST_ROI_XMIN;
				tofcamInfo.roiXMax = ADJUST_ROI_XMAX;
				tofcamInfo.roiYMin = ADJUST_ROI_YMIN;
				tofcamInfo.roiYMax = ADJUST_ROI_YMAX;
			}
			else{
				tofcamInfo.roiXMin = DEFAULT_ROI_XMIN;
				tofcamInfo.roiXMax = DEFAULT_ROI_XMAX;
				tofcamInfo.roiYMin = DEFAULT_ROI_YMIN;
				tofcamInfo.roiYMax = DEFAULT_ROI_YMAX;
			}

			char cmd[100];
			sprintf(cmd, "setROI %d %d %d %d\n", tofcamInfo.roiXMin, tofcamInfo.roiXMax, tofcamInfo.roiYMin, tofcamInfo.roiYMax);
			reqCommandStr(cmd, response[1]);
#endif
		}
		else if ( tofcamInfo.tofcamEvent_key == 'p' ){
			tofcamInfo.usedPointCloud = tofcamInfo.usedPointCloud ? 0 : 1; 
		}
		else if( tofcamInfo.tofcamEvent_key == 'h' ){
			tofcamInfo.tofcamEvent_key = 0;
			printf("-----------------------------------------------\n");
			printf("p key : change Point Cloud\n");
			printf("b key : change GRAYSCALE mode\n");
			printf("d key : change DISTANCE mode\n");
			printf("a key : change AMPLITUDE & DISTANCE mode\n");
			printf("e key : change AMPLITUDE(log) & DISTANCE mode\n");
			printf("g key : change Grayscale Gain corrected\n");
			printf("o key : change Grayscale Offset corrected\n");
			printf("l key : change Ambient light corrected\n");
			printf("s key : change saturation corrected\n");
			printf("f key : change overflow corrected\n");
			printf("u key : change DRNU(ABS,DRNU,Temp)\n");
			printf("r key : rotate 90(ROI reset)\n");
			printf("-----------------------------------------------\n");
	
			tofcamInfo.printBasicInfo = tofcamInfo.printBasicInfo ? 0 : 1;
		}
		else if( tofcamInfo.tofcamEvent_key == 'u' )
		{
			tofcamInfo.tofcamEvent_key = 0;
			tofcamInfo.drnuType = tofcamInfo.drnuType ? 0 : 1;
			if( tofcamInfo.drnuType != 0 ){
				reqCommandStr("setABS 1\n", response[1]);
				reqCommandStr("correctDRNU 2\n", response[1]);
				reqCommandStr("correctTemperature 2\n", response[1]);
			}
			else{
				reqCommandStr("setABS 0\n", response[1]);
				reqCommandStr("correctDRNU 0\n", response[1]);
				reqCommandStr("correctTemperature 0\n", response[1]);
			}
		}
	
	
	}
}

void *NSL1110AA::rxDme660(void *arg) 
{
	static unsigned char allbuff[2048];
	int loopCnt = 0;
	bool bStartVideo = false;
	
	while(!exit_thtread)
	{
		if( tofcamInfo.captureNetType == NONEMODEL_TYPE )
		{
			Sleep(10);
			loopCnt++;
//			printf("rxDme660 %d\n", loopCnt);
			continue;
		}

		if( bStartVideo == false ){
			bStartVideo = true;
			reqStartVideo(response[0]);			
			Sleep(30);
		}

//		printf("loopCnt = %d\n", loopCnt);
		loopCnt = 0;

		keyProc();
		
		const auto& time_cap0 = std::chrono::steady_clock::now();		

		int otherLen = reqSingleFrame(tofcamInfo.tofcamModeType, response[0]);

        const auto& time_cap1 = std::chrono::steady_clock::now();
        double time_cam = (time_cap1 - time_cap0).count() / 1000000.0;

		if( tofcamInfo.printBasicInfo ){
		    printf("Tofcam-Rx:%9.3lf [msec] mode : %s ambient : %d mean : %f\n", time_cam, getModeString(tofcamInfo.tofcamModeType).c_str(), tofcamInfo.nCorrectAmbientLight, tofcamInfo.meanAvg);
			printf("grayscale gain:%d offset:%d satu:%d overf:%d drnu:%d rxLen = %d\n", tofcamInfo.GrayscaleGainType, tofcamInfo.GrayscaleOffsetType, tofcamInfo.saturationType, tofcamInfo.overflowType, tofcamInfo.drnuType, otherLen);
		}

		if( otherLen > 0 ){
			EnterCriticalSection(&tofcamBuff.lock);

			memcpy(tofcamBuff.tofcamBuf[tofcamBuff.head_idx], response[0], DME660_BUFF_SIZE);
			tofcamBuff.bufLen[tofcamBuff.head_idx] = otherLen;
		
			ADD_TOFCAM_BUFF(tofcamBuff, DME660_ETH_BUFF_SIZE);

			LeaveCriticalSection(&tofcamBuff.lock);
		}

	}

	return NULL;
}


void NSL1110AA::waitClosingThread()
{
	if( exit_thtread == 0 ){
		exit_thtread = 1;
#ifdef _WINDOWS
		DWORD rst = WaitForSingleObject(hThread, 1000);
		if (rst != WAIT_OBJECT_0) {
			printf("wait error\n");
		}
#else
		pthread_join(threadID, NULL);
#endif
		reqStopVideo(response[0]);
	}
}

//point cloud
#ifdef _WINDOWS
pcl::PointCloud<pcl::PointXYZRGB>::Ptr NSL1110AA::pcbVis()
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	point_cloud_ptr->clear();
	point_cloud_ptr->is_dense = false;
	//point_cloud_ptr->reserve(IMAGE_WIDTH * IMAGE_HEIGHT);
	point_cloud_ptr->width = DME660_IMAGE_WIDTH;
	point_cloud_ptr->height = DME660_IMAGE_HEIGHT;

	return point_cloud_ptr;
}

pcl::visualization::PCLVisualizer::Ptr NSL1110AA::rgbVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("ROBOSCAN PointCloud"));
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "sample cloud");
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	//ÁÂÇ¥Ãà
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	viewer->setCameraPosition(0, 0, -5, 0, 0, 0, 0, -1, 0, 0);
	viewer->setShowFPS(false);
	//viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)viewer.get());
#if 0
	pcl::PointXYZ a1, a2;
	a1.x = -1.450;
	a1.y = -0.450;
	a1.z = 5.000;
	a2.x = 1.450;
	a2.y = -0.450;
	a2.z = 5.000;
	viewer->addLine(a1, a2, "a12");

	pcl::PointXYZ a3, a4;
	a3.x = -1.450;
	a3.y = 0.450;
	a3.z = 5.000;
	a4.x = 1.450;
	a4.y = 0.450;
	a4.z = 5.000;
	viewer->addLine(a3, a4, "a34");

	pcl::PointXYZ a5, a6;
	a5.x = -1.450;
	a5.y = -0.450;
	a5.z = 3.000;
	a6.x = 1.450;
	a6.y = -0.450;
	a6.z = 3.000;
	viewer->addLine(a5, a6, "a56");

	pcl::PointXYZ a7, a8;
	a7.x = -1.450;
	a7.y = 0.450;
	a7.z = 3.000;
	a8.x = 1.450;
	a8.y = 0.450;
	a8.z = 3.000;
	viewer->addLine(a7, a8, "a78");

	viewer->addLine(a1, a5, "a15");
	viewer->addLine(a2, a6, "a26");
	viewer->addLine(a3, a7, "a37");
	viewer->addLine(a4, a8, "a45");

	viewer->addLine(a1, a3, "a13");
	viewer->addLine(a2, a4, "a24");
	viewer->addLine(a5, a7, "a57");
	viewer->addLine(a6, a8, "a68");
#endif
	return (viewer);
}
#endif

//////////////////////////////////// External Interface ////////////////////////////////////////
// Create
NSL1110AA* NSL1110AA::Create( std::string ipaddr )
{
	// create camera instance
	return new NSL1110AA(ipaddr);
}

std::string NSL1110AA::getDistanceString(int distance )
{
	std::string distStr;

	if( distance == 0 || distance == DME660_LOW_AMPLITUDE || distance == 300000 )
		distStr = "LOW_AMPLITUDE";
	else if( distance == DME660_ADC_OVERFLOW )
		distStr = "ADC_OVERFLOW";
	else if( distance == DME660_SATURATION )
		distStr = "SATURATION";
	else if( distance == DME660_PIXEL_BADASYM )
		distStr = "PIXEL_BADASYM";
	else
		distStr = format("%d mm", distance);

	return distStr;
}

std::string NSL1110AA::getModeString(int modeType )
{
	std::string distStr;

	if( modeType == GRAYSCALE_MODE )
		distStr = "GRAYSCALE";
	else if( modeType == AMPLITEDE_DISTANCE_MODE)
		distStr = "AMPLITEDE_DISTANCE";
	else if( modeType == AMPLITEDE_DISTANCE_EX_MODE)
		distStr = "AMPLITEDE_DISTANCE_BW";
	else if( modeType == DISTANCE_GRAYSCALE_MODE)
		distStr = "GRAYSCALE_DISTANCE";
	else if( modeType == DISTANCE_MODE)
		distStr = "DISTANCE";
	else
		distStr = "None-mode";

	return distStr;
}

std::string NSL1110AA::getLeftViewName(void)
{
	std::string nameStr;
	if( tofcamInfo.tofcamModeType == AMPLITEDE_DISTANCE_MODE )
		nameStr = "AMPLITUDE";
	else if( tofcamInfo.tofcamModeType == AMPLITEDE_DISTANCE_EX_MODE )
		nameStr = "AMPL&Gray";
	else //if( tofcamInfo.tofcamModeType == DISTANCE_GRAYSCALE_MODE )
		nameStr = "GRAYSCALE";	

	return nameStr;
}

void NSL1110AA::drawPointCloud(void)
{
#ifdef _WINDOWS
	if( !viewer->wasStopped() && point_cloud_ptr->points.size() > 0 ){
		viewer->updatePointCloud(point_cloud_ptr, "sample cloud");
		viewer->spinOnce();
	}
#endif
}

bool NSL1110AA::isRotate90()
{
	if( tofcamInfo.rotate_90 != 0 ){
		return true;
	}

	return false;
}

int NSL1110AA::getVideoWidth(){
	if( tofcamInfo.rotate_90 != 0 ){
		return DME660_IMAGE_HEIGHT;
	}
	return DME660_IMAGE_WIDTH;
}
int NSL1110AA::getVideoHeight(){
	if( tofcamInfo.rotate_90 != 0 ){
		return DME660_IMAGE_WIDTH;
	}
	return DME660_IMAGE_HEIGHT;
}

int NSL1110AA::getWidthDiv()				
{ 
	if( tofcamInfo.rotate_90 != 0 ){
		return tofcamInfo.height/DME660_IMAGE_HEIGHT;
	}
	return tofcamInfo.width/DME660_IMAGE_WIDTH; 
}

int NSL1110AA::getHeightDiv()
{
	if( tofcamInfo.rotate_90 != 0 ){
		return tofcamInfo.width/DME660_IMAGE_WIDTH;
	}
	return tofcamInfo.height/DME660_IMAGE_HEIGHT; 
}


int NSL1110AA::getWidth()				
{ 
	if( tofcamInfo.rotate_90 != 0 ){
		return tofcamInfo.height;
	}
	return tofcamInfo.width; 
}

/**
 * Return the height of the stream, in pixels.
 */
int NSL1110AA::getHeight()
{
	if( tofcamInfo.rotate_90 != 0 ){
		return tofcamInfo.width;
	}
	return tofcamInfo.height; 
}

// Capture
bool NSL1110AA::Capture( void** output, int timeout )
{
	// verify the output pointer exists
	if( !output )
		return false;

	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

	int frame_cnt = 0;
	while(!exit_thtread)
	{

		if( GET_BUFF_CNT(tofcamBuff, DME660_ETH_BUFF_SIZE) > 0 ){
			
			cv::Mat image(DME660_IMAGE_HEIGHT, DME660_IMAGE_WIDTH, CV_8UC3, Scalar(255,255,255));	
			cv::Mat imageDist(DME660_IMAGE_HEIGHT, DME660_IMAGE_WIDTH, CV_8UC3, Scalar(255,255,255));	

			EnterCriticalSection(&tofcamBuff.lock);
			
			//printf("main-bufLen = %d:%d\n", tofcamBuff.bufGrayLen[tofcamBuff.tail_idx], tofcamBuff.bufLen[tofcamBuff.tail_idx]);
			if( tofcamInfo.tofcamModeType == AMPLITEDE_DISTANCE_MODE 
				|| tofcamInfo.tofcamModeType == AMPLITEDE_DISTANCE_EX_MODE
				|| tofcamInfo.tofcamModeType == DISTANCE_GRAYSCALE_MODE )
			{
				int halfLen = tofcamBuff.bufLen[tofcamBuff.tail_idx]>>1;
				memcpy(procBuff[0], &tofcamBuff.tofcamBuf[tofcamBuff.tail_idx][0], halfLen);
				memcpy(procBuff[1], &tofcamBuff.tofcamBuf[tofcamBuff.tail_idx][halfLen], halfLen);
			}
			else{
				memcpy(procBuff[0], &tofcamBuff.tofcamBuf[tofcamBuff.tail_idx][0], tofcamBuff.bufLen[tofcamBuff.tail_idx]);
				memcpy(procBuff[1], &tofcamBuff.tofcamBuf[tofcamBuff.tail_idx][0], tofcamBuff.bufLen[tofcamBuff.tail_idx]);
			}
			
			POP_TOFCAM_BUFF(tofcamBuff, DME660_ETH_BUFF_SIZE);

			LeaveCriticalSection(&tofcamBuff.lock);

			getDistanceAmplitude(imageDist, image, tofcamInfo.usedPointCloud != 0);


			Scalar avg = mean(image);
			tofcamInfo.meanAvg = avg[0];
	//		if( avg[0] == 0 ){
	//			rx_ok = 2; // ERROR ??
	//		}

			static cv::Mat resizeDist, resizeFrame;
			// up-sampling : INTER_CUBIC, INTER_LANCZOS4
			// down-sampling : INTER_AREA
#ifdef RESIZE_IMAGE_MAT
#ifdef HAVE_CV_CUDA
			cuda::GpuMat gpuImage, gpuOutImage;
			cuda::GpuMat gpuImageDist, gpuOutImageDist;
			gpuImage.upload(image);
			gpuImageDist.upload(imageDist);

			if( tofcamInfo.rotate_90 != 0 )
			{
				cuda::rotate(gpuImage, gpuImage, cv::Size( 240, 320 ), -90, 239, 0, cv::INTER_LINEAR);
				cuda::rotate(gpuImageDist, gpuImageDist, cv::Size( 240, 320 ), -90, 239, 0, cv::INTER_LINEAR);

				cuda::resize(gpuImage, gpuOutImage, cv::Size( 480, 640 ), cv::INTER_LANCZOS4);
				cuda::resize(gpuImageDist, gpuOutImageDist, cv::Size( 480, 640 ), cv::INTER_LANCZOS4);
			}
			else{
				cuda::resize(gpuImage, gpuOutImage, cv::Size( tofcamInfo.width, tofcamInfo.height ), cv::INTER_LANCZOS4);
				cuda::resize(gpuImageDist, gpuOutImageDist, cv::Size( tofcamInfo.width, tofcamInfo.height ), cv::INTER_LANCZOS4);
			}

			gpuOutImage.download(resizeFrame);
			gpuOutImageDist.download(resizeDist);
#else

			if( tofcamInfo.rotate_90 != 0 )
			{
				cv::rotate(image, image, ROTATE_90_CLOCKWISE);
				cv::rotate(imageDist, imageDist, ROTATE_90_CLOCKWISE);

				cv::resize( imageDist, resizeDist, cv::Size( 480, 640 ));
				cv::resize( image, resizeFrame, cv::Size( 480, 640 ));
			}
			else{
				cv::resize( imageDist, resizeDist, cv::Size( tofcamInfo.width, tofcamInfo.height ));
				cv::resize( image, resizeFrame, cv::Size( tofcamInfo.width, tofcamInfo.height ));
			}

#endif
	
#else	// RESIZE_IMAGE_MAT
			if( tofcamInfo.rotate_90 != 0 )
			{
				cv::rotate(image, image, ROTATE_90_CLOCKWISE);
				cv::rotate(imageDist, imageDist, ROTATE_90_CLOCKWISE);
			}

			resizeFrame = image;
			resizeDist = imageDist;
#endif // RESIZE_IMAGE_MAT

			tofcamImage.frameMat = &resizeFrame;
			tofcamImage.distMat = &resizeDist;			
			tofcamImage.pDistanceTable = distanceTable;
			break;
		}
		else{
			Sleep(10);

			std::chrono::steady_clock::time_point curTime = std::chrono::steady_clock::now();
			double passed_time = (curTime - begin).count() / 1000000.0;
			if( passed_time > timeout ){
				printf("timeout capture~~~~~~~ timeout = %d\n", timeout);
				return false;
			}

			
			continue;
		}
	}

	*output = &tofcamImage;
	return true;
}


// closeLidar
void NSL1110AA::closeLidar()
{
	printf("->closeLidar()~~ exit_thtread = %d\n", exit_thtread);

	tofcamInfo.captureNetType = NONEMODEL_TYPE;

	waitClosingThread();

	printf("<-closeLidar()~~ exit_thtread = %d\n", exit_thtread);
}



void NSL1110AA::startCaptureCommand(int netType, void *pCapOption)
{	
	CaptureOptions *pCapOpt = (CaptureOptions *)pCapOption;
	maxDistanceValue = (pCapOpt->maxDistance <= 0 || pCapOpt->maxDistance > MAX_DISTANCEVALUE) ? MAX_DISTANCEVALUE : pCapOpt->maxDistance;

	tofcamInfo.tofcamModeType = pCapOpt->captureType;
	tofcamInfo.integrationTimeGrayscale = pCapOpt->grayIntegrationTime;
	tofcamInfo.integrationTime3D = pCapOpt->integrationTime;
	tofcamInfo.minAmplitude = pCapOpt->minAmplitude;
	tofcamInfo.edgeThresHold = pCapOpt->edgeThresHold;
	tofcamInfo.bUsedMedianFilter = pCapOpt->medianFilterEnable;
	tofcamInfo.medianFilterSize = pCapOpt->medianFilterSize;
	tofcamInfo.medianFilterIterations = pCapOpt->medianFilterIterations;
	tofcamInfo.gaussIteration = pCapOpt->gaussIteration;

	if( tofcamInfo.gaussIteration > 0 ){
		tofcamInfo.bUsedGaussFilter = true;
	}
	else{
		tofcamInfo.bUsedGaussFilter = false;
	}

	if( tofcamInfo.bUsedMedianFilter
		&& tofcamInfo.medianFilterSize > 0 
		&& tofcamInfo.medianFilterIterations > 0 )
	{
		tofcamInfo.bUsedMedianFilter = true;
	}
	else{
		tofcamInfo.bUsedMedianFilter = false;
	}

	if( tofcamInfo.edgeThresHold > 0 ){		
		tofcamInfo.bUsedEdgeFilter = true;	
#if 0	// test code [[
//		tofcamInfo.bUsedMedianFilter = true;
//		tofcamInfo.medianFilterSize = 3;
//		tofcamInfo.medianFilterIterations = 3;

		tofcamInfo.bUsedGaussFilter = true;
		tofcamInfo.gaussIteration = 8;
#endif	// test code ]]
	}
	else {
		tofcamInfo.bUsedEdgeFilter = false;
	}

		
	reqGrayScaleIntegrationTime0(response[1]);
	reqIntegrationTime3D(response[1]);
	reqMinAmplitude(response[1]);

	
	if( tofcamInfo.tofcamModeType == DISTANCE_GRAYSCALE_MODE 
		|| tofcamInfo.tofcamModeType == GRAYSCALE_MODE )
	{
		maxAmplitudeValue = 2047;
	}
	else{
		maxAmplitudeValue = 4095;
	}

	tofcamInfo.captureNetType = netType;

	printf("start Capture~~~ intTime = %d grayIntTime = %d\n", pCapOpt->integrationTime, pCapOpt->grayIntegrationTime);
}

void NSL1110AA::setKey(int cmdKey)
{	
	tofcamInfo.tofcamEvent_key = cmdKey;
}


// constructor
NSL1110AA::NSL1110AA( std::string ipaddr )
{	
	mIpaddrStr = ipaddr;
	exit_thtread = 1;

	tofcamBuff.overflow = 0;
	tofcamBuff.head_idx = 0;
	tofcamBuff.tail_idx = 0;

	memset(&tofcamInfo, 0, sizeof(tofcamInfo));
	memset(&tofcamImage, 0, sizeof(tofcamImage));

//	tofcamInfo.printBasicInfo = 1;
	
#ifdef DME660_ROTATE_IMAGE_90
	tofcamInfo.rotate_90 = 1;
#endif
#ifdef ENABLE_DRNU_TEMPERATURE
	tofcamInfo.drnuType = 1;
#endif
	tofcamInfo.integrationTimeGrayscale = 500; // 40000
	tofcamInfo.integrationTime3D = 500;
	tofcamInfo.GrayscaleGainType = 0;
	tofcamInfo.GrayscaleOffsetType = 0;
//	tofcamInfo.tofcamModeType = AMPLITEDE_DISTANCE_MODE;// distance & amplitude
//	tofcamInfo.tofcamModeType = AMPLITEDE_DISTANCE_EX_MODE; // distance & amplitude(grayscale)
	tofcamInfo.tofcamModeType = DISTANCE_GRAYSCALE_MODE;// distance & grayscale
//	tofcamInfo.tofcamModeType = GRAYSCALE_MODE;

#ifdef ROTATE_IMAGE_ADJUST_ROI
	if( tofcamInfo.rotate_90 != 0 ){
		tofcamInfo.roiXMin = ADJUST_ROI_XMIN;
		tofcamInfo.roiXMax = ADJUST_ROI_XMAX;
		tofcamInfo.roiYMin = ADJUST_ROI_YMIN;
		tofcamInfo.roiYMax = ADJUST_ROI_YMAX;
	}
	else{
		tofcamInfo.roiXMin = DEFAULT_ROI_XMIN;
		tofcamInfo.roiXMax = DEFAULT_ROI_XMAX;
		tofcamInfo.roiYMin = DEFAULT_ROI_YMIN;
		tofcamInfo.roiYMax = DEFAULT_ROI_YMAX;
	}
#else
	tofcamInfo.roiXMin = DEFAULT_ROI_XMIN;
	tofcamInfo.roiXMax = DEFAULT_ROI_XMAX;
	tofcamInfo.roiYMin = DEFAULT_ROI_YMIN;
	tofcamInfo.roiYMax = DEFAULT_ROI_YMAX;
#endif
	tofcamInfo.captureNetType = NONEMODEL_TYPE;
	tofcamInfo.hdr_mode = DEFAULT_HDR_MODE;
	tofcamInfo.saturationType = 0;
	tofcamInfo.overflowType = 0;
	
#ifdef RESIZE_IMAGE_MAT
	tofcamInfo.width = 640;
	tofcamInfo.height = 480;
#else
	tofcamInfo.width = DefaultWidth;
	tofcamInfo.height = DefaultHeight;
#endif

	initializeDME660();

	exit_thtread = 0;
#ifdef _WINDOWS
	unsigned threadID;
	hThread = (HANDLE)_beginthreadex(NULL, 0, &NSL1110AA::rxWrapper, this, 0, &threadID);

	double psdAngle = 0.0f;	//seobi psd angle 0'
	sin_angle = sin(psdAngle*PI/180.0);
	cos_angle = cos(psdAngle*PI/180.0);
	
	tofcamInfo.usedPointCloud = 1;
	lensTransform.initLensDistortionTable(STANDARD_FIELD);

	point_cloud_ptr = pcbVis();
	viewer = rgbVis(point_cloud_ptr);
#else
	pthread_create(&threadID, NULL, NSL1110AA::rxWrapper, this);
#endif

}


// destructor	
NSL1110AA::~NSL1110AA()
{
	printf("~NSL1110AA\n");
	closeLidar();
	return;
}




