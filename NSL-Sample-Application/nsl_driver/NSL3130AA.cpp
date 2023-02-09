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
#include <asm/termbits.h>
#include <sys/ioctl.h>
#include <arpa/inet.h>
#include <sys/time.h> 
#endif

#include "NSL3130AA.h"
#include "NSLFrame.h"

#ifdef HAVE_CV_CUDA
#include <opencv2/cudawarping.hpp>
#endif

using namespace cv;


//#define TOFCAM660_ROTATE_IMAGE_90
//#define ROTATE_IMAGE_ADJUST_ROI

#define RESIZE_IMAGE_MAT

#define __STREAMING_COMMAND__


#define DEFAULT_HDR_MODE	HDR_NONE_MODE
//#define DEFAULT_HDR_MODE	HDR_SPATIAL_MODE
//#define DEFAULT_HDR_MODE	HDR_TEMPORAL_MODE

#define opcode_index		9

#define DEFAULT_DIST_OFFSET_VALUE		0
#define OPCODE_SET_OFFSET				0x14

#define MASK_USED_ADC_OVERFLOW 			0x01
#define MASK_USED_SATURATION 			0x02

#define MASK_DRNU_COMPENSATION			0x01
#define MASK_TEMPERATURE_COMPENSATION 	0x02
#define MASK_GRAYSCALE_COMPENSATION 	0x04
#define MASK_AMBIENT_LIGHT_COMPENSATION	0x08


#define HOST_START_MARK					0xFA									   ///<Start marker for command (host to camera)
#define CAM_START_MARK					0xFA										   ///<Start marker for the data (camera to host)
#define SERIAL_END_MARK					0xCAFEBABE										   ///<End marker if no CRC is used
#define BAUDRATE660 					10000000
#define SERIAL_BUFFER_SIZE 				1024

#define DEFAULT_ROI_XMIN	0
#define DEFAULT_ROI_XMAX	319
#define DEFAULT_ROI_YMIN	0
#define DEFAULT_ROI_YMAX	239

#define ADJUST_ROI_XMIN		0
#define ADJUST_ROI_XMAX		319
#define ADJUST_ROI_YMIN		60
#define ADJUST_ROI_YMAX		179


#define _unused(x) 			((void)(x))


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



#define MAX_DISTANCEVALUE 12500

static const double PI = 3.14159265358979323846264338328;


static int maxDistanceValue = 12500;
static int maxAmplitudeValue = 2897;
static int maxValidValue = 15000;

static const int indexAmplitudeFactorColor = TOF660_NUM_COLORS / maxAmplitudeValue;

/*
	length 4byte 이후부터 
	data index 0, 1 = opcode
	data index 2 ~ = data 시작 
	{
		const uint16_t COMMAND_SET_ROI = 0;                                            ///<Set the ROI
		const uint16_t COMMAND_SET_INT_TIMES = 1;                                      ///<Set the integration times (all at once)
		const uint16_t COMMAND_GET_DISTANCE_AMPLITUDE = 2;                             ///<Get distance and amplitude (single or stream)
		const uint16_t COMMAND_GET_DISTANCE = 3;                                       ///<Get distance (single or stream)
		const uint16_t COMMAND_GET_GRAYSCALE = 5;                                      ///<Get grayscale (single or stream)
		const uint16_t COMMAND_STOP_STREAM = 6;                                        ///<Stop the stream
		const uint16_t COMMAND_GET_DCS = 7;                                            ///<Get DCS data (single or stream)
		const uint16_t COMMAND_GET_DISTANCE_GRAYSCALE = 8;                             ///<Get distance and GRAYSCALE (single or stream) :: seobi add
		const uint16_t COMMAND_SET_OFFSET = 20;                                        ///<Set the offset
		const uint16_t COMMAND_SET_MIN_AMPLITUDE = 21;                                 ///<Set the minimal amplitude
		const uint16_t COMMAND_SET_FILTER = 22;                                        ///<Set the filter settings (all at once)
		const uint16_t COMMAND_SET_MODULATION = 23;                                    ///<Set the modulation settings
		const uint16_t COMMAND_SET_BINNING = 24;                                       ///<Set the binning settings
		const uint16_t COMMAND_SET_HDR = 25;                                           ///<Set the HDR settings
		const uint16_t COMMAND_SET_SHUTTER_MODE = 26;                                  ///<Set the shutter mode
		const uint16_t COMMAND_SET_ABS = 27;                                           ///<Set the ABS (enable/disable)
		const uint16_t COMMAND_SET_COMPENSATION = 28;                                  ///<Set the compensations (enable/disable)
		const uint16_t COMMAND_SET_DLL_STEP = 29;                                      ///<Set the DLL step
		const uint16_t COMMAND_SHUTTER = 100;                                          ///<Force a shutter (in case of external shutter)
		const uint16_t COMMAND_CALIBRATE = 30;                                         ///<Calibrate DRNU
		const uint16_t COMMAND_CALIBRATE_PRODUCTION = 31;                              ///<Calibrate for production
		const uint16_t COMMAND_DELETE_CALIBRATION = 32;                                ///<Delete the calibration data
		const uint16_t COMMAND_DEBUG = 33;                                             ///<Debug command with different sub commands
		const uint16_t COMMAND_CALIBRATE_GRAYSCALE = 34;                               ///<Calibrate Grayscale
		const uint16_t COMMAND_CALIBRATE_AMBIENT_LIGHT = 35;                           ///<Calibrate Ambient Light
		const uint16_t COMMAND_READ_CHIP_INFORMATION = 36;                             ///<Read chip ID and wafer ID
		const uint16_t COMMAND_READ_FIRMWARE_RELEASE = 37;                             ///<Read firmware release
		const uint16_t COMMAND_GET_TEMPERATURE = 0x4A;                              		///<Command to read the temperature dec 74
		const uint16_t COMMAND_SET_DATA_IP_ADDRESS = 38;                               ///<Set the IP address of the data
		const uint16_t COMMAND_SET_GRAYSCALE_ILLUMINATION = 39;  						///<Configure illumination during grayscale acquisition
		const uint16_t COMMAND_SET_CAMERA_IP_SETTINGS= 40;
		const uint16_t COMMAND_SET_CAMERA_MAC_ADDRESS= 41;
		const uint16_t COMMAND_WRITE_REGISTER= 42;
		const uint16_t COMMAND_READ_REGISTER= 43;
		const uint16_t COMMAND_SEND_FIRMWARE_UPDATE = 200;                             ///<Send firmware update file
		const uint16_t COMMAND_IO_RESET = 44;											   ///<reset chip in case of NACK
		const uint16_t COMMAND_SYSTEM_RESET = 45;										   ///<system reset
		const uint16_t COMMAND_PSU_5V_ENABLE = 46;										   ///<psu 5v enable
		const uint16_t COMMAND_PSU_5V_DISABLE = 47;										   ///<psu 5v disable
		const uint16_t COMMAND_JUMP_TO_BOOTLOADER = 111;									/// Jump to Bootloader
	}
*/
static uint8_t initialCode660[][100]={	
//	 | ------- start marker --------| |--- length(opcode+data) -----| |-- opcode--| |------- data feild -----------| |-------- end marker --------|
	// COMMAND_SET_LOG_IP_ADDRESS :: 0.0.0.0
	{ 0xff ,0xff ,0xaa ,0x55 ,0x00 ,0x00 ,0x00 ,0x06 ,0x00 ,0x37 ,0x00 ,0x00 ,0x00 ,0x00 ,0xff ,0xff ,0x55 ,0xaa}
	// COMMAND_SET_INT_TIMES :: int3d(800),  int3d1(100), int3d2(50), grayint(100)
	,{0xff ,0xff ,0xaa ,0x55 ,0x00 ,0x00 ,0x00 ,0x0a ,0x00 ,0x01 ,0x03 ,0x20 ,0x00 ,0x64 ,0x00 ,0x32 ,0x00 ,0x64 ,0xff ,0xff ,0x55 ,0xaa}
	// COMMAND_SET_HDR :: DEFAULT_HDR_MODE
	,{0xff ,0xff ,0xaa ,0x55 ,0x00 ,0x00 ,0x00 ,0x03 ,0x00 ,0x19 ,DEFAULT_HDR_MODE ,0xff ,0xff ,0x55 ,0xaa}
	// COMMAND_SET_FILTER:: temper filter(2) 0 : THRESHOLD(0) 0 :: all off
	,{0xff ,0xff ,0xaa ,0x55 ,0x00 ,0x00 ,0x00 ,0x11 ,0x00 ,0x16 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0xff ,0xff ,0x55 ,0xaa}
	// COMMAND_SET_MODULATION : 0(VALUE_12MHZ)
	,{0xff ,0xff ,0xaa ,0x55 ,0x00 ,0x00 ,0x00 ,0x05 ,0x00 ,0x17 ,0x00 ,0x00 ,0x00 ,0xff ,0xff ,0x55 ,0xaa}
	// COMMAND_SET_ROI :: 320 x 240
	,{0xff ,0xff ,0xaa ,0x55 ,0x00 ,0x00 ,0x00 ,0x0a ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x01 ,0x3f ,0x00 ,0xef ,0xff ,0xff ,0x55 ,0xaa}
	// COMMAND_SET_ADC_OVERFLOW :: adc overflow on, saturation on
	,{0xff ,0xff ,0xaa ,0x55 ,0x00 ,0x00 ,0x00 ,0x04 ,0x00 ,0x0a ,0x01 ,0x01 ,0xff ,0xff ,0x55 ,0xaa}
	// COMMAND_SET_COMPENSATION :: Drnu(1), Temperature(1), Grayscaled(1), RESERVED(0)
	,{0xff ,0xff ,0xaa ,0x55 ,0x00 ,0x00 ,0x00 ,0x06 ,0x00 ,0x1c ,0x01 ,0x01 ,0x01 ,0x00 ,0xff ,0xff ,0x55 ,0xaa}
	// COMMAND_SET_MIN_AMPLITUDE :: 50
	,{0xff ,0xff ,0xaa ,0x55 ,0x00 ,0x00 ,0x00 ,0x04 ,0x00 ,0x15 ,0x00 ,0x32 ,0xff ,0xff ,0x55 ,0xaa}
	// COMMAND_SET_OFFSET :: 0
	,{0xff ,0xff ,0xaa ,0x55 ,0x00 ,0x00 ,0x00 ,0x06 ,0x00 ,0x14 ,0x00 ,0x00 ,0x00 ,0x00 ,0xff ,0xff ,0x55 ,0xaa}
	// COMMAND_SET_DATA_IP_ADDRESS :: 0.0.0.0
	,{0xff ,0xff ,0xaa ,0x55 ,0x00 ,0x00 ,0x00 ,0x06 ,0x00 ,0x26 ,0x00 ,0x00 ,0x00 ,0x00 ,0xff ,0xff ,0x55 ,0xaa}
	// COMMAND_SET_GRAYSCALE_ILLUMINATION ON
	,{0xff ,0xff ,0xaa ,0x55 ,0x00 ,0x00 ,0x00 ,0x03 ,0x00 ,0x27 ,0x01 ,0xff ,0xff ,0x55 ,0xaa} 
};


bool NSL3130AA::hasValidStartMarking(unsigned char *data, int data_len)
{
    return memcmp(data, START_MARKER, 4) == 0 ? true : false;
}

bool NSL3130AA::hasValidEndMarking(unsigned char * data, int data_len)
{
    return memcmp(data, END_MARKER, 4) == 0 ? true : false;
}

bool NSL3130AA::lengthIsCorrect(unsigned char * data, int data_len)
{
    int32_t expectedPayloadBytes = (data[PAYLOAD_SIZE_INDEX] << 24) + (data[PAYLOAD_SIZE_INDEX+1] << 16) + (data[PAYLOAD_SIZE_INDEX+2] << 8) + (data[PAYLOAD_SIZE_INDEX+3] << 0);

    if ((data_len -  PROTOCOL_OVERHEAD_SIZE) == expectedPayloadBytes)
        return true;

    return false;
}


bool NSL3130AA::isValidData(unsigned char *data, int data_len)
{
    if(data_len < TOF660_HEADER_SIZE)
        return false;

    if(hasValidStartMarking(data, data_len) == false)
        return false;

    if(hasValidEndMarking(data, data_len) == false)
        return false;

    if(lengthIsCorrect(data, data_len) == false)
        return false;

    return true;
}

uint16_t NSL3130AA::getUint16FromCharBuffer(const char *data)
{
    uint16_t value = static_cast<uint16_t>(((static_cast<uint8_t>(data[0])) << 8) + static_cast<uint8_t>(data[1]));
    return value;
}

int16_t NSL3130AA::getInt16FromCharBuffer(const char *data)
{
    int16_t value = static_cast<int16_t>((static_cast<uint8_t>(data[0]) << 8) + static_cast<uint8_t>(data[1]));
    return value;
}


int NSL3130AA::getCamInfo( uint8_t *data )
{
    tofcamInfo.header.version = data[0];
    tofcamInfo.header.dataType = data[1]<<8|data[2];//getUint16FromCharBuffer(&data[1]);
    tofcamInfo.header.width = data[3]<<8|data[4];//getUint16FromCharBuffer(&data[3]);
    tofcamInfo.header.height = data[5]<<8|data[6];//getUint16FromCharBuffer(&data[5]);
    tofcamInfo.header.roiX0 = data[7]<<8|data[8];//getUint16FromCharBuffer(&data[7]);
    tofcamInfo.header.roiY0 = data[9]<<8|data[10];//getUint16FromCharBuffer(&data[9]);
    tofcamInfo.header.roiX1 = data[11]<<8|data[12];//getUint16FromCharBuffer(&data[11]);
    tofcamInfo.header.roiY1 = data[13]<<8|data[14];//getUint16FromCharBuffer(&data[13]);
    tofcamInfo.header.intTime0 = data[15]<<8|data[16];//getUint16FromCharBuffer(&data[15]);
    tofcamInfo.header.intTime1 = data[17]<<8|data[18];//getUint16FromCharBuffer(&data[17]);
    tofcamInfo.header.intTime2 = data[19]<<8|data[20];//getUint16FromCharBuffer(&data[19]);
    tofcamInfo.header.temperature = data[21]<<8|data[22];//getInt16FromCharBuffer(&data[21]);
    tofcamInfo.header.offset = data[23]<<8|data[24];//getUint16FromCharBuffer(&data[23]);

#if 0
	printf("width = %d height = %d roiX=%d/%d roiY=%d/%d intTime0 = %d intTime1 = %d intTime2 = %d temper = %d offset = %d dataType = %d\n"
		, tofcamInfo.header.width
		, tofcamInfo.header.height
		, tofcamInfo.header.roiX0
		, tofcamInfo.header.roiX1
		, tofcamInfo.header.roiY0
		, tofcamInfo.header.roiY1
		, tofcamInfo.header.intTime0
		, tofcamInfo.header.intTime1
		, tofcamInfo.header.intTime2
		, tofcamInfo.header.temperature
		, tofcamInfo.header.offset
		, tofcamInfo.header.dataType );
#endif

	return 1;
}

uint16_t NSL3130AA::getHeaderUint16(uint8_t *pData, const int offset)
{
    uint16_t value0 = (unsigned char)(pData[offset]) & 0xFF;
    uint16_t value1 = (unsigned char)(pData[offset+1]) & 0xFF;

    uint16_t value = (value0 << 8) | value1;

    return value;
}

uint32_t NSL3130AA::getHeaderUint32(uint8_t *pData, const int offset)
{
    uint32_t value0 = (unsigned char)(pData[offset]) & 0xFF;
    uint32_t value1 = (unsigned char)(pData[offset+1]) & 0xFF;
    uint32_t value2 = (unsigned char)(pData[offset+2]) & 0xFF;
    uint32_t value3 = (unsigned char)(pData[offset+3]) & 0xFF;

    uint32_t value = (value0 << 24) | (value1 << 16) | (value2 << 8) | value3;

    return value;
}



int NSL3130AA::processSerialData(uint8_t *tempBuffer, int recvedLen, int resp_idx)
{

	while( tofcamInfo.receivedBytes > tofcamInfo.actualNumber 
		&& response[resp_idx][tofcamInfo.actualNumber] != CAM_START_MARK )
	{
		tofcamInfo.actualNumber++;
	}

	if( tofcamInfo.actualNumber == tofcamInfo.receivedBytes ){
//		printf("err::processSerialData  :: act = %d recv = %d all zero\n", tofcamInfo.actualNumber, tofcamInfo.receivedBytes);
		tofcamInfo.actualNumber = 0;
		tofcamInfo.receivedBytes = 0;
	}
	else if( tofcamInfo.receivedBytes + recvedLen > TOF660_BUFF_SIZE ){
		printf("err::processSerialData  :: size over = %d/%d/%d\n", tofcamInfo.actualNumber, tofcamInfo.receivedBytes, recvedLen);
		tofcamInfo.actualNumber = 0;
		tofcamInfo.receivedBytes = 0;
		return 0;
	}
	else if( tofcamInfo.actualNumber > 0 && tofcamInfo.receivedBytes > 0 ){
		printf("err::processSerialData  :: act = %d recv = %d\n", tofcamInfo.actualNumber, tofcamInfo.receivedBytes);
		
		if( tofcamInfo.actualNumber < tofcamInfo.receivedBytes ){
			memmove(response[resp_idx], &response[resp_idx][tofcamInfo.actualNumber], tofcamInfo.receivedBytes-tofcamInfo.actualNumber);
			tofcamInfo.receivedBytes-=tofcamInfo.actualNumber;
			tofcamInfo.actualNumber = 0;
		}
		else{
			tofcamInfo.actualNumber = 0;
			tofcamInfo.receivedBytes = 0;
		}
	}
	
	//Store the received frame at the required offset
	memcpy(&response[resp_idx][tofcamInfo.receivedBytes], tempBuffer, recvedLen);
	tofcamInfo.receivedBytes += recvedLen;

	if( tofcamInfo.receivedBytes < 6 ) return 0;

	uint32_t expectedSize = getHeaderUint32(response[resp_idx], 2);	// INDEX_DATA_SIZE
    if ((expectedSize > TOF660_BUFF_SIZE) || (expectedSize < 0)){
		printf("err::processSerialData :: expectedSize = %u rxSize = %d recvedLen = %d\n", expectedSize, tofcamInfo.receivedBytes, recvedLen);
		tofcamInfo.actualNumber = 0;
		tofcamInfo.receivedBytes = 0;
		return 0;
	}

	if( (int)expectedSize+10 <= tofcamInfo.receivedBytes ){
		printf("expect = %u recvedLen = %d rx = %d \n", expectedSize, recvedLen, tofcamInfo.receivedBytes);
		return expectedSize+10;
	}

	return 0;
}


int NSL3130AA::processUpdData(uint8_t *tempBuffer, int recvedLen, int resp_idx)
{
	uint16_t number = tempBuffer[0]<<8|tempBuffer[1];//getHeaderUint16(tempBuffer, 0);
	uint32_t totalSize = tempBuffer[2]<<24|tempBuffer[3]<<16|tempBuffer[4]<<8|tempBuffer[5];//getHeaderUint32(tempBuffer, 2);
	uint16_t payloadSize = tempBuffer[6]<<8|tempBuffer[7];//getHeaderUint16(tempBuffer, 6);
	uint32_t offset = tempBuffer[8]<<24|tempBuffer[9]<<16|tempBuffer[10]<<8|tempBuffer[11];//getHeaderUint32(tempBuffer, 8);
	uint32_t numPacket = tempBuffer[12]<<24|tempBuffer[13]<<16|tempBuffer[14]<<8|tempBuffer[15];//getHeaderUint32(tempBuffer, 12);
	uint32_t packetNumber = tempBuffer[16]<<24|tempBuffer[17]<<16|tempBuffer[18]<<8|tempBuffer[19];//getHeaderUint32(tempBuffer, 16);
	static uint32_t prevPNumber = 0;
	
	//A new data number is received, so clear anything
	if (number != tofcamInfo.actualNumber)
	{
		if( tofcamInfo.actualNumber != -1 ){
			char dbgbuff[100];
			sprintf(dbgbuff,"actual = %d number = %d/%d packetNumber = %d/%d\n", tofcamInfo.actualNumber, number, numPacket, prevPNumber, packetNumber);
			puts(dbgbuff);
		}
		tofcamInfo.actualNumber = number;
		tofcamInfo.receivedBytes = 0;
	}

	prevPNumber = packetNumber;
	
	//Store the received frame at the required offset
	memcpy(&response[resp_idx][tofcamInfo.receivedBytes], &tempBuffer[UDP_PACKET_HEADER_SIZE], payloadSize);
	tofcamInfo.receivedBytes += payloadSize;
	
#if 0
	printf("number = %d/%d total = %d payload = %d offset = %d numPack = %d packNum = %d recLen = %d/%d\n"
		, number 
		, tofcamInfo.actualNumber 
		, totalSize
		, payloadSize
		, offset
		, numPacket
		, packetNumber
		, recvedLen
		, tofcamInfo.receivedBytes);		
#endif
	//If the last packet is received, the whole data
	if ( packetNumber == (numPacket - 1))
	{
		if( totalSize != tofcamInfo.receivedBytes ){
			tofcamInfo.actualNumber = -1;
			return -1;
		}

		//printf("last number = %d\n", (numPacket - 1)); // 219
		return tofcamInfo.receivedBytes;
	}

	return 0;
}


int NSL3130AA::recvFromTcp(SOCKET sock, uint8_t *total_buf)
{
	static char read_buf[1024];
	uint16_t read_total_size = 0;
	int nbyte = 1;
	int ntotal_length = 0; // start : 4, length 4, end : 4
	struct timeval timeout;    
	fd_set readfds;

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
		printf("recvFromTcp TCP no response state = %d\n", state);
		return 0;
	}
	
	while( nbyte > 0 )
	{
		nbyte = recv( sock, (char*)read_buf, sizeof(read_buf), 0);
		if( nbyte > 0 ){
			memcpy(&total_buf[read_total_size] , read_buf , nbyte);
			read_total_size += nbyte;

			if( read_total_size >= PACKET_INFO_SIZE && ntotal_length == 0 ){ // start prefix(4) + length(4)
				ntotal_length = getHeaderUint32(total_buf, 4);
				ntotal_length += (PACKET_INFO_SIZE + TAIL_PREFIX_SIZE); // ntotal_length = start prefix(4) + length(4) + total_length + end prefix(4) ;
#if 0
				printf("read_total_size = %d ntotal_length = %d nbyte = %d\n", read_total_size, ntotal_length, nbyte);
				if( ntotal_length == 10 ){
					for(int i = 0;i<nbyte;i++){
						printf("%02x ", total_buf[i]);
					}
					printf("\n");
				}
#endif				
			}

			if( read_total_size >= ntotal_length ){
				//printf("read ok : [%d:%d]\n", read_total_size, ntotal_length );
				break;
			}
			else{
				//printf("ing...[%d/%d]\n", nbyte, grayscale_idx );
			}
		}
		else{
			printf("read... 0\n");
			read_total_size = 0;
		}
	}

	return read_total_size;
}


void NSL3130AA::setGrayScaledColor(cv::Mat &imageLidar, int x, int y, int value, double end_range )
{
	if (value == TOF660_SATURATION)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 255;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 128; 
	}
	else if (value == TOF660_ADC_OVERFLOW)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 169;
		imageLidar.at<Vec3b>(y, x)[1] = 14;
		imageLidar.at<Vec3b>(y, x)[2] = 255; 
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


int NSL3130AA::setDistanceColor(cv::Mat &imageLidar, int x, int y, int value )
{
	if( value == TOF660_LOW_AMPLITUDE )
	{
		imageLidar.at<Vec3b>(y, x)[0] = 0;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 0; 
	}
	else if (value == TOF660_SATURATION)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 255;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 128; 
	}
	else if (value == TOF660_ADC_OVERFLOW)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 169;
		imageLidar.at<Vec3b>(y, x)[1] = 14;
		imageLidar.at<Vec3b>(y, x)[2] = 255; 
	}
	else if(value == TOF660_INTERFERENCE)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 0;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 0; 
	}
	else if (value == TOF660_EDGE_DETECTED)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 0;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 0; 
	}
	else if (value == TOF660_BAD)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 0;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 0; 
	}
	else if(value == 0)
	{
//		imageLidar.at<Vec3b>(y, x)[0] = 255;
//		imageLidar.at<Vec3b>(y, x)[1] = 255;
//		imageLidar.at<Vec3b>(y, x)[2] = 255; 
		imageLidar.at<Vec3b>(y, x) = colorVector.at(colorVector.size()-1);
	}
	else if (value < 0)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 0;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 0; 
	}
	else if (value > maxDistanceValue)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 0;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 0; 
	}
	else{
		int index = (int)colorVector.size() - (value*(TOF660_NUM_COLORS / maxDistanceValue));
		if( index < 0 ){
			printf("error index = %d\n", index);
			index = (int)colorVector.size()-1;
		}
		else if( index > (int)colorVector.size() ){
			index = 0;
		}
		
		imageLidar.at<Vec3b>(y, x) = colorVector.at(index);
	}

	return value;
}

void NSL3130AA::setAmplitudeColor(cv::Mat &imageLidar, int x, int y, int value )
{
	if( value == TOF660_LOW_AMPLITUDE )
	{
		imageLidar.at<Vec3b>(y, x)[0] = 0;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 0; 
	}
	else if (value == TOF660_SATURATION)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 255;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 128; 
	}
	else if (value == TOF660_ADC_OVERFLOW)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 169;
		imageLidar.at<Vec3b>(y, x)[1] = 14;
		imageLidar.at<Vec3b>(y, x)[2] = 255; 
	}
	else if(value == TOF660_INTERFERENCE)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 0;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 0; 
	}
	else if (value == TOF660_EDGE_DETECTED)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 0;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 0; 
	}
	else if (value == TOF660_BAD)
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
		int index = value * indexAmplitudeFactorColor - 1;
		if( index < 0 ){
			printf("error index = %d\n", index);
			index = 0;
		}
		else if( index > (int)colorVector.size() ){
			index = (int)colorVector.size()-1;
		}
		
		imageLidar.at<Vec3b>(y, x) = colorVector.at(index);
	}

}

int NSL3130AA::getDistanceAmplitude(cv::Mat &imageDistance, cv::Mat &imageAmplitude, bool bUsedPointCloud)
{
	int x, y, index = 0;
	int stepY = 1;
	int maxHeight = tofcamInfo.header.height;
	int maxWidth = tofcamInfo.header.width;
	//uint16_t *pixelPtr = (uint16_t *)&procBuff[1][tofcamInfo.header.offset];

	if( tofcamInfo.config.hdr_mode == HDR_SPATIAL_MODE 
		&& tofcamInfo.header.height <= (TOF660_IMAGE_HEIGHT>>1) )
	{
		stepY = 2;
		maxHeight <<= 1;
	}

	memset(distanceTable, 0, sizeof(distanceTable));

//	printf("width = %d height = %d/%d hdr = %d\r\n", tofcamInfo.header.width, tofcamInfo.header.height, maxHeight, tofcamInfo.config.hdr_mode);
#ifdef _WINDOWS
	point_cloud_ptr->clear();
#endif

    for (y = 0; y < maxHeight; y += stepY)
	{
		for (x = 0; x < maxWidth; x++)
		{
			int pixelDistance = (procBuff[1][4*index+1+tofcamInfo.header.offset] << 8) + procBuff[1][4*index+0+tofcamInfo.header.offset];
			int pixelAmplitude = (procBuff[1][4*index+3+tofcamInfo.header.offset] << 8) + procBuff[1][4*index+2+tofcamInfo.header.offset];

			setDistanceColor(imageDistance, x, y, pixelDistance);
			if( tofcamInfo.tofcamModeType == AMPLITEDE_DISTANCE_EX_MODE ) 
				setGrayScaledColor(imageAmplitude, x, y, pixelAmplitude, maxAmplitudeValue);
			else if( tofcamInfo.tofcamModeType == DISTANCE_GRAYSCALE_MODE )
				setGrayScaledColor(imageAmplitude, x, y, pixelAmplitude, 2048.0);
			else 
				setAmplitudeColor(imageAmplitude, x, y, pixelAmplitude);

			distanceTable[y*TOF660_IMAGE_WIDTH+x] = pixelDistance;


			if(stepY==2){
				distanceTable[(y+1)*TOF660_IMAGE_WIDTH+x] = pixelDistance;

				if( tofcamInfo.tofcamModeType == AMPLITEDE_DISTANCE_EX_MODE ) 
					setGrayScaledColor(imageAmplitude, x, y+1, pixelAmplitude, maxAmplitudeValue);
				else if( tofcamInfo.tofcamModeType == DISTANCE_GRAYSCALE_MODE )
					setGrayScaledColor(imageAmplitude, x, y+1, pixelAmplitude, 2048.0);
				else 
					setAmplitudeColor(imageAmplitude, x, y+1, pixelAmplitude);

				setDistanceColor(imageDistance, x, y+1, pixelDistance);
			}
			
#ifdef _WINDOWS
			if( bUsedPointCloud ){
				pcl::PointXYZRGB point;
				pcl::PointXYZRGB basePoint;

				double outX = 0.0f, outY = 0.0f, outZ = 0.0f;

				if( pixelDistance < TOF660_LIMIT_FOR_VALID_DATA ){
					lensTransform.transformPixel(tofcamInfo.config.roi_xMin+x, tofcamInfo.config.roi_yMin+y, pixelDistance, outX, outY, outZ, sin_angle, cos_angle);
				}
				
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

			index++;
		}
	}

//	printf("maxAmplitude = %d\n", maxAmplitude);
	return 0;
}


int NSL3130AA::getGrayscaled(cv::Mat &imageLidar, bool bUsedPointCloud)
{
	int index = 0;
	int maxHeight = tofcamInfo.header.height;
	int maxWidth = tofcamInfo.header.width;
	int stepY = 1;
	
	if( tofcamInfo.config.hdr_mode == HDR_SPATIAL_MODE 
		&& tofcamInfo.header.height <= (TOF660_IMAGE_HEIGHT>>1) )
	{
		stepY = 2;
		maxHeight <<= 1;
	}

	memset(distanceTable, 0, sizeof(distanceTable));
	//	printf("width = %d height = %d/%d hdr = %d\r\n", tofcamInfo.header.width, tofcamInfo.header.height, maxHeight, tofcamInfo.config.hdr_mode);
#ifdef _WINDOWS
	point_cloud_ptr->clear();
#endif

	for(int y = 0; y < maxHeight; y+=stepY)
	{
		for(int x = 0; x < maxWidth; x++)
		{
			int pixelGrayscale = ( procBuff[1][2*index+1+tofcamInfo.header.offset] << 8) +  procBuff[1][2*index+0+tofcamInfo.header.offset];		

			if( tofcamInfo.tofcamModeType == GRAYSCALE_MODE )
				setGrayScaledColor( imageLidar, x, y, pixelGrayscale, 2048.0);
			else{ // DISTANCE_MODE
				distanceTable[y*TOF660_IMAGE_WIDTH+x] = pixelGrayscale;
				setDistanceColor(imageLidar, x, y, pixelGrayscale);
			}

			if(stepY==2){
				if( tofcamInfo.tofcamModeType == GRAYSCALE_MODE )
					setGrayScaledColor( imageLidar, x, y+1, pixelGrayscale, 2048.0);
				else{
					distanceTable[(y+1)*TOF660_IMAGE_WIDTH+x] = pixelGrayscale;
					setDistanceColor(imageLidar, x, y+1, pixelGrayscale);
				}
			}

			if( tofcamInfo.tofcamModeType != GRAYSCALE_MODE ){
#ifdef _WINDOWS
				if( bUsedPointCloud ){
					pcl::PointXYZRGB point;
					pcl::PointXYZRGB basePoint;

					double outX = 0.0f, outY = 0.0f, outZ = 0.0f;

					if( pixelGrayscale < TOF660_LIMIT_FOR_VALID_DATA ){
						lensTransform.transformPixel(tofcamInfo.config.roi_xMin+x, tofcamInfo.config.roi_yMin+y, pixelGrayscale, outX, outY, outZ, sin_angle, cos_angle);
					}
					
					point.x = (double)(outX/1000);
					point.y = (double)(outY/1000);
					point.z = (double)(outZ/1000);

					point.b = imageLidar.at<Vec3b>(y, x)[0];
					point.g = imageLidar.at<Vec3b>(y, x)[1];
					point.r = imageLidar.at<Vec3b>(y, x)[2];

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
			
			index++;
		}
	}

	return 0;
}


uint8_t NSL3130AA::getCommandByType( int modeType )
{
	uint8_t cmd = 0;

	switch(modeType){
		case AMPLITEDE_DISTANCE_MODE:
		case AMPLITEDE_DISTANCE_EX_MODE:
			cmd = 0x02;
			break;
		case DISTANCE_MODE:
			cmd = 0x03;
			break;
		case DISTANCE_GRAYSCALE_MODE: // nxp 수정후 사용 가능 
			cmd = 0x08;
			break;
		case GRAYSCALE_MODE:
			cmd = 0x05;
		default:
			break;
	}
	
	return cmd;
}

int NSL3130AA::sendToDev(SOCKET sock, uint8_t *pData, int nLen)
{
	static uint8_t serialData[SERIAL_BUFFER_SIZE];
	int pktLen = 0;
	
	if( ttySerial ){
		uint32_t endMark = SERIAL_END_MARK;
		
		serialData[0] = HOST_START_MARK;
		serialData[1] = (uint8_t)((nLen >> 24) & 0xFF);
		serialData[2] = (uint8_t)((nLen >> 16) & 0xFF);
		serialData[3] = (uint8_t)((nLen >> 8) & 0xFF);
		serialData[4] = (uint8_t)((nLen >> 0) & 0xFF);
		
		memcpy(&serialData[5], pData, nLen);
		
		serialData[SERIAL_BUFFER_SIZE-4] = (uint8_t)((endMark >> 24) & 0xFF);
		serialData[SERIAL_BUFFER_SIZE-3] = (uint8_t)((endMark >> 16) & 0xFF);
		serialData[SERIAL_BUFFER_SIZE-2] = (uint8_t)((endMark >> 8) & 0xFF);
		serialData[SERIAL_BUFFER_SIZE-1] = (uint8_t)((endMark >> 0) & 0xFF);

//		int ret = write(sock, (const void *)serialData, SERIAL_BUFFER_SIZE);
//		_unused(ret);
//		pktLen = rxSerial(serialData, SERIAL_BUFFER_SIZE, false);
	}
	else{
		int nTotalLen = 0;
		memcpy(&serialData[0], START_MARKER, 4); 
		nTotalLen+=4;

		serialData[nTotalLen+0] = (uint8_t)((nLen >> 24) & 0xff);
		serialData[nTotalLen+1] = (uint8_t)((nLen >> 16) & 0xff);
		serialData[nTotalLen+2] = (uint8_t)((nLen >> 8) & 0xff);
		serialData[nTotalLen+3] = (uint8_t)((nLen >> 0) & 0xff);
		nTotalLen+=4;

		memcpy(&serialData[nTotalLen], pData, nLen);
		nTotalLen+=nLen;

		memcpy(&serialData[nTotalLen], END_MARKER, 4); 
		nTotalLen+=4;

		int ret = send(sock, (char *)serialData, nTotalLen, 0);
		_unused(ret);
		pktLen = recvFromTcp(sock, serialData);
	}

	return pktLen;
}

void NSL3130AA::reqOverflow(SOCKET control_sock)
{
	uint8_t data[10] = {0x00, 0x0a, 0x00, 0x00};
	uint32_t data_len = 4;

	data[2] = (tofcamInfo.config.saturatedFlag & MASK_USED_ADC_OVERFLOW) ? 1 : 0;
	data[3] = (tofcamInfo.config.saturatedFlag & MASK_USED_SATURATION) ? 1 : 0;

	int bComplete = sendToDev(control_sock, data, data_len);

	printf("reqOverflow : read data complete = %d \n", bComplete);
}


void NSL3130AA::reqHdrMode(SOCKET control_sock)
{
	uint8_t data[] = {0x00, 0x19, 0x00};
	uint32_t data_len = 3;	

	data[2] = tofcamInfo.config.hdr_mode;
	
	int bComplete = sendToDev(control_sock, data, data_len);

	printf("reqHdrMode : mode = %d int3d=%d, hdr1=%d, hdr2=%d gray=%d\n", tofcamInfo.config.hdr_mode, tofcamInfo.config.integrationTime3D, tofcamInfo.config.integrationTime3DHdr1, tofcamInfo.config.integrationTime3DHdr2, tofcamInfo.config.integrationTimeGrayScale);
}



void NSL3130AA::reqCompensation(SOCKET control_sock)
{
	uint8_t data[6] = {0x00, 0x1c, 0x00, 0x00, 0x00, 0x00}; // get grayscale
	uint32_t data_len = 6;	

	data[2] = (tofcamInfo.config.compensationFlag & MASK_DRNU_COMPENSATION) ? 1 : 0;
	data[3] = (tofcamInfo.config.compensationFlag & MASK_TEMPERATURE_COMPENSATION) ? 1 : 0;
	data[4] = (tofcamInfo.config.compensationFlag & MASK_GRAYSCALE_COMPENSATION) ? 1 : 0;
	data[5] = (tofcamInfo.config.compensationFlag & MASK_AMBIENT_LIGHT_COMPENSATION) ? 1 : 0;
	
	int bComplete = sendToDev(control_sock, data, data_len);
	printf("reqCompensation : read data complete = %d, comp = %x \n", bComplete, tofcamInfo.config.compensationFlag);
}



int NSL3130AA::reqStreamingFrame(SOCKET control_sock)
{
	uint8_t data[10] = {0x00, 0x02, VALUE_STREAMING_MEASUREMENT}; // get distance & amplitude
	uint32_t data_len = 3;	
	int cmdType = getCommandByType(tofcamInfo.tofcamModeType);	

	data[1] = cmdType;

	int bComplete = sendToDev(control_sock, data, data_len);
	printf("reqStreamingFrame : read data complete = %d \n", bComplete);

	return bComplete;
}



void NSL3130AA::reqSingleFrame(SOCKET control_sock, int modeType)
{
//	uint8_t data[3] = {0x00, 0x02, VALUE_AUTO_REPEAT_MEASUREMENT};
	uint8_t data[3] = {0x00, 0x02, VALUE_SINGLE_MEASUREMENT};
	uint32_t data_len = 3;	
	int cmdType = getCommandByType(modeType);	

	data[1] = cmdType;
	
	int bComplete = sendToDev(control_sock, data, data_len);

//	printf("reqSingleFrame : modeType = %d \n", modeType);


}


void NSL3130AA::reqStopStream(SOCKET control_sock)
{
	uint8_t data[2] = {0x00, 0x06};
	uint32_t data_len = 2;	

	int bComplete = sendToDev(control_sock, data, data_len);
	printf("reqStopStream : read data complete = %d \n", bComplete);
}

void NSL3130AA::reqIntegrationTime(SOCKET control_sock)
{
	uint8_t data[] = {0x00 ,0x01 ,0x00 ,0x64 ,0x03 ,0xe8 ,0x00 ,0x00 ,0x55 ,0xf0};
	uint32_t data_len = 10;	

	data[2] = (tofcamInfo.config.integrationTime3D >> 8) & 0xFF;
	data[3] = tofcamInfo.config.integrationTime3D & 0xFF;

	data[4] = (tofcamInfo.config.integrationTime3DHdr1 >> 8) & 0xFF;
	data[5] = tofcamInfo.config.integrationTime3DHdr1 & 0xFF;

	data[6] = (tofcamInfo.config.integrationTime3DHdr2 >> 8) & 0xFF;
	data[7] = tofcamInfo.config.integrationTime3DHdr2 & 0xFF;

	data[8] = (tofcamInfo.config.integrationTimeGrayScale >> 8) & 0xFF;
	data[9] = tofcamInfo.config.integrationTimeGrayScale & 0xFF;

	int bComplete = sendToDev(control_sock, data, data_len);
	printf("setIntegrationTime3D : %d\n", tofcamInfo.config.integrationTime3D);

}


void NSL3130AA::reqFilterParameter(SOCKET control_sock)
{
	uint8_t data[] = {0x00 ,0x16 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00};
	uint32_t data_len = 17;	

	data[2] = (tofcamInfo.config.temporalFilterFactorActual>>8)&0xFF;
	data[3] = (tofcamInfo.config.temporalFilterFactorActual>>0)&0xFF;
	data[4] = (tofcamInfo.config.temporalFilterThreshold>>8)&0xFF;
	data[5] = (tofcamInfo.config.temporalFilterThreshold>>0)&0xFF;
	data[6] = tofcamInfo.config.medianFilterEnable ? 1 : 0;
	data[7] = tofcamInfo.config.averageFilterEnable ? 1 : 0;
	data[8] = (tofcamInfo.config.edgeFilterThreshold>>8)&0xFF;
	data[9] = (tofcamInfo.config.edgeFilterThreshold>>0)&0xFF;
	data[10] = tofcamInfo.config.interferenceUseLashValueEnable ? 1 : 0;
	data[11] = (tofcamInfo.config.interferenceLimit>>8)&0xFF;
	data[12] = (tofcamInfo.config.interferenceLimit>>0)&0xFF;
	data[13] = 0;//(camInfo.config.edgefilterThresholdLow>>8)&0xFF;
	data[14] = 0;//(camInfo.config.edgefilterThresholdLow>>0)&0xFF;
	data[15] = 0;//(camInfo.config.edgefilterThresholdHigh>>8)&0xFF;
	data[16] = 0;//(camInfo.config.edgefilterThresholdHigh>>0)&0xFF;

	int bComplete = sendToDev(control_sock, data, data_len);
	printf("reqFilterParameter : %d\n", bComplete);

}


void NSL3130AA::reqMinAmplitude(SOCKET control_sock)
{
	uint8_t data[] = {0x00 ,0x15 ,0x00 ,0x1e};
	uint32_t data_len = 4;	

	data[2] = (tofcamInfo.config.minAmplitude>>8)&0xFF;
	data[3] = (tofcamInfo.config.minAmplitude>>0)&0xFF;
	
	int bComplete = sendToDev(control_sock, data, data_len);
	printf("reqMinAmplitude : %d\n", tofcamInfo.config.minAmplitude);

}

void NSL3130AA::reqSetROI(SOCKET control_sock)
{
	uint8_t data[] = {0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x01 ,0x3f ,0x00 ,0xef};
	uint32_t data_len = 10;	

#ifdef ROTATE_IMAGE_ADJUST_ROI
	if( tofcamInfo.rotate_90 != 0 ){
		tofcamInfo.config.roi_xMin = ADJUST_ROI_XMIN;
		tofcamInfo.config.roi_xMax = ADJUST_ROI_XMAX;
		tofcamInfo.config.roi_yMin = ADJUST_ROI_YMIN;
		tofcamInfo.config.roi_yMax = ADJUST_ROI_YMAX;
	}
	else{
		tofcamInfo.config.roi_xMin = DEFAULT_ROI_XMIN;
		tofcamInfo.config.roi_xMax = DEFAULT_ROI_XMAX;
		tofcamInfo.config.roi_yMin = DEFAULT_ROI_YMIN;
		tofcamInfo.config.roi_yMax = DEFAULT_ROI_YMAX;
	}

	data[2] = (tofcamInfo.config.roi_xMin>>8)&0xFF;
	data[3] = (tofcamInfo.config.roi_xMin>>0)&0xFF;

	data[4] = (tofcamInfo.config.roi_yMin>>8)&0xFF;
	data[5] = (tofcamInfo.config.roi_yMin>>0)&0xFF;

	data[6] = (tofcamInfo.config.roi_xMax>>8)&0xFF;
	data[7] = (tofcamInfo.config.roi_xMax>>0)&0xFF;

	data[8] = (tofcamInfo.config.roi_yMax>>8)&0xFF;
	data[9] = (tofcamInfo.config.roi_yMax>>0)&0xFF;

#endif

	int bComplete = sendToDev(control_sock, data, data_len);
	printf("reqSetROI rotate 90 : %d\n", tofcamInfo.rotate_90);
}

void NSL3130AA::reqGrayscaleLedControl(SOCKET control_sock)
{
	uint8_t data[] = {0x00 ,0x27 ,0x01};
	uint32_t data_len = 3;	

	data[2] = tofcamInfo.led_control;	// ON : 1, OFF : 0
	
	int bComplete = sendToDev(control_sock, data, data_len);
	printf("reqGrayscaleLedControl : read data complete = %d bOn = %d\n", bComplete, tofcamInfo.led_control);
}

int NSL3130AA::setSerialBaudrate(void)
{
	int fd = -1;
	return fd;
}


SOCKET NSL3130AA::InitializeControlsocket(void)
{
	SOCKET control_sock;
	struct sockaddr_in   server_addr;
	memset(&server_addr, 0, sizeof(server_addr));
	server_addr.sin_family = AF_INET;
	server_addr.sin_port = htons(TOF660_PORT);
	server_addr.sin_addr.s_addr = inet_addr(mIpaddr.c_str());

	control_sock = socket(PF_INET, SOCK_STREAM, 0);
	if(-1 == control_sock)
	{
		printf("Can not open socket\n");
		exit( 1);
	}
   
	if(-1 == connect(control_sock, (struct sockaddr*)&server_addr, sizeof(server_addr)))
	{
		printf("Socket not connect\n");
		exit(1);
   	}

	return control_sock;
}

SOCKET NSL3130AA::InitializeDataSocket(void)
{
	SOCKET data_socket;
	struct sockaddr_in si_me;

	data_socket = socket(PF_INET, SOCK_DGRAM, 0);
	if(-1 == data_socket)
	{
		printf("Can not open socket\n");
		exit(1);
	}
	
	
	memset(&si_me, 0, sizeof(si_me));
	si_me.sin_family = AF_INET;
	si_me.sin_port = htons(45454);
	si_me.sin_addr.s_addr = htonl(INADDR_ANY);
#ifdef _WINDOWS	
	int ret = bind(data_socket, (SOCKADDR*)&si_me, sizeof(si_me));
	if (ret == SOCKET_ERROR) {
		closesocket(data_socket);
		exit(0);
	}

	int sock_opt = 65535;
	int sock_len = sizeof(sock_opt);
	setsockopt(data_socket, SOL_SOCKET, SO_RCVBUF, (char*)&sock_opt, sock_len);
	printf("UDP socket id = %lld\n", data_socket);
#else
	if( bind(data_socket, (struct sockaddr*)&si_me, sizeof(si_me)) < 0 )
	{
		printf("error udp bind...\n");
		exit(0);
	}


	int sock_opt = 160000;
	socklen_t sock_len = sizeof(sock_opt);	
	setsockopt(data_socket, SOL_SOCKET, SO_RCVBUF, &sock_opt, sock_len);

	printf("UDP socket id = %d\n", data_socket);
#endif

	return data_socket;
}

double NSL3130AA::interpolate( double x, double x0, double y0, double x1, double y1){

    if( x1 == x0 ){
        return y0;
    } else {
        return ((x-x0)*(y1-y0)/(x1-x0) + y0);
    }

}


void NSL3130AA::createColorMapPixel(int numSteps, int indx, unsigned char &red, unsigned char &green, unsigned char &blue)
{
    double k = 1;
    double BIT0 = -0.125 * k - 0.25;
    double BIT1 = BIT0 + 0.25 * k;
    double BIT2 = BIT1 + 0.25 * k;
    double BIT3 = BIT2 + 0.25 * k;

    double G0 = BIT1;
    double G1 = G0 + 0.25 * k;
    double G2 = G1 + 0.25 * k;
    double G3 = G2 + 0.25 * k + 0.125;

    double R0 = BIT2;
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


    if( i>= BIT0 && i < BIT1 ){
        blue = (unsigned char)interpolate(i, BIT0, 0, BIT1, 255);
    } else if((i >= BIT1)&&(i < BIT2)){
        blue = 255;
    } else if((i >= BIT2)&&(i < BIT3)) {
        blue = (unsigned char)interpolate(i, BIT2, 255, BIT3, 0);
    } else{
        blue = 0;
    }

}

uint8_t *NSL3130AA::convertParameterFromConfig(uint8_t *pData, int nLen)
{
	switch(pData[opcode_index])
	{
		case COMMAND_SET_INT_TIMES:
			pData[10] = (tofcamInfo.config.integrationTime3D>>8)&0xFF;
			pData[11] = (tofcamInfo.config.integrationTime3D>>0)&0xFF;
			
			pData[12] = (tofcamInfo.config.integrationTime3DHdr1>>8)&0xFF;
			pData[13] = (tofcamInfo.config.integrationTime3DHdr1>>0)&0xFF;
			
			pData[14] = (tofcamInfo.config.integrationTime3DHdr2>>8)&0xFF;
			pData[15] = (tofcamInfo.config.integrationTime3DHdr2>>0)&0xFF;
			
			pData[16] = (tofcamInfo.config.integrationTimeGrayScale>>8)&0xFF;
			pData[17] = (tofcamInfo.config.integrationTimeGrayScale>>0)&0xFF;
			break;
		case COMMAND_SET_HDR:
			pData[10] = tofcamInfo.config.hdr_mode;
			break;
		case COMMAND_SET_MODULATION:
			pData[10] = tofcamInfo.config.mod_frequency;
			pData[11] = tofcamInfo.config.mod_channel;
			pData[12] = tofcamInfo.config.mod_autoChannelEnabled;
			break;
		case COMMAND_SET_ROI:
			pData[10] = (tofcamInfo.config.roi_xMin>>8)&0xFF;
			pData[11] = (tofcamInfo.config.roi_xMin>>0)&0xFF;
			
			pData[12] = (tofcamInfo.config.roi_yMin>>8)&0xFF;
			pData[13] = (tofcamInfo.config.roi_yMin>>0)&0xFF;
			
			pData[14] = (tofcamInfo.config.roi_xMax>>8)&0xFF;
			pData[15] = (tofcamInfo.config.roi_xMax>>0)&0xFF;
			
			pData[16] = (tofcamInfo.config.roi_yMax>>8)&0xFF;
			pData[17] = (tofcamInfo.config.roi_yMax>>0)&0xFF;
			break;
		case COMMAND_SET_ADC_OVERFLOW:
			pData[10] = (tofcamInfo.config.saturatedFlag & MASK_USED_ADC_OVERFLOW) ? 1 : 0;
			pData[11] = (tofcamInfo.config.saturatedFlag & MASK_USED_SATURATION) ? 1 : 0;
			break;
		case COMMAND_SET_COMPENSATION:
			pData[10] = (tofcamInfo.config.compensationFlag & MASK_DRNU_COMPENSATION) ? 1 : 0;
			pData[11] = (tofcamInfo.config.compensationFlag & MASK_TEMPERATURE_COMPENSATION) ? 1 : 0;
			pData[12] = (tofcamInfo.config.compensationFlag & MASK_GRAYSCALE_COMPENSATION) ? 1 : 0;
			pData[13] = (tofcamInfo.config.compensationFlag & MASK_AMBIENT_LIGHT_COMPENSATION) ? 1 : 0;
			break;
		case COMMAND_SET_OFFSET:
			pData[10] = (tofcamInfo.config.drnuOffset[tofcamInfo.config.mod_frequency]>>8)&0xFF;
			pData[11] = (tofcamInfo.config.drnuOffset[tofcamInfo.config.mod_frequency]>>0)&0xFF;
			break;
		case COMMAND_SET_MIN_AMPLITUDE:
			pData[10] = (tofcamInfo.config.minAmplitude>>8)&0xFF;
			pData[11] = (tofcamInfo.config.minAmplitude>>0)&0xFF;
			break;
		case COMMAND_SET_FILTER:
			pData[10] = (tofcamInfo.config.temporalFilterFactorActual>>8)&0xFF;
			pData[11] = (tofcamInfo.config.temporalFilterFactorActual>>0)&0xFF;
			pData[12] = (tofcamInfo.config.temporalFilterThreshold>>8)&0xFF;
			pData[13] = (tofcamInfo.config.temporalFilterThreshold>>0)&0xFF;
			pData[14] = tofcamInfo.config.medianFilterEnable ? 1 : 0;
			pData[15] = tofcamInfo.config.averageFilterEnable ? 1 : 0;
			pData[16] = (tofcamInfo.config.edgeFilterThreshold>>8)&0xFF;
			pData[17] = (tofcamInfo.config.edgeFilterThreshold>>0)&0xFF;
			pData[18] = tofcamInfo.config.interferenceUseLashValueEnable ? 1 : 0;
			pData[19] = (tofcamInfo.config.interferenceLimit>>8)&0xFF;
			pData[20] = (tofcamInfo.config.interferenceLimit>>0)&0xFF;
			pData[21] = 0;//(camInfo.config.edgefilterThresholdLow>>8)&0xFF;
			pData[22] = 0;//(camInfo.config.edgefilterThresholdLow>>0)&0xFF;
			pData[23] = 0;//(camInfo.config.edgefilterThresholdHigh>>8)&0xFF;
			pData[24] = 0;//(camInfo.config.edgefilterThresholdHigh>>0)&0xFF;
			break;
		case COMMAND_STOP_STREAM:
			break;
		default:
			printf("undefined OPCODE = 0x%02X\n", pData[opcode_index]);
			break;
	}

	return pData;
}


void NSL3130AA::initializeTofcam660(SOCKET socket)
{	
	int numSteps = TOF660_NUM_COLORS;
	unsigned char red, green, blue;

	for(int i=0;  i< numSteps; i++)
	{
	  createColorMapPixel(numSteps, i, red, green, blue);
	  colorVector.push_back(Vec3b(red, green, blue));
	}

#if 1
	int size = sizeof(initialCode660) / sizeof(initialCode660[0]);
	for(int i = 0; i < size ; i++){
		int pktLen = (initialCode660[i][4]<<24 | initialCode660[i][5]<<16 | initialCode660[i][6]<<8 | initialCode660[i][7]);

//		printf(":: tx_len = %d\n", pktLen);
		printf("initicode = %02X retLen = %d ttySerial = %d\n", initialCode660[i][9], pktLen, ttySerial);
		
		int bComplete = sendToDev(socket, &convertParameterFromConfig(initialCode660[i], pktLen)[8], pktLen);
	}
#endif
	reqIntegrationTime(socket);
	reqSetROI(socket);
	reqOverflow(socket);

	printf("end initializeTofcam660()\n");
}

void NSL3130AA::rxSocket(uint8_t *socketbuff, int buffLen) 
{
	struct sockaddr_in si_other;
	socklen_t addr_size;
	int nbyte = 0;
	struct timeval timeout;    
	fd_set readfds;
	int totalLen = 0;

	tofcamInfo.actualNumber = -1;
#ifndef __STREAMING_COMMAND__
	reqSingleFrame(tofcamInfo.control_sock, tofcamInfo.tofcamModeType);
#endif

	const auto& time_cap0 = std::chrono::steady_clock::now();

	do
	{
#if 1	
		FD_ZERO(&readfds);
		FD_SET(tofcamInfo.data_sock, &readfds);
		timeout.tv_sec = 0;
		timeout.tv_usec = 500000;  
		
#ifdef _WINDOWS
		int state = select(0, &readfds, NULL, NULL, &timeout);
#else
		int state = select(tofcamInfo.data_sock+1, &readfds, NULL, NULL, &timeout);
#endif
		if( state == 0 || state == -1 ){  //timeout , error
			printf("rxSock datagrame no response state = %d sock = %lld\n", state, tofcamInfo.data_sock);
			return;
		}
#endif		
		addr_size = sizeof(si_other);
		int data_len = recvfrom(tofcamInfo.data_sock, (char *)socketbuff, 1500, 0, (struct sockaddr*)&si_other, &addr_size);
		if (data_len > 0)
		{
			totalLen = processUpdData((uint8_t*)socketbuff, data_len, 1);
			if( totalLen < 0 ){
#ifndef __STREAMING_COMMAND__
				printf("----- retry reqSingleFrame\n");
				reqSingleFrame(tofcamInfo.control_sock, tofcamInfo.tofcamModeType);
#endif
			}
		}
		else{
			printf("err data_len = %d\n", data_len);
		}
	}while( totalLen <= 0 && !exit_thtread );

	if( !exit_thtread )
	{
		const auto& time_cap1 = std::chrono::steady_clock::now();
		double time_cam = (time_cap1 - time_cap0).count() / 1000000.0;
//		printf("  Tofcam-Rx:		   %9.3lf [msec] len = %d\n", time_cam, totalLen);		// 75ms
#if 1
		EnterCriticalSection(&tofcamBuff.lock);

		tofcamBuff.bufGrayLen[tofcamBuff.head_idx] = 0; 				
		memcpy(tofcamBuff.tofcamBuf[tofcamBuff.head_idx], response[1], totalLen);
		tofcamBuff.bufLen[tofcamBuff.head_idx] = totalLen;
		ADD_TOFCAM_BUFF(tofcamBuff, TOFCAM_ETH_BUFF_SIZE);

		LeaveCriticalSection(&tofcamBuff.lock);
#endif
	}
	
}


int NSL3130AA::rxSerial(uint8_t *socketbuff, int buffLen, bool addQue) 
{
	int nbyte = 0;
	struct timeval timeout;    
	fd_set readfds;
	int totalLen = 0;

#ifndef __STREAMING_COMMAND__
	if( GET_BUFF_CNT(tofcamBuff, TOFCAM_ETH_BUFF_SIZE) != 0 ) return 0;
	reqSingleFrame(tofcamInfo.control_sock, tofcamInfo.tofcamModeType);
#endif

	tofcamInfo.actualNumber = 0;

	do
	{
#if 1
		FD_ZERO(&readfds);
		FD_SET(tofcamInfo.control_sock, &readfds);
		timeout.tv_sec = 0;
		timeout.tv_usec = 500000;

#ifdef _WINDOWS
		int state = select(0, &readfds, NULL, NULL, &timeout);
#else
		int state = select(tofcamInfo.control_sock+1, &readfds, NULL, NULL, &timeout);
#endif
		if( state == 0 || state == -1 ){  //timeout , error
			printf("rxSerial no response state = %d\n", state);
			return 0;
		}
#endif
		int data_len = recv(tofcamInfo.control_sock, (char *)socketbuff, buffLen-tofcamInfo.receivedBytes, 0);
		if (data_len > 0)
		{
			totalLen = processSerialData(socketbuff, data_len, 1);
			if( totalLen < 0 ){
#ifndef __STREAMING_COMMAND__
				printf("----- retry reqSingleFrame\n");
				reqSingleFrame(tofcamInfo.control_sock, tofcamInfo.tofcamModeType);
#endif
			}
		}
		else{
			printf("err data_len = %d\n", data_len);
		}
	}while( totalLen <= 0 && !exit_thtread );

	if( !exit_thtread && addQue == true )
	{
		unsigned int type = response[1][1];

		if( type == 1 ){
			EnterCriticalSection(&tofcamBuff.lock);

			tofcamBuff.bufGrayLen[tofcamBuff.head_idx] = 0;
			memcpy(tofcamBuff.tofcamBuf[tofcamBuff.head_idx], &response[1][6], totalLen-10);
			tofcamBuff.bufLen[tofcamBuff.head_idx] = totalLen;
			ADD_TOFCAM_BUFF(tofcamBuff, TOFCAM_ETH_BUFF_SIZE);
			
			LeaveCriticalSection(&tofcamBuff.lock);
		}
		else{
			printf("err recv data type = [%x]%d totalLen = %d recv = %d\n", response[1][0], type, totalLen, tofcamInfo.receivedBytes);
		}

		if( tofcamInfo.receivedBytes > totalLen ){
			memmove(response[1], &response[1][totalLen], tofcamInfo.receivedBytes-totalLen);
			tofcamInfo.receivedBytes -= totalLen;

			printf("memmove recv = %d\n", tofcamInfo.receivedBytes);
		}
		else{
			tofcamInfo.receivedBytes = 0;
		}
	}
	else if( addQue == false ){
		tofcamInfo.receivedBytes = 0;
	}

	return totalLen;
}

void NSL3130AA::keyProc()
{
	if( tofcamInfo.tofcamEvent_key != 0 )
	{		
		// black & white (grayscale) mode
		if( tofcamInfo.tofcamEvent_key == 'b' )
		{
			if( tofcamInfo.tofcamModeType != GRAYSCALE_MODE )
			{
				tofcamInfo.tofcamModeType = GRAYSCALE_MODE;
#ifdef __STREAMING_COMMAND__
				reqStreamingFrame(tofcamInfo.control_sock);
#endif
			}
		}
		// distance (+grayscale) mode
		else if( tofcamInfo.tofcamEvent_key == 'd')
		{
			if( tofcamInfo.tofcamModeType != DISTANCE_GRAYSCALE_MODE ){
				tofcamInfo.tofcamModeType = DISTANCE_GRAYSCALE_MODE;
#ifdef __STREAMING_COMMAND__
				reqStreamingFrame(tofcamInfo.control_sock);
#endif
			}
		}
		// amplitude & distance mode(grayscale)
		else if( tofcamInfo.tofcamEvent_key == 'e' )
		{
			if( tofcamInfo.tofcamModeType != AMPLITEDE_DISTANCE_EX_MODE ){
				tofcamInfo.tofcamModeType = AMPLITEDE_DISTANCE_EX_MODE;
#ifdef __STREAMING_COMMAND__
				reqStreamingFrame(tofcamInfo.control_sock);
#endif
			}
		}
		// amplitude & distance mode
		else if( tofcamInfo.tofcamEvent_key == 'a' )
		{
			if( tofcamInfo.tofcamModeType != AMPLITEDE_DISTANCE_MODE ){
				tofcamInfo.tofcamModeType = AMPLITEDE_DISTANCE_MODE;
#ifdef __STREAMING_COMMAND__
				reqStreamingFrame(tofcamInfo.control_sock);
#endif
			}
		}
		// hdr off
		else if( tofcamInfo.tofcamEvent_key == '0' )
		{
			tofcamInfo.config.hdr_mode = HDR_NONE_MODE;
			reqHdrMode(tofcamInfo.control_sock);
		}
		// hdr spatial
		else if( tofcamInfo.tofcamEvent_key == '1' )
		{
			tofcamInfo.config.hdr_mode = HDR_SPATIAL_MODE;
			reqHdrMode(tofcamInfo.control_sock);
		}
		// hdr temporal
		else if( tofcamInfo.tofcamEvent_key == '2' )
		{
			tofcamInfo.config.hdr_mode = HDR_TEMPORAL_MODE;
			reqHdrMode(tofcamInfo.control_sock);
		}
		// grayscale corrected
		else if( tofcamInfo.tofcamEvent_key == 'g' )
		{
			if( tofcamInfo.config.compensationFlag & MASK_GRAYSCALE_COMPENSATION )
				tofcamInfo.config.compensationFlag &= ~MASK_GRAYSCALE_COMPENSATION;
			else
				tofcamInfo.config.compensationFlag |= MASK_GRAYSCALE_COMPENSATION;

			reqCompensation(tofcamInfo.control_sock);
		}
		// ambient light enable/disable
		else if( tofcamInfo.tofcamEvent_key == 'l' )
		{
			if( tofcamInfo.config.compensationFlag & MASK_AMBIENT_LIGHT_COMPENSATION )
				tofcamInfo.config.compensationFlag &= ~MASK_AMBIENT_LIGHT_COMPENSATION;
			else
				tofcamInfo.config.compensationFlag |= MASK_AMBIENT_LIGHT_COMPENSATION;
		
			reqCompensation(tofcamInfo.control_sock);
		}
		else if( tofcamInfo.tofcamEvent_key == 'u' )
		{

			if( tofcamInfo.config.compensationFlag & MASK_DRNU_COMPENSATION ){
				tofcamInfo.config.compensationFlag &= ~MASK_DRNU_COMPENSATION;
				tofcamInfo.config.compensationFlag &= ~MASK_TEMPERATURE_COMPENSATION;
			}
			else{
				tofcamInfo.config.compensationFlag |= MASK_DRNU_COMPENSATION;
				tofcamInfo.config.compensationFlag |= MASK_TEMPERATURE_COMPENSATION;
			}

			reqCompensation(tofcamInfo.control_sock);
		}
		// saturation enable/disable
		else if( tofcamInfo.tofcamEvent_key == 's' )
		{
			if( tofcamInfo.config.saturatedFlag & MASK_USED_SATURATION )
				tofcamInfo.config.saturatedFlag &= ~MASK_USED_SATURATION;
			else
				tofcamInfo.config.saturatedFlag |= MASK_USED_SATURATION;

			reqOverflow(tofcamInfo.control_sock);
		}
		// overflow enable/disable
		else if( tofcamInfo.tofcamEvent_key == 'f' )
		{
			if( tofcamInfo.config.saturatedFlag & MASK_USED_ADC_OVERFLOW )
				tofcamInfo.config.saturatedFlag &= ~MASK_USED_ADC_OVERFLOW;
			else
				tofcamInfo.config.saturatedFlag |= MASK_USED_ADC_OVERFLOW;
			
			reqOverflow(tofcamInfo.control_sock);
		}
		else if( tofcamInfo.tofcamEvent_key == 'r' )
		{
			tofcamInfo.rotate_90 = tofcamInfo.rotate_90 ? 0 : 1;
#ifdef ROTATE_IMAGE_ADJUST_ROI
			reqSetROI(tofcamInfo.control_sock);
#endif
		}
		else if ( tofcamInfo.tofcamEvent_key == 't'){
			tofcamInfo.led_control = tofcamInfo.led_control ? 0 : 1;

			reqGrayscaleLedControl(tofcamInfo.control_sock);
		}
		else if ( tofcamInfo.tofcamEvent_key == 'p' ){
			tofcamInfo.usedPointCloud = tofcamInfo.usedPointCloud ? 0 : 1; 
		}
		else if( tofcamInfo.tofcamEvent_key == 'h' ){
			printf("-----------------------------------------------\n");
			printf("p key : Print FPS\n");
			printf("b key : change GRAYSCALE mode\n");
			printf("d key : change DISTANCE & Grayscale mode\n");
			printf("a key : change AMPLITUDE & DISTANCE mode\n");
			printf("e key : change AMPLITUDE(log) & DISTANCE mode\n");
			printf("t key : change Grayscale LED\n");
			printf("g key : change Grayscale corrected\n");
			printf("l key : change Ambient light corrected\n");
			printf("s key : change saturation corrected\n");
			printf("f key : change overflow corrected\n");
			printf("u key : change DRNU\n");
			printf("r key : rotate 90(ROI reset)\n");
			printf("0 key : change HDR off\n");
			printf("1 key : change HDR Spatial\n");
			printf("2 key : change HDR Temporal\n");
			printf("-----------------------------------------------\n");

			tofcamInfo.printBasicInfo = tofcamInfo.printBasicInfo ? 0 : 1;
		}

		tofcamInfo.tofcamEvent_key = 0;
	}
}


void *NSL3130AA::rxTofcam660(void *arg) 
{	
	static uint8_t socketbuff[TOF660_BUFF_SIZE];

	while(!exit_thtread)
	{
		if( tofcamInfo.captureNetType == NONEMODEL_TYPE )
		{
			Sleep(10);
			continue;
		}

		keyProc();

		if( ttySerial ) {
			rxSerial(socketbuff, sizeof(socketbuff), true);
		}
		else{
			rxSocket(socketbuff, sizeof(socketbuff));
		}
		
		Sleep(1);
	}

	return NULL;
}

//point cloud
#ifdef _WINDOWS
pcl::PointCloud<pcl::PointXYZRGB>::Ptr NSL3130AA::pcbVis()
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	point_cloud_ptr->clear();
	point_cloud_ptr->is_dense = false;
	//point_cloud_ptr->reserve(IMAGE_WIDTH * IMAGE_HEIGHT);
	point_cloud_ptr->width = TOF660_IMAGE_WIDTH;
	point_cloud_ptr->height = TOF660_IMAGE_HEIGHT;

	return point_cloud_ptr;
}

pcl::visualization::PCLVisualizer::Ptr NSL3130AA::rgbVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("ROBOSCAN PointCloud"));
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "sample cloud");
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	//좌표축
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
NSL3130AA* NSL3130AA::Create( std::string ipaddr )
{
	// create camera instance
	return new NSL3130AA(ipaddr);
}


int NSL3130AA::getVideoWidth(){
	if( tofcamInfo.rotate_90 != 0 ){
		return TOF660_IMAGE_HEIGHT;
	}
	return TOF660_IMAGE_WIDTH;
}

int NSL3130AA::getVideoHeight(){
	if( tofcamInfo.rotate_90 != 0 ){
		return TOF660_IMAGE_WIDTH;
	}
	return TOF660_IMAGE_HEIGHT;
}

int NSL3130AA::getWidthDiv()				
{ 
	if( tofcamInfo.rotate_90 != 0 ){
		return tofcamInfo.height/TOF660_IMAGE_HEIGHT;
	}
	return tofcamInfo.width/TOF660_IMAGE_WIDTH; 
}

int NSL3130AA::getHeightDiv()
{
	if( tofcamInfo.rotate_90 != 0 ){
		return tofcamInfo.width/TOF660_IMAGE_WIDTH;
	}
	return tofcamInfo.height/TOF660_IMAGE_HEIGHT; 
}



int NSL3130AA::getWidth()				
{ 
	if( tofcamInfo.rotate_90 != 0 ){
		return tofcamInfo.height;
	}
	return tofcamInfo.width; 
}

/**
 * Return the height of the stream, in pixels.
 */
int NSL3130AA::getHeight()
{
	if( tofcamInfo.rotate_90 != 0 ){
		return tofcamInfo.width;
	}
	return tofcamInfo.height; 
}



bool NSL3130AA::isRotate90()
{
	if( tofcamInfo.rotate_90 != 0 ){
		return true;
	}

	return false;
}

void NSL3130AA::drawPointCloud(void)
{
#ifdef _WINDOWS
	if( !viewer->wasStopped() && point_cloud_ptr->points.size() > 0 ){
		viewer->updatePointCloud(point_cloud_ptr, "sample cloud");
		viewer->spinOnce();
	}
#endif
}

std::string NSL3130AA::getDistanceString(int distance )
{
	std::string distStr;

	if( distance == TOF660_LOW_AMPLITUDE || distance == 300000 )
		distStr = "LOW_AMPLITUDE";
	else if( distance == TOF660_ADC_OVERFLOW )
		distStr = "ADC_OVERFLOW";
	else if( distance == TOF660_SATURATION )
		distStr = "SATURATION";
	else if( distance == TOF660_INTERFERENCE )
		distStr = "INTERFERENCE";
	else if( distance == TOF660_EDGE_DETECTED )
		distStr = "EDGE_DETECTED";
	else if( distance == TOF660_BAD )
		distStr = "BAD_FIXEL";
	else
		distStr = format("%d mm", distance);

	return distStr;
}

std::string NSL3130AA::getLeftViewName(void)
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


// Capture
bool NSL3130AA::Capture( void** output, int timeout )
{
	// verify the output pointer exists
	if( !output )
		return false;

	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
	
	int frame_cnt = 0;
	while(!exit_thtread)
	{
		if( GET_BUFF_CNT(tofcamBuff, TOFCAM_ETH_BUFF_SIZE) > 0 ){

			cv::Mat image(TOF660_IMAGE_HEIGHT, TOF660_IMAGE_WIDTH, CV_8UC3, Scalar(255,255,255));	
			cv::Mat imageDist(TOF660_IMAGE_HEIGHT, TOF660_IMAGE_WIDTH, CV_8UC3, Scalar(255,255,255));			

			EnterCriticalSection(&tofcamBuff.lock);
			//printf("main-bufLen = %d:%d\n", tofcamBuff.bufGrayLen[tofcamBuff.tail_idx], tofcamBuff.bufLen[tofcamBuff.tail_idx]);
			if( tofcamBuff.bufGrayLen[tofcamBuff.tail_idx] > 0 ) memcpy(procBuff[0], tofcamBuff.tofcamGrayBuf[tofcamBuff.tail_idx], tofcamBuff.bufGrayLen[tofcamBuff.tail_idx]);
			memcpy(procBuff[1], tofcamBuff.tofcamBuf[tofcamBuff.tail_idx], tofcamBuff.bufLen[tofcamBuff.tail_idx]);
			POP_TOFCAM_BUFF(tofcamBuff, TOFCAM_ETH_BUFF_SIZE);

			LeaveCriticalSection(&tofcamBuff.lock);


			getCamInfo(procBuff[1]);

			if( tofcamInfo.tofcamModeType == AMPLITEDE_DISTANCE_MODE
				|| tofcamInfo.tofcamModeType == DISTANCE_GRAYSCALE_MODE
				|| tofcamInfo.tofcamModeType == AMPLITEDE_DISTANCE_EX_MODE )
			{
				getDistanceAmplitude(imageDist, image, tofcamInfo.usedPointCloud != 0 );
			}
			else{
				getGrayscaled(image, tofcamInfo.usedPointCloud != 0);
				imageDist = image;
			}

			static cv::Mat resizeDist, resizeFrame;

#ifdef RESIZE_IMAGE_MAT
#ifdef HAVE_CV_CUDA
			cuda::GpuMat gpuImage, gpuOutImage, gpuImageDist, gpuOutImageDist;

			gpuImage.upload(image);
			gpuImageDist.upload(imageDist);

//			Scalar avg = mean(image);
//			tofcamInfo.meanAvg = avg[0];

			if( tofcamInfo.rotate_90 != 0 ){
				cuda::rotate(gpuImage, gpuImage, Size( 240, 320 ), -90, 239, 0, cv::INTER_LINEAR);
				cuda::rotate(gpuImageDist, gpuImageDist, Size( 240, 320 ), -90, 239, 0, cv::INTER_LINEAR);

				cuda::resize(gpuImage, gpuOutImage, cv::Size( 480, 640 ),  cv::INTER_LANCZOS4);
				cuda::resize(gpuImageDist, gpuOutImageDist, cv::Size( 480, 640 ),  cv::INTER_LANCZOS4);
			}
			else{	// detectnet, poseNet, imagenet
				cuda::resize(gpuImage, gpuOutImage, cv::Size( tofcamInfo.width, tofcamInfo.height ),  cv::INTER_LANCZOS4);
				cuda::resize(gpuImageDist, gpuOutImageDist, cv::Size( tofcamInfo.width, tofcamInfo.height ),  cv::INTER_LANCZOS4);
			}

			// up-sampling : INTER_CUBIC, INTER_LANCZOS4
			// down-sampling : INTER_AREA

			gpuOutImage.download(resizeFrame);
			gpuOutImageDist.download(resizeDist);
#else
			if( tofcamInfo.rotate_90 != 0 )
			{
				cv::rotate(image, image, ROTATE_90_CLOCKWISE);
				cv::rotate(imageDist, imageDist, ROTATE_90_CLOCKWISE);

				cv::resize( image, resizeFrame, cv::Size( 480, 640 ) );
				cv::resize( imageDist, resizeDist, cv::Size( 480, 640 ));
			}
			else{	// poseNet, imagenet
				cv::resize( image, resizeFrame, cv::Size( tofcamInfo.width, tofcamInfo.height ) );
				cv::resize( imageDist, resizeDist, cv::Size( tofcamInfo.width, tofcamInfo.height ));
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
void NSL3130AA::closeLidar()
{
	tofcamInfo.captureNetType = NONEMODEL_TYPE;

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
		reqStopStream(tofcamInfo.control_sock);

		if( tofcamInfo.control_sock != 0 ){
			closesocket(tofcamInfo.control_sock);
			tofcamInfo.control_sock = 0;
		}

		if( tofcamInfo.data_sock != 0 ){
			closesocket(tofcamInfo.data_sock);
			tofcamInfo.data_sock = 0;
		}
	}

}

void NSL3130AA::startCaptureCommand(int netType, void *pCapOption )
{
	CaptureOptions *pCapOpt = (CaptureOptions *)pCapOption;
	maxDistanceValue = (pCapOpt->maxDistance <= 0 || pCapOpt->maxDistance > MAX_DISTANCEVALUE) ? MAX_DISTANCEVALUE : pCapOpt->maxDistance;

	tofcamInfo.tofcamModeType = pCapOpt->captureType;
	tofcamInfo.config.integrationTimeGrayScale = pCapOpt->grayIntegrationTime;
	tofcamInfo.config.integrationTime3D = pCapOpt->integrationTime;
	tofcamInfo.config.minAmplitude = pCapOpt->minAmplitude;

	tofcamInfo.config.medianFilterEnable = pCapOpt->medianFilterEnable;
	tofcamInfo.config.edgeFilterThreshold = pCapOpt->edgeThresHold;
	tofcamInfo.config.averageFilterEnable = pCapOpt->averageFilterEnable;
	tofcamInfo.config.temporalFilterFactorActual = pCapOpt->temporalFilterFactorActual;
	tofcamInfo.config.temporalFilterThreshold = pCapOpt->temporalFilterThreshold;
	tofcamInfo.config.interferenceUseLashValueEnable = pCapOpt->interferenceUseLashValueEnable;
	tofcamInfo.config.interferenceLimit = pCapOpt->interferenceLimit;
	
	reqIntegrationTime(tofcamInfo.control_sock);
	reqMinAmplitude(tofcamInfo.control_sock);
	reqFilterParameter(tofcamInfo.control_sock);

	if( tofcamInfo.tofcamModeType == DISTANCE_GRAYSCALE_MODE ){
		tofcamInfo.led_control = 1;		
		reqGrayscaleLedControl(tofcamInfo.control_sock);
	}

	
#ifdef __STREAMING_COMMAND__
	reqStreamingFrame(tofcamInfo.control_sock);
#endif	
	tofcamInfo.captureNetType = netType;

	printf("start Capture~~~ intTime = %d/%d modeType =%d\n", pCapOpt->integrationTime, pCapOpt->grayIntegrationTime, pCapOpt->captureType);
	
}

void NSL3130AA::setKey(int cmdKey)
{	
	tofcamInfo.tofcamEvent_key = cmdKey;
}



// constructor
NSL3130AA::NSL3130AA( std::string ipaddr )
{	
	mIpaddr = ipaddr;
	ttySerial = false;

#if 0 // not  yet
	if( mIpaddr.compare("/dev/ttyLiDAR") == 0 ) 
		ttySerial = true;
	else
		ttySerial = false;
#endif

	exit_thtread = 0;
	
	tofcamBuff.overflow = 0;
	tofcamBuff.head_idx = 0;
	tofcamBuff.tail_idx = 0;


	memset(&tofcamInfo, 0, sizeof(tofcamInfo));
//	tofcamInfo.tofcamModeType = AMPLITEDE_DISTANCE_MODE;
	tofcamInfo.tofcamModeType = AMPLITEDE_DISTANCE_EX_MODE; // distance & amplitude(grayscale)
//	tofcamInfo.tofcamModeType = DISTANCE_GRAYSCALE_MODE;
//	tofcamInfo.tofcamModeType = GRAYSCALE_MODE;
//	tofcamInfo.tofcamModeType = DISTANCE_MODE;


#ifdef TOFCAM660_ROTATE_IMAGE_90
	tofcamInfo.rotate_90 = 1;
	tofcamInfo.config.roi_xMin = ADJUST_ROI_XMIN;
	tofcamInfo.config.roi_xMax = ADJUST_ROI_XMAX;
	tofcamInfo.config.roi_yMin = ADJUST_ROI_YMIN;
	tofcamInfo.config.roi_yMax = ADJUST_ROI_YMAX;
#else
	tofcamInfo.rotate_90 = 0;
	tofcamInfo.config.roi_xMin = DEFAULT_ROI_XMIN;
	tofcamInfo.config.roi_xMax = DEFAULT_ROI_XMAX;
	tofcamInfo.config.roi_yMin = DEFAULT_ROI_YMIN;
	tofcamInfo.config.roi_yMax = DEFAULT_ROI_YMAX;
#endif


	tofcamInfo.led_control = 1;
	tofcamInfo.captureNetType = NONEMODEL_TYPE;
	tofcamInfo.config.hdr_mode = DEFAULT_HDR_MODE;
	tofcamInfo.config.integrationTime3D = 800;
	tofcamInfo.config.integrationTime3DHdr1 = 100;
	tofcamInfo.config.integrationTime3DHdr2 = 50;
	tofcamInfo.config.integrationTimeGrayScale = 100;
	
//	tofcamInfo.config.saturatedFlag |= MASK_USED_ADC_OVERFLOW;
//	tofcamInfo.config.saturatedFlag |= MASK_USED_SATURATION;
	tofcamInfo.config.compensationFlag |= MASK_DRNU_COMPENSATION;
	tofcamInfo.config.compensationFlag |= MASK_TEMPERATURE_COMPENSATION;
	tofcamInfo.config.compensationFlag |= MASK_GRAYSCALE_COMPENSATION;
//	if( tofcamInfo.config.integrationTimeGrayScale == 0 || tofcamInfo.config.integrationTimeGrayScale == 50 )
//		tofcamInfo.config.compensationFlag |= MASK_AMBIENT_LIGHT_COMPENSATION;
	tofcamInfo.config.minAmplitude = 50;	// 0x32

#ifdef RESIZE_IMAGE_MAT
	tofcamInfo.width = 640;
	tofcamInfo.height = 480;
#else
	tofcamInfo.width = DefaultWidth;
	tofcamInfo.height = DefaultHeight;
#endif
	if( ttySerial ) {
		tofcamInfo.control_sock = setSerialBaudrate();
	}
	else{
		tofcamInfo.control_sock = InitializeControlsocket();
		tofcamInfo.data_sock = InitializeDataSocket();
	}

	
	initializeTofcam660(tofcamInfo.control_sock);

#ifdef _WINDOWS

	unsigned threadID;
	hThread = (HANDLE)_beginthreadex(NULL, 0, &NSL3130AA::rxWrapper, this, 0, &threadID);

	double psdAngle = 0.0f; //seobi psd angle 0'
	sin_angle = sin(psdAngle*PI/180.0);
	cos_angle = cos(psdAngle*PI/180.0);

	tofcamInfo.usedPointCloud = 1;
	lensTransform.initLensDistortionTable(STANDARD_FIELD);
	
	point_cloud_ptr = pcbVis();
	viewer = rgbVis(point_cloud_ptr);
#else
	pthread_create(&threadID, NULL, NSL3130AA::rxWrapper, this);
#endif

}


// destructor	
NSL3130AA::~NSL3130AA()
{
	printf("~NSL3130AA\n");
	closeLidar();
	return;
}




