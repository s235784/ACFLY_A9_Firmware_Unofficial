#include "drv_YT_INYYO.hpp"
#include "Commulink.hpp"
#include "Basic.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "SensorsBackend.hpp"
#include "MeasurementSystem.hpp"
#include "AuxFuncs.hpp"

struct DriverInfo
{
	uint32_t param;
	Port port;
};

static bool needTakePhoto = false;
static uint8_t takePhoto()
{
	needTakePhoto = true;
	return 3;
}

static void YT_INYYO_Server(void* pvParameters)
{
	DriverInfo driver_info = *(DriverInfo*)pvParameters;
	delete (DriverInfo*)pvParameters;
	
	#define freq 20
	#define minPhotoGap 0.7
	
	//注册数字云台功能
	DgYTFuncStruct dgYTFunc = {0};
	dgYTFunc.takePhoto = takePhoto;
	dgYtFunc_Register(dgYTFunc);
	
	TIME pTIME, yTIME, zTIME, fTIME, sTIME, RtTIME;
	float last_fChann, last_sChann, last_RtChann;
	bool shooting_video = false;	uint8_t taking_photo = 255;	uint16_t photo_counter = 0;
	bool zooming = false;
	TIME lastPhotoTime(false);
	
	//初始化变量
	YTCtrl ty_ctrl;
	if( get_DgYTFocusCtrl(&ty_ctrl) )
		last_fChann = ty_ctrl.value;
	if( get_DgYTShootCtrl(&ty_ctrl) )
		last_sChann = ty_ctrl.value;
	if( get_DgYTReturnCtrl(&ty_ctrl) )
		last_RtChann = ty_ctrl.value;
	
	extern float debug_test[30];
	while(1)
	{
		os_delay(1.0/freq);
		
		bool RateCtrl = false;
		float yawRate = 0;	float pitRate = 0;
		if( get_DgYTPitCtrl(&ty_ctrl) )
		{	//俯仰控制
			if( ty_ctrl.update_TIME != pTIME )
			{
				pTIME = ty_ctrl.update_TIME;
				
				if( ty_ctrl.mode == 0 )
				{	//角度控制
					uint8_t YT_cmd[] = { 0xff, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01 };
					driver_info.port.write( YT_cmd, sizeof(YT_cmd), 0.1, 0.1 );
				}
				else if( ty_ctrl.mode == 1 )
				{	//角速度控制
					RateCtrl = true;
					pitRate = -ty_ctrl.value;
				}
			}
		}
		if( get_DgYTYawCtrl(&ty_ctrl) )
		{	//偏航控制
			if( ty_ctrl.update_TIME != yTIME )
			{
				yTIME = ty_ctrl.update_TIME;
				
				if( ty_ctrl.mode == 0 )
				{	//角度控制
					uint8_t YT_cmd[] = { 0xff, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01 };
					driver_info.port.write( YT_cmd, sizeof(YT_cmd), 0.1, 0.1 );
				}
				else if( ty_ctrl.mode == 1 )
				{	//角速度控制
					RateCtrl = true;
					yawRate = -ty_ctrl.value;
				}
			}
		}
		if( get_DgYTZoomCtrl(&ty_ctrl) )
		{	//倍数控制
			if( ty_ctrl.update_TIME != zTIME )
			{
				zTIME = ty_ctrl.update_TIME;
				
				if( ty_ctrl.mode == 0 )
				{	//倍数控制
					
				}
				else if( ty_ctrl.mode == 1 )
				{	//放大缩小控制
					uint8_t zoom = 0;
					if( ty_ctrl.value > 10 )
						zoom = 0x6;
					else if( ty_ctrl.value < -10 )
						zoom = 0x5;
					
					if( zoom )
					{
						uint8_t YT_cmd[] = { 0xff, 0x01, 0x00, 0x40, 0x04, 0x00, 0x45 };
						driver_info.port.write( YT_cmd, sizeof(YT_cmd), 0.1, 0.1 );
						zooming = true;
					}
					else if( zooming )
					{
						uint8_t YT_cmd[] = { 0xff, 0x01, 0x00, 0x20, 0x04, 0x00, 0x25 };
						driver_info.port.write( YT_cmd, sizeof(YT_cmd), 0.1, 0.1 );
						zooming = false;
					}
				}
			}
		}
		if( get_DgYTFocusCtrl(&ty_ctrl) )
		{	//聚焦控制
			if( ty_ctrl.update_TIME != fTIME )
			{
				fTIME = ty_ctrl.update_TIME;
				
				if( ty_ctrl.mode == 0 )
				{	//按钮按下聚焦
					if( ty_ctrl.value > 25 )
					{
						
					}
				}
				else if( ty_ctrl.mode == 1 )
				{	//通道改变聚焦
					if( fabsf( ty_ctrl.value - last_fChann ) > 10 )
					{
						
					}
					last_fChann = ty_ctrl.value;
				}
			}
		}
		if( needTakePhoto && lastPhotoTime.get_pass_time()>minPhotoGap )
		{	//自动触发
			uint8_t YT_cmd[] = { 0xff, 0x01, 0x12, 0x00, 0x00, 0x00, 0x13 };
			driver_info.port.write( YT_cmd, sizeof(YT_cmd), 0.1, 0.1 );
			recordPos(false);
			needTakePhoto = false;
			lastPhotoTime = TIME::now();
		}
		if( get_DgYTShootCtrl(&ty_ctrl) )
		{	//拍照录像控制
			if( ty_ctrl.update_TIME != sTIME )
			{
				sTIME = ty_ctrl.update_TIME;
				
				if( ty_ctrl.mode == 0 )
				{	//按钮上拍照 按钮下录像
					if( ty_ctrl.value > 25 )
					{
						if( taking_photo >= freq )
						{
							if( lastPhotoTime.get_pass_time() > minPhotoGap )
							{
								uint8_t YT_cmd[] = { 0xff, 0x01, 0x12, 0x00, 0x00, 0x00, 0x13 };
								driver_info.port.write( YT_cmd, sizeof(YT_cmd), 0.1, 0.1 );
								recordPos(true);
								taking_photo = 0;
								lastPhotoTime = TIME::now();
							}
						}
						else
							++taking_photo;
					}
					else if( ty_ctrl.value < -25 )
					{
						uint8_t YT_cmd[] = { 0xff, 0x01, 0x12, 0x01, 0x00, 0x00, 0x14 };
						driver_info.port.write( YT_cmd, sizeof(YT_cmd), 0.1, 0.1 );
						shooting_video = true;
						taking_photo = 255;
					}
					else
					{
						uint8_t YT_cmd[] = { 0xff, 0x01, 0x12, 0x02, 0x00, 0x00, 0x15 };
						driver_info.port.write( YT_cmd, sizeof(YT_cmd), 0.1, 0.1 );
							
						shooting_video = false;
						taking_photo = 255;
						photo_counter = 0;
					}
				}
				else if( ty_ctrl.mode == 1 )
				{	//通道改变拍照
					if( fabsf( ty_ctrl.value - last_sChann ) > 10 )
					{
						if( lastPhotoTime.get_pass_time() > minPhotoGap )
						{
							uint8_t YT_cmd[] = { 0xff, 0x01, 0x12, 0x00, 0x00, 0x00, 0x13 };
							driver_info.port.write( YT_cmd, sizeof(YT_cmd), 0.1, 0.1 );
							recordPos(true);
							lastPhotoTime = TIME::now();
						}
					}
					last_sChann = ty_ctrl.value;
				}
			}
		}
//		if( get_DgYTReturnCtrl(&ty_ctrl) )
//		{	//归位控制
//			if( ty_ctrl.update_TIME != RtTIME )
//			{
//				RtTIME = ty_ctrl.update_TIME;
//				
//				if( ty_ctrl.mode == 0 )
//				{	//按钮按下归位
//					if( ty_ctrl.value > 25 )
//					{
//						YT_cmd[2] = 0x0B;
//						driver_info.port.write( YT_cmd, sizeof(YT_cmd), 0.1, 0.1 );
//					}
//				}
//				else if( ty_ctrl.mode == 1 )
//				{	//通道改变归位
//					if( fabsf( ty_ctrl.value - last_RtChann ) > 10 )
//					{
//						YT_cmd[2] = 0x0B;
//						driver_info.port.write( YT_cmd, sizeof(YT_cmd), 0.1, 0.1 );
//					}
//					last_RtChann = ty_ctrl.value;
//				}
//			}
//		}
		
		if( RateCtrl )
		{	//控制角速度
			float yaw_rate=0, pit_rate=0;
			yaw_rate = remove_deadband( yawRate, 5.0f );
			pit_rate = remove_deadband( pitRate, 5.0f );
			if( pit_rate > 0 )
			{ //向上
				uint8_t YT_cmd[] = { 0xff, 0x01, 0x00, 0x08, 0x00, 0x22, 0x08 };
				driver_info.port.write( YT_cmd, sizeof(YT_cmd), 0.1, 0.1 );
			}
			else if( pit_rate < 0 )
			{	//向下
				uint8_t YT_cmd[] = { 0xff, 0x01, 0x00, 0x10, 0x00, 0x22, 0x10 };
				driver_info.port.write( YT_cmd, sizeof(YT_cmd), 0.1, 0.1 );
			}
			else
			{
				uint8_t YT_cmd[] = { 0xff, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01 };
				driver_info.port.write( YT_cmd, sizeof(YT_cmd), 0.1, 0.1 );
			}
			
			if( yaw_rate > 0 )
			{
				uint8_t YT_cmd[] = { 0xff, 0x01, 0x00, 0x04, 0xff, 0x00, 0x04 };
				driver_info.port.write( YT_cmd, sizeof(YT_cmd), 0.1, 0.1 );
			}
			else if( yaw_rate < 0 )
			{
				uint8_t YT_cmd[] = { 0xff, 0x01, 0x00, 0x02, 0xff, 0x00, 0x02 };
				driver_info.port.write( YT_cmd, sizeof(YT_cmd), 0.1, 0.1 );
			}
			else
			{
				uint8_t YT_cmd[] = { 0xff, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01 };
				driver_info.port.write( YT_cmd, sizeof(YT_cmd), 0.1, 0.1 );
			}
		}
	}
}

static bool YT_INYYO_DriverInit( Port port, uint32_t param )
{
	//波特率19200
	port.SetBaudRate( 115200, 2, 2 );

	DriverInfo* driver_info = new DriverInfo;
	driver_info->param = param;
	driver_info->port = port;
	xTaskCreate( YT_INYYO_Server, "YT_INYYO", 812, (void*)driver_info, SysPriority_ExtSensor, NULL);
	return true;
}

void init_drv_YT_INYYO()
{
	PortFunc_Register( 102, YT_INYYO_DriverInit );
}