#include "Commulink.hpp"
#include "Basic.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "drv_LED.hpp"
#include "mavlink.h"
#include "MavlinksendFuncs.hpp"
#include "MavlinkRCProcess.hpp"
#include "MeasurementSystem.hpp"
#include "Sensors.hpp"
#include "Parameters.hpp"
#include <map>
#include "ctrl_Main.hpp"
#include "usb_composite.h"
#include "ReceiverBackend.hpp"
#include "ControlSystem.hpp"
#include "StorageSystem.hpp"
#include "drv_Can.hpp"

using namespace std;

/*声光提示*/
	static float ledSignalCounter = -1;	static LEDSignal ledSignal;
	static LEDMode ledmode = LEDMode_Processing1;
	static float ledR = 0, ledG = 0, ledB = 0;
	static bool buzzerOn = false;	static uint16_t buzzerFreq;

	void sendLedSignal( LEDSignal signal )
	{
		ledSignalCounter = 0;
		ledSignal = signal;
	}
	void setLedMode( LEDMode mode )
	{
		ledmode = mode;
	}
	void setLedManualCtrl( float R, float G, float B, bool BuzzerOn, uint16_t BuzzerFreq )
	{
		ledR = R;	ledG = G;	ledB = B;
		buzzerOn = BuzzerOn;	buzzerFreq = BuzzerFreq;
		ledmode = LEDMode_Manual;
	}
	static inline void LEDRefresh(float dt)
	{
		if( ledSignalCounter >= 0 )
		{
			switch(ledSignal)
			{
				case LEDSignal_Start1:
				{
					if( ledSignalCounter > 0.3 )
					{
						ledSignalCounter = -1;
						return;
					}
					if( ledSignalCounter < 0.15f )
					{
						set_BuzzerFreq(900);
						set_BuzzerOnOff(true);
						set_LedBrightness( 0, 0, 100 );
					}
					else
					{
						set_BuzzerFreq(1500);
						set_BuzzerOnOff(true);
						set_LedBrightness( 0, 100, 0 );
					}
					break;
				}
				case LEDSignal_Start2:
				{
					if( ledSignalCounter > 0.45 )
					{
						ledSignalCounter = -1;
						return;
					}
					if( ledSignalCounter < 0.15 )
					{
						set_BuzzerFreq(800);
						set_BuzzerOnOff(true);
						set_LedBrightness( 100, 0, 0 );
					}
					else if( ledSignalCounter < 0.3 )
					{
						set_BuzzerFreq(1000);
						set_BuzzerOnOff(true);
						set_LedBrightness( 0, 100, 0 );
					}
					else
					{
						set_BuzzerFreq(1200);
						set_BuzzerOnOff(true);
						set_LedBrightness( 0, 0, 100 );
					}
					break;
				}
				
				case LEDSignal_Continue1:
				{
					if( ledSignalCounter > 0.8 )
					{
						ledSignalCounter = -1;
						return;
					}
					set_BuzzerFreq(1500);				
					if( ledSignalCounter < 0.2f )
					{
						set_LedBrightness( 0, 0, 100 );
						set_BuzzerOnOff(true);
					}
					else if( ledSignalCounter < 0.4f )
					{
						set_LedBrightness( 0, 0, 0 );
						set_BuzzerOnOff(false);
					}
					else if( ledSignalCounter < 0.6f )
						set_LedBrightness( 0, 0, 100 );
					else
						set_LedBrightness( 0, 0, 0 );
					break;
				}
				
				case LEDSignal_Success1:
				{
					if( ledSignalCounter > 0.8 )
					{
						ledSignalCounter = -1;
						return;
					}
					set_BuzzerFreq(1500);
					if( ledSignalCounter < 0.2f )
					{						
						set_LedBrightness( 0, 100, 0 );
						set_BuzzerOnOff(true);
					}
					else if( ledSignalCounter < 0.4f )
					{
						set_LedBrightness( 0, 0, 0 );
						set_BuzzerOnOff(false);
					}
					else if( ledSignalCounter < 0.6f )
					{
						set_LedBrightness( 0, 100, 0 );
						set_BuzzerOnOff(true);
					}
					else
					{
						set_LedBrightness( 0, 0, 0 );
						set_BuzzerOnOff(false);
					}
					break;
				}
				
				
				case LEDSignal_Err1:
				{
					if( ledSignalCounter > 1.0 )
					{
						ledSignalCounter = -1;
						return;
					}
					set_BuzzerFreq(800);
					set_BuzzerOnOff(true);
					if( ledSignalCounter < 0.25f )
						set_LedBrightness( 100, 0, 0 );
					else if( ledSignalCounter < 0.5f )
						set_LedBrightness( 0, 0, 0 );
					else if( ledSignalCounter < 0.75f )
						set_LedBrightness( 100, 0, 0 );
					else
						set_LedBrightness( 0, 0, 0 );
					break;
				}
				case LEDSignal_Err2:
				{//低电量报警
					if( ledSignalCounter > 0.3 )
					{
						ledSignalCounter = -1;
						return;
					}
					set_BuzzerFreq(2000);
					set_BuzzerOnOff(true);
					if( ledSignalCounter < 0.15f )
						set_LedBrightness( 100, 0, 0 );
					else if( ledSignalCounter < 0.3f )
						set_LedBrightness( 0, 0, 0 );
					if( ledSignalCounter < 0.35f )
						set_LedBrightness( 100, 0, 0 );
					else if( ledSignalCounter < 0.5f )
						set_LedBrightness( 0, 0, 0 );					
					break;					
				}				
			}
			ledSignalCounter += dt;
			return;
		}
		
		static float counter = 0;
		switch(ledmode)
		{
			/*正常模式*/
				case LEDMode_Normal1:
				{
					if( counter > 2 )
						counter = 0;
					set_BuzzerOnOff(false);
					if( counter < 1 )
						set_LedBrightness( 0 , counter*100 , 0 );
					else
						set_LedBrightness( 0 , 200 - counter*100 , 0 );
					break;
				}
				case LEDMode_Normal2:
				{
					if( counter > 2 )
						counter = 0;
					set_BuzzerOnOff(false);
					if( counter < 1 )
						set_LedBrightness( 0 , 0 , counter*100 );
					else
						set_LedBrightness( 0 , 200 - counter*100 , 0 );
					break;
				}
			/*正常模式*/
				
			/*飞行模式*/
				case LEDMode_Flying1:
				{
					if( counter > 1.4 )
						counter = 0;
					set_BuzzerOnOff(false);
					if( counter < 1 )
						set_LedBrightness( 0 , counter*0 , 0-counter*0 );
					else if( counter < 1.1 )
						set_LedBrightness( 80 , 80 , 0 );
					else if( counter < 1.2 )
						set_LedBrightness( 0 , 0 , 0 );
					else if( counter < 1.3 )
						set_LedBrightness( 80 , 80 , 0 );
					else
						set_LedBrightness( 0 , 0 , 0 );
					break;
				}
				case LEDMode_Flying2:
				{
					if( counter > 1.4 )
						counter = 0;
					set_BuzzerOnOff(false);
					if( counter < 1 )
						set_LedBrightness( 0 , counter*0 , 0-counter*0 );
					else if( counter < 1.1 )
						set_LedBrightness( 0 , 0 , 100 );
					else if( counter < 1.2 )
						set_LedBrightness( 0 , 0 , 0 );
					else if( counter < 1.3 )
						set_LedBrightness( 0 , 0 , 100 );
					else
						set_LedBrightness( 0 , 0 , 0 );
					break;
				}
			/*飞行模式*/	
				
			/*处理中*/
				case LEDMode_Processing1:
				{
					if( counter > 0.5 )
						counter = 0;
					set_BuzzerOnOff(false);
					set_LedBrightness( 0 , 0 , counter*200 );
					break;
				}
				case LEDMode_Processing2:
				{
					if( counter > 0.5 )
						counter = 0;
					set_BuzzerOnOff(false);
					if( counter < 0.25 )
						set_LedBrightness( 0 , 100-counter*400 , counter*400 );
					else
						set_LedBrightness( 0 , (counter-0.25)*400 , 100-(counter-0.25)*400 );
					break;
				}
			/*处理中*/
				
			default:
			{	//用户手动控制
				if(buzzerOn)
					set_BuzzerFreq(buzzerFreq);
				set_BuzzerOnOff(buzzerOn);
				set_LedBrightness( ledR, ledG, ledB );
				break;
			}
		}
		counter += dt;
	}
/*声光提示*/

/*功能接口*/
	//功能接口定义
	typedef struct
	{
		//初始化函数
		bool (*init)( Port port, uint32_t param );
	}PortFunc;
	static PortFunc PortFuncs[256] = {0};
	
	bool PortFunc_Register( uint8_t FuncInd, bool (*init)( Port port, uint32_t param ) )
	{
		if( FuncInd < 8 )
			return false;
		if( init == 0 )
			return false;
		if( PortFuncs[FuncInd].init )
			return false;
		PortFuncs[FuncInd].init = init;
		return true;
	}
/*功能接口*/
	
/*Can功能*/
	#define MAX_CANFUNCS 128
	
	//功能接口定义
	typedef struct
	{
		//初始化函数
		bool (*init)();
		//开始运行函数
		bool (*run)();
	}CanFunc;
	static CanFunc CanFuncs[MAX_CANFUNCS] = {0};
	
	/*Can功能注册函数
		init函数中进行Can邮箱初始化
		run函数运行（run函数前不能执行读邮箱操作）
	*/
	bool CanFunc_Register( uint8_t FuncInd, 
													bool (*init)(), bool (*run)() )
	{
		if( FuncInd >= MAX_CANFUNCS )
			return false;
		if( init==0 || run==0 )
			return false;
		if( CanFuncs[FuncInd].init )
			return false;
		CanFuncs[FuncInd].init = init;
		CanFuncs[FuncInd].run = run;
		return true;
	}
/*Can功能*/
	
/*I2C功能*/
	#define MAX_I2CFUNCS 128
	
	//功能接口定义
	typedef struct
	{
		//初始化函数
		bool (*init)();
		//开始运行函数
		bool (*run)();
	}I2CFunc;
	static I2CFunc I2CFuncs[MAX_I2CFUNCS] = {0};
	
	/*I2C功能注册函数
		init函数中进行Can邮箱初始化
		run函数运行（run函数前不能执行读邮箱操作）
	*/
	bool I2CFunc_Register( uint8_t FuncInd, 
													bool (*init)(), bool (*run)() )
	{
		if( FuncInd >= MAX_I2CFUNCS )
			return false;
		if( init==0 || run==0 )
			return false;
		if( I2CFuncs[FuncInd].init )
			return false;
		I2CFuncs[FuncInd].init = init;
		I2CFuncs[FuncInd].run = run;
		return true;
	}
/*I2C功能*/
	
/*端口*/
	static Port Ports[MAXPorts] = {0};
	//注册端口
	bool PortRegister( uint8_t ind, Port port )
	{
		if( ind==0 || ind>=MAXPorts )
			return false;
		if( Ports[ind].read!=0 || Ports[ind].write!=0 )
			return false;
		
		Ports[ind] = port;
		return true;
	}
/*端口*/
	
/*通信端口*/
	//端口
	static uint8_t CommuPorts[MAVLINK_COMM_NUM_BUFFERS] = {0};
	//发送消息列表
	struct SDMsg
	{
		uint16_t counter;
		uint16_t rate;
	};
	static map<uint16_t,SDMsg> SDMessages[MAVLINK_COMM_NUM_BUFFERS];
	static SemaphoreHandle_t SDMessagesMutex[MAVLINK_COMM_NUM_BUFFERS];
	
	//在指定端口设置消息速率
	bool SetMsgRate( uint8_t port_index, uint16_t Msg, float RateHz, double TIMEOUT )
	{
		if( port_index >= MAVLINK_COMM_NUM_BUFFERS )
			return false;
		if( Msg >= Mavlink_Send_Funcs_Count )
			return false;
		if( Mavlink_Send_Funcs[ Msg ] == 0 )
			return false;
		
		TickType_t TIMEOUT_Ticks;
		if( TIMEOUT >= 0 )
			TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
		else
			TIMEOUT_Ticks = portMAX_DELAY;
		if( xSemaphoreTake( SDMessagesMutex[port_index], TIMEOUT_Ticks ) == pdTRUE )
		{
			uint16_t Rate = 0;
			if( RateHz > 0.01f )
				Rate = 400.0f / RateHz;
			
			if( Rate==0 && RateHz!=0 )
				Rate = 1;
			map<uint16_t,SDMsg>::iterator it = SDMessages[port_index].find(Msg);
			if( it == SDMessages[port_index].end() )
			{	//无此消息 添加消息
				if( Rate != 0 )
				{
					SDMsg sdmsg;
					sdmsg.rate = Rate;
					sdmsg.counter = 0;
					SDMessages[port_index].insert( pair<uint16_t,SDMsg>(Msg, sdmsg) );
				}
			}
			else
			{	//消息存在 更改速率
				if( Rate != 0 )
					it->second.rate = Rate;
				else
					SDMessages[port_index].erase(it);
			}
			
			xSemaphoreGive(SDMessagesMutex[port_index]);
			return true;
		}
		return false;
	}
	
	//位置传感器发送
	struct PosSDMsg
	{
		TIME last_updataT;
		uint16_t counter;
		int16_t rate;
	};
	static map<uint16_t,PosSDMsg> SDPosMsgs[MAVLINK_COMM_NUM_BUFFERS];
	//在指定端口设置消息速率
	//rate: 0-不发送 <0-发送指定次数，-1一次 >0-按分频系数发送,1不分频,2频率/2
	bool SetPosSensorMsg( uint8_t port_index, uint8_t ind, int16_t rate, double TIMEOUT )
	{
		if( port_index >= MAVLINK_COMM_NUM_BUFFERS )
			return false;
		if( ind >= Position_Sensors_Count )
			return false;
		
		TickType_t TIMEOUT_Ticks;
		if( TIMEOUT >= 0 )
			TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
		else
			TIMEOUT_Ticks = portMAX_DELAY;
		if( xSemaphoreTake( SDMessagesMutex[port_index], TIMEOUT_Ticks ) == pdTRUE )
		{
			map<uint16_t,PosSDMsg>::iterator it = SDPosMsgs[port_index].find(ind);
			if( it == SDPosMsgs[port_index].end() )
			{	//无此消息 添加消息
				if( rate != 0 ) {
					PosSDMsg sdmsg;
					sdmsg.rate = rate;
					sdmsg.counter = 0;
					SDPosMsgs[port_index].insert( pair<uint16_t,PosSDMsg>(ind, sdmsg) );
				}
			}
			else
			{	//消息存在 更改速率
				if( rate != 0 )
					it->second.rate = rate;
				else
					SDPosMsgs[port_index].erase(it);
			}
			
			xSemaphoreGive(SDMessagesMutex[port_index]);
			return true;
		}
		return false;
	}
	bool ClearPosSensorMsg( uint8_t port_index, uint8_t ind, double TIMEOUT )
	{
		if( port_index >= MAVLINK_COMM_NUM_BUFFERS )
			return false;
		if( ind >= Position_Sensors_Count )
			return false;
		
		TickType_t TIMEOUT_Ticks;
		if( TIMEOUT >= 0 )
			TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
		else
			TIMEOUT_Ticks = portMAX_DELAY;
		if( xSemaphoreTake( SDMessagesMutex[port_index], TIMEOUT_Ticks ) == pdTRUE )
		{
			for( map<uint16_t,PosSDMsg>::iterator it = SDPosMsgs[port_index].begin(); it != SDPosMsgs[port_index].end(); /*++it*/ )
			{
				if( it->second.rate >= 0 )
					SDPosMsgs[port_index].erase(it++);
				else
					++it;
			}
			xSemaphoreGive(SDMessagesMutex[port_index]);
			return true;
		}
		return false;
	}
	
	//注册端口用于协议通信
	static bool CommuPortRegister( uint8_t ind )
	{
		if( ind==0 || ind>=MAXPorts )
			return false;
		if( Ports[ind].read==0 || Ports[ind].write==0 )
			return false;
		
		//寻找可用的位置
		int8_t p_index = -1;
		for( uint8_t i = 0 ; i < MAVLINK_COMM_NUM_BUFFERS ; ++i )
		{
			if( CommuPorts[i] == 0 )
			{
				p_index = i;
				break;
			}
		}
		//放满了
		if( p_index < 0 )
			return false;
		
		mavlink_init_chan( p_index );
		CommuPorts[ p_index ] = ind;
		mavlink_set_proto_version( p_index , 1 );
		SetMsgRate( p_index, MAVLINK_MSG_ID_ATTITUDE, 15 );
		SetMsgRate( p_index, MAVLINK_MSG_ID_LOCAL_POSITION_NED, 15 );
		SetMsgRate( p_index, MAVLINK_MSG_ID_GPS_RAW_INT, 2 );
		SetMsgRate( p_index, MAVLINK_MSG_ID_GPS2_RAW, 2 );
		SetMsgRate( p_index, MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 5 );
		SetMsgRate( p_index, MAVLINK_MSG_ID_SYS_STATUS, 1 );
		SetMsgRate( p_index, MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 1 );
		SetMsgRate( p_index, MAVLINK_MSG_ID_VFR_HUD, 2 );
		SetMsgRate( p_index, MAVLINK_MSG_ID_SCALED_IMU, 2 );
		SetMsgRate( p_index, MAVLINK_MSG_ID_SCALED_IMU2, 2 );
		SetMsgRate( p_index, MAVLINK_MSG_ID_SCALED_IMU3, 2 );
		SetMsgRate( p_index, MAVLINK_MSG_ID_RC_CHANNELS, 2 );
		SetMsgRate( p_index, MAVLINK_MSG_ID_RC_CHANNELS_SCALED, 2 );
		SetMsgRate( p_index, MAVLINK_MSG_ID_MISSION_CURRENT, 0.2 );
		SetMsgRate( p_index, MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT, 0.2 );
		SetMsgRate( p_index, MAVLINK_MSG_ID_SCALED_PRESSURE, 1 );
		SetMsgRate( p_index, MAVLINK_MSG_ID_HOME_POSITION, 0.2 );
		SetMsgRate( p_index, MAVLINK_MSG_ID_VIBRATION, 1.0 );
		SetMsgRate( p_index, MAVLINK_MSG_ID_OBSTACLE_DISTANCE, 1.0 );
		return true;
	}
	//获取端口
	const Port* get_CommuPort( uint8_t ind )
	{
		if( ind < MAVLINK_COMM_NUM_BUFFERS )
			return &Ports[CommuPorts[ind]];
		else
			return 0;
	}
	
	//设备Id
	static uint8_t CommulinkSysId = 1;
	static uint8_t CommulinkCompId = 1;
	uint8_t get_CommulinkSysId(){ return CommulinkSysId; }
	uint8_t get_CommulinkCompId(){ return CommulinkCompId; }
/*通信端口*/
	
/*Rtk端口*/
	//端口
	#define MaxRtkPortsCount 3
	static RtkPort RtkPorts[ MaxRtkPortsCount ] = {0};

	//注册Rtk端口
	int8_t RtkPortRegister( RtkPort port )
	{
		if( port.write==0 || port.lock==0 || port.unlock==0 )
			return false;
		
		//寻找可用的位置
		int8_t p_index = -1;
		for( uint8_t i = 0 ; i < MaxRtkPortsCount ; ++i )
		{
			if( RtkPorts[ i ].write == 0 )
			{
				p_index = i;
				break;
			}
		}
		//放满了
		if( p_index < 0 )
			return -1;
		
		RtkPorts[ p_index ] = port;
		return p_index;
	}
	//使能失能Rtk端口
	bool RtkPort_setEna( uint8_t port, bool ena )
	{
		if( port < MaxRtkPortsCount )
		{
			RtkPorts[port].ena = ena;
			return true;
		}
		return false;
	}
	//获取端口
	const RtkPort* get_RtkPort( uint8_t port )
	{
		if( port < MaxRtkPortsCount )
			return &RtkPorts[port];
		else
			return 0;
	}
	//往rtk端口发送注入数据
	void inject_RtkPorts( const uint8_t data[], uint16_t length )
	{
		for( uint8_t i = 0 ; i < MaxRtkPortsCount ; ++i )
		{
			if( RtkPorts[i].write!=0 && RtkPorts[i].ena )
				RtkPorts[i].write( data, length, 0.02, 0.02 );
		}
	}
/*Rtk端口*/
	
static MAV_STATE mav_state = MAV_STATE_STANDBY;
static uint16_t mav_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
static uint16_t mav_main_mode = PX4_CUSTOM_MAIN_MODE_STABILIZED;
static uint16_t mav_sub_mode = 0;
bool set_mav_state(MAV_STATE state)
{
	//屏蔽用户控制
	bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
	if( !isMSafe && ForceMSafeCtrl )
		return false;
	
	mav_state = state;
	return true;
}
bool set_mav_mode_arm()
{
	//屏蔽用户控制
	bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
	if( !isMSafe && ForceMSafeCtrl )
		return false;
	
	mav_mode |= MAV_MODE_STABILIZE_ARMED;
	return true;
}
bool set_mav_mode_disarm()
{
	//屏蔽用户控制
	bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
	if( !isMSafe && ForceMSafeCtrl )
		return false;
	
	mav_mode &= ~MAV_MODE_STABILIZE_ARMED;
	return true;
}
bool set_mav_mode( uint16_t req_mav_mode, uint16_t req_mav_main_mode, uint16_t req_mav_sub_mode )
{
	//屏蔽用户控制
	bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
	if( !isMSafe && ForceMSafeCtrl )
		return false;
	
	mav_mode = req_mav_mode;
	mav_main_mode = req_mav_main_mode;
	mav_sub_mode = req_mav_sub_mode;
	return true;
}
bool get_mav_mode( uint16_t* req_mav_mode, uint16_t* req_mav_main_mode, uint16_t* req_mav_sub_mode )
{
	*req_mav_mode = mav_mode;
	*req_mav_main_mode = mav_main_mode;
	*req_mav_sub_mode = mav_sub_mode;
	return true;
}
static void Commulink_Server(void* pvParameters)
{
	//初始化开始声音
	sendLedSignal(LEDSignal_Start1);
	//等待初始化完成
	while( getInitializationCompleted() == false )
	{
		//刷新led声光提示
		LEDRefresh(0.01f);
		os_delay(0.01);
	}

	//分配端口功能
	if( Port_isBasicFunc(Ports[MAXPorts-1]) )
		CommuPortRegister(MAXPorts-1);
	for( uint8_t i = 1; i < MAXPorts - 1; ++i )
	{
		if( Port_isFullFunc(Ports[i]) )
		{
			char FuncName_buf[17];
			char ParamName_buf[17];
			sprintf( FuncName_buf, "Lk_Uart%dFunc", i );
			sprintf( ParamName_buf, "Lk_Uart%dParam", i );
			
			uint8_t Func_temp[8];
			uint32_t Param_temp[2];			
			if( ReadParam( FuncName_buf, 0, 0, (uint64_t*)Func_temp, 0 )==PR_OK &&
					ReadParam( ParamName_buf, 0, 0, (uint64_t*)Param_temp, 0 )==PR_OK
			)
			{
				if( Func_temp[0] == 1 )
				{	//分配为Mavlink通信端口
					if( Param_temp[0] > 1000 )
						Ports[i].SetBaudRate(Param_temp[0],1,1);
					CommuPortRegister(i);
				}
				else if( Func_temp[0] >= 8 )
				{	//分配为自定义功能
					if( PortFuncs[Func_temp[0]].init )
						PortFuncs[Func_temp[0]].init( Ports[i], Param_temp[0] );
				}
			}		
		}
	}
	
	//分配CAN功能
	init_drv_Can();
	CANConfig CAN_config;
	bool CAN_driver_on[MAX_CANFUNCS] = {0};
	if( ReadParamGroup( "CAN", (uint64_t*)&CAN_config, 0 ) == PR_OK )
	{
		for( uint8_t i = 0; i < MAX_CANFUNCS/32; ++i )
		{
			for( uint8_t k = 0; k < 32; ++k )
			{
				if( CAN_config.Ena1[i*2] & (1<<k) )
				{	//需要打开驱动
					uint8_t drv_ind = i*32 + k;
					if( CanFuncs[drv_ind].init )
					{
						if( CanFuncs[drv_ind].init() )
							CAN_driver_on[drv_ind] = true;
					}
				}
			}
		}
	}
	for( uint8_t i = 0; i < MAX_CANFUNCS; ++i )
	{	//初始化完成执行所有驱动
		if( CAN_driver_on[i] && CanFuncs[i].run )
			CanFuncs[i].run();
	}
	
	//分配I2C功能
	I2CConfig I2C_config;
	bool I2C_driver_on[MAX_I2CFUNCS] = {0};
	if( ReadParamGroup( "I2C", (uint64_t*)&I2C_config, 0 ) == PR_OK )
	{
		for( uint8_t i = 0; i < MAX_I2CFUNCS/32; ++i )
		{
			for( uint8_t k = 0; k < 32; ++k )
			{
				if( I2C_config.Ena1[i*2] & (1<<k) )
				{	//需要打开驱动
					uint8_t drv_ind = i*32 + k;
					if( I2CFuncs[drv_ind].init )
					{
						if( I2CFuncs[drv_ind].init() )
							I2C_driver_on[drv_ind] = true;
					}
				}
			}
		}
	}
	for( uint8_t i = 0; i < MAX_I2CFUNCS; ++i )
	{	//初始化完成执行所有驱动
		if( I2C_driver_on[i] && I2CFuncs[i].run )
			I2CFuncs[i].run();
	}
	
	uint8_t CommulinkId_temp[8];
	ReadParam( "Lk_SysId", 0, 0, (uint64_t*)CommulinkId_temp, 0 );
	CommulinkSysId = CommulinkId_temp[0];
	ReadParam( "Lk_CompId", 0, 0, (uint64_t*)CommulinkId_temp, 0 );
	CommulinkCompId = CommulinkId_temp[0];
	
	//心跳包计数器
	uint16_t HeartBeat_counter = 0;
	//参数发送计数器
	uint16_t ParamSend_counter = 0;
	//系统状态记录计数器
	uint16_t SysStateLog_counter = 0;
	
	//低电量检测
	TIME LowPowerMessageSend_TIME(1);
	uint16_t LowPowerCnt = 0;
	uint8_t lowPowerState = 0;
	uint16_t lowPowerState1_counter = 0;
	uint16_t lowPowerState2_counter = 0;
	
	//位置传感器异常报警
	uint16_t PosAlarmCnt = 0;
	
	//航向报警
	uint16_t YawHealthCnt = 0;
	bool last_DAOSensorFixed = false;
	int8_t last_init_YawSensor = -1;
	uint8_t counter100Hz = 0;
	while(1)
	{
		vTaskDelay( 2 );
		
		bool tick100hz = false;
		bool sendHB = false;
		if( ++counter100Hz >= 4 )
		{
			counter100Hz = 0;
			tick100hz = true;
			
			//刷新led声光提示
			LEDRefresh(0.01f);

			//心跳包分频计数
			if( ++HeartBeat_counter >= 100 )
			{
				HeartBeat_counter = 0;
				sendHB = true;
			}
			
			bool inFlight;
			get_is_inFlight(&inFlight);
			
			/*记录系统状态Log*/
				bool unLocked;
				is_Attitude_Control_Enabled(&unLocked);
				if( unLocked )
				{
					uint32_t log[2];
					if( ReadParam( "SDLog_SysState", 0, 0, (uint64_t*)&log, 0 ) == PR_OK )
					{	//记录姿态
						if( log[0] )
						{
							if( ++SysStateLog_counter >= log[0] )
							{
								SysStateLog_counter = 0;
								SDLog_Msg_SysState();
							}
						}
					}
				}
			/*记录系统状态Log*/
			
			/*低电量报警*/								
				static float FcRTLVolt[2] = {0};
				static float FcLandVolt[2] = {0};
				static float FcAlarmVolt[2] = {0};
				if(++LowPowerCnt >= 500)
				{//5秒更新读取一次配置
					ReadParam( "Sf_FcRTLVolt", 0, 0, (uint64_t*)&FcRTLVolt[0], 0);
					ReadParam( "Sf_FcLandVolt", 0, 0, (uint64_t*)&FcLandVolt[0], 0);
					ReadParam( "Sf_AlarmVolt", 0, 0, (uint64_t*)&FcAlarmVolt[0], 0);
					LowPowerCnt = 0;
				}		
				//获取电压
				float mainBatVolt = 0;
				getCurrentBatteryTotalVoltFilted(&mainBatVolt);
				if( mainBatVolt > 7 ) 
				{
					if( FcLandVolt[0]>5 && mainBatVolt<FcLandVolt[0] )
					{	//电量低于降落电压
						if( lowPowerState < 3 )
						{
							if( ++lowPowerState1_counter >= 300 )
							{
								lowPowerState1_counter = 0;
								lowPowerState = 3;
							}
						}
						else
							lowPowerState1_counter = 0;						
					}
					else if( FcRTLVolt[0]>5 && mainBatVolt<FcRTLVolt[0] )
					{	//电量低于返航电压
						if( lowPowerState < 2 )
						{
							if( ++lowPowerState1_counter >= 300 )
							{
								lowPowerState1_counter = 0;
								lowPowerState = 2;
							}
						}
						else
							lowPowerState1_counter = 0;
					}
					else if( FcAlarmVolt[0]>5 && mainBatVolt<FcAlarmVolt[0] )
					{	//电量低于报警
						if( lowPowerState < 1 )
						{
							if( ++lowPowerState1_counter >= 300 )
							{
								lowPowerState1_counter = 0;
								lowPowerState = 1;
							}
						}
						else
							lowPowerState1_counter = 0;
					}
					else
						lowPowerState = 0;
				}else
					lowPowerState = 0;
				
				if( lowPowerState>0 ){
					sendLedSignal(LEDSignal_Err1);
					if(LowPowerMessageSend_TIME.get_pass_time()>5)
					{//发送低电量提示信息
						char text[50];
						switch( lowPowerState )
						{
							default:
							case 1:
								sprintf( text, "Voltage alarm!" );
								break;
							case 2:
								sprintf( text, "Low power! Return voltage." );
								break;
							case 3:
								sprintf( text, "Low power! Land voltage." );
								break;
						}
						
						for( uint8_t i = 0 ; i < MAVLINK_COMM_NUM_BUFFERS ; ++i )
						{
							const Port* port = get_CommuPort(i);
							if( port->write != 0 )
							{	
								mavlink_message_t msg_sd;
								if(mavlink_lock_chan( i, 0.01 )){
									mavlink_msg_statustext_pack_chan( 
										get_CommulinkSysId() ,	//system id
										get_CommulinkCompId() ,	//component id
										i ,	//chan
										&msg_sd,
										MAV_SEVERITY_ALERT,
										text,
										0,0
									);				
									mavlink_msg_to_send_buffer(port->write, 
																						 port->lock,
																						 port->unlock,
																						 &msg_sd, 0, 0.01);
									mavlink_unlock_chan(i);
								}
							}
						}
						LowPowerMessageSend_TIME = TIME::now();
					}				
				}
			/*低电量报警*/

			/*定位报警*/
				float msHealthXY = get_MSHealthXY();
				if( inFlight && msHealthXY>0 && msHealthXY<25 )
				{
					if( ++PosAlarmCnt >= 500 )
					{	//发送航向异常提示信息		
						PosAlarmCnt = 0;					
						const char text[] = "Positioning abnormal! Please return immediately!";						
						for( uint8_t i = 0 ; i < MAVLINK_COMM_NUM_BUFFERS ; ++i )
						{
							const Port* port = get_CommuPort(i);
							if( port->write != 0 )
							{	
								mavlink_message_t msg_sd;
								if(mavlink_lock_chan( i, 0.01 )){
									mavlink_msg_statustext_pack_chan( 
										get_CommulinkSysId() ,	//system id
										get_CommulinkCompId() ,	//component id
										i ,	//chan
										&msg_sd,
										MAV_SEVERITY_ALERT,
										text,
										0,0
									);				
									mavlink_msg_to_send_buffer(port->write, 
																						 port->lock,
																						 port->unlock,
																						 &msg_sd, 0, 0.01);
									mavlink_unlock_chan(i);
								}								
							}
						}
					}
				}
				else
					PosAlarmCnt = 60000;
			/*定位报警*/
				
			/*航向报警*/
				if( isGlobalXYSensorExist() && get_YawHealthEst()<0 )
				{
					if( ++YawHealthCnt >= 500 )
					{	//发送航向异常提示信息		
						YawHealthCnt = 0;					
						const char text[] = "Abnormal heading, check yaw angle!";						
						for( uint8_t i = 0 ; i < MAVLINK_COMM_NUM_BUFFERS ; ++i )
						{
							const Port* port = get_CommuPort(i);
							if( port->write != 0 )
							{	
								mavlink_message_t msg_sd;
								if(mavlink_lock_chan( i, 0.01 )){
									mavlink_msg_statustext_pack_chan( 
										get_CommulinkSysId() ,	//system id
										get_CommulinkCompId() ,	//component id
										i ,	//chan
										&msg_sd,
										MAV_SEVERITY_ALERT,
										text,
										0,0
									);				
									mavlink_msg_to_send_buffer(port->write, 
																						 port->lock,
																						 port->unlock,
																						 &msg_sd, 0, 0.01);
									mavlink_unlock_chan(i);
								}								
							}
						}
					}
				}
				else
					YawHealthCnt = 60000;
				
				int8_t init_YawSensor = get_init_YawSensor();
				if( last_init_YawSensor!=init_YawSensor && init_YawSensor>=0 && init_YawSensor<Internal_Magnetometer_Index )
				{	//发送初始磁罗盘传感器状态提示信息
					char text[30];
					sprintf( text, "External compass connected." );
					for( uint8_t i = 0 ; i < MAVLINK_COMM_NUM_BUFFERS ; ++i )
					{
						const Port* port = get_CommuPort(i);
						if( port->write != 0 )
						{	
							mavlink_message_t msg_sd;
							if(mavlink_lock_chan( i, 0.01 )){
								mavlink_msg_statustext_pack_chan( 
									get_CommulinkSysId() ,	//system id
									get_CommulinkCompId() ,	//component id
									i ,	//chan
									&msg_sd,
									MAV_SEVERITY_NOTICE,
									text,
									0,0
								);				
								mavlink_msg_to_send_buffer(port->write, 
																					 port->lock,
																					 port->unlock,
																					 &msg_sd, 0, 0.01);
								mavlink_unlock_chan(i);
							}								
						}
					}
				}
				last_init_YawSensor = init_YawSensor;
				
				if( last_DAOSensorFixed != isDAOSensorFixed() )
				{	//发送测向传感器状态提示信息
					char text[30];
					if( isDAOSensorFixed() )
						sprintf( text, "DF sensor connected." );
					else
						sprintf( text, "DF sensor disconnected." );
					for( uint8_t i = 0 ; i < MAVLINK_COMM_NUM_BUFFERS ; ++i )
					{
						const Port* port = get_CommuPort(i);
						if( port->write != 0 )
						{	
							mavlink_message_t msg_sd;
							if(mavlink_lock_chan( i, 0.01 )){
								mavlink_msg_statustext_pack_chan( 
									get_CommulinkSysId() ,	//system id
									get_CommulinkCompId() ,	//component id
									i ,	//chan
									&msg_sd,
									isDAOSensorFixed() ? MAV_SEVERITY_NOTICE : MAV_SEVERITY_ALERT,
									text,
									0,0
								);				
								mavlink_msg_to_send_buffer(port->write, 
																					 port->lock,
																					 port->unlock,
																					 &msg_sd, 0, 0.01);
								mavlink_unlock_chan(i);
							}								
						}
					}					
					last_DAOSensorFixed = isDAOSensorFixed();
				}
			/*航向报警*/
		}
			
		for( uint8_t i = 0 ; i < MAVLINK_COMM_NUM_BUFFERS ; ++i )
		{	//遍历所有端口
			mavlink_message_t msg_sd;
			if( Ports[CommuPorts[i]].write != 0 )
			{	//
				if( sendHB )
				{	//发送心跳包
					if( mavlink_lock_chan(i,0.01) )
					{
						extern bool GCS_is_MP;
						px4_custom_mode custom_mode;
						custom_mode.reserved = 0;
						custom_mode.main_mode = mav_main_mode;
						if( (mav_mode&MAV_MODE_FLAG_CUSTOM_MODE_ENABLED) && GCS_is_MP && mav_main_mode==PX4_CUSTOM_MAIN_MODE_AUTO )
							custom_mode.main_mode = 0;
						custom_mode.sub_mode = mav_sub_mode;						
						mavlink_msg_heartbeat_pack_chan( 
							get_CommulinkSysId() ,	//system id
							get_CommulinkCompId() ,	//component id
							i	,	//chan
							&msg_sd,
							MAV_TYPE_QUADROTOR ,	//type
							MAV_AUTOPILOT_PX4 ,	//autopilot
							mav_mode > 0 ? MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | mav_mode : 0 ,	//base mode
							custom_mode.data ,	//custom mode
							mav_state	//sys status
						);
						mavlink_msg_to_send_buffer( Ports[CommuPorts[i]].write, 
																			  Ports[CommuPorts[i]].lock,
																			  Ports[CommuPorts[i]].unlock,
																			 &msg_sd, 0, 0.01);
						mavlink_unlock_chan(i);
					}
				}
				
				/*发送消息列表中的消息*/
					#define MAX_SDMsgs 20
					uint16_t sdmsgs[MAX_SDMsgs];
					uint16_t sdmsgs_count = 0;
					if( xSemaphoreTake( SDMessagesMutex[i], 0.01*configTICK_RATE_HZ ) == pdTRUE )
					{					
						for( map<uint16_t,SDMsg>::iterator it = SDMessages[i].begin(); it != SDMessages[i].end(); ++it )
						{
							if( ++(it->second.counter) >= it->second.rate )
							{
								it->second.counter = 0;
								if( sdmsgs_count < MAX_SDMsgs )
									sdmsgs[sdmsgs_count++] = it->first;
								else
									break;
							}
						}
						xSemaphoreGive(SDMessagesMutex[i]);
					}
					for( uint16_t k = 0; k < sdmsgs_count; ++k )
					{
						if( sdmsgs[k]<Mavlink_Send_Funcs_Count && Mavlink_Send_Funcs[sdmsgs[k]]!=0 )
						{
							if( mavlink_get_proto_version(i)<2 && sdmsgs[k]>0xff )
								continue;
							if( mavlink_lock_chan( i, 0.01 ) )
							{
								if( Mavlink_Send_Funcs[sdmsgs[k]]( i, &msg_sd ) )
									mavlink_msg_to_send_buffer( Ports[CommuPorts[i]].write, 
																							Ports[CommuPorts[i]].lock,
																							Ports[CommuPorts[i]].unlock,
																						  &msg_sd, 0, 0.01 );
								mavlink_unlock_chan(i);
							}
						}
					}
				/*发送消息列表中的消息*/
					
				/*发送位置传感器信息*/
					sdmsgs_count = 0;
					if( xSemaphoreTake( SDMessagesMutex[i], 0.01*configTICK_RATE_HZ ) == pdTRUE )
					{					
						for( map<uint16_t,PosSDMsg>::iterator it = SDPosMsgs[i].begin(); it != SDPosMsgs[i].end(); /*++it*/ )
						{
							Position_Sensor_Data sensor;
							GetPositionSensorData( it->first, &sensor );
							if( it->second.last_updataT!=sensor.last_update_time || it->second.last_updataT.get_pass_time()>0.5 ) {
								it->second.last_updataT = sensor.last_update_time;
								if( it->second.rate > 0 ) 
								{	//按频率发送
									if( ++(it->second.counter) >= it->second.rate ) {
										it->second.counter = 0;
										if( sdmsgs_count < MAX_SDMsgs )
											sdmsgs[sdmsgs_count++] = it->first;
										else
											break;
									}
									++it;
								}
								else 
								{	//发送指定次数
									if( sdmsgs_count < MAX_SDMsgs )
										sdmsgs[sdmsgs_count++] = it->first;
									else
										break;
									if( ++(it->second.counter) >= 0 )
										SDPosMsgs[i].erase(it++);
									else
										++it;
								}
							}
							else
								++it;
						}
						xSemaphoreGive(SDMessagesMutex[i]);
					}
					for( uint16_t k = 0; k < sdmsgs_count; ++k )
					{
						if( mavlink_lock_chan( i, 0.01 ) )
						{
							if( Msg206_ACFlyPosSensor_INFO( i, &msg_sd, sdmsgs[k] ) ) {
								mavlink_msg_to_send_buffer( Ports[CommuPorts[i]].write, 
																						Ports[CommuPorts[i]].lock,
																						Ports[CommuPorts[i]].unlock,
																						&msg_sd, 0, 0.01 );
							}
							mavlink_unlock_chan(i);
						}
					}
				/*发送位置传感器信息*/
			}
			
			if( tick100hz )
			{
				//发送参数列表
				paramProtocolTask();
				//ftp任务
				ftpProtocolTask();
				
				/*发送航点请求*/
					//任务超时再次请求变量
					extern bool RqMissionInt[MAVLINK_COMM_NUM_BUFFERS];
					extern int32_t RqMissionInd[MAVLINK_COMM_NUM_BUFFERS];
					extern int32_t RqMissionCounter[MAVLINK_COMM_NUM_BUFFERS];
					extern uint8_t RqMissiontarget_sysid[MAVLINK_COMM_NUM_BUFFERS];
					extern uint8_t RqMissiontarget_compid[MAVLINK_COMM_NUM_BUFFERS];
					
					for( uint8_t i = 0; i < MAVLINK_COMM_NUM_BUFFERS; ++i )
					{
						if( RqMissionCounter[i] > 0 )
						{
							--RqMissionCounter[i];
							if( (RqMissionCounter[i] & 0xf) == 0 )
							{	//超时发送请求
								if( RqMissionInd[i] == 0 )
								{	//0号航点同时发送int和普通请求
									const Port* port = get_CommuPort(i);
									if( port->write != 0 )
									{
										mavlink_message_t msg_sd;
										if( mavlink_lock_chan( i, 0.01 ) )
										{
											mavlink_msg_mission_request_int_pack_chan( 
												get_CommulinkSysId() ,	//system id
												get_CommulinkCompId() ,	//component id
												i ,	//chan
												&msg_sd,
												RqMissiontarget_sysid[i] ,	//target system
												RqMissiontarget_compid[i] ,	//target component
												0 ,	//seq
												MAV_MISSION_TYPE_MISSION	//mission type
											
											);
											mavlink_msg_to_send_buffer(port->write, 
																								 port->lock,
																								 port->unlock,
																								 &msg_sd, 0, 0.01);
											mavlink_unlock_chan(i);
										}
										
										if( mavlink_lock_chan( i, 0.01 ) )
										{
											mavlink_msg_mission_request_pack_chan( 
												get_CommulinkSysId() ,	//system id
												get_CommulinkCompId() ,	//component id
												i ,	//chan
												&msg_sd,
												RqMissiontarget_sysid[i] ,	//target system
												RqMissiontarget_compid[i] ,	//target component
												0 ,	//seq
												MAV_MISSION_TYPE_MISSION	//mission type
											
											);
											mavlink_msg_to_send_buffer(port->write, 
																								 port->lock,
																								 port->unlock,
																								 &msg_sd, 0, 0.01);
											mavlink_unlock_chan(i);
										}
									}
								}
								else if( RqMissionInt[i] )
								{
									const Port* port = get_CommuPort(i);
									if( port->write != 0 )
									{
										mavlink_message_t msg_sd;
										if( mavlink_lock_chan( i, 0.01 ) )
										{
											mavlink_msg_mission_request_int_pack_chan( 
												get_CommulinkSysId() ,	//system id
												get_CommulinkCompId() ,	//component id
												i ,	//chan
												&msg_sd,
												RqMissiontarget_sysid[i] ,	//target system
												RqMissiontarget_compid[i] ,	//target component
												RqMissionInd[i] ,	//seq
												MAV_MISSION_TYPE_MISSION	//mission type
											
											);
											mavlink_msg_to_send_buffer(port->write, 
																								 port->lock,
																								 port->unlock,
																								 &msg_sd, 0, 0.01);
											mavlink_unlock_chan(i);
										}
									}
								}
								else
								{
									const Port* port = get_CommuPort(i);
									if( port->write != 0 )
									{
										mavlink_message_t msg_sd;									
										if( mavlink_lock_chan( i, 0.01 ) )
										{
											mavlink_msg_mission_request_pack_chan( 
												get_CommulinkSysId() ,	//system id
												get_CommulinkCompId() ,	//component id
												i ,	//chan
												&msg_sd,
												RqMissiontarget_sysid[i] ,	//target system
												RqMissiontarget_compid[i] ,	//target component
												RqMissionInd[i] ,	//seq
												MAV_MISSION_TYPE_MISSION	//mission type
											
											);
											mavlink_msg_to_send_buffer(port->write, 
																								 port->lock,
																								 port->unlock,
																								 &msg_sd, 0, 0.01);
											mavlink_unlock_chan(i);
										}
									}
								}
							}
						}
					}
				/*发送航点请求*/
					
				/*发送航点请求*/
					//任务超时再次请求变量
					extern bool RqFenceInt[MAVLINK_COMM_NUM_BUFFERS];
					extern int32_t RqFenceInd[MAVLINK_COMM_NUM_BUFFERS];
					extern int32_t RqFenceCounter[MAVLINK_COMM_NUM_BUFFERS];
					extern uint8_t RqFencetarget_sysid[MAVLINK_COMM_NUM_BUFFERS];
					extern uint8_t RqFencetarget_compid[MAVLINK_COMM_NUM_BUFFERS];
					
					for( uint8_t i = 0; i < MAVLINK_COMM_NUM_BUFFERS; ++i )
					{
						if( RqFenceCounter[i] > 0 )
						{
							--RqFenceCounter[i];
							if( (RqFenceCounter[i] & 0xf) == 0 )
							{	//超时发送请求
								if( RqFenceInd[i] == 0 )
								{	//0号航点同时发送int和普通请求
									const Port* port = get_CommuPort(i);
									if( port->write != 0 )
									{
										mavlink_message_t msg_sd;
										if( mavlink_lock_chan( i, 0.01 ) )
										{
											mavlink_msg_mission_request_int_pack_chan( 
												get_CommulinkSysId() ,	//system id
												get_CommulinkCompId() ,	//component id
												i ,	//chan
												&msg_sd,
												RqFencetarget_sysid[i] ,	//target system
												RqFencetarget_compid[i] ,	//target component
												0 ,	//seq
												MAV_MISSION_TYPE_FENCE	//mission type
											
											);
											mavlink_msg_to_send_buffer(port->write, 
																								 port->lock,
																								 port->unlock,
																								 &msg_sd, 0, 0.01);
											mavlink_unlock_chan(i);
										}
										
										if( mavlink_lock_chan( i, 0.01 ) )
										{
											mavlink_msg_mission_request_pack_chan( 
												get_CommulinkSysId() ,	//system id
												get_CommulinkCompId() ,	//component id
												i ,	//chan
												&msg_sd,
												RqFencetarget_sysid[i] ,	//target system
												RqFencetarget_compid[i] ,	//target component
												0 ,	//seq
												MAV_MISSION_TYPE_FENCE	//mission type
											
											);
											mavlink_msg_to_send_buffer(port->write, 
																								 port->lock,
																								 port->unlock,
																								 &msg_sd, 0, 0.01);
											mavlink_unlock_chan(i);
										}
									}
								}
								else if( RqFenceInt[i] )
								{
									const Port* port = get_CommuPort(i);
									if( port->write != 0 )
									{
										mavlink_message_t msg_sd;
										if( mavlink_lock_chan( i, 0.01 ) )
										{
											mavlink_msg_mission_request_int_pack_chan( 
												get_CommulinkSysId() ,	//system id
												get_CommulinkCompId() ,	//component id
												i ,	//chan
												&msg_sd,
												RqFencetarget_sysid[i] ,	//target system
												RqFencetarget_compid[i] ,	//target component
												RqFenceInd[i] ,	//seq
												MAV_MISSION_TYPE_FENCE	//mission type
											
											);
											mavlink_msg_to_send_buffer(port->write, 
																								 port->lock,
																								 port->unlock,
																								 &msg_sd, 0, 0.01);
											mavlink_unlock_chan(i);
										}
									}
								}
								else
								{
									const Port* port = get_CommuPort(i);
									if( port->write != 0 )
									{
										mavlink_message_t msg_sd;									
										if( mavlink_lock_chan( i, 0.01 ) )
										{
											mavlink_msg_mission_request_pack_chan( 
												get_CommulinkSysId() ,	//system id
												get_CommulinkCompId() ,	//component id
												i ,	//chan
												&msg_sd,
												RqFencetarget_sysid[i] ,	//target system
												RqFencetarget_compid[i] ,	//target component
												RqFenceInd[i] ,	//seq
												MAV_MISSION_TYPE_FENCE	//mission type
											
											);
											mavlink_msg_to_send_buffer(port->write, 
																								 port->lock,
																								 port->unlock,
																								 &msg_sd, 0, 0.01);
											mavlink_unlock_chan(i);
										}
									}
								}
							}
						}
					}
				/*发送航点请求*/
			}
				
			if( Ports[CommuPorts[i]].read != 0 )
			{	//接收数据处理
				//每次接收64个字节进行处理
				mavlink_message_t msg;
				uint8_t buf[64];
				uint8_t length;			
				do
				{
					length = Ports[CommuPorts[i]].read( buf, 64, 0, 0.01 );
					for( uint8_t k = 0; k < length; ++k )
					{
						//消息解包
						if( mavlink_parse_char( i, buf[k], &msg, NULL ) == MAVLINK_FRAMING_OK )
						{
							//消息解包完成
							
							//如果消息处理函数存在
							//处理消息
							if( msg.msgid < Mavlink_RC_Process_Count )
							{
								if( Mavlink_RC_Process[ msg.msgid ] != 0 )
									Mavlink_RC_Process[ msg.msgid ]( i , &msg );
							}
						}
					}
				}while( length > 10 );
			}
		}
	}
}

#define Commulink_StackSize 3950
Static_SRAM3Buf __attribute__((aligned(8))) StackType_t Commulink_Stack[Commulink_StackSize];
Static_SRAM1Buf StaticTask_t Commulink_TaskBuffer;
void init_Commulink()
{
	//初始化互斥锁
	for( uint8_t i = 0; i < MAVLINK_COMM_NUM_BUFFERS; ++i )
	{
		SDMessagesMutex[i] = xSemaphoreCreateMutex();
	}
	
	//注册虚拟摇杆
	ReceiverRegister( "JoyStickMv", true );
	
	//注册通信参数	
	CommulinkConfig initial_cfg;
	initial_cfg.sys_id[0] = 1;
	initial_cfg.comp_id[0] = 1;
	initial_cfg.Uart1_Func[0] = 1;
	initial_cfg.Uart1_Param[0] = 115200;
	initial_cfg.Uart3_Func[0] = 100;
	initial_cfg.Uart3_Param[0] = 0;
	initial_cfg.Uart5_Func[0] = 32;
	initial_cfg.Uart5_Param[0] = 0;
	initial_cfg.Uart7_Func[0] = 52;
	initial_cfg.Uart7_Param[0] = 0;
	initial_cfg.Uart8_Func[0] = 12;
	initial_cfg.Uart8_Param[0] = 0;
	MAV_PARAM_TYPE param_types[] = {
		MAV_PARAM_TYPE_UINT8 ,	//sysid
		MAV_PARAM_TYPE_UINT8 ,	//compid
		MAV_PARAM_TYPE_UINT8 ,	//Uart1_Func
		MAV_PARAM_TYPE_UINT32 ,	//Uart1_Param
		MAV_PARAM_TYPE_UINT8 ,	//Uart3_Func
		MAV_PARAM_TYPE_UINT32 ,	//Uart3_Param
		MAV_PARAM_TYPE_UINT8 ,	//Uart5_Func
		MAV_PARAM_TYPE_UINT32 ,	//Uart5_Param
		MAV_PARAM_TYPE_UINT8 ,	//Uart7_Func
		MAV_PARAM_TYPE_UINT32 ,	//Uart7_Param
		MAV_PARAM_TYPE_UINT8 ,	//Uart8_Func
		MAV_PARAM_TYPE_UINT32 ,	//Uart8_Param
	};
	SName param_names[] = {
		"Lk_SysId" ,	//sysid
		"Lk_CompId" ,	//compid
		"Lk_Uart1Func" ,
		"Lk_Uart1Param" ,
		"Lk_Uart3Func" ,
		"Lk_Uart3Param" ,
		"Lk_Uart5Func" ,
		"Lk_Uart5Param" ,
		"Lk_Uart7Func" ,
		"Lk_Uart7Param" ,
		"Lk_Uart8Func" ,
		"Lk_Uart8Param" ,
	};
	ParamGroupRegister( "Commulink", 5, sizeof(initial_cfg)/8, param_types, param_names, (uint64_t*)&initial_cfg );
	
	//注册Can参数
	CANConfig initial_CANConfigcfg;
	initial_CANConfigcfg.Baud[0] = 500000;
	initial_CANConfigcfg.DataBaud[0] = 5000000;
	initial_CANConfigcfg.Ena1[0] = 0;
	initial_CANConfigcfg.Ena2[0] = 0;
	initial_CANConfigcfg.Ena3[0] = 0;
	initial_CANConfigcfg.Ena4[0] = 0;
	MAV_PARAM_TYPE CANparam_types[] = {
		MAV_PARAM_TYPE_UINT32 ,	//Baud
		MAV_PARAM_TYPE_UINT32 ,	//DataBaud
		MAV_PARAM_TYPE_UINT32 ,	//Ena1
		MAV_PARAM_TYPE_UINT32 ,	//Ena2
		MAV_PARAM_TYPE_UINT32 ,	//Ena3
		MAV_PARAM_TYPE_UINT32 ,	//Ena4
	};
	SName CANparam_names[] = {
		"CAN_Baud" ,
		"CAN_DataBaud" ,
		"CAN_DrvEna1" ,
		"CAN_DrvEna2" ,
		"CAN_DrvEna3" ,
		"CAN_DrvEna4" ,
	};
	ParamGroupRegister( "CAN", 5, sizeof(initial_CANConfigcfg)/8, CANparam_types, CANparam_names, (uint64_t*)&initial_CANConfigcfg );
	
	//注册I2C参数
	I2CConfig initial_I2CConfigcfg;
	initial_I2CConfigcfg.Ena1[0] = 2;
	initial_I2CConfigcfg.Ena2[0] = 1;
	initial_I2CConfigcfg.Ena3[0] = 0;
	initial_I2CConfigcfg.Ena4[0] = 0;
	MAV_PARAM_TYPE I2Cparam_types[] = {
		MAV_PARAM_TYPE_UINT32 ,	//Ena1
		MAV_PARAM_TYPE_UINT32 ,	//Ena2
		MAV_PARAM_TYPE_UINT32 ,	//Ena3
		MAV_PARAM_TYPE_UINT32 ,	//Ena4
	};
	SName I2Cparam_names[] = {
		"I2C_DrvEna1" ,
		"I2C_DrvEna2" ,
		"I2C_DrvEna3" ,
		"I2C_DrvEna4" ,
	};
	ParamGroupRegister( "I2C", 2, sizeof(initial_I2CConfigcfg)/8, I2Cparam_types, I2Cparam_names, (uint64_t*)&initial_I2CConfigcfg );
	
	//xTaskCreate( Commulink_Server, "Commulink", 3000, NULL, SysPriority_UserTask, NULL);
	xTaskCreateStatic( Commulink_Server , "Commulink" ,Commulink_StackSize, NULL, SysPriority_UserTask, Commulink_Stack, &Commulink_TaskBuffer);
}