#include "drv_UM982.hpp"
#include "drv_RTK_DAO_Base.hpp"
#include "Basic.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "SensorsBackend.hpp"
#include "Parameters.hpp"
#include "Commulink.hpp"
#include "StorageSystem.hpp"
#include "drv_CRC.hpp"
#include <string.h>
#include <assert.h>
#include <stdlib.h>

const uint32_t aulCrcTable[256] =
{
	0x00000000UL, 0x77073096UL, 0xee0e612cUL, 0x990951baUL, 0x076dc419UL, 0x706af48fUL,
	0xe963a535UL, 0x9e6495a3UL, 0x0edb8832UL, 0x79dcb8a4UL, 0xe0d5e91eUL, 0x97d2d988UL,
	0x09b64c2bUL, 0x7eb17cbdUL, 0xe7b82d07UL, 0x90bf1d91UL, 0x1db71064UL, 0x6ab020f2UL,
	0xf3b97148UL, 0x84be41deUL, 0x1adad47dUL, 0x6ddde4ebUL, 0xf4d4b551UL, 0x83d385c7UL,
	0x136c9856UL, 0x646ba8c0UL, 0xfd62f97aUL, 0x8a65c9ecUL, 0x14015c4fUL, 0x63066cd9UL,
	0xfa0f3d63UL, 0x8d080df5UL, 0x3b6e20c8UL, 0x4c69105eUL, 0xd56041e4UL, 0xa2677172UL,
	0x3c03e4d1UL, 0x4b04d447UL, 0xd20d85fdUL, 0xa50ab56bUL, 0x35b5a8faUL, 0x42b2986cUL,
	0xdbbbc9d6UL, 0xacbcf940UL, 0x32d86ce3UL, 0x45df5c75UL, 0xdcd60dcfUL, 0xabd13d59UL,
	0x26d930acUL, 0x51de003aUL, 0xc8d75180UL, 0xbfd06116UL, 0x21b4f4b5UL, 0x56b3c423UL,
	0xcfba9599UL, 0xb8bda50fUL, 0x2802b89eUL, 0x5f058808UL, 0xc60cd9b2UL, 0xb10be924UL,
	0x2f6f7c87UL, 0x58684c11UL, 0xc1611dabUL, 0xb6662d3dUL, 0x76dc4190UL, 0x01db7106UL,
	0x98d220bcUL, 0xefd5102aUL, 0x71b18589UL, 0x06b6b51fUL, 0x9fbfe4a5UL, 0xe8b8d433UL,
	0x7807c9a2UL, 0x0f00f934UL, 0x9609a88eUL, 0xe10e9818UL, 0x7f6a0dbbUL, 0x086d3d2dUL,
	0x91646c97UL, 0xe6635c01UL, 0x6b6b51f4UL, 0x1c6c6162UL, 0x856530d8UL, 0xf262004eUL,
	0x6c0695edUL, 0x1b01a57bUL, 0x8208f4c1UL, 0xf50fc457UL, 0x65b0d9c6UL, 0x12b7e950UL,
	0x8bbeb8eaUL, 0xfcb9887cUL, 0x62dd1ddfUL, 0x15da2d49UL, 0x8cd37cf3UL, 0xfbd44c65UL,
	0x4db26158UL, 0x3ab551ceUL, 0xa3bc0074UL, 0xd4bb30e2UL, 0x4adfa541UL, 0x3dd895d7UL,
	0xa4d1c46dUL, 0xd3d6f4fbUL, 0x4369e96aUL, 0x346ed9fcUL, 0xad678846UL, 0xda60b8d0UL,
	0x44042d73UL, 0x33031de5UL, 0xaa0a4c5fUL, 0xdd0d7cc9UL, 0x5005713cUL, 0x270241aaUL,
	0xbe0b1010UL, 0xc90c2086UL, 0x5768b525UL, 0x206f85b3UL, 0xb966d409UL, 0xce61e49fUL,
	0x5edef90eUL, 0x29d9c998UL, 0xb0d09822UL, 0xc7d7a8b4UL, 0x59b33d17UL, 0x2eb40d81UL,
	0xb7bd5c3bUL, 0xc0ba6cadUL, 0xedb88320UL, 0x9abfb3b6UL, 0x03b6e20cUL, 0x74b1d29aUL,
	0xead54739UL, 0x9dd277afUL, 0x04db2615UL, 0x73dc1683UL, 0xe3630b12UL, 0x94643b84UL,
	0x0d6d6a3eUL, 0x7a6a5aa8UL, 0xe40ecf0bUL, 0x9309ff9dUL, 0x0a00ae27UL, 0x7d079eb1UL,
	0xf00f9344UL, 0x8708a3d2UL, 0x1e01f268UL, 0x6906c2feUL, 0xf762575dUL, 0x806567cbUL,
	0x196c3671UL, 0x6e6b06e7UL, 0xfed41b76UL, 0x89d32be0UL, 0x10da7a5aUL, 0x67dd4accUL,
	0xf9b9df6fUL, 0x8ebeeff9UL, 0x17b7be43UL, 0x60b08ed5UL, 0xd6d6a3e8UL, 0xa1d1937eUL,
	0x38d8c2c4UL, 0x4fdff252UL, 0xd1bb67f1UL, 0xa6bc5767UL, 0x3fb506ddUL, 0x48b2364bUL,
	0xd80d2bdaUL, 0xaf0a1b4cUL, 0x36034af6UL, 0x41047a60UL, 0xdf60efc3UL, 0xa867df55UL,
	0x316e8eefUL, 0x4669be79UL, 0xcb61b38cUL, 0xbc66831aUL, 0x256fd2a0UL, 0x5268e236UL,
	0xcc0c7795UL, 0xbb0b4703UL, 0x220216b9UL, 0x5505262fUL, 0xc5ba3bbeUL, 0xb2bd0b28UL,
	0x2bb45a92UL, 0x5cb36a04UL, 0xc2d7ffa7UL, 0xb5d0cf31UL, 0x2cd99e8bUL, 0x5bdeae1dUL,
	0x9b64c2b0UL, 0xec63f226UL, 0x756aa39cUL, 0x026d930aUL, 0x9c0906a9UL, 0xeb0e363fUL,
	0x72076785UL, 0x05005713UL, 0x95bf4a82UL, 0xe2b87a14UL, 0x7bb12baeUL, 0x0cb61b38UL,
	0x92d28e9bUL, 0xe5d5be0dUL, 0x7cdcefb7UL, 0x0bdbdf21UL, 0x86d3d2d4UL, 0xf1d4e242UL,
	0x68ddb3f8UL, 0x1fda836eUL, 0x81be16cdUL, 0xf6b9265bUL, 0x6fb077e1UL, 0x18b74777UL,
	0x88085ae6UL, 0xff0f6a70UL, 0x66063bcaUL, 0x11010b5cUL, 0x8f659effUL, 0xf862ae69UL,
	0x616bffd3UL, 0x166ccf45UL, 0xa00ae278UL, 0xd70dd2eeUL, 0x4e048354UL, 0x3903b3c2UL,
	0xa7672661UL, 0xd06016f7UL, 0x4969474dUL, 0x3e6e77dbUL, 0xaed16a4aUL, 0xd9d65adcUL,
	0x40df0b66UL, 0x37d83bf0UL, 0xa9bcae53UL, 0xdebb9ec5UL, 0x47b2cf7fUL, 0x30b5ffe9UL,
	0xbdbdf21cUL, 0xcabac28aUL, 0x53b39330UL, 0x24b4a3a6UL, 0xbad03605UL, 0xcdd70693UL,
	0x54de5729UL, 0x23d967bfUL, 0xb3667a2eUL, 0xc4614ab8UL, 0x5d681b02UL, 0x2a6f2b94UL,
	0xb40bbe37UL, 0xc30c8ea1UL, 0x5a05df1bUL, 0x2d02ef8dUL
};

// Calculate and return the CRC for usA binary buffer
uint32_t CalculateCRC32(uint8_t *szBuf, int iSize)
{
	int iIndex;
	uint32_t ulCRC = 0;
	for (iIndex=0; iIndex<iSize; iIndex++)
	{
		ulCRC = aulCrcTable[(ulCRC ^ szBuf[iIndex]) & 0xff] ^ (ulCRC >> 8);
	}
	return ulCRC;
} 


#define INIT_LENGHT 80

struct DriverInfo
{
	uint32_t param;
	Port port;
};

struct GpsConfig
{
	uint8_t GNSS_Mode[8];
	float delay[2];
};

enum GPS_Scan_Operation
{
	//在指定波特率下发送初始化信息
	GPS_Scan_Baud9600 = 9600 ,
	GPS_Scan_Baud38400 = 38400 ,
	GPS_Scan_Baud460800 = 460800 ,
	GPS_Scan_Baud115200 = 115200 ,
	GPS_Scan_Baud57600 = 57600 ,
	
	//检查是否设置成功
	GPS_Check_Baud ,
	//在当前波特率下再次发送配置
	GPS_ResendConfig ,
	//GPS已检测存在
	GPS_Present ,
};

enum FrameName
{
	FrameNone=0,
	FrameKSXT,
	FrameGGA,
	FrameGSA,
	FrameGST,
	FrameGSV,
	FrameHDT,
	FrameHDT2,
	FrameGNRMC,
	FrameGPRMC,
	FrameGPVTG,
	FrameGPGLL,
	FrameGPZDA,
	FrameEVENTMARK
};

struct GPS_State_Machine
{
	FrameName frame_name = FrameNone;
	uint32_t frame_datas_ind = 0;
	uint16_t frame_datas_length;
	uint8_t read_state = 0;	
	uint16_t crc=0;
};

uint8_t AsciiToByte(uint8_t b)
{
	uint8_t ret = 0;
	if(b >= '0' && b <= '9')
		ret = b - '0';
	else if(b >= 'A' && b <= 'F')
		ret = b - 'A' + 10;
	else if(b >= 'a' && b <= 'f')
		ret = b - 'a' + 10;
	else
		ret = 0;
	return ret;
}

uint8_t NMEA_Split_Pos(uint8_t *str,uint8_t pos)
{	 		    
	uint8_t offset = 0;
	while(pos)
	{		 
		if(*str=='*'||*str<' '||*str>'z')
			return 0xff;
		if(*str==',')
			pos--;
		str++;
		offset++;
	}
	return offset;	 
}

double NMEA_Pow(double m,uint8_t n)
{
	double result=1;	 
	while(n--)
		result*=m;    
	return result;
}

bool NMEA_Str2num(uint8_t *str, double& result)
{
	uint8_t *p=str;
	uint8_t ilen=0,flen=0,i;
	uint8_t mask=0;
	int64_t integerNum=0;
	double floatNum=0;
	int readCnt=0;
	while(1)
	{// 获取整数和小数的位数		
		readCnt++;
		if(readCnt>100)
			return false;
		
		if(*p=='-'){
			mask|=0x02;
			p++;
		}
		
		if(*p==','||(*p=='*'))
			break;		
		
	  if(*p=='.'){
			mask|=0X01;
			p++;
		}
		else if(*p>'9'||(*p<'0'))
		{	
			ilen=0;
			flen=0;
			break;
		}	
		if(mask&0X01)
			flen++;
		else 
			ilen++;
		p++;
	}
	
	// 跳过负号
	if(mask&0X02)
		str++;
	
	if(ilen>16)
		return false;

	if(flen>16)
		return false;
	
	// 计算整数大小
	for(i=0;i<ilen;i++)	 
		integerNum+=NMEA_Pow(10,ilen-1-i)*(str[i]-'0');

	// 计算小数大小	
	for(i=0;i<flen;i++) 
		floatNum+=NMEA_Pow(0.1,i+1)*(str[ilen+1+i]-'0');
	
	double res = integerNum+floatNum;
	
	// 是否是负数
	if(mask&0X02)
		res=-res;
	
	result = res;
	return true;
}	  			


static inline void ResetRxStateMachine( GPS_State_Machine* state_machine )
{
	state_machine->read_state=state_machine->frame_datas_ind=0;
}

extern float debug_test[30];
static inline bool NEMA_Parse( GPS_State_Machine* state_machine, uint8_t* frame_datas, uint8_t r_data )
{
	frame_datas[ state_machine->frame_datas_ind++ ] = r_data;
	switch( state_machine->read_state )
	{
		case 0:	//找包头
		{				
			if( state_machine->frame_datas_ind == 1 )
			{
				if( r_data != '$' )
					state_machine->frame_datas_ind = 0;
				else
					state_machine->read_state = 1;
			}
			break;
		}
		case 1:
		{	
			if( state_machine->frame_datas_ind == 6 )
			{			
				if(strncmp((char*)&frame_datas[1],"KSXT",4)==0){
					state_machine->frame_name = FrameKSXT;
					state_machine->read_state = 2;
				}
				else if(strncmp((char*)&frame_datas[1],"GNGGA",5)==0){
					state_machine->frame_name = FrameGGA;
					state_machine->read_state = 2;
				}
				else if(strncmp((char*)&frame_datas[1],"GNGSA",5)==0){
					state_machine->frame_name = FrameGSA;
					state_machine->read_state = 2;
				}else if(strncmp((char*)&frame_datas[1],"eventMark",9)==0){
					state_machine->frame_name = FrameEVENTMARK;
					state_machine->read_state = 2;
				}
				else
					ResetRxStateMachine(state_machine);	
			}
			break;
		}
		case 2:	//*
		{
			if( r_data == '*' ){
				state_machine->read_state = 3;
				state_machine->crc=0;	
			}				
			break;
		}
	  case 3:	//校验第一个字节
		{
			state_machine->crc = AsciiToByte(r_data)*16;
			state_machine->read_state = 4;
			break;
		}
		case 4:	//校验第二个字节
		{
			state_machine->crc += AsciiToByte(r_data);
			state_machine->read_state = 5;	
			break;
		}
		case 5:	//0x0D
		{
			if( r_data == 0x0D ){
				state_machine->read_state = 6;	
			}else
				ResetRxStateMachine(state_machine);
			break;
		}
		case 6:	//0x0A
		{	
			ResetRxStateMachine(state_machine);
			if( r_data == 0x0A )
			{
 				uint8_t result = frame_datas[1];
				for(int i = 2; frame_datas[i]!='*';i++ )
					result ^= frame_datas[i];
				if( result == state_machine->crc )
					return true;	
				else
					return false;
			}
			break;
		}
	}
	return false;
}

static bool send_init_msg(DriverInfo& driver_info)
{
  int len=0;
	char UM982Clr[100];
	len = sprintf( UM982Clr, "UNLOG COM2\r\n");	
	driver_info.port.write( (uint8_t*)&UM982Clr[0], len, portMAX_DELAY, portMAX_DELAY );	
	driver_info.port.wait_sent(1);
	driver_info.port.reset_rx(0.1);
	len = sprintf( UM982Clr, "CONFIG\r\n");	
	driver_info.port.write( (uint8_t*)&UM982Clr[0], len, portMAX_DELAY, portMAX_DELAY );	
	driver_info.port.wait_sent(1);
	os_delay(0.1);
	const char CmdResult[] = "$command,CONFIG,response: OK*54";
	const uint8_t CmdResultLen = strlen(CmdResult);
	len=0;
	TIME waitCorrectResponseTime = TIME::now();
	while( waitCorrectResponseTime.get_pass_time() < 3 )
	{
		if(driver_info.port.read( (uint8_t*)&UM982Clr[0], 1, 0.5, 1 )==0)
			return false;
		if( len < CmdResultLen )
		{
			if( UM982Clr[0] == CmdResult[len] )
				++len;
			else
				len = 0;
		}
		else
				break;
	}
	
	if( len != CmdResultLen )
		return false;

	// 波特率匹配完成
	for( uint8_t i=0; i < 20; i++ )
	{
		if(i==0)      // 停止所有信息输出
			len = sprintf( UM982Clr,  "UNLOG COM2\r\n");	
			
		else if(i==1) // 流动站工作模式
			len = sprintf( UM982Clr,  "MODE ROVER\r\n");	
		
		else if (i==2)// 设置 RTK 模块指令，数据最大龄期，秒为单位
			len = sprintf( UM982Clr,  "CONFIG RTK TIMEOUT 600\r\n");
		
		else if (i==3)// 设置接收的 DGPS差分数据的最大龄期，秒为单位，接收到的滞后于指定龄期的 DGPS 差分数据被忽略，也用于禁止 DGPS 定位计算
			len = sprintf( UM982Clr,  "CONFIG DGPS TIMEOUT 600\r\n");
		
		else if (i==4)// 设置双天线接收机的主天线（ANT1）与从天线（ANT2）之间距离保持固定
			len = sprintf( UM982Clr,  "CONFIG HEADING FIXLENGTH\r\n");			
		
		else if (i==5)// 设置 EVENT 功能，下降沿有效、两个有效脉冲之间的最短时间要求, 单位ms。若小于 TGuard，则第二个 Event 被忽视。默认值: 4，最小： 2；最大：3,599,999
			len = sprintf( UM982Clr,  "CONFIG EVENT ENABLE NEGATIVE 2\r\n");	
		
		else if (i==6)// 设置关闭单频定位状态的判定条件，基线长度超过 5km 时，精度满足需求依然可以使用固定解
			len = sprintf( UM982Clr,  "CONFIG SFRTK Enable\r\n");			
		
		else if (i==7)// 将 BDS 系统卫星的 B1C&B2a 信号编入 RTCM 协议
			len = sprintf( UM982Clr,  "CONFIG RTCMB1CB2a Enable\r\n");
		
		else if (i==8)// 使能接收机跟踪 GPS 卫星系
			len = sprintf( UM982Clr,  "UNMASK GPS\r\n");		
		
		else if (i==9)// 使能接收机跟踪 BDS 卫星系统
			len = sprintf( UM982Clr,  "UNMASK BDS\r\n");
		
		else if (i==10)// 使能接收机跟踪 GLO 卫星系统
			len = sprintf( UM982Clr, "UNMASK GLO\r\n");
		
		else if (i==11)// 使能接收机跟踪 GAL 卫星系统
			len = sprintf( UM982Clr,  "UNMASK GAL\r\n");

		else if (i==12)// 使能接收机跟踪 QZSS 卫星系统
			len = sprintf( UM982Clr,  "UNMASK QZSS\r\n");
		
		else if (i==13)// 输出 EVENT 发生时刻的精确绝对时间及相对时间
			len = sprintf( UM982Clr,  "LOG EVENTMARKA ONCHANGED\r\n");	

	  else if (i==14)// 设置接收机跟踪卫星的卫星截止角度
			len = sprintf( UM982Clr,  "MASK 10\r\n");	
		
		else if (i==15)// 接收机默设置为动态模式
			len = sprintf( UM982Clr,  "RTKDYNAMICS DYNAMIC\r\n");	
		
		else if (i==16)// 在 com2 输出 20Hz 的 KSXT 信息
			len = sprintf( UM982Clr,  "LOG KSXT ONTIME 0.05\r\n");
		
		else if (i==17)// 在 com2 输出 2 Hz 的 GPGSA 信息
			len = sprintf( UM982Clr,  "LOG GPGSA ONTIME 0.5\r\n");	
		
		else if (i==18)// 在 com2 输出 20Hz 的 GNGGA 信息	
			len = sprintf( UM982Clr,  "LOG GNGGA ONTIME 0.05\r\n");
		
		else if (i==19)// 设置com2波特率为460800,8位数据位,无校验,一位停止位
			len = sprintf( UM982Clr,  "CONFIG COM2 460800 8 n 1\r\n");
	
		driver_info.port.write( (uint8_t*)&UM982Clr[0], len, portMAX_DELAY, portMAX_DELAY );	
		driver_info.port.wait_sent(1);
		os_delay(0.2);
	}	
	return true;
}


static void RTK_Server(void* pvParameters)
{	
	DriverInfo driver_info = *(DriverInfo*)pvParameters;
	delete (DriverInfo*)pvParameters;
	
	//GPS识别状态
	GPS_Scan_Operation current_GPS_Operation = GPS_Scan_Baud115200;
	//数据读取状态机
	Static_AXIDMABuf __attribute__((aligned(4))) uint8_t frame_datas[4096];
	//上次更新时间
	TIME last_update_time;
	
	//等待初始化完成
	while( getInitializationCompleted() == false )
		os_delay(0.1);	
	
	//注册Rtk端口
	RtkPort rtk_port;
	rtk_port.ena = false;
	rtk_port.write = driver_info.port.write;
	rtk_port.lock = driver_info.port.lock;
	rtk_port.unlock = driver_info.port.unlock;
	int8_t rtk_port_ind = RtkPortRegister(rtk_port);	
	
	bool rtc_updated = false;
	
	//读取是否需要记录PPK
	bool record_ppk = false;
	uint8_t log_ppk[8];
	if( ReadParam( "SDLog_PPK", 0, 0, (uint64_t*)log_ppk, 0 ) == PR_OK )
	{
		if( log_ppk[0] == 4 )
			record_ppk = true;
	}
//	driver_info.port.SetBaudRate( 460800, 1, 0.1 );	
//	goto GPS_Present;
GPS_CheckBaud:
	//os_delay(15.0);
	while(1)
	{
		//更改指定波特率
		driver_info.port.SetBaudRate( current_GPS_Operation, 3, 0.1 );
		//切换波特率
		switch(current_GPS_Operation)
		{
			case GPS_Scan_Baud9600:
				current_GPS_Operation = GPS_Scan_Baud57600;
				break;
			case GPS_Scan_Baud57600:
				current_GPS_Operation = GPS_Scan_Baud38400;
				break;
			case GPS_Scan_Baud38400:
				current_GPS_Operation = GPS_Scan_Baud460800;
				break;
			case GPS_Scan_Baud460800:
				current_GPS_Operation = GPS_Scan_Baud115200;
				break;
			default:
			case GPS_Scan_Baud115200:
				current_GPS_Operation = GPS_Scan_Baud9600;
				break;
		}
		//发送配置
    if( send_init_msg(driver_info) == false )
			continue;

		//更改波特率
		driver_info.port.SetBaudRate( 460800, 1, 0.1 );		
		//清空接收缓冲区准备接收数据
		driver_info.port.reset_rx(0.1);
		GPS_State_Machine gps_state;
		ResetRxStateMachine(&gps_state);
		TIME RxChkStartTime = TIME::now();
		
		while(	RxChkStartTime.get_pass_time()<3 )
		{
			uint8_t r_data;
			if( driver_info.port.read( &r_data, 1, 0.5, 0.1 ) )
			{
				bool res = NEMA_Parse( &gps_state, frame_datas, r_data );
				if( res )
				{
					if( gps_state.frame_name == FrameKSXT )
					{	//已识别,跳转到gps接收程序
						goto GPS_Present;
					}
				}
			}
		}		
	}	
	
GPS_Present:
	//重发配置
	//send_init_msg(driver_info);
		
	uint32_t sensor_key = 0;
	GpsDAOConfig gps_cfg;
	if( ReadParamGroup( "GPSDAOCfg", (uint64_t*)&gps_cfg, 0 ) == PR_OK )
	{	
		//注册传感器
		sensor_key = PositionSensorRegister( default_rtk_sensor_index , \
																					"RTK_UM982" ,\
																					Position_Sensor_Type_GlobalPositioning , \
																					Position_Sensor_DataType_sv_xy , \
																					Position_Sensor_frame_ENU , \
																					gps_cfg.delay[0] , //延时
																					30 , //xy信任度
																					30 //z信任度
																				);	
		
		//注册侧向传感器
		DAOSensorRegister( 0, "DRTK", vector3<double>(gps_cfg.DRTK_VecX[0],gps_cfg.DRTK_VecY[0],gps_cfg.DRTK_VecZ[0]), gps_cfg.delay[0] );
	}
	else
	{	//注册传感器
		sensor_key = PositionSensorRegister( default_rtk_sensor_index , \
																					"RTK_UM982" ,\
																					Position_Sensor_Type_GlobalPositioning , \
																					Position_Sensor_DataType_sv_xy , \
																					Position_Sensor_frame_ENU , \
																					0.1 , //延时
																					30 , //xy信任度
																					30 //z信任度
																				);
		gps_cfg.DRTK_VecX[0] = gps_cfg.DRTK_VecY[0] = gps_cfg.DRTK_VecZ[0] = 0;
	}
	//开启Rtk注入
	RtkPort_setEna( rtk_port_ind, true );
 
	//gps状态
	bool gps_available = false;
	bool z_available=false;
	double alt_offset = -1000000;
	TIME GPS_stable_start_time(false);
	double gps_alt;
	TIME gps_update_TIME;
	
	//清空接收缓冲区准备接收数据
	driver_info.port.reset_rx(0.1);
	GPS_State_Machine gps_state;
	ResetRxStateMachine(&gps_state);
	frame_datas[0] = frame_datas[1] = 0;
	last_update_time = TIME::now();
	
	
	//附加数据
	double addition_inf[8] = {0};
	
	while(1)
	{
		uint8_t r_data;
		if( driver_info.port.read( &r_data, 1, 2, 0.1 ) )
		{
			if( NEMA_Parse( &gps_state, frame_datas, r_data ) )
			{
				struct NEMA_Nav_Pack
				{	
					uint16_t year;
					uint8_t month;
					uint8_t day;
					uint8_t hour;
					uint8_t min;
					uint8_t sec;
					float subSec;
					double lon;
					double lat;
					double height;
					float heading;
					float speedHeading;	
					float vel;	
					uint8_t posQuality;
					uint8_t headingQuality;
					uint8_t numSvANT1;
					uint8_t numSvANT2;
					float VelE;
					float VelN;
					float VelU;
					float hdop;
					float vdop;
					uint8_t mode;	
				}__attribute__((packed));
				NEMA_Nav_Pack pack;		
				
				extern float debug_test[30];	
				static float HZ=0,HZ2=0,HZ3=0;	
        static TIME tt1(1),tt2(1),tt3(1);					
//				if(gps_state.frame_name == FrameGSA){
//					HZ = 1.0f/tt1.get_pass_time();
//					//debug_test[20]=HZ;
//					tt1 = TIME::now();
//				}
				
				if(gps_state.frame_name == FrameKSXT){
					HZ2 = 1.0f/tt2.get_pass_time();
					debug_test[21]=HZ2;
					tt2 = TIME::now();
				}
				
				if(gps_state.frame_name == FrameGGA){
					HZ3 = 1.0f/tt3.get_pass_time();
					debug_test[22]=HZ3;
					tt3 = TIME::now();
				}		
					
				if( gps_state.frame_name == FrameKSXT )
				{ 
					//记录
//					if(record_ppk)
//						SDLog_Ubx( (const char*)frame_datas, 100 );

//					//解析测试
//					uint8_t data[]="113.38957854,23.04239933";
//					static double temp1=0;
//					NMEA_Str2num((uint8_t*)&data[0],temp1);
					
					last_update_time = TIME::now();
					uint8_t point_pos;			 
					uint8_t split_pos; 					
					//UTC时间
					split_pos=NMEA_Split_Pos((uint8_t*)&frame_datas[0],1);								
					if(split_pos!=0XFF && frame_datas[split_pos]!=',' && (pack.mode & 0x0f)==3 || pack.posQuality==2 || pack.posQuality==3){
						pack.year =  AsciiToByte(frame_datas[6])*1e3 + AsciiToByte(frame_datas[7])*1e2 + AsciiToByte(frame_datas[8])*1e1 + AsciiToByte(frame_datas[9]);
						pack.month = AsciiToByte(frame_datas[10])*10 + AsciiToByte(frame_datas[11]);
						pack.day =   AsciiToByte(frame_datas[12])*10 + AsciiToByte(frame_datas[13]);
						pack.hour =  AsciiToByte(frame_datas[14])*10 + AsciiToByte(frame_datas[15]);
						pack.min =   AsciiToByte(frame_datas[16])*10 + AsciiToByte(frame_datas[17]);
						pack.sec =   AsciiToByte(frame_datas[18])*10 + AsciiToByte(frame_datas[19]);
						pack.subSec =AsciiToByte(frame_datas[21])*10 + AsciiToByte(frame_datas[22]);
						/*更新RTC时间(本地时间)*/	
						if( rtc_updated==false && gps_available ){
							if( Lock_RTC() )
							{
								if( get_RTC_Updated() == false )
								{
									int8_t TimeZone = 8;//默认东8区(北京时区)
									RTC_TimeStruct rtc;
									TimeZone = GetTimeZone(pack.lat,pack.lon);
									UTC2LocalTime(&rtc, pack.year, pack.month, pack.day, pack.hour, pack.min, pack.sec, TimeZone, 0);
									Set_RTC_Time(&rtc);	
								}
								rtc_updated = true;
								UnLock_RTC();
							}
						}
						/*更新RTC时时间(本地时间)*/			
					}
			
					//经度（单位：度），保留小数点后 8 位有效数字
					split_pos=NMEA_Split_Pos((uint8_t*)&frame_datas[0],2);						
					if(split_pos!=0XFF && frame_datas[split_pos]!=','){
						double temp=0;
						if(NMEA_Str2num((uint8_t*)&frame_datas[split_pos],temp))
							pack.lon = temp;
					}
					
					//纬度（单位：度），保留小数点后 8 位有效数字
					split_pos=NMEA_Split_Pos((uint8_t*)&frame_datas[0],3);						
					if(split_pos!=0XFF && frame_datas[split_pos]!=','){
						double temp=0;
						if(NMEA_Str2num((uint8_t*)&frame_datas[split_pos],temp))
							pack.lat = temp;
					}

					//海拔高（单位：米），保留小数点后 4 位有效数字
					split_pos=NMEA_Split_Pos((uint8_t*)&frame_datas[0],4);						
					if(split_pos!=0XFF && frame_datas[split_pos]!=','){
						double temp=0;
						if(NMEA_Str2num((uint8_t*)&frame_datas[split_pos],temp))						
							pack.height = temp;					
					}					
				
					//双天线定向方位角
					split_pos=NMEA_Split_Pos((uint8_t*)&frame_datas[0],5);						
					if(split_pos!=0XFF && frame_datas[split_pos]!=','){
						double temp=0;
						if(NMEA_Str2num((uint8_t*)&frame_datas[split_pos],temp))						
							pack.heading = temp;					
					}					
					
					//GNSS 定位质量指示符,0 = 定位不可用或无效,1 = 单点定位,2 = RTK 浮点解,3 = RTK 固定解
					split_pos=NMEA_Split_Pos((uint8_t*)&frame_datas[0],10);						
					if(split_pos!=0XFF && frame_datas[split_pos]!=','){
						double temp=0;
						if(NMEA_Str2num((uint8_t*)&frame_datas[split_pos],temp))											
							pack.posQuality = temp;					
					}	
					
					//GNSS 定向质量指示符, 0 = 定位不可用或无效,1 = 单点定位,2 = RTK 浮点解,3 = RTK 固定解
					split_pos=NMEA_Split_Pos((uint8_t*)&frame_datas[0],11);						
					if(split_pos!=0XFF && frame_datas[split_pos]!=','){
						double temp=0;
						if(NMEA_Str2num((uint8_t*)&frame_datas[split_pos],temp))								
							pack.headingQuality = temp;		
					}							

					//从天线当前参与解算的卫星数量
					split_pos=NMEA_Split_Pos((uint8_t*)&frame_datas[0],12);						
					if(split_pos!=0XFF && frame_datas[split_pos]!=','){
						double temp=0;
						if(NMEA_Str2num((uint8_t*)&frame_datas[split_pos],temp))								
							pack.numSvANT2 = temp;					
					}						

					//主天线当前参与解算的卫星数量
					split_pos=NMEA_Split_Pos((uint8_t*)&frame_datas[0],13);						
					if(split_pos!=0XFF && frame_datas[split_pos]!=','){
						double temp=0;
						if(NMEA_Str2num((uint8_t*)&frame_datas[split_pos],temp))							
							pack.numSvANT1 = temp;							
					}		
					
					//东向速度，小数点后 3 位，单位：Km/h(如无为空)
					split_pos=NMEA_Split_Pos((uint8_t*)&frame_datas[0],17);						
					if(split_pos!=0XFF && frame_datas[split_pos]!=','){
						double temp=0;
						if(NMEA_Str2num((uint8_t*)&frame_datas[split_pos],temp))							
							pack.VelE = temp;		
					}		
					
					//北向速度，小数点后 3 位，单位：Km/h(如无为空)
					split_pos=NMEA_Split_Pos((uint8_t*)&frame_datas[0],18);						
					if(split_pos!=0XFF && frame_datas[split_pos]!=','){
						double temp=0;
						if(NMEA_Str2num((uint8_t*)&frame_datas[split_pos],temp))						
							pack.VelN = temp;							
					}							
					
					//天向速度，小数点后 3 位，单位：Km/h(如无为空)
					split_pos=NMEA_Split_Pos((uint8_t*)&frame_datas[0],19);						
					if(split_pos!=0XFF && frame_datas[split_pos]!=','){
						double temp=0;
						if(NMEA_Str2num((uint8_t*)&frame_datas[split_pos],temp))						
							pack.VelU = temp;					
					}					
				
					uint8_t gps_fix = 0;
					if(pack.posQuality==1)
					{// 单点定位
						uint8_t mode = pack.mode & 0x0f;
						if( mode == 1 )    // 不可用
							gps_fix = 1;
						else if( mode==2 ) // 2D
							gps_fix = 2;
						else if( mode==3 ) // 3D
							gps_fix = 3; 
					}else if(pack.posQuality==2)//RTK 浮点解
						gps_fix = 5;	
					else if(pack.posQuality==3)// RTK 固定解
						gps_fix = 6;
					
					if( ( gps_fix!=0 ) && ( (pack.numSvANT1+pack.numSvANT2) >= 7) )
					{
						if( gps_available == false )
						{
							if( pack.hdop < 2.5 )
							{
								if( GPS_stable_start_time.is_valid() == false )
									GPS_stable_start_time = TIME::now();
								else if( GPS_stable_start_time.get_pass_time() > 3.0f )
								{
									gps_available = true;
									GPS_stable_start_time.set_invalid();
								}
							}
							else
								GPS_stable_start_time.set_invalid();
						}
						else
						{
							if( pack.hdop > 3.5 )
								gps_available = z_available = false;
						}
					}
					else
					{
						gps_available = z_available = false;
						GPS_stable_start_time.set_invalid();
					}	
					addition_inf[0] = pack.numSvANT1+pack.numSvANT2;
					addition_inf[1] = gps_fix;
					addition_inf[4] = pack.hdop*100;
					addition_inf[5] = pack.vdop*100;
					addition_inf[6] = 0;
					
					if( z_available )
					{
						if( pack.vdop > 4.5 )
							z_available = false;
					}
					else
					{
						if( pack.vdop < 2.5 )
						{
							gps_alt = pack.height * 1e2;
							z_available = true;
						}
					}
					double t = gps_update_TIME.get_pass_time_st();
					if( t > 1 )
						t = 1;	

					vector3<double> velocity;
					velocity.y = pack.VelN*100/3.6;	//North
					velocity.x = pack.VelE*100/3.6;	//East
					velocity.z = pack.VelU*100/3.6;	  //Up
					gps_alt += velocity.z*t;
					if( pack.vdop < 6 )
					{	//高精度高度位置结果
						//使用绝对高度
						double r_height = pack.height*100;
						if( alt_offset <= -100000 )
							alt_offset = r_height - gps_alt;
						gps_alt += 0.5*t * ( r_height - gps_alt - alt_offset );
					}
					else
						alt_offset = -1000000;
					
					vector3<double> position_Global;
					position_Global.x = pack.lat;
					position_Global.y = pack.lon;
					position_Global.z = gps_alt;

					if( z_available )
						PositionSensorChangeDataType( default_rtk_sensor_index,sensor_key, Position_Sensor_DataType_sv_xyz );
					else
						PositionSensorChangeDataType( default_rtk_sensor_index,sensor_key, Position_Sensor_DataType_sv_xy );
					
					//信任度
					double xy_trustD = pack.hdop*100;
					double z_trustD = pack.vdop*100;
					PositionSensorUpdatePositionGlobalVel( default_rtk_sensor_index,sensor_key, position_Global, velocity, gps_available, -1, xy_trustD, z_trustD, addition_inf );
				}
				else if( gps_state.frame_name == FrameGGA )
				{//不做处理，请勿关闭此pack输出

					last_update_time = TIME::now();
				}
				else if( gps_state.frame_name == FrameGSA )
				{ 
					uint8_t point_pos;			 
					uint8_t split_pos; 					
					//模式：1= 固定或不可用；2=2D；3 = 3D 
					split_pos=NMEA_Split_Pos((uint8_t*)&frame_datas[0],2);						
					if(split_pos!=0XFF && frame_datas[split_pos]!=','){
						double temp=0;
						if(NMEA_Str2num((uint8_t*)&frame_datas[split_pos],temp))
							pack.mode = temp;
					}					
					
					//平面精度因子
					split_pos=NMEA_Split_Pos((uint8_t*)&frame_datas[0],16);						
					if(split_pos!=0XFF && frame_datas[split_pos]!=','){
						double temp=0;
						if(NMEA_Str2num((uint8_t*)&frame_datas[split_pos],temp))
							pack.hdop = temp;
					}							
					
					//高程精度因子
					split_pos=NMEA_Split_Pos((uint8_t*)&frame_datas[0],17);						
					if(split_pos!=0XFF && frame_datas[split_pos]!=','){
						double temp=0;
						if(NMEA_Str2num((uint8_t*)&frame_datas[split_pos],temp))
							pack.vdop = temp;
					}	
					
					last_update_time = TIME::now();
				}
				else if(gps_state.frame_name == FrameEVENTMARK)
				{
					debug_test[20]++;
				}
			}
			if( last_update_time.get_pass_time() > 2 )
			{	//接收不到数据
				PositionSensorUnRegister( default_rtk_sensor_index,sensor_key );
				DAOSensorUnRegister(0);
				//关闭Rtk注入
				RtkPort_setEna( rtk_port_ind, false );
				goto GPS_CheckBaud;
			}	
		}
		else
		{	//接收不到数据
			PositionSensorUnRegister( default_rtk_sensor_index,sensor_key );
			DAOSensorUnRegister(0);
			//关闭Rtk注入
			RtkPort_setEna( rtk_port_ind, false );
			goto GPS_CheckBaud;
		}	
	}		
}

static bool RTK_DriverInit( Port port, uint32_t param )
{
	DriverInfo* driver_info = new DriverInfo;
	driver_info->param = param;
	driver_info->port = port;
	xTaskCreate( RTK_Server, "RTK3", 1024, driver_info, SysPriority_ExtSensor, NULL);
	return true;
}

void init_drv_RTK_UM982()
{
	PortFunc_Register( 17, RTK_DriverInit );
}
