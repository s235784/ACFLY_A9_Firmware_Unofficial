#include "Basic.hpp"
#include "FreeRTOS.h"
#include "drv_USB.hpp"
#include "drv_Uart1.hpp"
#include "Sensors.hpp"
#include "MeasurementSystem.hpp"
#include "mavlink.h"
#include "Commulink.hpp"
#include "StorageSystem.hpp"
#include "ControlSystem.hpp"
#include "Avoidance.hpp"

#include "M32_PosCtrl.hpp" // nuotian

float debug_test[30];

static void Debug_task(void* pvParameters)
{
	while(1)
	{
		mavlink_message_t msg_sd;
		
		// 向上位机发送Position/Altitude控制模式和LandedState
		for( uint8_t i = 0 ; i < MAVLINK_COMM_NUM_BUFFERS ; ++i ) {
			const Port* portNew = get_CommuPort(i);
			if( portNew==0 || portNew->write==0 ) {
				continue;
			}
			Position_ControlMode pscmode,altmode;
			get_Position_ControlMode(&pscmode);
			get_Altitude_ControlMode(&altmode);
			
			bool inFlight;
			get_is_inFlight(&inFlight);
			uint8_t landed_state;
			bool takingoff;
			get_m32_is_takingoff(&takingoff);
			bool landing;
			get_m32_is_landing(&landing);	
			if(takingoff)landed_state = MAV_LANDED_STATE_TAKEOFF;
			else if(landing)landed_state = MAV_LANDED_STATE_LANDING;
			else if(!inFlight)landed_state = MAV_LANDED_STATE_ON_GROUND;
			else landed_state = MAV_LANDED_STATE_IN_AIR;
			
			mavlink_msg_debug_vect_pack_chan(
				get_CommulinkSysId() ,	//system id
				get_CommulinkCompId() ,	//component id
				i , 	//chan
				&msg_sd ,
				"PSCMODE",
				TIME::get_System_Run_Time() * 1e6 ,	//latitude
				pscmode,
				altmode,
				landed_state
			);
			mavlink_msg_to_send_buffer(portNew->write, 
																 portNew->lock,
																 portNew->unlock,
																 &msg_sd, 0, 0.01);
		}
		os_delay(0.5);
		
		
		uint8_t port_id = 0;
		const Port* port = get_CommuPort(port_id);
		if( port==0 || port->write==0 )
		{
			os_delay(1);
			continue;
		}
		
		vector3<double> tvel(1,0,0);
		double dis;
		get_AvLineDistanceFlu(&dis,tvel,200);
		
//		if( mavlink_lock_chan(1,0.01) )
//		{
//			mavlink_msg_vfr_hud_pack_chan(
//				1 ,	//system id
//				MAV_COMP_ID_AUTOPILOT1 ,	//component id
//				port_id , 	//chan
//				&msg_sd ,
//				0,0,0,0,0,0
//			);
//			mavlink_msg_to_send_buffer(Write_Uart1, 
//																 Lock_Uart1,
//																 Unlock_Uart1,
//																 &msg_sd, 0, -1);
//			mavlink_unlock_chan(1);
//		}
		extern int aaw[16];
		IMU_Sensor imu;
		GetMagnetometer( 2, &imu, 0.1 );
		if( mavlink_lock_chan(port_id,0.01) )
		{
			vector3<double> vec;
			IMU_Sensor sensor;
			Position_Sensor_Data pos;
			get_AccelerationNC_filted(&vec);
			GetPositionSensorData(1,&pos);
			GetMagnetometer( 2, &sensor );
			mavlink_msg_debug_vect_pack_chan( 
				get_CommulinkSysId() ,	//system id
				get_CommulinkCompId() ,	//component id
				port_id , 	//chan
				&msg_sd ,
				"1" ,	//name
				TIME::get_System_Run_Time() * 1e6 , 	//boot ms
				debug_test[0] ,
				debug_test[1] ,
				debug_test[2] );
			mavlink_msg_to_send_buffer(port->write, 
																 port->lock,
																 port->unlock,
																 &msg_sd, 0, -1);
					
			
			Position_Sensor_Data pos2;
			GetPositionSensorData(default_rtk_sensor_index,&pos);
			GetPositionSensorData(internal_baro_sensor_index,&pos2);
			mavlink_msg_debug_vect_pack_chan( 
				get_CommulinkSysId() ,	//system id
				get_CommulinkCompId() ,	//component id
				port_id , 	//chan
				&msg_sd ,
				"2" ,	//name
				TIME::get_System_Run_Time() * 1e6 , 	//boot ms
				get_YawHealthEst() ,
				dis ,
				sensor.data.z );
			mavlink_msg_to_send_buffer(port->write, 
																 port->lock,
																 port->unlock,
																 &msg_sd, 0, -1);
																 
			get_AccelerationENU_Ctrl(&vec);
			mavlink_msg_debug_vect_pack_chan( 
				get_CommulinkSysId() ,	//system id
				get_CommulinkCompId() ,	//component id
				port_id , 	//chan
				&msg_sd ,
				"3" ,	//name
				TIME::get_System_Run_Time() * 1e6 , 	//boot ms
				debug_test[10] ,
				debug_test[11] ,
				debug_test[12] );
			mavlink_msg_to_send_buffer(port->write, 
																 port->lock,
																 port->unlock,
																 &msg_sd, 0, -1);
								
GetPositionSensorData(15,&pos);								
		mavlink_msg_debug_vect_pack_chan( 
				get_CommulinkSysId() ,	//system id
				get_CommulinkCompId() ,	//component id
				port_id , 	//chan
				&msg_sd ,
				"4" ,	//name
				TIME::get_System_Run_Time() * 1e6 , 	//boot ms
				debug_test[8],
				debug_test[9] ,
				pos.position.y*0.01 );
			mavlink_msg_to_send_buffer(port->write, 
																 port->lock,
																 port->unlock,
																 &msg_sd, 0, -1);
//																 
//			GetMagnetometer( 2, &sensor );
//			mavlink_msg_debug_vect_pack_chan( 
//				1 ,	//system id
//				MAV_COMP_ID_AUTOPILOT1 ,	//component id
//				port_id , 	//chan
//				&msg_sd ,
//				"Mag" ,	//name
//				TIME::get_System_Run_Time() * 1e6 , 	//boot ms
//				debug_test[21] ,
//				debug_test[22] ,
//				debug_test[24] );
//			mavlink_msg_to_send_buffer(port->write, 
//																 port->lock,
//																 port->unlock,
//																 &msg_sd, 0, -1);
			mavlink_unlock_chan(port_id);
		}
		
		bool unLocked;
		is_Attitude_Control_Enabled(&unLocked);
		if( unLocked )
		{
			//uint8_t mt_count = get_MainMotorCount();
			double logbuf[8];
			logbuf[0] = TIM4->CCR3;
			logbuf[1] = TIM4->CCR4;
			logbuf[2] = TIM8->CCR1;
			logbuf[3] = TIM8->CCR2;
			logbuf[4] = TIM3->CCR1;
			logbuf[5] = TIM3->CCR2;
			logbuf[6] = TIM15->CCR1;
			logbuf[7] = TIM15->CCR2;
			SDLog_Msg_DebugVect( "dianji", logbuf, 8 );
			
			logbuf[0] = debug_test[0];
			logbuf[1] = debug_test[1];
			logbuf[2] = debug_test[2];
			SDLog_Msg_DebugVect( "lingP", logbuf, 3 );
		}
//		/*姿态*/
//			uint8_t buf[30];
//			uint8_t i = 0;
//			uint8_t checksum = 0;
//			buf[i] = 0xaa;
//			checksum += buf[i++];
//			buf[i] = 0xaa;
//			checksum += buf[i++];
//			buf[i] = 0x01;
//			checksum += buf[i++];
//			buf[i] = 6;
//			checksum += buf[i++];
//			
//			int16_t temp = debug_test[0]*5729.578f;
//			buf[i] = temp >> 8;
//			checksum += buf[i++];
//			buf[i] = temp;
//			checksum += buf[i++];
//			
//			temp = debug_test[1]*5729.578f;
//			buf[i] = temp >> 8;
//			checksum += buf[i++];
//			buf[i] = temp;
//			checksum += buf[i++];
//			
//			temp = debug_test[2]*5729.578f;
//			buf[i] = temp >> 8;
//			checksum += buf[i++];
//			buf[i] = temp;
//			checksum += buf[i++];
//			
//			buf[i++] = checksum;
//			Write_USBD_VCOM( buf , i );
//		/*姿态*/
//		
//		/*其它*/
//			checksum = i = 0;
//			buf[i] = 0xaa;
//			checksum += buf[i++];
//			buf[i] = 0xaa;
//			checksum += buf[i++];
//			buf[i] = 0x02;
//			checksum += buf[i++];
//			buf[i] = 18;
//			checksum += buf[i++];
//			
//			temp = debug_test[4];
//			buf[i] = temp >> 8;
//			checksum += buf[i++];
//			buf[i] = temp;
//			checksum += buf[i++];
//			
//			temp = debug_test[5];
//			buf[i] = temp >> 8;
//			checksum += buf[i++];
//			buf[i] = temp;
//			checksum += buf[i++];
//			
//			temp = debug_test[6];
//			buf[i] = temp >> 8;
//			checksum += buf[i++];
//			buf[i] = temp;
//			checksum += buf[i++];
//			
//			temp = debug_test[7];
//			buf[i] = temp >> 8;
//			checksum += buf[i++];
//			buf[i] = temp;
//			checksum += buf[i++];
//			
//			temp = debug_test[8];
//			buf[i] = temp >> 8;
//			checksum += buf[i++];
//			buf[i] = temp;
//			checksum += buf[i++];
//			
//			temp = debug_test[9];
//			buf[i] = temp >> 8;
//			checksum += buf[i++];
//			buf[i] = temp;
//			checksum += buf[i++];
//			
//			temp = debug_test[10];
//			buf[i] = temp >> 8;
//			checksum += buf[i++];
//			buf[i] = temp;
//			checksum += buf[i++];
//			
//			temp = debug_test[11];
//			buf[i] = temp >> 8;
//			checksum += buf[i++];
//			buf[i] = temp;
//			checksum += buf[i++];
//			
//			temp = debug_test[12];
//			buf[i] = temp >> 8;
//			checksum += buf[i++];
//			buf[i] = temp;
//			checksum += buf[i++];
//			
//			buf[i++] = checksum;
//			Write_USBD_VCOM( buf , i );
//		/*其它*/
		
		os_delay(0.01);
	}
}

void init_Debug()
{
	xTaskCreate( Debug_task , "Debug" ,1000,NULL,SysPriority_UserTask,NULL);
}