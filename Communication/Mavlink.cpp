#include "mavlink.h"
#include "FreeRTOS.h"
#include "semphr.h"

static SemaphoreHandle_t m_mavlink_TxMutex[MAVLINK_COMM_NUM_BUFFERS] = {0};

mavlink_status_t m_mavlink_status[MAVLINK_COMM_NUM_BUFFERS];

mavlink_message_t m_mavlink_buffer[MAVLINK_COMM_NUM_BUFFERS];

extern "C"
{
	void mavlink_init_chan( uint8_t chan )
	{
		if( m_mavlink_TxMutex[chan] == 0 )
			m_mavlink_TxMutex[chan] = xSemaphoreCreateRecursiveMutex();
	}
	bool mavlink_lock_chan( uint8_t chan, double TIMEOUT )
	{
		if( m_mavlink_TxMutex[chan] == 0 )
			return false;
		TickType_t TIMEOUT_Ticks;
		if( TIMEOUT >= 0 )
			TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
		else
			TIMEOUT_Ticks = portMAX_DELAY;
		if( xSemaphoreTakeRecursive( m_mavlink_TxMutex[chan], TIMEOUT_Ticks ) )
			return true;
		return false;
	}
	void mavlink_unlock_chan( uint8_t chan )
	{
		xSemaphoreGiveRecursive( m_mavlink_TxMutex[chan] );
	}
}

//uint16_t mavlink_msg_to_send_buffer(
//							uint16_t (*write_port)( const uint8_t* data, uint16_t length, double Sync_waitTime, double Write_waitTime ) ,
//							bool (*lock)( double Sync_waitTime ) ,
//							void (*unlock)() ,
//							const mavlink_message_t *msg ,
//							double SyncTIMEOUT, double TIMEOUT)
//{
//	if( write_port==0 || lock==0 || unlock==0 )
//		return 0;
//	
//	uint8_t signature_len, header_len;
//	uint16_t length = msg->len;

//	uint8_t* buf;
//	if( lock(SyncTIMEOUT) )
//	{
//		if (msg->magic == MAVLINK_STX_MAVLINK1) {
//			signature_len = 0;
//			header_len = MAVLINK_CORE_HEADER_MAVLINK1_LEN;
//			buf = new uint8_t[header_len + 1 + 2 + (uint16_t)length + (uint16_t)signature_len];
//			buf[0] = msg->magic;
//			buf[1] = length;
//			buf[2] = msg->seq;
//			buf[3] = msg->sysid;
//			buf[4] = msg->compid;
//			buf[5] = msg->msgid & 0xFF;
//			memcpy( &buf[6], (uint8_t*)_MAV_PAYLOAD(msg), length );
//		} else {
//			length = _mav_trim_payload(_MAV_PAYLOAD(msg), length);
//			header_len = MAVLINK_CORE_HEADER_LEN;
//			signature_len = (msg->incompat_flags & MAVLINK_IFLAG_SIGNED)?MAVLINK_SIGNATURE_BLOCK_LEN:0;
//			buf = new uint8_t[header_len + 1 + 2 + (uint16_t)length + (uint16_t)signature_len];
//			buf[0] = msg->magic;
//			buf[1] = length;
//			buf[2] = msg->incompat_flags;
//			buf[3] = msg->compat_flags;
//			buf[4] = msg->seq;
//			buf[5] = msg->sysid;
//			buf[6] = msg->compid;
//			buf[7] = msg->msgid & 0xFF;
//			buf[8] = (msg->msgid >> 8) & 0xFF;
//			buf[9] = (msg->msgid >> 16) & 0xFF;
//			memcpy( &buf[10], (uint8_t*)_MAV_PAYLOAD(msg), length );
//		}
//		buf[1+header_len+length + 0] = (uint8_t)(msg->checksum & 0xFF);
//		buf[1+header_len+length + 1] = (uint8_t)(msg->checksum >> 8);
//		if (signature_len > 0) {
//			memcpy(&buf[1+header_len+length + 2], msg->signature, signature_len);
//		}
//		uint16_t written_bytes = write_port( buf , 1+header_len+length+2+signature_len, SyncTIMEOUT, TIMEOUT );
//		delete[] buf;
//		unlock();
//		
//		return written_bytes;
//	}
//	else
//		return 0;
//}