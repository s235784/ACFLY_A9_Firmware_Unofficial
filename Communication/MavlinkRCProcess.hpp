#pragma once

#include "mavlink.h"

//Mavlink消息处理函数表
extern void (*const Mavlink_RC_Process[])( uint8_t Port_index , const mavlink_message_t* msg_sd );
//Mavlink消息处理函数个数
extern const uint16_t Mavlink_RC_Process_Count;

//参数更新协议任务
void paramProtocolTask();
//ftp协议任务
void ftpProtocolTask();

// nuotian
bool get_RCProcess_debug( int* extra, float* data, int* RCT);
