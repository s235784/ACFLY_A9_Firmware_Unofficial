#pragma once

#include "Modes.hpp"

class M35_Auto1:public Mode_Base 
{
	private:
		
	public:
		M35_Auto1();
		virtual ModeResult main_func( void* param1, uint32_t param2 );
		virtual void get_MavlinkMode( ModeFuncCfg cfg, Receiver rc, uint8_t btn_zones[4], AFunc* mode ); // nuotian
};


	bool get_m35_is_takingoff( bool* result, double TIMEOUT = -1); // nuotian
	bool get_m35_is_landing( bool* result, double TIMEOUT = -1);
	bool get_m35_offboard_lasting( unsigned int* result);
	bool get_m35_debug_mode( int* result);
	bool get_m35_debug_msg( int* result1, int* result2, float* result3);