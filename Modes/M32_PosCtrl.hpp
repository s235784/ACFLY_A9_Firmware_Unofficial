#pragma once

#include "Modes.hpp"

class M32_PosCtrl:public Mode_Base 
{
	private:
		
	public:
		M32_PosCtrl();
		virtual void get_MavlinkMode( ModeFuncCfg cfg, Receiver rc, 
																	uint8_t btn_zones[4], AFunc* mode );
		virtual ModeResult main_func( void* param1, uint32_t param2 );
};

	bool get_m32_is_takingoff( bool* result, double TIMEOUT = -1); // nuotian
	bool get_m32_is_landing( bool* result, double TIMEOUT = -1);
	bool get_m32_offboard_lasting( unsigned int* result);
	bool get_m32_debug_mode( int* result);
	bool get_m32_debug_msg( int* result1, int* result2, float* result3);