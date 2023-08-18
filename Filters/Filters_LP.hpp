#pragma once

#include "AC_Math.hpp"

inline bool BUT_IIR_calc_freq( double k[3] , double sample_freq, const double cutoff_freq , double cp )
{	
	if ( (cutoff_freq <= 0.0001) || (sample_freq <= 1.99 * cutoff_freq) ) {
			// no filtering
		return false;
	}
	double cos_PI_cp = cos( Pi * cp );
	
	double fr = sample_freq / cutoff_freq;
	double ohm = tan(Pi/fr);
	double ohm2 = ohm*ohm;
	double c = 1.0+2.0*cos_PI_cp*ohm + ohm2;
	double inv_c = 1.0 / c;
	k[0] = ohm2 * inv_c;
	k[1] = 2.0*(ohm2-1.0) * inv_c;
	k[2] = (1.0-2.0*cos_PI_cp*ohm+ohm2) * inv_c;
	
	return true;
}

/*一阶低通*/
	class Filter_LP_IIR_1
	{
		private:
			double k;
			double out;
		
		public:
			inline void set_cutoff_frequency( const double sample_freq, const double cutoff_freq )
			{
				if( sample_freq < 0.001f || cutoff_freq < 0.001f )
					return;
				this->k = 2 * Pi * cutoff_freq / sample_freq;
			}
			inline void set_value( const double value )
			{
				this->out = value;
			}
		
			inline Filter_LP_IIR_1()
			{
				this->k = 0;
				this->set_value( 0 );
			}		
			inline Filter_LP_IIR_1( double sample_freq , double cutoff_freq )
			{
				set_cutoff_frequency( sample_freq , cutoff_freq );
				this->set_value( 0 );
			}
			inline Filter_LP_IIR_1( double sample_freq , double cutoff_freq , double initial_offset ):Filter_LP_IIR_1( sample_freq , cutoff_freq )
			{
				this->set_value ( initial_offset );
			}
			
			inline void reset( const double value )
			{
				this->out = value;
			}
			
			inline double run( double new_data )
			{
				out += k * ( new_data - out );
				return out;
			}
			
			inline double get_result() {
				return this->out;
			}
			
			//添加偏移
			inline double add_offset( double offset )
			{
				this->out += offset;
				return this->out;
			}
	};
/*一阶低通*/

/*二阶ButterWorth低通*/
	class Filter_Butter2_LP
	{
		private:
			bool available;	
		
			double k_1[3];
		
			double in_1[2];
			double out_1[2];
		public:
			//滤波器可用状态
			inline bool is_available()
			{
				return this->available;
			}
			inline void set_inavailable()
			{
				this->available = false;
			}
			
			//设置滤波频率
			inline bool set_cutoff_frequency( double FS , double FC )
			{
				this->available = BUT_IIR_calc_freq( this->k_1 , FS , FC , 1.0 / 4 );
				return this->available;
			}
			inline void set_cutoff_frequency_from( const Filter_Butter2_LP& filter )
			{
				this->k_1[0] = filter.k_1[0];	this->k_1[1] = filter.k_1[1];	this->k_1[2] = filter.k_1[2];
				this->available = filter.available;		
			}
			
			//设置初始值
			inline void reset( double initial_value )
			{
				this->in_1[0] = this->in_1[1] = initial_value;
				this->out_1[0] = this->out_1[1] = initial_value;
			}
			
			//构造函数
			Filter_Butter2_LP()
			{
				this->available = false;
				this->reset(0);
			}
			Filter_Butter2_LP( double FS , double FC )
			{
				set_cutoff_frequency( FS , FC );
				this->reset(0);
			}
			
			//滤波
			inline double get_result(){return this->out_1[0];}
			inline double run( double newdata )
			{
				if( this->available )
				{
					double out_1_2 = this->out_1[1];	this->out_1[1] = this->out_1[0];
					this->out_1[0] = this->k_1[0]*( newdata + 2*this->in_1[0] + this->in_1[1] ) - this->k_1[1]*this->out_1[1] - this->k_1[2]*out_1_2;			
					this->in_1[1] = this->in_1[0];	this->in_1[0] = newdata;
				}
				else
					reset( newdata );
				return this->out_1[0];
			}
			
			//添加偏移
			inline double add_offset( double offset )
			{
				this->in_1[0] += offset;	this->in_1[1] += offset;
				this->out_1[0] += offset;	this->out_1[1] += offset;				
				return this->out_1[0];
			}
	};
/*二阶ButterWorth低通*/

/*四阶ButterWorth低通*/
	class Filter_Butter4_LP
	{
		private:
			bool available;	
		
			double k_1[3];
			double k_2[3];
		
			double in_1[2];
			double out_1[2];
			double in_2[2];
			double out_2[2];
		
		public:
			//滤波器可用状态
			inline bool is_available()
			{
				return this->available;
			}
			inline void set_inavailable()
			{
				this->available = false;
			}
			
			//设置滤波频率
			inline bool set_cutoff_frequency( double FS , double FC )
			{
				if( BUT_IIR_calc_freq( this->k_1 , FS , FC , 1.0 / 8 ) )
				{
					BUT_IIR_calc_freq( this->k_2 , FS , FC , 3.0 / 8 );
					this->available = true;
					return true;
				}
				else
				{
					this->available = false;
					return false;
				}
			}
			inline void set_cutoff_frequency_from( const Filter_Butter4_LP& filter )
			{
				this->k_1[0] = filter.k_1[0];	this->k_1[1] = filter.k_1[1];	this->k_1[2] = filter.k_1[2];
				this->k_2[0] = filter.k_2[0];	this->k_2[1] = filter.k_2[1];	this->k_2[2] = filter.k_2[2];
				this->available = filter.available;		
			}
			
			//设置初始值
			inline void reset( double initial_value )
			{
				this->in_1[0] = this->in_1[1] = initial_value;
				this->out_1[0] = this->out_1[1] = initial_value;
				
				this->in_2[0] = this->in_2[1] = initial_value;
				this->out_2[0] = this->out_2[1] = initial_value;
			}
			
			//构造函数
			Filter_Butter4_LP()
			{
				this->available = false;
				this->reset(0);
			}
			Filter_Butter4_LP( double FS , double FC )
			{
				set_cutoff_frequency( FS , FC );
				this->reset(0);
			}
			
			//滤波
			inline double get_result(){return this->out_2[0];}
			inline double run( double newdata )
			{
				if( this->available )
				{
					double out_1_2 = this->out_1[1];	this->out_1[1] = this->out_1[0];
					this->out_1[0] = this->k_1[0]*( newdata + 2*this->in_1[0] + this->in_1[1] ) - this->k_1[1]*this->out_1[1] - this->k_1[2]*out_1_2;			
					this->in_1[1] = this->in_1[0];	this->in_1[0] = newdata;
					
					newdata = this->out_1[0];
					double out_2_2 = this->out_2[1];	this->out_2[1] = this->out_2[0];
					this->out_2[0] = this->k_2[0]*( newdata + 2*this->in_2[0] + this->in_2[1] ) - this->k_2[1]*this->out_2[1] - this->k_2[2]*out_2_2;			
					this->in_2[1] = this->in_2[0];	this->in_2[0] = newdata;
				}
				else
					this->reset(newdata);
				return this->out_2[0];
			}
			
			//添加偏移
			inline double add_offset( double offset )
			{
				this->in_1[0] += offset;	this->in_1[1] += offset;
				this->out_1[0] += offset;	this->out_1[1] += offset;

				this->in_2[0] += offset;	this->in_2[1] += offset;
				this->out_2[0] += offset;	this->out_2[1] += offset;
				
				return this->out_2[0];
			}
	};
/*四阶ButterWorth低通*/

/*八阶ButterWorth低通*/
	class Filter_Butter8_LP
	{
		private:
			bool available;	
		
			double k_1[3];
			double k_2[3];
			double k_3[3];
			double k_4[3];
		
			double in_1[2];
			double out_1[2];
			double in_2[2];
			double out_2[2];
			double in_3[2];
			double out_3[2];
			double in_4[2];
			double out_4[2];
		
		public:
			//滤波器可用状态
			inline bool is_available()
			{
				return this->available;
			}
			inline void set_inavailable()
			{
				this->available = false;
			}
		
			//设置滤波频率
			inline void set_cutoff_frequency_from( const Filter_Butter8_LP& filter )
			{
				this->k_1[0] = filter.k_1[0];	this->k_1[1] = filter.k_1[1];	this->k_1[2] = filter.k_1[2];
				this->k_2[0] = filter.k_2[0];	this->k_2[1] = filter.k_2[1];	this->k_2[2] = filter.k_2[2];
				this->k_3[0] = filter.k_3[0];	this->k_3[1] = filter.k_3[1];	this->k_3[2] = filter.k_3[2];
				this->k_4[0] = filter.k_4[0];	this->k_4[1] = filter.k_4[1];	this->k_4[2] = filter.k_4[2];
				this->available = filter.available;		
			}
			inline bool set_cutoff_frequency( double FS , double FC )
			{
				if( BUT_IIR_calc_freq( this->k_1 , FS , FC , 1.0f / 16 ) )
				{
					BUT_IIR_calc_freq( this->k_2 , FS , FC , 3.0f / 16 );
					BUT_IIR_calc_freq( this->k_3 , FS , FC , 5.0f / 16 );
					BUT_IIR_calc_freq( this->k_4 , FS , FC , 7.0f / 16 );
					this->available = true;
					return true;
				}
				else
				{
					this->available = false;
					return false;
				}
			}
			
			//设置初始值
			inline void reset( double initial_value )
			{
				this->in_1[0] = this->in_1[1] = initial_value;
				this->out_1[0] = this->out_1[1] = initial_value;
				
				this->in_2[0] = this->in_2[1] = initial_value;
				this->out_2[0] = this->out_2[1] = initial_value;
				
				this->in_3[0] = this->in_3[1] = initial_value;
				this->out_3[0] = this->out_3[1] = initial_value;
				
				this->in_4[0] = this->in_4[1] = initial_value;
				this->out_4[0] = this->out_4[1] = initial_value;
			}
			
			//构造函数
			Filter_Butter8_LP()
			{
				this->available = false;
				this->reset(0);
			}
			Filter_Butter8_LP( double FS , double FC )
			{
				set_cutoff_frequency( FS , FC );
				this->reset(0);
			}
			
			//滤波
			inline double get_result(){return this->out_4[0];}
			inline double run( double newdata )
			{
				if( this->available )
				{
					double out_1_2 = this->out_1[1];	this->out_1[1] = this->out_1[0];
					this->out_1[0] = this->k_1[0]*( newdata + 2*this->in_1[0] + this->in_1[1] ) - this->k_1[1]*this->out_1[1] - this->k_1[2]*out_1_2;			
					this->in_1[1] = this->in_1[0];	this->in_1[0] = newdata;
					
					newdata = this->out_1[0];
					double out_2_2 = this->out_2[1];	this->out_2[1] = this->out_2[0];
					this->out_2[0] = this->k_2[0]*( newdata + 2*this->in_2[0] + this->in_2[1] ) - this->k_2[1]*this->out_2[1] - this->k_2[2]*out_2_2;			
					this->in_2[1] = this->in_2[0];	this->in_2[0] = newdata;
					
					newdata = this->out_2[0];
					double out_3_2 = this->out_3[1];	this->out_3[1] = this->out_3[0];
					this->out_3[0] = this->k_3[0]*( newdata + 2*this->in_3[0] + this->in_3[1] ) - this->k_3[1]*this->out_3[1] - this->k_3[2]*out_3_2;			
					this->in_3[1] = this->in_3[0];	this->in_3[0] = newdata;
					
					newdata = this->out_3[0];
					double out_4_2 = this->out_4[1];	this->out_4[1] = this->out_4[0];
					this->out_4[0] = this->k_4[0]*( newdata + 2*this->in_4[0] + this->in_4[1] ) - this->k_4[1]*this->out_4[1] - this->k_4[2]*out_4_2;			
					this->in_4[1] = this->in_4[0];	this->in_4[0] = newdata;
				}
				else
					reset( newdata );
				return this->out_4[0];
			}
			
			//添加偏移
			inline double add_offset( double offset )
			{
				this->in_1[0] += offset;	this->in_1[1] += offset;
				this->out_1[0] += offset;	this->out_1[1] += offset;

				this->in_2[0] += offset;	this->in_2[1] += offset;
				this->out_2[0] += offset;	this->out_2[1] += offset;
				
				this->in_3[0] += offset;	this->in_3[1] += offset;
				this->out_3[0] += offset;	this->out_3[1] += offset;

				this->in_4[0] += offset;	this->in_4[1] += offset;
				this->out_4[0] += offset;	this->out_4[1] += offset;
				
				return this->out_4[0];
			}
	};
/*八阶ButterWorth低通*/
	
/*可变阶数ButterWorth低通(偶数阶 最高8阶)*/
	class Filter_Butter_LP
	{
		private:
			bool available;	
		
			uint8_t order;
			double k[4][3];
			
			double in[4][2];
			double out[4][2];
		
		public:
			//滤波器可用状态
			inline bool is_available()
			{
				return this->available;
			}
			inline void set_inavailable()
			{
				this->available = false;
			}
		
			//设置滤波频率
			inline void set_cutoff_frequency_from( const Filter_Butter_LP& filter )
			{
				this->order = filter.order;
				this->k[0][0] = filter.k[0][0];	this->k[0][1] = filter.k[0][1];	this->k[0][2] = filter.k[0][2];
				this->k[1][0] = filter.k[1][0];	this->k[1][1] = filter.k[1][1];	this->k[1][2] = filter.k[1][2];
				this->k[2][0] = filter.k[2][0];	this->k[2][1] = filter.k[2][1];	this->k[2][2] = filter.k[2][2];
				this->k[3][0] = filter.k[3][0];	this->k[3][1] = filter.k[3][1];	this->k[3][2] = filter.k[3][2];
				this->available = filter.available;		
			}
			inline bool set_cutoff_frequency( uint8_t order, double FS , double FC )
			{
				if( order!=2 && order!=4 && order!=6 && order!=8 )
				{
					this->available = false;
					return false;
				}
				
				uint8_t n = order*2;
				if( BUT_IIR_calc_freq( this->k[0] , FS , FC , 1.0f / n ) )
				{
					this->order = order;
					for( uint8_t i=1; i<order/2; ++i ) {
						BUT_IIR_calc_freq( this->k[i] , FS , FC , (1.0f+i*2) / n );
					}
					this->available = true;
					return true;
				}
				else
				{
					this->available = false;
					return false;
				}
			}
			
			//设置初始值
			inline void reset( double initial_value )
			{
				uint8_t n = order/2;
				for( uint8_t i=0; i<n; ++i ) {
					this->in[i][0] = this->in[i][1] = initial_value;
					this->out[i][0] = this->out[i][1] = initial_value;
				}
			}
			
			//构造函数
			Filter_Butter_LP()
			{
				this->order = 2;
				this->available = false;
				this->reset(0);
			}
			Filter_Butter_LP( uint8_t order, double FS , double FC )
			{
				this->order = 2;
				set_cutoff_frequency( order, FS , FC );
				this->reset(0);
			}
			
			//滤波
			inline double get_result(){return this->out[order/2-1][0];}
			inline double run( double newdata )
			{
				uint8_t n = order/2;
				if( this->available )
				{
					for( uint8_t i=0; i<n; ++i ) {
						double out = this->out[i][1];	this->out[i][1] = this->out[i][0];
						this->out[i][0] = this->k[i][0]*( newdata + 2*this->in[i][0] + this->in[i][1] ) - this->k[i][1]*this->out[i][1] - this->k[i][2]*out;			
						this->in[i][1] = this->in[i][0];	this->in[i][0] = newdata;
						newdata = this->out[i][0];
					}
				}
				else
					reset( newdata );
				return newdata;
			}
			
			//添加偏移
			inline double add_offset( double offset )
			{
				uint8_t n = order/2;
				for( uint8_t i=0; i<n; ++i ) {
					this->in[i][0] += offset;	this->in[i][1] += offset;
					this->out[i][0] += offset;	this->out[i][1] += offset;
				}
				return this->out[n-1][0];
			}
	};
/*可变阶数ButterWorth低通(最高8阶)*/
