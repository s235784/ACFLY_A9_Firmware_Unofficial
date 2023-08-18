#include "Basic.hpp"
#include "Avoidance.hpp"
#include "ctrl_Main.hpp"
#include "AC_Math.hpp"
#include "vector3.hpp"
#include "MeasurementSystem.hpp"
#include <list>
#include "mavlink.h"
#include "Fences.hpp"
#include "StorageSystem.hpp"

#include "FreeRTOS.h"
#include "semphr.h"

/*Χ��*/
	#define isInclusionFence(x) (x==MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION || x==MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION)
	#define isExclusionFence(x) (x==MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION || x==MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION)
	enum FENCE_RS
	{
		//����
		FENCE_RS_NOFENCE = -1,
		FENCE_RS_FENCEERR = -2,
		FENCE_RS_NOPOSITIONING = -3,
		FENCE_RS_TYPEERR = -4,
		//ʧ�ܽ��
		FENCE_RS_NotInFence = 0,	//����Χ�������ٶȷ���ָ��Χ���ڲ�
		FENCE_RS_NotExFence = 1,	//����Χ���������������ٶȷ���ָ��������
		FENCE_RS_NotInFence_VelInFence = 2,	//����Χ���ڣ����ٶȷ���ָ��Χ���ڲ�(distance=�ٶȷ������Χ���߽����)
		FENCE_RS_NotExFence_VelExFence = 3,	//����Χ�����������⣬���ٶȷ���ָ��������(distance=�ٶȷ������Χ���߽����)
		//�ɹ����
		FENCE_RS_InFence = 10,	//��Χ����(distance=�ٶȷ������Χ���߽����)
		FENCE_RS_ExFence = 11,	//��Χ�����Ʒ�Χ��(distance=�ٶȷ������Χ���߽���� ������ʾ��)
	};
	enum FENCE_TYPE
	{
		FENCE_TYPE_ALL = 3,
		FENCE_TYPE_INCLUSION = 1,
		FENCE_TYPE_EXCLUSION = 2,
	};
	//���㵥һΧ��
	static inline FENCE_RS check_SingleFence(int16_t& fenceInd, FENCE_TYPE fenceType, const vector3<double>& pos, const vector3<double>& targetVel, const double& sinVel, const double& cosVel, double* distance)
	{
		if( fenceInd < 0 )
			return FENCE_RS_NOFENCE;
		
		FenceInf fenceInf;
		if( ReadFence(fenceInd, &fenceInf) )
		{	//��ȡ��Χ����Ϣ
			if( (fenceType & FENCE_TYPE_INCLUSION) && isInclusionFence(fenceInf.cmd) )
				goto CHKFENCE;
			if( (fenceType & FENCE_TYPE_EXCLUSION) && isExclusionFence(fenceInf.cmd) )
				goto CHKFENCE;
			switch(fenceInf.cmd)
			{
				default:
				case MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION:
				case MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION:
				{	//Բ��
					++fenceInd;
					break;
				}
				
				case MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
				case MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:
				{	//�����
					uint16_t vertexs = fenceInf.params[0];
					fenceInd += vertexs;
					break;
				}
			}
			return FENCE_RS_TYPEERR;
CHKFENCE:
			switch(fenceInf.cmd)
			{
				case MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION:
				{	//��Բ��
					++fenceInd;
					//��ȡ����ȫ��λ��������Ϣ
					PosSensorHealthInf2 global_inf;
					if( get_OptimalGlobal_XY( &global_inf ) == false )
					{	//�޾�γ�ȴ�����
						return FENCE_RS_NOPOSITIONING;
					}
					//��ȡָ����γ��ƽ������
					double x, y;
					double offsetX = global_inf.HOffset.x + pos.x;
					double offsetY = global_inf.HOffset.y + pos.y;
					map_projection_project( &global_inf.mp, fenceInf.params[4], fenceInf.params[5], &x, &y );
					x -= offsetX;
					y -= offsetY;
					
					//ͶӰԲ�ĵ��ٶȷ���
					vector2<double> CPos;
					CPos.x = ENU2BodyHeading_x( x, y, sinVel, cosVel );
					CPos.y = ENU2BodyHeading_y( x, y, sinVel, cosVel );
					double r = fenceInf.params[0]*100;
					if( fabs(CPos.y)>=r || fabs(CPos.x)>=r )
					{	//Բ��ȫ��ԭ����
						return FENCE_RS_NotInFence;
					}
					else
					{
						double xC = safe_sqrt( sq(r) - sq(CPos.y) );
						double A = CPos.x - xC;
						double B = CPos.x + xC;
						if( A<0 && B>=0 )
						{	//��Բ��
							if(distance)
								*distance = B;
							return FENCE_RS_InFence;
						}
						else if( A<0 && B<=0 )
						{	//�����㶼��x�Ḻ����
							return FENCE_RS_NotInFence;
						}
						else
						{	//�����㶼��x+
							if(distance)
								*distance = A;
							return FENCE_RS_NotInFence_VelInFence;
						}
					}
					break;
				}
				case MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION:
				{	//��Բ��
					++fenceInd;
					//��ȡ����ȫ��λ��������Ϣ
					PosSensorHealthInf2 global_inf;
					if( get_OptimalGlobal_XY( &global_inf ) == false )
					{	//�޾�γ�ȴ�����
						return FENCE_RS_NOPOSITIONING;
					}
					//��ȡָ����γ��ƽ������
					double x, y;
					double offsetX = global_inf.HOffset.x + pos.x;
					double offsetY = global_inf.HOffset.y + pos.y;
					map_projection_project( &global_inf.mp, fenceInf.params[4], fenceInf.params[5], &x, &y );
					x -= offsetX;
					y -= offsetY;
					
					//ͶӰԲ�ĵ��ٶȷ���
					vector2<double> CPos;
					CPos.x = ENU2BodyHeading_x( x, y, sinVel, cosVel );
					CPos.y = ENU2BodyHeading_y( x, y, sinVel, cosVel );
					double r = fenceInf.params[0]*100;
					if( fabs(CPos.y)>=r || CPos.x<-r )
					{	//Բ��ȫ��ԭ����
						if(distance)
							*distance = -1;
						return FENCE_RS_ExFence;
					}
					else
					{
						double xC = safe_sqrt( sq(r) - sq(CPos.y) );
						double A = CPos.x - xC;
						double B = CPos.x + xC;
						if( A<0 && B>=0 )
						{	//��Բ��
							if(distance)
								*distance = B;
							if( B > -A )
								return FENCE_RS_NotExFence;
							else
								return FENCE_RS_NotExFence_VelExFence;
						}
						else if( A<0 && B<=0 )
						{	//�����㶼��x�Ḻ����
							if(distance)
								*distance = -1;
							return FENCE_RS_ExFence;
						}
						else
						{	//�����㶼��x+
							if(distance)
								*distance = A;
							return FENCE_RS_ExFence;
						}
					}
					break;
				}
				
				case MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
				{	//�ڶ������
					uint16_t vertexs = fenceInf.params[0];
					uint16_t cmd = fenceInf.cmd;
					//��ȡ����ȫ��λ��������Ϣ
					PosSensorHealthInf2 global_inf;
					if( get_OptimalGlobal_XY( &global_inf ) == false )
					{	//�޾�γ�ȴ�����
						fenceInd += vertexs;
						return FENCE_RS_NOPOSITIONING;
					}
					++fenceInd;
					//��ȡָ����γ��ƽ������
					double x, y;
					double offsetX = global_inf.HOffset.x + pos.x;
					double offsetY = global_inf.HOffset.y + pos.y;
					map_projection_project( &global_inf.mp, fenceInf.params[4], fenceInf.params[5], &x, &y );
					x -= offsetX;
					y -= offsetY;
					vector2<double> vertex0;
					vector2<double> last_vertex;
					last_vertex.x = ENU2BodyHeading_x( x, y, sinVel, cosVel );
					last_vertex.y = ENU2BodyHeading_y( x, y, sinVel, cosVel );
					vertex0 = last_vertex;
					//ͳ�ƽ�������ж��Ƿ��ڷ�Χ��
					uint16_t crCount = 0;
					double min_x = -1;
					for( uint8_t i=1; i<vertexs; ++i )
					{
						if( ReadFence(fenceInd, &fenceInf) )
						{	//��ȡ��Χ����Ϣ
							if( fenceInf.cmd != cmd )
								return FENCE_RS_FENCEERR;
							if( fenceInf.params[0] != vertexs )
								return FENCE_RS_FENCEERR;
							++fenceInd;
							//������תͶӰ�����
							map_projection_project( &global_inf.mp, fenceInf.params[4], fenceInf.params[5], &x, &y );
							x -= offsetX;
							y -= offsetY;
							vector2<double> vertex;
							vertex.x = ENU2BodyHeading_x( x, y, sinVel, cosVel );
							vertex.y = ENU2BodyHeading_y( x, y, sinVel, cosVel );
checkFENCE_POLYGON_VERTEX_INCLUSION:
							//�ж��Ƿ�Ϊ����
							if( last_vertex.y*vertex.y <= 0 )
							{	//���ܴ��ڽ���
								double crX = vertex.y - last_vertex.y;
								if( is_zero(crX) )
								{	//ˮƽ��
									if( vertex.x * last_vertex.x < 0 )
									{	//ˮƽ�ߴ���ԭ��
										//ѹΧ���жϲ���Χ����
										fenceInd += vertexs - i - 1;
										return FENCE_RS_NotInFence;
									}
									else
									{	//ˮƽ�߲�����ԭ��
										//��������㶼��x���Ҳ���ѡȡ������С�ĵ�������
										if( vertex.x > 0 )
										{
											++crCount;
											double dis = vertex.x < last_vertex.x ? vertex.x : last_vertex.x;
											//������С����
											if( min_x<0 || dis<min_x )
												min_x = dis;
										}
									}
								}
								else
								{	//��ˮƽ��
									//����x�ύ������
									crX = ( last_vertex.x*vertex.y - vertex.x*last_vertex.y ) / crX;
									if( crX > 0 )
									{	//������x���Ҳ�
										++crCount;
										//������С����
										if( min_x<0 || crX<min_x )
											min_x = crX;
									}
								}
							}
							//����������
							last_vertex = vertex;
							if( i==vertexs-1 && vertexs>=3 )
							{	//���n-0��
								++i;
								vertex = vertex0;
								goto checkFENCE_POLYGON_VERTEX_INCLUSION;
							}
						}
						else
							return FENCE_RS_NOFENCE;
					}
					//�������
					//����������֤����Χ����
					if( crCount & 1 )
					{	//��Χ����
						if( distance )
							*distance = min_x;
						return FENCE_RS_InFence;
					}
					else
					{	//��Χ����
						if( crCount == 0 )
							//�޽���֤���˷�����Χ��
							return FENCE_RS_NotInFence;
						else
						{	//�ٶȷ�����Χ��
							if( distance )
								*distance = min_x;
							return FENCE_RS_NotInFence_VelInFence;
						}
					}
					break;
				}
				case MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:
				{	//�ڶ������
					uint16_t vertexs = fenceInf.params[0];
					uint16_t cmd = fenceInf.cmd;
					//��ȡ����ȫ��λ��������Ϣ
					PosSensorHealthInf2 global_inf;
					if( get_OptimalGlobal_XY( &global_inf ) == false )
					{	//�޾�γ�ȴ�����
						fenceInd += vertexs;
						return FENCE_RS_NOPOSITIONING;
					}
					++fenceInd;
					//��ȡָ����γ��ƽ������
					double x, y;
					double offsetX = global_inf.HOffset.x + pos.x;
					double offsetY = global_inf.HOffset.y + pos.y;
					map_projection_project( &global_inf.mp, fenceInf.params[4], fenceInf.params[5], &x, &y );
					x -= offsetX;
					y -= offsetY;
					vector2<double> vertex0;
					vector2<double> last_vertex;
					last_vertex.x = ENU2BodyHeading_x( x, y, sinVel, cosVel );
					last_vertex.y = ENU2BodyHeading_y( x, y, sinVel, cosVel );
					vertex0 = last_vertex;
					//ͳ�ƽ�������ж��Ƿ��ڷ�Χ��
					uint16_t crCount = 0;
					double min_x = -1;
					double min_Nx = -1;
					for( uint8_t i=1; i<vertexs; ++i )
					{
						if( ReadFence(fenceInd, &fenceInf) )
						{	//��ȡ��Χ����Ϣ
							if( fenceInf.cmd != cmd )
								return FENCE_RS_FENCEERR;
							if( fenceInf.params[0] != vertexs )
								return FENCE_RS_FENCEERR;
							++fenceInd;
							//������תͶӰ�����
							map_projection_project( &global_inf.mp, fenceInf.params[4], fenceInf.params[5], &x, &y );
							x -= offsetX;
							y -= offsetY;
							vector2<double> vertex;
							vertex.x = ENU2BodyHeading_x( x, y, sinVel, cosVel );
							vertex.y = ENU2BodyHeading_y( x, y, sinVel, cosVel );
checkFENCE_POLYGON_VERTEX_EXCLUSION:
							//�ж��Ƿ�Ϊ����
							if( last_vertex.y*vertex.y <= 0 )
							{	//���ܴ��ڽ���
								double crX = vertex.y - last_vertex.y;
								if( is_zero(crX) )
								{	//ˮƽ��
									if( vertex.x * last_vertex.x < 0 )
									{	//ˮƽ�ߴ���ԭ��
										//ѹΧ���жϲ���Χ����
										fenceInd += vertexs - i - 1;
										return FENCE_RS_NotExFence;
									}
									else
									{	//ˮƽ�߲�����ԭ��
										//��������㶼��x���Ҳ���ѡȡ������С�ĵ�������
										if( vertex.x > 0 )
										{	//X+����
											++crCount;
											double dis = vertex.x < last_vertex.x ? vertex.x : last_vertex.x;
											//������С����
											if( min_x<0 || dis<min_x )
												min_x = dis;
										}
										else
										{	//X-����
											double dis = vertex.x > last_vertex.x ? -vertex.x : -last_vertex.x;
											//������С����
											if( min_Nx<0 || dis<min_Nx )
												min_Nx = dis;
										}
									}
								}
								else
								{	//��ˮƽ��
									//����x�ύ������
									crX = ( last_vertex.x*vertex.y - vertex.x*last_vertex.y ) / crX;
									if( crX > 0 )
									{	//������x���Ҳ�
										++crCount;
										//������С����
										if( min_x<0 || crX<min_x )
											min_x = crX;
									}
									else
									{	//X-����
										double dis = -crX;
										if( min_Nx<0 || dis<min_Nx )
											min_Nx = dis;
									}
								}
							}
							//����������
							last_vertex = vertex;
							if( i==vertexs-1 && vertexs>=3 )
							{	//���n-0��
								++i;
								vertex = vertex0;
								goto checkFENCE_POLYGON_VERTEX_EXCLUSION;
							}
						}
						else
							return FENCE_RS_NOFENCE;
					}
					//�������
					//����������֤����Χ����
					if( crCount & 1 )
					{	//�ڽ�������
						if( min_x < min_Nx )
						{	//��ǰ����ɷɳ�������
							if( distance )
								*distance = min_x;
							return FENCE_RS_NotExFence_VelExFence;
						}
						else
							return FENCE_RS_NotExFence;
					}
					else
					{	//��Χ����
						if( crCount == 0 )
						{	//�޽���֤���˷�����Χ��
							if( distance )
								*distance = -1;
							return FENCE_RS_ExFence;
						}
						else
						{	//�ٶȷ�����Χ��
							if( distance )
								*distance = min_x;
							return FENCE_RS_ExFence;
						}
					}
					break;
				}
				
				default:
				{	//δָ֪��
					++fenceInd;
					return FENCE_RS_FENCEERR;
					break;
				}
			}
		}
		else
		{	//��Χ����Ϣ
			return FENCE_RS_NOFENCE;
		}
	}
		
	//����ɻ���ǰ�Ƿ���Χ����
	//true:��Χ����Χ�ڣ�distance����Ϊ����Χ���߽���룬����Ϊ��Χ����Ϣ
	//false:����Χ����Χ�ڣ�distance����ʱ��ʾ�ٶȷ���ָ��ǰΧ���ڲ�������ʱ��ʾָ��ǰΧ���ڲ��뵱ǰΧ���ľ���
	//����Χ����Χ��ʱ��fenceRsΪ-1��ʾ���ڿɷ��������ڣ�-2��ʾ��ǰ�ڽ�������
	bool is_insideFence(const vector3<double>& pos, const vector3<double>& targetVel, double* distance, FRS* fenceRs )
	{
		//��Ԥ����Ϣ
		double sinVel, cosVel;
		double velLen = safe_sqrt( targetVel.get_square() );
		if( is_zero(velLen) )
		{	//�ٶȳ���Ϊ0
			sinVel = 0;
			cosVel = 1;
		}
		else
		{
			double invVelLen = 1.0/velLen;
			sinVel = targetVel.y * invVelLen;
			cosVel = targetVel.x * invVelLen;
		}
		
		//��ȡ��ǰΧ�����
		int16_t currentFence = getCurrentFenceInd();
		//��ǰ��ȡ��Χ�����
		int16_t fenceInd = 0;
		//inclusionΧ������
		uint16_t inclusionFenceCount = 0;
		//��ǰΧ��״̬ -1-��ǰΧ�������� 0-�ڱ߽��� 1-���ڱ߽��� 2-���ڱ߽��ڵ��ٶȷ���ָ��߽���
		int8_t currentFenceState = -1;
		//������Զ��inclusionΧ��
		double max_InFenceDistance = -1;	int16_t max_InFenceDistanceInd = -1;
		//������״̬ -1-�����ڽ����� 0-�ڽ������� 1-�ڽ������ڵ��ٶ�ָ�������� 2-�ڽ�������
		int8_t exFenceState = -1;
		//���������exclusionΧ��
		double min_ExFenceDistance = -1;	double max_NotExFenceDistance = -1;
		//�ٶ�ָ��Χ����ʱ����Χ������
		double NotInFence_VelInFence_distance = -1;
		while(1)
		{
			//��������Χ������״̬
			double dis;
			int16_t currentFenceInd = fenceInd;
			FENCE_RS rs = check_SingleFence( fenceInd, FENCE_TYPE_ALL, pos, targetVel, sinVel, cosVel, &dis );
			if( rs==FENCE_RS_InFence )
			{	//��Χ����
				//���¾���Χ���߽�������
				++inclusionFenceCount;
				if( max_InFenceDistance<0 || dis>max_InFenceDistance )
				{
					max_InFenceDistance = dis;
					max_InFenceDistanceInd = currentFenceInd;
				}
				if( currentFence==currentFenceInd )
				{	//��ǰΧ���ڱ߽���
					currentFenceState = 0;
				}
			}
			else if( rs==FENCE_RS_NotInFence || rs==FENCE_RS_NotInFence_VelInFence )
			{	//����Χ����
				++inclusionFenceCount;
				if( currentFence==currentFenceInd )
				{	//������ǰΧ����Χ
					if( rs==FENCE_RS_NotInFence_VelInFence && velLen>1 )
						currentFenceState = 2;
					else
						currentFenceState = 1;
					NotInFence_VelInFence_distance = dis;
				}
			}
			else if( rs==FENCE_RS_ExFence )
			{	//��������
				if( exFenceState < 0 )
					exFenceState = 0;
				if( min_ExFenceDistance<0 || dis<min_ExFenceDistance )
					min_ExFenceDistance = dis;
			}
			else if( rs==FENCE_RS_NotExFence_VelExFence )
			{	//�������ڵ��ٶ�ָ��������
				if( exFenceState < 1 )
					exFenceState = 1;
				if( max_NotExFenceDistance<0 || dis>max_NotExFenceDistance )
					max_NotExFenceDistance = dis;
			}
			else if( rs==FENCE_RS_NotExFence )
			{	//��������
				if( exFenceState < 2 )
					exFenceState = 2;
				//ֱ���˳�
				if(distance)
					*distance = -1;
				if(fenceRs)
					*fenceRs = FRS_NotOuNFlyZone;
				return false;
			}
			else if( rs == FENCE_RS_NOFENCE )
			{	//��Χ���˳�
				break;
			}
		}
		
		double minDis = -1;
		if( exFenceState == 1 )
		{	//�������ڷ�����߽����
			if(distance)
				*distance = max_NotExFenceDistance;
			if(fenceRs)
				*fenceRs = FRS_NotOuNFlyZone;
			return false;
		}
		else if( exFenceState == 0 )
		{	//��������±߽����
			minDis = min_ExFenceDistance;
		}
		if( inclusionFenceCount > 0 )
		{	//��inclusionΧ��
			if( currentFenceState == 0 )
			{	//���������ڵ�ǰΧ����Χ��
				if( max_InFenceDistanceInd != currentFence )
				{	//������Χ����Ϊ��ǰΧ��
					//���µ�ǰΧ��λ������Χ��
					setCurrentFence(max_InFenceDistanceInd);
				}
				if( minDis<0 || minDis>max_InFenceDistance )
					minDis = max_InFenceDistance;
				if(distance)
					*distance = minDis;
				if(fenceRs)
					*fenceRs = FRS_InFlyZone;
				return true;
			}
			else if( currentFenceState==-1 || currentFenceState==1 )
			{	//��ǰΧ�������ڻ��ڱ߽���
				if( max_InFenceDistanceInd >= 0 )
				{	//�������ڲ���Χ��
					//���µ�ǰΧ��
					setCurrentFence(max_InFenceDistanceInd);
					if( minDis<0 || minDis>max_InFenceDistance )
						minDis = max_InFenceDistance;
					if(distance)
						*distance = minDis;
					if(fenceRs)
						*fenceRs = FRS_InFlyZone;
					return true;
				}
				else
				{	//���������ڲ���Χ��
					if(distance)
						*distance = -1;
					if(fenceRs)
						*fenceRs = FRS_NotInFlyZone;
					return false;
				}
			}
			else if( currentFenceState == 2 )
			{	//���ڵ�ǰΧ���ڵ��ٶȷ���ָ��߽���
				if( max_InFenceDistanceInd >= 0 )
				{	//�������ڲ���Χ��
					//���µ�ǰΧ��
					setCurrentFence(max_InFenceDistanceInd);
					if( minDis<0 || minDis>max_InFenceDistance )
						minDis = max_InFenceDistance;
					if(distance)
						*distance = minDis;
					if(fenceRs)
						*fenceRs = FRS_InFlyZone;
					return true;
				}
				else
				{	//���������ڲ���Χ��
					if(distance)
						*distance = NotInFence_VelInFence_distance;
					if(fenceRs)
						*fenceRs = FRS_NotInFlyZone;
					return false;
				}
			}
		}
		else
		{	//��inclusionΧ��
			if(distance)
				*distance = minDis;
			if( exFenceState < 0 )
			{
				if(fenceRs)
					*fenceRs = FRS_NoFence;
			}
			else
			{
				if(fenceRs)
					*fenceRs = FRS_InFlyZone;
			}
			return true;
		}
		if(distance)
			*distance = -1;
		return false;
	}
/*Χ��*/
	
/*����*/
	/*����ϵͳ������*/
		static SemaphoreHandle_t AvMutex = xSemaphoreCreateRecursiveMutex();
		bool LockAv( double TIMEOUT )
		{
			TickType_t TIMTOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMTOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMTOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTakeRecursive( AvMutex, TIMTOUT_Ticks ) == pdTRUE )
				return true;
			else
				return false;
		}
		void UnlockAv()
		{
			xSemaphoreGiveRecursive(AvMutex);
		}
	/*����ϵͳ������*/

	static AvoidanceTarget AvTargets[max_AvTargets] = {0};
	static uint8_t AvTargetsCount = 0;
	
	uint8_t getAvTargetsCount()
	{
		return AvTargetsCount;
	}
	
	int8_t registere_AvTarget( double TIMEOUT )
	{
		if( LockAv(TIMEOUT) )
		{
			int8_t id = -1;
			for( uint8_t i = 0; i < max_AvTargets; ++i )
			{	//ѡ��λ���
				if( AvTargets[i].registered==false )
				{
					id = i;
					break;
				}
			}
			if( id < 0 )
			{	//�޿�λ
				UnlockAv();
				return -1;
			}
			
			//����Ŀ��
			AvTargets[id].registered = true;
			AvTargets[id].last_update_TIME = TIME::now();
			AvTargets[id].available = false;
			
			++AvTargetsCount;
			
			UnlockAv();
			return id;
		}
		return -1;
	}
	int8_t unregistere_AvTarget( uint8_t id, double TIMEOUT )
	{
		if( id >= max_AvTargets )
			return false;
		if( LockAv(TIMEOUT) )
		{
			if( AvTargets[id].registered == false )
			{	//�������˳�
				UnlockAv();
				return false;
			}
			
			//����Ŀ��
			AvTargets[id].registered = false;
			AvTargets[id].available = false;
			
			--AvTargetsCount;
			
			UnlockAv();
			return id;
		}
		return -1;
	}
	
	bool get_AvTarget( uint8_t id, AvoidanceTarget* resTarget, double TIMEOUT )
	{
		if( id >= max_AvTargets )
			return false;
		
		if( LockAv(TIMEOUT) )
		{
			if( AvTargets[id].registered == false )
			{	//�������˳�
				UnlockAv();
				return false;
			}
			//����Ŀ��
			*resTarget = AvTargets[id];
			
			UnlockAv();
			return true;
		}
		return false;
	}
	
	bool set_AvTargetInavailable( uint8_t id, double TIMEOUT )
	{
		if( id >= max_AvTargets )
			return false;
		
		if( LockAv(TIMEOUT) )
		{
			if( AvTargets[id].registered == false )
			{	//�������˳�
				UnlockAv();
				return false;
			}
			//����Ŀ��
			AvTargets[id].available = false;
			AvTargets[id].last_update_TIME = TIME::now();
			
			UnlockAv();
			return true;
		}
		return false;
	}
	
	/*��ά��Ŀ��*/
		bool set_AvTarget2dPoint_RelativeEnu( uint8_t id, vector2<double> dis, double TIMEOUT )
		{
			if( !isvalid(dis.x) || !isvalid(dis.y) )
				return false;
			if( id >= max_AvTargets )
				return false;
			
			//��ȡ��ǰλ��
			vector3<double> pos;
			get_Position_Ctrl(&pos);
			if( LockAv(TIMEOUT) )
			{
				if( AvTargets[id].registered == false )
				{	//�������˳�
					UnlockAv();
					return false;
				}

				//����Ŀ��
				AvTargets[id].type = AvTargetType_2dPoint;
				AvTargets[id].available = true;
				AvTargets[id].pos.x = pos.x + dis.x;
				AvTargets[id].pos.y = pos.y + dis.y;
				AvTargets[id].pos.z = pos.z;
				AvTargets[id].last_update_TIME = TIME::now();
				
				UnlockAv();
				return true;
			}
			return false;
		}
		bool set_AvTargetPoint2dPoint_RelativeFlu( uint8_t id, vector2<double> dis, double TIMEOUT )
		{
			Quaternion quat;
			get_AirframeY_quat(&quat);
			
			double yaw = quat.getYaw();
			double sin_Yaw, cos_Yaw;
			fast_sin_cos( yaw, &sin_Yaw, &cos_Yaw );
			vector2<double> disENU;
			disENU.x = BodyHeading2ENU_x( dis.x , dis.y , sin_Yaw , cos_Yaw );
			disENU.y = BodyHeading2ENU_y( dis.x , dis.y , sin_Yaw , cos_Yaw );
			
			return set_AvTarget2dPoint_RelativeEnu(id, disENU, TIMEOUT);
		}
	/*��ά��Ŀ��*/
	
	/*��ά��Ŀ��*/
		bool set_AvTarget3dPoint_RelativeEnu( uint8_t id, vector3<double> dis, double TIMEOUT )
		{
			if( !isvalid(dis.x) || !isvalid(dis.y) || !isvalid(dis.z) )
				return false;
			if( id >= max_AvTargets )
				return false;
			
			//��ȡ��ǰλ��
			vector3<double> pos;
			get_Position_Ctrl(&pos);
			if( LockAv(TIMEOUT) )
			{
				if( AvTargets[id].registered == false )
				{	//�������˳�
					UnlockAv();
					return false;
				}

				//����Ŀ��
				AvTargets[id].type = AvTargetType_3dPoint;
				AvTargets[id].available = true;
				AvTargets[id].pos.x = pos.x + dis.x;
				AvTargets[id].pos.y = pos.y + dis.y;
				AvTargets[id].pos.z = pos.z + dis.z;
				AvTargets[id].last_update_TIME = TIME::now();
				
				UnlockAv();
				return true;
			}
			return false;
		}
		bool set_AvTargetPoint3dPoint_RelativeFlu( uint8_t id, vector3<double> dis, double TIMEOUT )
		{
			Quaternion quat;
			get_AirframeY_quat(&quat);
			
			double yaw = quat.getYaw();
			double sin_Yaw, cos_Yaw;
			fast_sin_cos( yaw, &sin_Yaw, &cos_Yaw );
			vector3<double> disENU;
			disENU.x = BodyHeading2ENU_x( dis.x , dis.y , sin_Yaw , cos_Yaw );
			disENU.y = BodyHeading2ENU_y( dis.x , dis.y , sin_Yaw , cos_Yaw );
			disENU.z = dis.z;
			
			return set_AvTarget3dPoint_RelativeEnu(id, disENU, TIMEOUT);
		}
	/*��ά��Ŀ��*/
		
	/*XYǽ��*/
		bool set_AvTargetXYStraightLine_RelativeEnu( uint8_t id, vector2<double> dis, double angle, double TIMEOUT )
		{
			if( !isvalid(dis.x) || !isvalid(dis.y) || !isvalid(angle) )
				return false;
			if( id >= max_AvTargets )
				return false;
			
			//��ȡ��ǰλ��
			vector3<double> pos;
			get_Position_Ctrl(&pos);
			if( LockAv(TIMEOUT) )
			{
				if( AvTargets[id].registered == false )
				{	//�������˳�
					UnlockAv();
					return false;
				}

				//����Ŀ��
				AvTargets[id].type = AvTargetType_XYStraightLine;
				AvTargets[id].available = true;
				AvTargets[id].pos.x = pos.x + dis.x;
				AvTargets[id].pos.y = pos.y + dis.y;
				AvTargets[id].pos.z = angle;
				AvTargets[id].last_update_TIME = TIME::now();
				
				UnlockAv();
				return true;
			}
			return false;
		}
		bool set_AvTargetXYStraightLine_RelativeFlu( uint8_t id, vector2<double> dis, double angle, double TIMEOUT )
		{
			Quaternion quat;
			get_AirframeY_quat(&quat);
			
			double yaw = quat.getYaw();
			double sin_Yaw, cos_Yaw;
			fast_sin_cos( yaw, &sin_Yaw, &cos_Yaw );
			vector2<double> disENU;
			disENU.x = BodyHeading2ENU_x( dis.x , dis.y , sin_Yaw , cos_Yaw );
			disENU.y = BodyHeading2ENU_y( dis.x , dis.y , sin_Yaw , cos_Yaw );
			
			double disAngle = atan2(dis.y,dis.x);
			
			double lineK = yaw + disAngle + Pi*0.5 + angle;
			lineK = Mod( lineK, 2*Pi );
			if(lineK > Pi)
				lineK -= 2*Pi;
			else if(lineK < -Pi)
				lineK += 2*Pi;
			
			return set_AvTargetXYStraightLine_RelativeEnu(id, disENU, lineK, TIMEOUT);
		}
	/*XYǽ��*/
	
	bool get_AvLineDistanceEnu(double* resAvDistance, vector3<double> targetVel, double inRange, const vector3<double>* posOffsets, uint8_t offsetsCount, double TIMEOUT)
	{
		double sqBA = targetVel.get_square();
		if( sqBA < 0.00001 )
			return false;
		
		//Ԥ�ȼ������
		vector3<double> pos;
		get_Position_Ctrl(&pos);
		double m = 1.0 / sqBA;
		
		AvoidanceTarget targets[max_AvTargets];
		if( LockAv(TIMEOUT) )
		{
			memcpy( targets, AvTargets, sizeof(AvoidanceTarget)*max_AvTargets );
			UnlockAv();
		}
		else
			return false;
		
		//��λ���
		vector3<double>* rOffsets;
		if( posOffsets!=0 && offsetsCount>0 )
		{	//����ƫ�Ƽ���Ŀ��
			rOffsets = new vector3<double>[offsetsCount];
			for( uint16_t i=0; i<=offsetsCount; ++i )
			{
				resAvDistance[i] = -1;
				//����ƫ��
				if( i > 0 )
					rOffsets[i-1] = posOffsets[i-1] - targetVel*( (posOffsets[i-1]*targetVel)*m );
			}
		}
		else
			resAvDistance[0] = -1;
		
		for( uint8_t i = 0; i < max_AvTargets; ++i )
		{	//��������Ŀ��
			AvoidanceTarget* target = &targets[i];
			if( target->available )
			{
				switch(target->type)
				{
					case AvTargetType_3dPoint:
					{	//��ά��Ŀ��
						vector3<double> C = target->pos - pos;
						double CdA = C*targetVel;
						if( CdA > 0 ) 
						{ //Ŀ����ǰ��
							//���㴹��
							double k = CdA * m;
							vector3<double> foot_point = targetVel * k;
							
							//ѡȡ��Χ����С�����
							uint16_t i = 0;
							vector3<double> cOffset( 0, 0, 0 );
							do
							{
								if( i > 0 )
									cOffset = rOffsets[i-1];
								double hDis = (foot_point + cOffset - C).get_square();
								if( hDis <= sq(inRange) )
								{
									double rDis = foot_point.get_square();
									rDis = safe_sqrt(rDis);
									if( resAvDistance[i] < 0 )
										resAvDistance[i] = rDis;
									else if( rDis < resAvDistance[i] ) 
										resAvDistance[i] = rDis;
								}
								++i;
							}while( posOffsets && i<=offsetsCount );
						}
						break;
					}
					
					case AvTargetType_2dPoint:
					{	//��ά��Ŀ��
						vector3<double> C = target->pos - pos;
						double CdA = C*targetVel;
						if( CdA > 0 )
						{	//Ŀ����ǰ��
							//��)AΪx��ӳ��C������
							double Atheta = atan2( targetVel.y, targetVel.x );
							double cosTheta, sinTheta;
							fast_sin_cos( Atheta, &sinTheta, &cosTheta );
							
							uint16_t i = 0;
							vector3<double> cOffset( 0, 0, 0 );
							do
							{
								if( i > 0 )
									cOffset = rOffsets[i-1];
								C = target->pos - pos - cOffset;
								double hDis = -C.x*sinTheta + C.y*cosTheta;
								if( fabs(hDis) < inRange )
								{
									double rDis = C.x*cosTheta + C.y*sinTheta;
									if( rDis >= 0 )
									{
										if( resAvDistance[i] < 0 )
											resAvDistance[i] = rDis;
										else if( rDis < resAvDistance[i] ) 
											resAvDistance[i] = rDis;
									}
								}
								++i;
							}while( posOffsets && i<=offsetsCount );
						}
						break;
					}
					
					case AvTargetType_XYStraightLine:
					{	//XY���޳�ֱ��ǽ��
						vector3<double> C = target->pos - pos;
						if( targetVel.get_square() > 0.001 )
						{	//��)AΪx��ӳ��C������
							double Atheta = atan2( targetVel.y, targetVel.x );
							double cosTheta, sinTheta;
							fast_sin_cos( Atheta, &sinTheta, &cosTheta );
							
							uint16_t i = 0;
							vector3<double> cOffset( 0, 0, 0 );
							do
							{
								if( i > 0 )
									cOffset = rOffsets[i-1];
								C = target->pos - pos - cOffset;
								vector2<double> Crotated;
								Crotated.x = C.x*cosTheta + C.y*sinTheta;
								Crotated.y = -C.x*sinTheta + C.y*cosTheta;
								
								//��ͶӰ��Cֱ��б��
								double Ctheta = target->pos.z - Atheta;
								//��ӽ���λ��
								double CthetaTan = tan(Ctheta);
								if( is_zero(CthetaTan) )
								{	//ƽ�������
									//ֻҪˮƽ�����ڷ�Χ�ڼ�����0����
									if( fabs(Crotated.y) < inRange )
									{
										resAvDistance[i] = 0;
									}
								}
								else
								{	//��ƽ����
									//�������ӽ���
									double invTan = 1.0 / CthetaTan;
									double x1 = Crotated.x + (inRange - Crotated.y) * invTan;
									double x2 = Crotated.x + (-inRange - Crotated.y) * invTan;
									double rDis = -1;
									if( x1*x2 < 0 )
									{	//���ӽ�����ԭ������
										//����ӦΪ0
										resAvDistance[i] = 0;
									}
									else if( x1 > 0 )
									{	//�ӽ��㶼��ԭ���Ҳ�
										if( x1 < x2 )
											rDis = x1;
										else
											rDis = x2;
									}
									if( rDis >= 0 )
									{
										if( resAvDistance[i] < 0 )
											resAvDistance[i] = rDis;
										else if( rDis < resAvDistance[i] ) 
											resAvDistance[i] = rDis;
									}
								}
								++i;
							}while( posOffsets && i<=offsetsCount );
						}
						break;
					}
				}
			}
		}
		if( posOffsets!=0 && offsetsCount>0 )
			delete[] rOffsets;
		return true;
	}
	bool get_AvLineDistanceFlu(double* resAvDistance, vector3<double> targetVel, double inRange, const vector3<double>* posOffsets, uint8_t offsetsCount, double TIMEOUT)
	{
		Quaternion quat;
		get_AirframeY_quat(&quat);
		
		double yaw = quat.getYaw();
		double sin_Yaw, cos_Yaw;
		fast_sin_cos( yaw, &sin_Yaw, &cos_Yaw );
		vector3<double> targetVelENU;
		targetVelENU.x = BodyHeading2ENU_x( targetVel.x , targetVel.y , sin_Yaw , cos_Yaw );
		targetVelENU.y = BodyHeading2ENU_y( targetVel.x , targetVel.y , sin_Yaw , cos_Yaw );
		targetVelENU.z = targetVel.z;
		
		vector3<double>* posOffsetsENU = 0;
		if( posOffsets!=0 && offsetsCount>0 )
		{	//����ƫ�Ƽ���Ŀ��
			posOffsetsENU = new vector3<double>[offsetsCount];
			if(!posOffsetsENU)
				return false;
			for( uint16_t i=0; i<offsetsCount; ++i )
			{	//FLUƫ��ת��ΪENU
				posOffsetsENU[i].x = BodyHeading2ENU_x( posOffsets[i].x , posOffsets[i].y , sin_Yaw , cos_Yaw );
				posOffsetsENU[i].y = BodyHeading2ENU_y( posOffsets[i].x , posOffsets[i].y , sin_Yaw , cos_Yaw );
				posOffsetsENU[i].z = posOffsets[i].z;
			}
		}
		bool res = get_AvLineDistanceEnu(resAvDistance, targetVelENU, inRange, posOffsetsENU, offsetsCount, TIMEOUT);
		if( posOffsets!=0 && offsetsCount>0 )
			delete[] posOffsetsENU;
		return res;
	}
/*����*/