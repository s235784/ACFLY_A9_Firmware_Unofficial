#pragma once
#include <stdint.h>
#include "TimeBase.hpp"

struct canBatteryStatus
{
	//���̱��
	int16_t manufacturerID;
	//����ͺű���
	int16_t batteryTypeID;
	//����ܵ�ѹ,��λ��mv�� 
	uint16_t batteryMainVoltage;
	//��ŵ����,��λ��10mA��ע��������磬�����ŵ�
	int16_t current;
	//����¶�,��λ��1�棩  
	int16_t batteryTemp;
	//�����ٷֱ�,��λ��%�� 
	uint16_t batteryCapacityPercent;
	//ѭ������
	uint16_t useCircleCount;
	//����״��,��λ��%��  
	int16_t health;	
	//���ڵ�ص�ѹ,��λ��mv��
	uint16_t batteryCellVoltage[14];					
	//����������,��λ��mAh��
	uint16_t batteryCapacityExpected;		
	//���ʣ������,��λ��mAh��
	uint16_t batteryCapacityRemain;
	//��ش�����Ϣ,ÿλ��ʾһ�ִ������͵�״̬
	uint32_t errorFlag;
	// ������к�
	uint8_t BatteryID[16];	
	
	// ����ʱ��
	TIME upDateTime;
  // ��������
	uint8_t cellNum;
	
}__PACKED;


// ��ȡ���״̬
bool getTATTUBatteryStatus(canBatteryStatus& battery);
// ������ʼ��
void init_drv_CAN_BMS_TATTU();