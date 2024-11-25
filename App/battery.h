#ifndef _BATTERY_H
#define _BATTERY_H

#include <preDef.h>

#define BATTERY_MAX_NUMS            4       //���������
#define BATTERY_CELL_MAX_NUMS       24      //��о�����

#define BATTERY_INFO_TYPE_SOC       0x01
#define BATTERY_INFO_TYPE_TEMP      0x02

#define BATTERY485_READ_PERIOD        2000
#define BATTERY485_READ_TIMEOUT_MS 	  120

/*
    ��ر�־λĬ�϶���:
    b0: 1:single voltage over 4.25V; 0:is OK;
    b1: 1:single voltage under 2.8V; 0:is OK;
    b2: 1:max temperature over 55 deg; 0:is OK;
    b3: 1:min temperature under 0 deg; 0:is OK;
    b4: 1:discharge current over 90A ; 0:is OK;
    b5: 1:charge current over 90A ; 0:is OK;
    b6: 1:Soc under 20% ; 0:is OK;
    b7: 1:single voltage differential over 0.5V; 0:is OK;
    b8-b31:Ԥ����

    ��ر�־λ�ű���V3����:
    b0 : �ŵ����(0:δ����������1:��������)
    b1 : �����ŵ����(0:δ����������1:��������)
    b2 : Ӳ������(0:δ����������1:��������)
    b3 : Ӳ����ѹ(0:δ����������1:��������)
    b4 : Ӳ��Ƿѹ(0:δ����������1:��������)
    b5 : �ܵ�ѹǷѹ(0:δ����������1:��������)
    b6 : �ܵ�ѹ��ѹ(0:δ����������1:��������)
    b7 : ��·����(0:δ����������1:��������)
    b8 : ������(0:δ����������1:��������)
    b9 : �ŵ����(0:δ����������1:��������)
    b10: ���·ŵ�(0:δ����������1:��������)
    b11: ���³��(0:δ����������1:��������)
    b12: MOS����(0:δ����������1:��������)
    b13: ��ع���(0:δ����������1:��������)
    b14: ��ѹ(0:δ����������1:��������)
    b15: Ƿѹ(0:δ����������1:��������)
    b16: reserve;
    b17: reserve;
    b18: reserve;
    b19: ��T1ͨѶ״̬(0:������1:�쳣)
    b20: reserve;
    b21: reserve;
    b22: ���⿪����־λ(0:����رգ�1:���⿪��)
    b23: reserve;
    b24: �¶Ȳ���״̬(0:������1:�쳣)
    b25: ����оƬ״̬(0:������1:�쳣)
    b26: �ŵ�MOS״̬(0:�رգ�1:��)
    b27: ���MOS״̬(0:�رգ�1:��)
    b28: reserve;
    b29: reserve;
    b30: reserve;
    b31: reserve;
    
*/
typedef union
{
    struct
    {
        uint8_t b0   :1; 	        
    	uint8_t b1   :1;	        
    	uint8_t b2   :1;	        
    	uint8_t b3   :1;	        
        uint8_t b4   :1;          
    	uint8_t b5   :1;	        
    	uint8_t b6   :1;	        
    	uint8_t b7   :1;	        
        uint8_t b8   :1;            
        uint8_t b9   :1;            
        uint8_t b10  :1;    
        uint8_t b11  :1; 	        
    	uint8_t b12  :1;	        
    	uint8_t b13  :1;	        
    	uint8_t b14  :1;	        
        uint8_t b15  :1;          
    	uint8_t b16  :1;	        
    	uint8_t b17  :1;	        
    	uint8_t b18  :1;	        
        uint8_t b19  :1;              
        uint8_t b20  :1;              
        uint8_t b21  :1;    
        uint8_t b22  :1; 	        
    	uint8_t b23  :1;	        
    	uint8_t b24  :1;	        
    	uint8_t b25  :1;	        
        uint8_t b26  :1;          
    	uint8_t b27  :1;	        
    	uint8_t b28  :1;	        
    	uint8_t b29  :1;	        
        uint8_t b30  :1;              
        uint8_t b31  :1;              
    }bits;
    uint8_t byt[4];
    uint16_t word[2];
    uint32_t dword;
} Battery_error ;

typedef struct
{
	uint16_t voltage; 	            /*	unit: 0.1V/bit	 */
	int16_t current;	            /*	unit: 0.1A/bit	 */
	uint8_t  soc;	                /*	unit: 0.01Ah  1%/bit*/
    uint16_t soh;                   /*	unit: 0.1%/bit*/
	Battery_error  battery_error ;	/*	unit: 1:is error ;0:is OK */
    uint8_t  maxvnum; 	            /*	unit: 1-13 */
	uint8_t  minvnum;	            /*	unit: 1-13 */
	uint16_t maxvoltage;	        /*	unit: 0.001V/bit */
	uint16_t minvoltage;	        /*	unit: 0.001V/bit */
	uint8_t  maxtemp ;              /*	unit: +40�� */
	uint8_t  mintemp ;              /*	unit: +40�� */
	uint8_t  maxtempnum ;           /*	unit: 1-37 */
	uint8_t  mintempnum ;           /*	unit: 1-37 */
    uint8_t  error;          //��ع���
    uint16_t singleVol[BATTERY_CELL_MAX_NUMS];  //�����о��ѹ����λ0.001V
	uint8_t  validFlag;      //�������������Ч��־��1-��Ч��0-��Ч		
	uint8_t  driverVolValidFlag;    //��������ѹֵ��Ч
	uint8_t motorRunFlag;    //������б�־
	uint32_t motorIdleTime;  //�������ʱ��
	uint32_t lastRevTime;    //�ϴν������ݵ�ʵ��
} Battery_status;

void InitBatteryData(void);
void BatterPrintInfo(void);
void BatteryCollectProcess(rt_bool_t canFlag);
extern Battery_status gStBatteryState;

#endif
