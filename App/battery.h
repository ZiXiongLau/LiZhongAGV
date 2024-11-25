#ifndef _BATTERY_H
#define _BATTERY_H

#include <preDef.h>

#define BATTERY_MAX_NUMS            4       //电池最大个数
#define BATTERY_CELL_MAX_NUMS       24      //电芯最大串数

#define BATTERY_INFO_TYPE_SOC       0x01
#define BATTERY_INFO_TYPE_TEMP      0x02

#define BATTERY485_READ_PERIOD        2000
#define BATTERY485_READ_TIMEOUT_MS 	  120

/*
    电池标志位默认定义:
    b0: 1:single voltage over 4.25V; 0:is OK;
    b1: 1:single voltage under 2.8V; 0:is OK;
    b2: 1:max temperature over 55 deg; 0:is OK;
    b3: 1:min temperature under 0 deg; 0:is OK;
    b4: 1:discharge current over 90A ; 0:is OK;
    b5: 1:charge current over 90A ; 0:is OK;
    b6: 1:Soc under 20% ; 0:is OK;
    b7: 1:single voltage differential over 0.5V; 0:is OK;
    b8-b31:预留；

    电池标志位优贝特V3定义:
    b0 : 放电高温(0:未发生保护；1:发生保护)
    b1 : 二级放电过流(0:未发生保护；1:发生保护)
    b2 : 硬件过流(0:未发生保护；1:发生保护)
    b3 : 硬件过压(0:未发生保护；1:发生保护)
    b4 : 硬件欠压(0:未发生保护；1:发生保护)
    b5 : 总电压欠压(0:未发生保护；1:发生保护)
    b6 : 总电压过压(0:未发生保护；1:发生保护)
    b7 : 短路保护(0:未发生保护；1:发生保护)
    b8 : 充电过流(0:未发生保护；1:发生保护)
    b9 : 放电过流(0:未发生保护；1:发生保护)
    b10: 低温放电(0:未发生保护；1:发生保护)
    b11: 低温充电(0:未发生保护；1:发生保护)
    b12: MOS过温(0:未发生保护；1:发生保护)
    b13: 电池过温(0:未发生保护；1:发生保护)
    b14: 过压(0:未发生保护；1:发生保护)
    b15: 欠压(0:未发生保护；1:发生保护)
    b16: reserve;
    b17: reserve;
    b18: reserve;
    b19: 与T1通讯状态(0:正常；1:异常)
    b20: reserve;
    b21: reserve;
    b22: 均衡开启标志位(0:均衡关闭；1:均衡开启)
    b23: reserve;
    b24: 温度采样状态(0:正常；1:异常)
    b25: 采样芯片状态(0:正常；1:异常)
    b26: 放电MOS状态(0:关闭；1:打开)
    b27: 充电MOS状态(0:关闭；1:打开)
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
	uint8_t  maxtemp ;              /*	unit: +40℃ */
	uint8_t  mintemp ;              /*	unit: +40℃ */
	uint8_t  maxtempnum ;           /*	unit: 1-37 */
	uint8_t  mintempnum ;           /*	unit: 1-37 */
    uint8_t  error;          //电池故障
    uint16_t singleVol[BATTERY_CELL_MAX_NUMS];  //单体电芯电压，单位0.001V
	uint8_t  validFlag;      //电池数据数据有效标志，1-有效，0-无效		
	uint8_t  driverVolValidFlag;    //驱动器电压值有效
	uint8_t motorRunFlag;    //电机运行标志
	uint32_t motorIdleTime;  //电机空闲时间
	uint32_t lastRevTime;    //上次接收数据的实际
} Battery_status;

void InitBatteryData(void);
void BatterPrintInfo(void);
void BatteryCollectProcess(rt_bool_t canFlag);
extern Battery_status gStBatteryState;

#endif
