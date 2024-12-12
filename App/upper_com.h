#ifndef UPPER_COM_
#define UPPER_COM_

#include <stdint.h>
#include "predef.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "usart.h"
#include "odom.h"


// ֡ͷ
#define FRAME_HEADER 0xFF       // ֡ͷ
#define PROTOCOL_VERSION 0xFF   // �汾��

#define UPPER_COM_PROTOCOL_CARSTATE_LENGTH    			23
#define UPPER_COM_PROTOCOL_CHASISVELCMD_LENGTH         	15

/* �Ϸ����ݵ�Ƶ�� */
#define UPPER_COM_TX_CAR_STA_FREQUENCY 20 //Hz




#pragma pack (1) /*ָ����1�ֽڶ���*/

enum upper_com_protocol_type{
    UPPER_COM_PROTOCOL_TYPE_DEFAULT = 0,
    UPPER_COM_PROTOCOL_TYPE_CARSTATE = 0x01,
    UPPER_COM_PROTOCOL_TYPE_CHASISVELCMD = 0x10
};

struct upper_com_protocol_head
{
    uint8_t head;
    uint8_t protocol_version;
    enum upper_com_protocol_type protocol_type;
    uint16_t data_length;
};


//�·�����֡1 �����ٶ�
typedef struct upper_com_protocol_chasisvel upper_com_protocol_chasisvel_t;
struct upper_com_protocol_chasisvel
{
    struct upper_com_protocol_head protocol_head;
    float chasis_linear_vel_x;
    float chasis_angular_vel_z;
    uint16_t crc16; //all of the protocol
};

/*------------------------------------------------------------------------------------------------------------------*/

//�Ϸ�����֡1 ������̼ơ������ٶ�
typedef struct upper_com_protocol_carstate upper_com_protocol_carstate_t;
struct upper_com_protocol_carstate
{
    struct upper_com_protocol_head protocol_head;
    float odometry_left_wheel;
    float odometry_right_wheel;
    uint16_t crc16; //all of the protocol
};

#pragma pack () /*ȡ��ָ�����룬�ָ�ȱʡ����*/






uint16_t CRC16(uint8_t *_pBuf, uint16_t _usLen);

#endif /* UPPER_COM_ */
