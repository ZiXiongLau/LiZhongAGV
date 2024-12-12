#include "upper_com.h"


uint8_t gUpper_com_send_buf[512];
uint8_t gUpper_com_rev_buf[512];
uint8_t gRx_buff[100];



upper_com_protocol_chasisvel_t upper_com_protocol_chasisvel_temp;


upper_com_protocol_carstate_t upper_com_protocol_carstate_pack(odom_t* odom)
{
    struct upper_com_protocol_carstate upper_com_protocol_carstate_temp;

    upper_com_protocol_carstate_temp.protocol_head.head = FRAME_HEADER;
    upper_com_protocol_carstate_temp.protocol_head.protocol_version = PROTOCOL_VERSION;
    upper_com_protocol_carstate_temp.protocol_head.protocol_type = UPPER_COM_PROTOCOL_TYPE_CARSTATE;
    upper_com_protocol_carstate_temp.protocol_head.data_length = sizeof(struct upper_com_protocol_carstate);//��λ��ǰ

    upper_com_protocol_carstate_temp.odometry_left_wheel = odom->odometry_left_wheel;
    upper_com_protocol_carstate_temp.odometry_right_wheel = odom->odometry_left_wheel;

    upper_com_protocol_carstate_temp.crc16 = CRC16((uint8_t *)&upper_com_protocol_carstate_temp, sizeof(struct upper_com_protocol_carstate) - 2);

    return  upper_com_protocol_carstate_temp;
}

int8_t upper_com_protocol_get_type_length(enum upper_com_protocol_type type, uint16_t *type_length)
{
    int8_t result = 0;

    switch (type)
    {
        case UPPER_COM_PROTOCOL_TYPE_CARSTATE:
            *type_length = UPPER_COM_PROTOCOL_CARSTATE_LENGTH;
            break;
        case UPPER_COM_PROTOCOL_TYPE_CHASISVELCMD:
            *type_length = UPPER_COM_PROTOCOL_CHASISVELCMD_LENGTH;
            break;
        default:
            *type_length = 0;
            result = -1;//���ʹ���
            break;
    }
    return result;
}


int8_t upper_com_protocol_chasisvel_unpack(uint8_t *buf, uint16_t buf_length, upper_com_protocol_chasisvel_t *upper_com_protocol_chasisvel_temp)
{
    int8_t result;

    do{
        /* buf���ȣ��������·���Խ�� */
        if(buf_length != sizeof(struct upper_com_protocol_chasisvel))
        {
            result = -1;
            break;
        }
        /* ��ʼλ */
        if(buf[0] != FRAME_HEADER)
        {
            result = -2;
            break;
        }
        /* Э��汾 */
        if(buf[1] != PROTOCOL_VERSION)
        {
            result = -3;
            break;
        }
        /* �������� */
        if(buf[2] != UPPER_COM_PROTOCOL_TYPE_CHASISVELCMD)
        {
            result = -4;
            break;
        }
        /* ���ݳ��� */
        if(sizeof(struct upper_com_protocol_chasisvel) != (((uint16_t)buf[4]<<8) + buf[3]))
        {
            result = -5;
            break;
        }
        /* crc16У�� */
        if((((uint16_t)buf[buf_length - 1]<<8) + buf[buf_length -2] )!= CRC16(buf, buf_length - 2))
        {
            result = -6;
            break;
        }

        memcpy(upper_com_protocol_chasisvel_temp, buf, buf_length);
        return 0;
    }while(0);

    return result;
}

static void upper_com_rx_succeed_cb(enum upper_com_protocol_type type, uint8_t *buf, uint16_t buf_length)
{
    uint16_t type_length;
    do{
        /* ������ */
        upper_com_protocol_get_type_length(type, &type_length);
        if(type_length != buf_length)
        {
            rt_kprintf("upper_com_rx_succeed_cb:type_length != buf_length");
            break;
        }
        /* ȡ�����ݣ�������Ϣ���� */
        switch(type)
        {
            case  UPPER_COM_PROTOCOL_TYPE_CHASISVELCMD:
                memcpy(&upper_com_protocol_chasisvel_temp, buf, type_length);
                break;
            default:
                break;
        }
    }while(0);
}



void StartTaskRx(void const * argument)
{
	uint8_t _data;
	uint8_t _state = 0;
	uint32_t _receive_len =0;
	uint16_t _index = 0;
	static uint8_t _length = 0;

	enum upper_com_protocol_type protocol_type = UPPER_COM_PROTOCOL_TYPE_DEFAULT;
	uint16_t protocol_length;

	while(1)
	{
		_receive_len = 0;
		_index = 0;
		_receive_len = UsartDeviceRead(USART1_DEVICE, gUpper_com_rev_buf, sizeof(gUpper_com_rev_buf));

		while(_receive_len > 0)
		{
			_data = gUpper_com_rev_buf[_index];
			_receive_len--;
			_index++;

			if(_length >= 100)
			{
				_length = 0;
				_state = 0;
			}
			gRx_buff[_length++] = _data;
			switch (_state)
			{
				case 0:
					/* У��֡ͷ */
					if(1 == _length)
					{	
						if(FRAME_HEADER == gRx_buff[0])
						{
							_state = 1;
						}
						else
						{
							_length = 0;
							_state = 0;
							rt_kprintf("step 0 failed!\r\n");
						}
					}
					else
					{
						_length = 0;
						_state = 0;
						rt_kprintf("step 0 error!\r\n");
					}
					break;
				case 1:
					/* У��֡ͷ */
					if(2 == _length)
					{
						if(gRx_buff[1] == PROTOCOL_VERSION)
						{
							_state = 2;
						}
						else if(gRx_buff[1] == FRAME_HEADER)
						{
							_state = 1;
							_length = 1;
						}
						else
						{
							_length = 0;
							_state = 0;
							rt_kprintf("step 1 failed!\r\n");
						}
					}
					else
					{
						_length = 0;
						_state = 0;
						rt_kprintf("step 1 error!\r\n");
					}
					break;
				case 2:
					/* ����ͷ������ */
					if(_length >= 5)
					{
						_state = 3;
					}
					break;
				case 3:
					/* У��ͷ��: �ж�֡���ͣ���ȡ֡����                 */
					protocol_type = gRx_buff[2];
					upper_com_protocol_get_type_length(protocol_type, &protocol_length);
					if(((((uint16_t)gRx_buff[4] << 8) + gRx_buff[3]) == protocol_length)) //У�����ݳ���
					{
						if((protocol_length > 100) || (protocol_length < 6))//��Ϊ������Ҫ 6 ���ֽ������һ����Ч������֡������֡ͷ���汾�����͡����Ⱥ�CRCУ�飩
						{
							rt_kprintf("protocol_length out of range!\r\n");
							_length = 0;
							_state = 0;
							break;
						}
						else
						{
							_state = 4;
						}
					}
					else
					{
						rt_kprintf("step 3 failed!\r\n");
						_length = 0;
						_state = 0;
					}
					break;	
				case 4:
					/* ��������֡ */
					if(_length >= protocol_length - 1) //�����յ���type_length������ʱ ����step5
					{
						_state = 5;
					}
					break;
				case 5:
					/* У������֡ */
					if((((uint16_t)gRx_buff[protocol_length - 1] << 8) + gRx_buff[protocol_length - 2]) == CRC16(gRx_buff, protocol_length - 2))
					{
						rt_kprintf("upper_com_rx_parse success!\r\n");
						upper_com_rx_succeed_cb(protocol_type,gRx_buff,_length);
					}
					else
					{
						rt_kprintf("step 5 failed!\r\n");
					}
					_length = 0;//�ɹ�����ʧ�� ��������ʼ
					_state = 0;
					break;
				default:
					rt_kprintf("setp input error!\r\n");
					break;
			}
		}
		osDelay(10);
	}
}
void StartTaskTx(void const * argument)
{
	upper_com_protocol_carstate_t _carstateinfo2upper;
	uint16_t _send_size = 0;
	static TickType_t _cur_tick;
	TickType_t _tick[10];
	odom_t _odom;


	while(1)
	{
		_cur_tick = xTaskGetTickCount();
		_send_size = 0;
		memset(gUpper_com_rev_buf,0,sizeof(gUpper_com_rev_buf));

		if((_cur_tick - _tick[0]) >= (1000 / UPPER_COM_TX_CAR_STA_FREQUENCY))
		{
			_odom = wheel_odom_get();
/*			rt_kprintf("left_odom : %.2f , right_odom : %.2f",_odom.odometry_left_wheel,_odom.odometry_right_wheel);*/
			rt_kprintf("test\r\n");
			_carstateinfo2upper = upper_com_protocol_carstate_pack(&_odom);
			memcpy(&gRx_buff[_send_size], (uint8_t *)&_carstateinfo2upper, sizeof(_carstateinfo2upper));
			_send_size = _send_size + sizeof(_carstateinfo2upper);
			_tick[0] = _cur_tick;
		}

		if(_send_size > 0)
		{
			UsartDeviceWrite(USART1_DEVICE, gRx_buff, _send_size);
		}
		osDelay(1);
	}
}







/*
*********************************************************************************************************
*   �� �� ��: CRC16_Modbus
*   ����˵��: ����CRC�� ����ModbusЭ�顣
*   ��    ��: _pBuf : ����У�������
*             _usLen : ���ݳ���
*   �� �� ֵ: 16λ����ֵ�� ����Modbus ���˽�����ֽ��ȴ��ͣ����ֽں��͡�
*
*   ���п��ܵ�CRCֵ����Ԥװ���������鵱�У������㱨������ʱ���Լ򵥵��������ɣ�
*   һ�����������16λCRC�������256�����ܵĸ�λ�ֽڣ���һ�����麬�е�λ�ֽڵ�ֵ��
*   ������������CRC�ķ�ʽ�ṩ�˱ȶԱ��Ļ�������ÿһ�����ַ��������µ�CRC����ķ�����
*
*  ע�⣺�˳����ڲ�ִ�и�/��CRC�ֽڵĽ������˺������ص����Ѿ�����������CRCֵ��Ҳ����˵���ú����ķ���ֵ����ֱ�ӷ���
*        �ڱ������ڷ��ͣ�
*********************************************************************************************************
*/

static const uint8_t s_CRCHi[] = {
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
        0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
        0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
        0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
        0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
        0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
        0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
        0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
        0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
        0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
        0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
        0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
        0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
        0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
        0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
        0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
        0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
};


// CRC ��λ�ֽ�ֵ��
const uint8_t s_CRCLo[] = {
        0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,
        0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
        0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
        0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
        0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,
        0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
        0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
        0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
        0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
        0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
        0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,
        0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
        0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,
        0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
        0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
        0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
        0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,
        0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
        0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,
        0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
        0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
        0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
        0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,
        0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
        0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,
        0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};

uint16_t CRC16(uint8_t *_pBuf, uint16_t _usLen)
{
    uint8_t ucCRCHi = 0xFF; /* ��CRC�ֽڳ�ʼ�� */
    uint8_t ucCRCLo = 0xFF; /* ��CRC �ֽڳ�ʼ�� */
    uint16_t usIndex;  /* CRCѭ���е����� */
    while (_usLen--)
    {
        usIndex = ucCRCHi ^ (*_pBuf); /* ����CRC */
        _pBuf++;
        ucCRCHi = ucCRCLo ^ s_CRCHi[usIndex];
        ucCRCLo = s_CRCLo[usIndex];
    }
    return ((uint16_t)ucCRCHi << 8 | ucCRCLo);
}


