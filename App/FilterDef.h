#ifndef __FilterDef_H
#define __FilterDef_H

//���������淶
//in_ǰ׺��ʾ�������
//out_ǰ׺��ʾ�������
//temp_ǰ׺��ʾ��ʱ����
//ǰ׺�����һ�������Ĵ�дA����ʾ������
//ǰ׺�����һ�������Ĵ�дC����ʾ�����ǳ��� 

/*****************************************************************************
 ��������  : ��������������ֵ
 �������  : in_A       ����һ
             in_B       ������
             temp_one   ��ʱ����
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2023��3��23��
*****************************************************************************/
#define ExchangeTwoValue(in_A, in_B, temp_one)  {\
    temp_one = in_A;\
    in_A = in_B;\
    in_B = temp_one;\
}
/*****************************************************************************
 ��������  : ȡ�м�ξ�ֵ�˲���ȥ��inExcludeNum����Сֵ��inExcludeNum�����ֵ
             ע:��ı�Դֵ,��ʹ�Ҫע���Ƿ���ܳ�����������
 �������  : inA_src        Դ��������
             inC_len        Դ���ݳ���
             in_excludeNum  ͬʱȥ�����ֵ����Сֵ�ĸ���
             out_average    ����м�ε�ƽ��ֵ
             temp_i         ��ʱ���ڵ����ı���
             temp_len       ��ʱ���ȱ���
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2023��3��23��
*****************************************************************************/
#define FilterMidAverage(inA_src, inC_len, inC_excludeNum, out_average, temp_i, temp_len)  {\
    temp_len = 0;\
    while((temp_len < (inC_excludeNum << 1)) && (temp_len + 2 < inC_len))    {   /*ѭ������С���ֵ�ᵽǰ��*/\
        for(temp_i = temp_len + 1; temp_i < inC_len; temp_i++)   {\
            if(inA_src[temp_len] > inA_src[temp_i]) /*��Сֵ*/\
                ExchangeTwoValue(inA_src[temp_len], inA_src[temp_i], out_average)\
            if(inA_src[temp_len + 1] < inA_src[temp_i]) /*���ֵ*/\
                ExchangeTwoValue(inA_src[temp_len + 1], inA_src[temp_i], out_average)\
        }\
        temp_len += 2;\
    }\
    out_average = 0;\
    if(temp_len < inC_len)    {\
        for(temp_i = temp_len; temp_i < inC_len; temp_i++)   {\
            out_average += inA_src[temp_i]; /*���*/\
        }\
        out_average /= (inC_len - temp_len);    /*��ƽ��ֵ*/\
    }\
}

/*****************************************************************************
 ��������  : �������ṹ���ʼ������
 �������  : LastP  �ϴι���Э����
             Q      ��������Э����
             R      �۲�����Э����
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2023��8��18��
*****************************************************************************/
#define SimpleInitKalmanFilterData(stKalmanData, lLastP, lQ, lR) {\
    stKalmanData.LastP = lLastP;\
    stKalmanData.Q = lQ;\
    stKalmanData.R = lR;\
    stKalmanData.Now_P = 0.0f;\
    stKalmanData.out = 0.0f;\
    stKalmanData.Kg = 0.0f;\
}

/**
 * �������˲��ṹ��
 */
typedef struct
{
    float LastP;        //�ϴι���Э���� �����ʼ��ֵΪ0.02
    float Now_P;        //��ǰ����Э���� ��ʼ��ֵΪ0
    float out;          //�������˲������ ��ʼ��ֵΪ0
    float Kg;           //���������� ��ʼ��ֵΪ0
    float Q;            //��������Э���� �����ʼ��ֵΪ0.05
    float R;            //�۲�����Э���� �����ʼ��ֵΪ0.05
}ST_KALMAN_DATA;

//�ߵ����ٶȿ������˲��ṹ���ʼ��
#define QUICKLY_FOLLOW_LASTP      0.4f    //�������˲����ٸ������ֵʱ���ϴ�Э���ֵ
#define IMMEDIATELY_FOLLOW_LASTP  0.8f    //�������˲������������ֵʱ���ϴ�Э���ֵ
#define SimpleInitCarNavAccKalmanData SimpleInitKalmanFilterData(gStCarNavAccKalmanData, 0.02f, 0.05f, 0.2f)
#define SimpleInitMotorAccKalmanData SimpleInitKalmanFilterData(gStMotorAccKalmanData, 0.02f, 0.003f, 0.2f)
#define SimpleInitSetAccKalmanData(motorNum) SimpleInitKalmanFilterData(gStMotorRunState[motorNum].setAccKalmanData, QUICKLY_FOLLOW_LASTP, 0.01f, 0.15f)

/*****************************************************************************
 ��������  : �������˲���
 �������  : stKalmanData  ���������ݽṹ��
             input         ����ֵ����
 �������  : ��
 ��    ��  : ����
 ��    ��  : 2023��8��18��
*****************************************************************************/
#define KalmanFilter(stKalmanData, input) {\
    /*Ԥ��Э����̣�kʱ��ϵͳ����Э���� = k-1ʱ�̵�ϵͳЭ���� + ��������Э����*/\
    stKalmanData.Now_P = stKalmanData.LastP + stKalmanData.Q;\
    /*���������淽�̣����������� = kʱ��ϵͳ����Э���� / ��kʱ��ϵͳ����Э���� + �۲�����Э���*/\
    stKalmanData.Kg = stKalmanData.Now_P / (stKalmanData.Now_P + stKalmanData.R);\
    /*��������ֵ���̣�kʱ��״̬����������ֵ = ״̬������Ԥ��ֵ + ���������� * ������ֵ - ״̬������Ԥ��ֵ��*/\
    stKalmanData.out = stKalmanData.out + stKalmanData.Kg * (input -stKalmanData.out);  /*��Ϊ��һ�ε�Ԥ��ֵ������һ�ε����ֵ*/\
    /*����Э�����: ���ε�ϵͳЭ����� stKalmanData.LastP ����һ������׼����*/\
    stKalmanData.LastP = (1-stKalmanData.Kg) * stKalmanData.Now_P;\
}

#endif

