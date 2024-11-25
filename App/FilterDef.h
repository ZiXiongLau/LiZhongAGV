#ifndef __FilterDef_H
#define __FilterDef_H

//变量命名规范
//in_前缀表示输入变量
//out_前缀表示输出变量
//temp_前缀表示临时变量
//前缀如果带一个单独的大写A，表示是数组
//前缀如果带一个单独的大写C，表示可以是常量 

/*****************************************************************************
 功能描述  : 更换两个变量的值
 输入参数  : in_A       变量一
             in_B       变量二
             temp_one   临时变量
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2023年3月23日
*****************************************************************************/
#define ExchangeTwoValue(in_A, in_B, temp_one)  {\
    temp_one = in_A;\
    in_A = in_B;\
    in_B = temp_one;\
}
/*****************************************************************************
 功能描述  : 取中间段均值滤波，去除inExcludeNum个最小值和inExcludeNum个最大值
             注:会改变源值,求和处要注意是否可能出现溢出的情况
 输入参数  : inA_src        源数据数组
             inC_len        源数据长度
             in_excludeNum  同时去除最大值和最小值的个数
             out_average    输出中间段的平均值
             temp_i         临时用于递增的变量
             temp_len       临时长度变量
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2023年3月23日
*****************************************************************************/
#define FilterMidAverage(inA_src, inC_len, inC_excludeNum, out_average, temp_i, temp_len)  {\
    temp_len = 0;\
    while((temp_len < (inC_excludeNum << 1)) && (temp_len + 2 < inC_len))    {   /*循环将最小最大值搬到前面*/\
        for(temp_i = temp_len + 1; temp_i < inC_len; temp_i++)   {\
            if(inA_src[temp_len] > inA_src[temp_i]) /*最小值*/\
                ExchangeTwoValue(inA_src[temp_len], inA_src[temp_i], out_average)\
            if(inA_src[temp_len + 1] < inA_src[temp_i]) /*最大值*/\
                ExchangeTwoValue(inA_src[temp_len + 1], inA_src[temp_i], out_average)\
        }\
        temp_len += 2;\
    }\
    out_average = 0;\
    if(temp_len < inC_len)    {\
        for(temp_i = temp_len; temp_i < inC_len; temp_i++)   {\
            out_average += inA_src[temp_i]; /*求和*/\
        }\
        out_average /= (inC_len - temp_len);    /*求平均值*/\
    }\
}

/*****************************************************************************
 功能描述  : 卡尔曼结构体初始化参数
 输入参数  : LastP  上次估算协方差
             Q      过程噪声协方差
             R      观测噪声协方差
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2023年8月18日
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
 * 卡尔曼滤波结构体
 */
typedef struct
{
    float LastP;        //上次估算协方差 经验初始化值为0.02
    float Now_P;        //当前估算协方差 初始化值为0
    float out;          //卡尔曼滤波器输出 初始化值为0
    float Kg;           //卡尔曼增益 初始化值为0
    float Q;            //过程噪声协方差 经验初始化值为0.05
    float R;            //观测噪声协方差 经验初始化值为0.05
}ST_KALMAN_DATA;

//惯导加速度卡尔曼滤波结构体初始化
#define QUICKLY_FOLLOW_LASTP      0.4f    //卡尔曼滤波快速跟随测量值时的上次协方差赋值
#define IMMEDIATELY_FOLLOW_LASTP  0.8f    //卡尔曼滤波立即跟随测量值时的上次协方差赋值
#define SimpleInitCarNavAccKalmanData SimpleInitKalmanFilterData(gStCarNavAccKalmanData, 0.02f, 0.05f, 0.2f)
#define SimpleInitMotorAccKalmanData SimpleInitKalmanFilterData(gStMotorAccKalmanData, 0.02f, 0.003f, 0.2f)
#define SimpleInitSetAccKalmanData(motorNum) SimpleInitKalmanFilterData(gStMotorRunState[motorNum].setAccKalmanData, QUICKLY_FOLLOW_LASTP, 0.01f, 0.15f)

/*****************************************************************************
 功能描述  : 卡尔曼滤波器
 输入参数  : stKalmanData  卡尔曼数据结构体
             input         测量值输入
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2023年8月18日
*****************************************************************************/
#define KalmanFilter(stKalmanData, input) {\
    /*预测协方差方程：k时刻系统估算协方差 = k-1时刻的系统协方差 + 过程噪声协方差*/\
    stKalmanData.Now_P = stKalmanData.LastP + stKalmanData.Q;\
    /*卡尔曼增益方程：卡尔曼增益 = k时刻系统估算协方差 / （k时刻系统估算协方差 + 观测噪声协方差）*/\
    stKalmanData.Kg = stKalmanData.Now_P / (stKalmanData.Now_P + stKalmanData.R);\
    /*更新最优值方程：k时刻状态变量的最优值 = 状态变量的预测值 + 卡尔曼增益 * （测量值 - 状态变量的预测值）*/\
    stKalmanData.out = stKalmanData.out + stKalmanData.Kg * (input -stKalmanData.out);  /*因为这一次的预测值就是上一次的输出值*/\
    /*更新协方差方程: 本次的系统协方差付给 stKalmanData.LastP 威下一次运算准备。*/\
    stKalmanData.LastP = (1-stKalmanData.Kg) * stKalmanData.Now_P;\
}

#endif

