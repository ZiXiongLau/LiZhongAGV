/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.c
  * @brief   This file provides code for the configuration
  *          of the ADC instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "adc.h"

/* USER CODE BEGIN 0 */
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc3;

__ALIGNED(4)
static uint16_t gAdcDmaBuffer[ADC_FILTER_TIMES * ADCA_MAX_CHANNELS] = {0};    //dma缓冲
static float gAdcLastValue[ADCA_MAX_CHANNELS] = {0, 0, 0};
static uint16_t gAdcTmpDmaBuffer[ADC_FILTER_TIMES * ADCTMP_MAX_CHANNELS] = {0};    //dma缓冲
static float gAdcTmpLastValue[ADCTMP_MAX_CHANNELS] = {0};
static uint16_t gAdcTmpIndex = 0;
static uint16_t gAdcTempBuffer[ADC_FILTER_TIMES];

const float RTD_TAB_PT100[211] =   // 表格是以5度为一步，即-200, -195, - 190.....
{
 18.52,20.68,22.83,24.97,27.10,29.22,31.34,33.44,35.54,37.64,                  // -200 ~ -155   10
 
 39.72,41.80,43.88,45.94,48.00,50.06,52.11,54.15,56.19,58.23,                  // -150 ~ -105
 
 60.26,62.28,64.30,66.31,68.33,70.33,72.33,74.33,76.33,78.32,                  // -100 ~ -55
 
 80.31,82.29,84.27,86.25,88.22,90.19,92.16,94.12,96.09,98.04,                  //        -50 ~ -5
 
 100.00,101.95,103.90,105.85,107.79,109.73,111.67,113.61,115.54,117.47,        // 0   ~ 45
 
 119.40,121.32,123.24,125.16,127.08,128.99,130.90,132.80,134.71,136.61,        // 50  ~ 95
 
 138.51,140.40,142.29,144.18,146.07,147.95,149.83,151.71,153.58,155.46,        // 100 ~ 145
 
 157.33,159.19,161.05,162.91,164.77,166.63,168.48,170.33,172.17,174.02,        // 150 ~ 195
 
 175.86,177.69,179.53,181.36,183.19,185.01,186.84,188.66,190.47,192.29,        // 200 ~ 245
 
 194.10,195.91,197.71,199.51,201.31,203.11,204.90,206.70,208.48,210.27,        // 250 ~ 295
 
 212.05,213.83,215.61,217.38,219.15,220.92,222.68,224.45,226.21,227.96,        // 300 ~ 345
 
 229.72,231.47,233.21,234.96,236.70,238.44,240.18,241.91,243.64,245.37,        // 350 ~ 395
 
 247.09,248.81,250.53,252.25,253.96,255.67,257.38,259.08,260.78,262.48,        // 400 ~ 445
 
 264.18,265.87,267.56,269.25,270.93,272.61,274.29,275.97,277.64,279.31,        // 450 ~ 495
 
 280.98,282.64,284.30,285.96,287.62,289.27,290.92,292.56,294.21,295.85,        // 500 ~ 545
 
 297.49,299.12,300.75,302.38,304.01,305.63,307.25,308.87,310.49,312.10,        // 550 ~ 595
 
 313.71,315.31,316.92,318.52,320.12,321.71,323.30,324.89,326.48,328.06,        // 600 ~ 645
 
 329.64,331.22,332.79,334.36,335.93,337.50,339.06,340.62,342.18,343.73,        // 650 ~ 695
 
 345.28,346.83,348.38,349.92,351.46,353.00,354.53,356.06,357.59,359.12,        // 700 ~ 745
 
 360.64,362.16,363.67,365.19,366.70,368.21,369.71,371.21,372.71,374.21,        // 750 ~ 795
 
 375.70,377.19,378.68,380.17,381.65,383.13,384.60,386.08,387.55,389.02,        // 800 ~ 845
 
 390.48        // 850
};

/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc3;

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}
/* ADC3 init function */
void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 10;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_8;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_9;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_10;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* ADC1 clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PA4     ------> ADC1_IN4
    PA5     ------> ADC1_IN5
    PA6     ------> ADC1_IN6
    */
    GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC1_MspInit 1 */
    /* ADC1 Init */

    __HAL_RCC_DMA2_CLK_ENABLE();
    
    hdma_adc1.Instance = DMA2_Stream4;
    hdma_adc1.Init.Channel = DMA_CHANNEL_0;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    hdma_adc1.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;
    hdma_adc1.Init.MemBurst = DMA_MBURST_SINGLE;
    hdma_adc1.Init.PeriphBurst = DMA_PBURST_SINGLE;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_DMA_Start(&hdma_adc1, (uint32_t)(&(ADC1->DR)), (uint32_t)gAdcDmaBuffer, ADC_FILTER_TIMES * ADCA_MAX_CHANNELS);
    
  /* USER CODE END ADC1_MspInit 1 */
  }
  else if(adcHandle->Instance==ADC3)
  {
  /* USER CODE BEGIN ADC3_MspInit 0 */

  /* USER CODE END ADC3_MspInit 0 */
    /* ADC3 clock enable */
    __HAL_RCC_ADC3_CLK_ENABLE();

    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**ADC3 GPIO Configuration
    PF3     ------> ADC3_IN9
    PF4     ------> ADC3_IN14
    PF5     ------> ADC3_IN15
    PF6     ------> ADC3_IN4
    PF7     ------> ADC3_IN5
    PF8     ------> ADC3_IN6
    PF9     ------> ADC3_IN7
    PF10     ------> ADC3_IN8
    PC2     ------> ADC3_IN12
    PC3     ------> ADC3_IN13
    */
    GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC3_MspInit 1 */
  /* ADC3 Init */
  
      __HAL_RCC_DMA2_CLK_ENABLE();
      
      hdma_adc3.Instance = DMA2_Stream0;
      hdma_adc3.Init.Channel = DMA_CHANNEL_2;
      hdma_adc3.Init.Direction = DMA_PERIPH_TO_MEMORY;
      hdma_adc3.Init.PeriphInc = DMA_PINC_DISABLE;
      hdma_adc3.Init.MemInc = DMA_MINC_ENABLE;
      hdma_adc3.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
      hdma_adc3.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
      hdma_adc3.Init.Mode = DMA_CIRCULAR;
      hdma_adc3.Init.Priority = DMA_PRIORITY_HIGH;
      hdma_adc3.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
      hdma_adc3.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;
      hdma_adc3.Init.MemBurst = DMA_MBURST_SINGLE;
      hdma_adc3.Init.PeriphBurst = DMA_PBURST_SINGLE;
      if (HAL_DMA_Init(&hdma_adc3) != HAL_OK)
      {
          Error_Handler();
      }
  
      HAL_DMA_Start(&hdma_adc3, (uint32_t)(&(ADC3->DR)), (uint32_t)gAdcTmpDmaBuffer, ADC_FILTER_TIMES * ADCTMP_MAX_CHANNELS);

  /* USER CODE END ADC3_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC1_CLK_DISABLE();

    /**ADC1 GPIO Configuration
    PA4     ------> ADC1_IN4
    PA5     ------> ADC1_IN5
    PA6     ------> ADC1_IN6
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6);

  /* USER CODE BEGIN ADC1_MspDeInit 1 */
    HAL_DMA_DeInit(adcHandle->DMA_Handle);
  /* USER CODE END ADC1_MspDeInit 1 */
  }
  else if(adcHandle->Instance==ADC3)
  {
  /* USER CODE BEGIN ADC3_MspDeInit 0 */

  /* USER CODE END ADC3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC3_CLK_DISABLE();

    /**ADC3 GPIO Configuration
    PF3     ------> ADC3_IN9
    PF4     ------> ADC3_IN14
    PF5     ------> ADC3_IN15
    PF6     ------> ADC3_IN4
    PF7     ------> ADC3_IN5
    PF8     ------> ADC3_IN6
    PF9     ------> ADC3_IN7
    PF10     ------> ADC3_IN8
    PC2     ------> ADC3_IN12
    PC3     ------> ADC3_IN13
    */
    HAL_GPIO_DeInit(GPIOF, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10);

    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_2|GPIO_PIN_3);

  /* USER CODE BEGIN ADC3_MspDeInit 1 */

  /* USER CODE END ADC3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/*****************************************************************************
 功能描述  : 启动adc
 输入参数  : void  
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2021年4月15日
*****************************************************************************/
void AdcStart(void)
{
    LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_UNLIMITED);

    HAL_ADC_Start(&hadc1);

    LL_ADC_REG_SetDMATransfer(ADC3, LL_ADC_REG_DMA_TRANSFER_UNLIMITED);

    HAL_ADC_Start(&hadc3);
}
/*****************************************************************************
 功能描述  : 二分滤波
 输入参数  : 无
 输出参数  : 无
 作    者  :  
 日    期  : 2020年4月16日
*****************************************************************************/
uint16_t Dichotomyfiltering(uint16_t* data, uint16_t dataLen)
{
    uint16_t i;
    uint16_t lMin = data[0];
    uint16_t lMax = lMin;
    uint16_t lMid, lHighNum, lMinTemp, lMaxTemp;
    uint16_t lEndFlag = 0x02;
    
    //找最小最大值
    for(i = 0; i < dataLen; i++)
    {
        if(data[i] < lMin)
        {
            lMin = data[i];
        }
        else if(data[i] > lMax)
        {
            lMax = data[i];
        }
    }
    
    //寻找中值，二分查找
    while((lEndFlag <= dataLen) && (lMin + 2 < lMax))
    {
        lEndFlag <<= 1;
        lMid = (lMin + lMax) >> 1;
        lHighNum = 0;
        lMinTemp = lMax;
        lMaxTemp = lMin;
        for(i = 0; i < dataLen; i++)
        {   
            if(data[i] > lMid)
            {
                lHighNum++;
                if(data[i] < lMinTemp)
                {
                    lMinTemp = data[i];
                }
            }
            else if(data[i] > lMaxTemp)
            {
                lMaxTemp = data[i];
            }
        }
        //小于中值的较多，则下次在小于中值的数之间二分查找
        if(lHighNum + 1 < (dataLen >> 1))
        {
            lMax = lMaxTemp;
        }
        //大于中值的较多，则下次在大于中值的数之间二分查找
        else if(lHighNum > (dataLen >> 1) + 1)
        {
            lMin = lMinTemp;
        }
        else
        {
            break;
        }
    }
    
    lMid = (lMin + lMax) >> 1;

    return lMid;
}
/*****************************************************************************
 功能描述  : 获取指定通道ADC转换缓存的中间值(中值滤波)
 输入参数  : uint8_t channel  
 输出参数  : uint16_t   AD值
 作    者  : 刘鹏
 日    期  : 2018年12月18日
*****************************************************************************/
uint8_t ADCGetMidAdcValue(uint8_t channel, uint16_t *value)
{ 
    uint16_t i;
    
    if(channel >= ADCA_MAX_CHANNELS)
    {
        return 1;
    }
    /*if((SET == DMA_GetFlagStatus(ADCA_DMA_PORT, ADCA_DMA_ERROR_TEIF))
        || (SET == DMA_GetFlagStatus(ADCA_DMA_PORT, ADCA_DMA_ERROR_FEIF))
        || (SET == ADC_GetFlagStatus(ADC1, ADC_FLAG_OVR)))//DMA传输错误，则重新初始化
    {
        rt_kprintf("ADC1 DMA Error!");
        log_kprintf("ADC1 DMA Error!");
        ADCInit();
        return 1;
    }*/
    
    //赋值滤波数组
    for(i = 0; i < ADC_FILTER_TIMES; i++)
    {
        gAdcTempBuffer[i] = gAdcDmaBuffer[i * ADCA_MAX_CHANNELS + channel] & 0x0fff;//12位精度 
    }

    *value = Dichotomyfiltering(gAdcTempBuffer, ADC_FILTER_TIMES);
    
    return 0;
}
/*****************************************************************************
 功能描述  : 获取指定通道ADC转换缓存的中间值(中值滤波)
 输入参数  : uint8_t channel  
 输出参数  : uint16_t   AD值
 作    者  : 刘鹏
 日    期  : 2018年12月18日
*****************************************************************************/
uint8_t ADCTmpGetMidAdcValue(uint8_t channel, uint16_t *value)
{ 
    uint16_t i;
    
    if(channel >= ADCTMP_MAX_CHANNELS)
    {
        return 1;
    }
    /*if((SET == DMA_GetFlagStatus(ADCA_DMA_PORT, ADCA_DMA_ERROR_TEIF))
        || (SET == DMA_GetFlagStatus(ADCA_DMA_PORT, ADCA_DMA_ERROR_FEIF))
        || (SET == ADC_GetFlagStatus(ADC1, ADC_FLAG_OVR)))//DMA传输错误，则重新初始化
    {
        rt_kprintf("ADC1 DMA Error!");
        log_kprintf("ADC1 DMA Error!");
        ADCInit();
        return 1;
    }*/
    
    //赋值滤波数组
    for(i = 0; i < ADC_FILTER_TIMES; i++)
    {
        gAdcTempBuffer[i] = gAdcTmpDmaBuffer[i * ADCTMP_MAX_CHANNELS + channel] & 0x0fff;//12位精度 
    }

    *value = Dichotomyfiltering(gAdcTempBuffer, ADC_FILTER_TIMES);
    
    return 0;
}
/*****************************************************************************
 功能描述  : 获取传感器值
 输入参数  : uint8_t channel  指定adc通道
             int32_t offset   零位偏移值
             float ratio      电压转传感器值系数
 输出参数  : float            传感器值
 作    者  :  
 日    期  : 2021年4月15日
*****************************************************************************/
float AdcGetSensorValue(uint8_t channel, int32_t offset, float ratio)
{
    float lAngle = CVT_RATIO;
    uint16_t lValue;
    //中值滤波
    if(0 == ADCGetMidAdcValue(channel, &lValue))
    {
        lAngle *= lValue;   //计算电压
        lAngle *= ratio;    //电压转传感器值
        lAngle += offset;   //加上零位偏移值
        gAdcLastValue[channel] = lAngle; 
    }
    else
    {
        lAngle = gAdcLastValue[channel];
    }    

    return lAngle;
}
/***********************************************************************
 *FunName:        float CalculateTemperature(float fR)
 *
 *In:                fR -> PT100的电阻值。
 *                        
 *Out:                fTem -> 测得的温度值。               
 *
*Discription: 将电阻值查表算出温度值。
 *
 *Notes:         采用2分查找法。          ************************************************************************/
float CalculateTemperature(float fR)
{
    float fTem, fLowRValue, fHighRValue;        
    int   iTem;
    uint8_t i, cBottom, cTop;

    if (fR < RTD_TAB_PT100[0])                // 电阻值小于表格最值低于量程下限。
    {
         return -200;
    }
    if (fR > RTD_TAB_PT100[210])        // 电阻值大于表格最大值，超出量程上限 。
    {
         return 850;
    }

    cBottom = 0; 
    cTop    = 210;

    for(i=105; (cTop-cBottom)!=1;)        // 2分法查表。
    {
        if (fR < RTD_TAB_PT100[i])
        {
           cTop = i;
           i = (cTop + cBottom) / 2;
        }
        else if (fR > RTD_TAB_PT100[i])
        {
             cBottom = i;
             i = (cTop + cBottom) / 2;
        }
        else
        {
            iTem = (uint32_t)i * 5 - 200;
            fTem = (float)iTem;
            return fTem;
        }
    }

    iTem = (uint32_t)i * 5 - 200;
    fLowRValue  = RTD_TAB_PT100[cBottom];
    fHighRValue = RTD_TAB_PT100[cTop];
    fTem = ( ((fR - fLowRValue)*5) / (fHighRValue - fLowRValue) ) + iTem;        // 表格是以5度为一步的。
                                                                                   // 两点内插进行运算。
    return fTem;
}

/*****************************************************************************
 功能描述  : 获取传感器值
 输入参数  : uint8_t channel  指定adc通道
             int32_t offset   零位偏移值
             float ratio      电压转传感器值系数
 输出参数  : float            传感器值
 作    者  :  
 日    期  : 2021年4月15日
*****************************************************************************/
float AdcTmpGetSensorValue(uint8_t channel)
{
    float lTmp = CVT_RATIO;
    uint16_t lValue;
    //中值滤波
    if(0 == ADCTmpGetMidAdcValue(channel, &lValue))
    {
        lTmp *= lValue;   //计算电压
        lTmp = (2 * lTmp + 2.36f) * 1000.0f / (28.82f - lTmp); //计算pt100阻值
        lTmp = CalculateTemperature(lTmp);  //计算温度值
        gAdcTmpLastValue[channel] = lTmp;
    }
    else
    {
        lTmp = gAdcTmpLastValue[channel];
    }    

    return lTmp;
}
/*****************************************************************************
 功能描述  : 读adc传感器值
 输入参数  : uint8_t* cmdData  命令数据
             uint8_t size      命令长度
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2021年4月15日
*****************************************************************************/
void ReadAdcSensorValue(uint8_t* cmdData, uint8_t size)
{
    float lSensorValue;

    if(cmdData[0] > ADC1_IN6)
    {
        return;
    }
    
    switch(cmdData[0])
    {
        case ADC1_IN4:
            lSensorValue = AdcGetSensorValue(cmdData[0], ADC1_OIL_OFFSET, ADC1_OIL_RATIO);
            break;
        case ADC1_IN5:
            lSensorValue = AdcGetSensorValue(cmdData[0], ADC_AIR_PRESSURE_OFFSET, ADC_AIR_PRESSURE_RATIO);
            break;
        case ADC1_IN6:
            lSensorValue = AdcGetSensorValue(cmdData[0], ADC_AIR_PRESSURE_OFFSET, ADC_AIR_PRESSURE_RATIO);
            break;
    }

    rt_kprintf("Adc channle%d: %.2f.\r\n", cmdData[0], lSensorValue);
}
/*****************************************************************************
 功能描述  : 温度传感器轮询线程
 输入参数  : void  
 输出参数  : 无
 作    者  : 刘鹏
 日    期  : 2022年8月12日
*****************************************************************************/
void ReadTmpProcess(void)
{
    float lfTmp;
    uint8_t lu8Tmp;
    
    lfTmp = AdcTmpGetSensorValue(gAdcTmpIndex);

    lfTmp = Limit(lfTmp, -40, 200) + 40;
    lu8Tmp = (uint8_t)lfTmp;

    if(ADC3_IN15 == gAdcTmpIndex)   //主控卡
    {
        gRespondState.moduleTmp[0] = lu8Tmp;
    }
    else if(ADC3_IN8 == gAdcTmpIndex)   //交换机
    {
        gRespondState.moduleTmp[1] = lu8Tmp;
    }
    else if(ADC3_IN14 == gAdcTmpIndex)  //电源板
    {
        gRespondState.moduleTmp[2] = lu8Tmp;
    }
    else if(ADC3_IN13 == gAdcTmpIndex)  //电机5/6
    {
        gRespondState.motorTmp[M_LEFT_ONE] = lu8Tmp;
    }
    else if(ADC3_IN6 == gAdcTmpIndex)   //电机5/6
    {
        gRespondState.motorTmp[M_RIGHT_ONE] = lu8Tmp;
    }
    else if(ADC3_IN12 == gAdcTmpIndex)  //电机1/2
    {
        gRespondState.motorTmp[M_LEFT] = lu8Tmp;
    }
    else if(ADC3_IN5 == gAdcTmpIndex)   //电机1/2
    {
        gRespondState.motorTmp[M_RIGHT] = lu8Tmp;
    }
    
    gAdcTmpIndex++;
    if(gAdcTmpIndex >= ADCTMP_MAX_CHANNELS)
    {
        gAdcTmpIndex = 0;
    }
}
/* USER CODE END 1 */
