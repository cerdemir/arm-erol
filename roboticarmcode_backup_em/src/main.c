/*
**
**                           Main.c
**
**
**********************************************************************/
/*
   Last committed:     $Revision: 00 $
   Last changed by:    $Author: $
   Last changed date:  $Date:  $
   ID:                 $Id:  $

**********************************************************************/
#include "stm32f4xx_conf.h"

void Init()
{

    TIM_TimeBaseInitTypeDef TIM_BaseStruct;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    //timer_tick_frequency = Timer_default_frequency / (prescaller_set + 1)
    TIM_BaseStruct.TIM_Prescaler = 0;
    TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
     //TIM_Period = timer_tick_frequency / PWM_frequency - 1
    TIM_BaseStruct.TIM_Period = 3359;//0xFFFFFFFF;//3359; /* 50kHz PWM */
    TIM_BaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_BaseStruct.TIM_RepetitionCounter = 0;

    TIM_TimeBaseInit(TIM1, &TIM_BaseStruct);
    TIM_Cmd(TIM1, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    GPIO_PinAFConfig (GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);
    GPIO_PinAFConfig (GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);
    // set them up as encoder inputs
    // set both inputs to rising polarity to let it use both edges
    TIM_EncoderInterfaceConfig (TIM1, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_SetAutoreload (TIM1, 0xffff);
    TIM_ITConfig(TIM1,TIM_IT_Update , ENABLE);
}
int main(void)
{
    int i;
    Init();

    while(1)
    {

    i = 100000;
        while(i--);
        printf("TIM1 Counter :%d - %d\n", TIM_GetCounter (TIM1),TIM1->CNT);

    }
}
