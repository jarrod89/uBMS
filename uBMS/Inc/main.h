/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/
#define MIN_V (2.5)
#define MAX_V (3.655)
#define BRICK_V_HYST (0.01)
//(4.2)
#define BLEED_THRESHOLD (3.0)
#define OVT (65.0)
#define ACTIVE_CELLS {0,1,2,3,4, 6,7,8,9,10, 12,13,14,15, 16}
//current shunt
#define RSHUNT 0.001
#define AMP_GAIN 31.6
#define AMPS_PER_VOLT (1/(RSHUNT*AMP_GAIN))
#define AMP_OFFSET_ERROR (-0.0011)

#define ADC_RESOLUTION (0.0001)

//Inputs:
#define TEST_BTN_GPIO_PORT GPIOB
#define TEST_BTN_PIN GPIO_PIN_0
#define CLEAR_BTN_GPIO_PORT GPIOB
#define CLEAR_BTN_PIN GPIO_PIN_1
#define test_btn() HAL_GPIO_ReadPin(TEST_BTN_GPIO_PORT, TEST_BTN_PIN)
#define clear_btn() HAL_GPIO_ReadPin(CLEAR_BTN_GPIO_PORT, CLEAR_BTN_PIN)

//Outputs:
#define BMB_CS_GPIO_PORT GPIOA
#define BMB_CS_PIN GPIO_PIN_4

#define CAN_STBY_GPIO_PORT GPIOB
#define CAN_STBY_PIN GPIO_PIN_7

#define LED1_GPIO_PORT GPIOB
#define LED1_PIN GPIO_PIN_2
#define LED2_GPIO_PORT GPIOB
#define LED2_PIN GPIO_PIN_3
#define LED3_GPIO_PORT GPIOB
#define LED3_PIN GPIO_PIN_4
#define busContactor_GPIO_PORT GPIOA
#define busContactor_PIN GPIO_PIN_15
#define chargeContactor_GPIO_PORT GPIOA
#define chargeContactor_PIN GPIO_PIN_3
#define prechargeContactor_GPIO_PORT GPIOB
#define prechargeContactor_PIN GPIO_PIN_10
#define chargeSSR_GPIO_PORT GPIOB
#define chargeSSR_PIN GPIO_PIN_5
#define HIGH 1
#define LOW 0

#define MD 0x2

#define WRCFGA 0x0001
#define WRCFGB 0x0024
#define ADCVSC 0x0467
#define ADAX 0x0460
#define RDAUXA 0x000C
#define RDAUXB 0x000E
#define RDAUXC 0x000D
#define RDAUXD 0x000F
#define RDCVA 0x0004
#define RDCVB 0x0006
#define RDCVC 0x0008
#define RDCVD 0x000A
#define RDCVE 0x0009
#define RDCVF 0x000B
#define RDSTATA 0x0010
#define ADSTAT 0x0468
//Macros for timing and simple IO
#define cs_low() HAL_GPIO_WritePin(BMB_CS_GPIO_PORT, BMB_CS_PIN, LOW)
#define cs_high() HAL_GPIO_WritePin(BMB_CS_GPIO_PORT, BMB_CS_PIN, HIGH)

#define chargeContactor_off() HAL_GPIO_WritePin(chargeContactor_GPIO_PORT, chargeContactor_PIN, LOW)
#define chargeContactor_on() HAL_GPIO_WritePin(chargeContactor_GPIO_PORT, chargeContactor_PIN, HIGH)
#define busContactor_off() HAL_GPIO_WritePin(busContactor_GPIO_PORT, busContactor_PIN, LOW)
#define busContactor_on() HAL_GPIO_WritePin(busContactor_GPIO_PORT, busContactor_PIN, HIGH)
#define prechargeContactor_off() HAL_GPIO_WritePin(prechargeContactor_GPIO_PORT, prechargeContactor_PIN, LOW)
#define prechargeContactor_on() HAL_GPIO_WritePin(prechargeContactor_GPIO_PORT, prechargeContactor_PIN, HIGH)
#define chargeSSR_off() HAL_GPIO_WritePin(chargeSSR_GPIO_PORT, chargeSSR_PIN, LOW)
#define chargeSSR_on() HAL_GPIO_WritePin(chargeSSR_GPIO_PORT, chargeSSR_PIN, HIGH)

//systick is 100us
#define delay_u(_micro_) HAL_Delay((int)(_micro_/100))
#define delay_m(_milli_) HAL_Delay(_milli_*10)

#define SIZE_OF_ARRAY(_array) (sizeof(_array) / sizeof(_array[0]))
/* Private define ------------------------------------------------------------*/

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
