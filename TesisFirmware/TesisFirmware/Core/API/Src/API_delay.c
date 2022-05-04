/**
 ******************************************************************************
 * @file    UART/UART_Printf/Src/main.c
 * @author  MCD Application Team
 * @brief   This example shows how to retarget the C library printf function
 *          to the UART.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/

#include "API_delay.h"


/** @addtogroup STM32F4xx_HAL_Examples
 * @{
 */

/** @addtogroup UART_Printf
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/





/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* UART handler declaration */
/*UART_HandleTypeDef UartHandle;*/

/* Private function prototypes -----------------------------------------------*/



/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Main program
 * @param  None
 * @retval None
 */

void delayInit( delay_t * delay, tick_t duration )
{

	if(delay != NULL )
	{
		delay->duration = duration;
	    delay->running = false;
	}
}

bool_t delayRead( delay_t * delay )
{

	bool_t tiempo_cumplido = false;
	if(delay != NULL )
	{
	     if(!delay->running)
	     {
		     delay->startTime = HAL_GetTick();
		     delay->running = 1;
	      }
		 if (((tick_t)HAL_GetTick() - delay->startTime) >= delay->duration)
		 {
			 tiempo_cumplido = true;
		     delay->running= 0;
		 }
	}
	return tiempo_cumplido;
}

void delayWrite( delay_t * delay, tick_t duration )
{

	if(delay != NULL )
    {  //delay->startTime = HAL_GetTick();
	     delay->duration = duration;
    }
}




/**
 * @brief  System Clock Configuration
 *         The system Clock is configured as follow :
 *            System Clock source            = PLL (HSE)
 *            SYSCLK(Hz)                     = 180000000
 *            HCLK(Hz)                       = 180000000
 *            AHB Prescaler                  = 1
 *            APB1 Prescaler                 = 4
 *            APB2 Prescaler                 = 2
 *            HSE Frequency(Hz)              = 8000000
 *            PLL_M                          = 8
 *            PLL_N                          = 360
 *            PLL_P                          = 2
 *            PLL_Q                          = 7
 *            PLL_R                          = 2
 *            VDD(V)                         = 3.3
 *            Main regulator output voltage  = Scale1 mode
 *            Flash Latency(WS)              = 5
 * @param  None
 * @retval None
 */

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1)
	{
	}
}
#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
