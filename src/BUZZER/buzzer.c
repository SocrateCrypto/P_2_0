#include "BUZZER/buzzer.h" // include header file


/* USER CODE BEGIN Includes */
#include <buzzer.h>
/* USER CODE END Includes */

/* USER CODE BEGIN PV */

typedef struct BuzzerDebug
{
	uint16_t Tout;
	uint16_t ToutSave;
	uint8_t Tick;
	uint8_t Help;
	   int8_t run; // 0 - off, 1 - on, 2 - toggle
}buzdebug_t;

buzdebug_t Buz = {0, 0, 0, 0,0};

/* USER CODE END PV */

/**
  * @brief Initialize timer with PWM-output
  * @note The timer handler and channel should be defined in .h file
  * @retval No
  */
void BUZZER_Init(void)
{
#if(BUZZER_PASSIVE)
	  HAL_TIM_PWM_Start(TIM_BUZZER_HANDLER, TIM_BUZZER_CHANNEL);
#endif
}

/**
  * @brief Turn on buzzer by PWM signal
  * @note No need init
  * @param 0%-100%
  * @retval No
  */
void BUZZER_PWM_SetPercent(uint8_t percent)
{
	/* Check range */
	if(percent > 100){
		percent = 100;
	}
#if(BUZZER_PASSIVE)

	uint16_t tim_arr = __HAL_TIM_GET_AUTORELOAD(TIM_BUZZER_HANDLER);

	uint16_t tim_ccr_new = (percent*tim_arr*0.01);

	__HAL_TIM_SET_COMPARE(TIM_BUZZER_HANDLER, TIM_BUZZER_CHANNEL, tim_ccr_new);
#endif

}

/**
  * @brief Turn on buzzer
  * @note No need init
  * @param No
  * @retval No
  */
void BUZZER_On(void)
{

#if(BUZZER_PASSIVE)

	BUZZER_PWM_SetPercent(50);

#else

#if(BUZZER_ACTIVE_HIGH)
	/* For buzzer with generator,  High Level = BuzzerOn */
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
	   Buz.run=1; // Set run state to on
#else
	/* For buzzer with generator,  Low Level = BuzzerOn */
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
#endif
#endif

}

/**
  * @brief Turn off buzzer
  * @note No need init
  * @param No
  * @retval No
  */
void BUZZER_Off(void)
{
#if(BUZZER_PASSIVE)

	BUZZER_PWM_SetPercent(0);

#else

#if(BUZZER_ACTIVE_HIGH)
	/* For buzzer with generator,  Low Level = BuzzerOff */
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
	   Buz.run=0; // Set run state to off
#else
	/* For buzzer with generator,  High Level = BuzzerOff */
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
#endif
#endif
}

/**
  * @brief Toggle buzzer state
  * @note No need init
  * @param No
  * @retval No
  */


/**
  * @brief Disable buzzer
  * @note No need init
  * @param time between ticks in ms, ticks quantity
  * @retval No
  */
void BUZZER_Go(buztime_t period, buztick_t ticks)
{
	BUZZER_On();
	Buz.Help = 0;
	Buz.ToutSave = period;
	Buz.Tout = period;
	Buz.Tick = ticks;
}


/**
  * @brief Disable buzzer
  * @note Insert func.in the 1kHz interrupt handler (e.g. SysTick)
  * @param No
  * @retval No
  */
void BUZZER_Handler(void)
{
	if(Buz.Tick)
	{
		if(--Buz.Tout <= 0)
		{
			Buz.Tick--;

			if(Buz.Help)
			{
				BUZZER_On();
			}
			else
			{
				BUZZER_Off();
			}

			Buz.Tout = Buz.ToutSave;

			/* Toggle */
			Buz.Help = ~Buz.Help;
		}
	}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM1) {
		if (Buz.run == 1) // Check if buzzer is running
		{
			 HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8); // Toggle the state of pin PB0
		} 
		
      
       
     }
 }
