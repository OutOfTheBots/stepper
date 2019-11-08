#include "main.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"


TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim7;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM4_Init(void);

void stepper_init();
void set_speed(float target_speed);

uint32_t freq_source = 80000000;

uint16_t prescaler = 40;
uint32_t freq_counter;
int32_t n = 0;
uint16_t init_speed = 10000;
float speed;
float max_speed = 0;
float RPM;
float tick_freq;
uint32_t change;
uint8_t speed_up;
int8_t new_dir=1;
int8_t old_dir=1;
uint8_t second_time = 0;

int main(void){

  SystemClock_Config();

  stepper_init();


  set_speed(100);
  HAL_Delay(2000);

  set_speed(0.1);
  HAL_Delay(2000);

  set_speed(200);
  HAL_Delay(3000);

  set_speed(-200);
  HAL_Delay(4000);

  set_speed(100);


  while (1)  {
  	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
	  HAL_Delay(1);
  }
}





void stepper_init(){
	  MX_GPIO_Init();
	  MX_TIM3_Init();
	  MX_TIM7_Init();
	  MX_TIM5_Init();
	  MX_TIM4_Init();

	  speed = init_speed;
	  freq_counter = freq_source / (prescaler + 1);

	  RCC->AHB1ENR |= RCC_APB1ENR_TIM3EN;
	  TIM3->PSC = prescaler;         // Set prescaler
	  TIM3->ARR = init_speed;           // Auto reload value
	  TIM3->DIER = TIM_DIER_UIE; // Enable update interrupt (timer level)
	  TIM3->CR1 |= TIM_CR1_CEN;   // Enable timer
	  //NVIC_EnableIRQ(TIM3_IRQn); // Enable interrupt from TIM3 (NVIC level)
	  GPIOE->ODR |= GPIO_ODR_OD1; //set dir pin high



	  TIM4->PSC = 50;         // Set prescaler
	  TIM4->ARR = 10;           // Auto reload value
	  TIM4->DIER = TIM_DIER_UIE; // Enable update interrupt (timer level)
	  TIM4->CR1 = TIM_CR1_CEN;   // Enable timer
	  //NVIC_EnableIRQ(TIM4_IRQn); // Enable interrupt(NVIC level)

	  TIM5->PSC = 50;         // Set prescaler
	  TIM5->ARR = 65535;           // Auto reload value
	  TIM5->DIER = TIM_DIER_UIE; // Enable update interrupt (timer level)
	  TIM5->CR1 = TIM_CR1_CEN;   // Enable timer
	  //NVIC_EnableIRQ(TIM5_IRQn); // Enable interrupt(NVIC level)

	  TIM7->PSC = 50;         // Set prescaler
	  TIM7->ARR = 1;           // Auto reload value
	  TIM7->DIER = TIM_DIER_UIE; // Enable update interrupt (timer level)
	  TIM7->CR1 = TIM_CR1_CEN;   // Enable timer
	  //NVIC_EnableIRQ(TIM7_IRQn); // Enable interrupt(NVIC level)
}

void set_speed(float target_speed){
	if(target_speed > 0){
		new_dir = 1;
	}else{
		new_dir = 0;
		target_speed = -1 * target_speed;
	}

	tick_freq = 2 * 3200 * target_speed / 60;
	max_speed = freq_counter / tick_freq;

	if(max_speed > 65535){
		max_speed = 65535;
	}


	if(max_speed < speed){
		speed_up = 1;
	}else {
		speed_up = 0;
	}
}

void TIM3_IRQHandler(void){
	if(TIM3->SR & TIM_SR_UIF){ // if UIF flag is set
		TIM3->SR &= ~TIM_SR_UIF; // clear UIF flag

		if(max_speed != 0){
			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_0);
			if(second_time){

				if(new_dir == old_dir){
					if((max_speed > init_speed - 100) && (speed > init_speed - 100)){
						speed = max_speed;
					}else if ((speed > init_speed - 100) && (max_speed < init_speed - 100) && (speed_up)){
						speed = init_speed;
						n = 0;
					}

					if ((speed_up) && (speed > max_speed)){
						n++;
						speed = speed - ( (2 * speed) / (4 * n + 1) );
					}else if ((!speed_up) && (speed < max_speed)){
						n--;
						speed = (speed * (4 * n + 1) / (4 * n - 1));
					}
				}else{
					if(speed == 65000){
						HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_1);
						old_dir = new_dir;
						speed_up = 1;
					}else if(speed > init_speed - 100){
						speed = 65000;

					}else{
						n--;
						speed = (speed * (4 * n + 1) / (4 * n - 1));
					}
				}

				if(speed > 65535){
					speed = 65535;
				}

				TIM3->ARR = (uint32_t)speed;//update ARR
			}
			second_time = !second_time;

		}   	//GPIOE->ODR &= ~GPIO_ODR_OD0;//set step pin low
	}
}

void TIM4_IRQHandler(void){
if(TIM4->SR & TIM_SR_UIF){ // if UIF flag is set
  TIM4->SR &= ~TIM_SR_UIF; // clear UIF flag
  //GPIOE->ODR &= ~GPIO_ODR_OD3;
  //GPIOE->ODR |= GPIO_ODR_OD3;
  HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_3);

  }
}

void TIM5_IRQHandler(void){
if(TIM5->SR & TIM_SR_UIF){ // if UIF flag is set
  TIM5->SR &= ~TIM_SR_UIF; // clear UIF flag

  GPIOE->ODR |= GPIO_ODR_OD5;
  GPIOE->ODR &= ~GPIO_ODR_OD5;

  }
}

void TIM7_IRQHandler(void){
if(TIM7->SR & TIM_SR_UIF){ // if UIF flag is set
  TIM7->SR &= ~TIM_SR_UIF; // clear UIF flag

  GPIOE->ODR |= GPIO_ODR_OD7;
  GPIOE->ODR &= ~GPIO_ODR_OD7;

  }
}


/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM5 init function */
static void MX_TIM5_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 0;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}


/* TIM7 init function */
static void MX_TIM7_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 0;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 0;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE3 PE4 PE5
                           PE6 PE7 PE0 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
