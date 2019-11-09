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
void set_speed(uint8_t motor, float target_speed);

#define freq_source 80000000 //internal clock source freq

uint16_t prescaler = 40; //prescaler used by timer
uint32_t freq_counter; //the freq of the timer calculated from freq_source and prescaler
uint16_t init_speed = 5000; //this sets the acceleration by setting the timing of first step, the smaller the number the faster the acceleration
uint16_t SPR = 3200; //steps per revolution of the stepper motor
float tick_freq[4]; //the freq that the steps need to be calculated from frq_counter RPM and SPR
float speed[4]; //the current speed measured by timer ticks in ARR value to count up to
float target_speed[4]; //the target speed that speed is accelerating towards

int32_t n[4];
uint8_t speed_up[4]; //boolean used to tell whether in speed up process or slow down
int8_t new_dir[4]; //the final direction desired
int8_t old_dir[4]; //the direction before
uint8_t second_time[4]; //boolean used to create double toggle of step pin i.e high/low


int main(void){
  SystemClock_Config();
  stepper_init();

  //test some speeds :)
  set_speed(0, 5);
  set_speed(1, 10);
  set_speed(2, 20);
  set_speed(3, 40);

  HAL_Delay(8000);

  set_speed(0, -100);
  set_speed(1, -100);
  set_speed(2, -100);
  set_speed(3, -100);

  HAL_Delay(8000);

  set_speed(0, 0);
  set_speed(1, 0);
  set_speed(2, 0);
  set_speed(3, 0);



  while (1)  {
  	  //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
	  //HAL_Delay(500);
  }
}


void stepper_init(){
	  MX_GPIO_Init();
	  MX_TIM3_Init();
	  MX_TIM7_Init();
	  MX_TIM5_Init();
	  MX_TIM4_Init();

	  freq_counter = freq_source / (prescaler + 1);


	  TIM3->PSC = prescaler;         // Set prescaler
	  TIM3->ARR = init_speed;           // Auto reload value
	  TIM3->DIER = TIM_DIER_UIE; // Enable update interrupt (timer level)
	  TIM3->CR1 |= TIM_CR1_CEN;   // Enable timer
	  NVIC_EnableIRQ(TIM3_IRQn); // Enable interrupt from TIM3 (NVIC level)
	  GPIOE->ODR &= ~GPIO_ODR_OD1; //set dir pin high



	  TIM4->PSC = 50;         // Set prescaler
	  TIM4->ARR = 10;           // Auto reload value
	  TIM4->DIER = TIM_DIER_UIE; // Enable update interrupt (timer level)
	  TIM4->CR1 = TIM_CR1_CEN;   // Enable timer
	  NVIC_EnableIRQ(TIM4_IRQn); // Enable interrupt(NVIC level)
	  GPIOE->ODR &= ~GPIO_ODR_OD3; //set dir pin high

	  TIM5->PSC = 50;         // Set prescaler
	  TIM5->ARR = 65535;           // Auto reload value
	  TIM5->DIER = TIM_DIER_UIE; // Enable update interrupt (timer level)
	  TIM5->CR1 = TIM_CR1_CEN;   // Enable timer
	  NVIC_EnableIRQ(TIM5_IRQn); // Enable interrupt(NVIC level)
	  GPIOE->ODR &= ~GPIO_ODR_OD5; //set dir pin high

	  TIM7->PSC = 50;         // Set prescaler
	  TIM7->ARR = 1;           // Auto reload value
	  TIM7->DIER = TIM_DIER_UIE; // Enable update interrupt (timer level)
	  TIM7->CR1 = TIM_CR1_CEN;   // Enable timer
	  NVIC_EnableIRQ(TIM7_IRQn); // Enable interrupt(NVIC level)
	  GPIOE->ODR &= ~GPIO_ODR_OD7; //set dir pin high
}


void set_speed(uint8_t motor, float RPM){

	if(RPM > 0){ //set dirction boolean
		new_dir[motor] = 1;
	}else if (RPM < 0) {
		new_dir[motor] = 0;
		RPM = -1 * RPM; //if RPM is negative then flip sign
	}

	if(RPM != 0){ //only enters this loop if desired RPM isn't zero

		tick_freq[motor]  = 2 * SPR * RPM / 60;
		target_speed[motor]  = freq_counter / tick_freq[motor] ;

		if(target_speed[motor]  > 65535){ //check that the desired RPM hasn't caused the ARR to overflow
			target_speed[motor]  = 65535;
		}

		if((speed[motor] == 0) && (target_speed[motor] !=0)){ //if stopped and want to start again
			speed[motor] = init_speed;
			n[motor] = 0;
		}

		if(target_speed[motor]  < speed[motor]){ //set boolean to either speed up or slow down
			speed_up[motor] = 1;
		}else {
			speed_up[motor] = 0;
		}

	}else{ //only does this if desired to stop
		target_speed[motor] = 0;
		speed_up[motor] = 0;
	}
}


void TIM3_IRQHandler(void){
	if(TIM3->SR & TIM_SR_UIF){ // if UIF flag is set
		TIM3->SR &= ~TIM_SR_UIF; // clear UIF flag

		if((speed[0] != 0) || (target_speed[0] != 0)){ //only enters this loop if not at stand still
			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_0); //toggle the step pin, it requires this to happen twice to complete 1 step i.e high/low

			if(second_time[0]){ //only enters this loop every second time that way the toggle happens twice to every 1 loop

				if(new_dir[0] == old_dir[0]){ //only enters this loop if current direction is same as desired direction

					if(((target_speed[0] > init_speed - 100) || (target_speed[0] == 0)) && (n[0] < 2)){
						speed[0] = target_speed[0]; //if desired speed is slow and current speed is slow then goto straight to it
					}else if ((speed[0] > init_speed - 100) && (target_speed[0] < init_speed - 100) && (speed_up[0]) && (target_speed[0] !=0)){
						speed[0] = init_speed; //if current speed is slow and desired speed is fast then goto start acceleration speed
						n[0] = 0;
					}

					if ((speed_up[0]) && (speed[0] > target_speed[0])){ //enter this loop if speeding up
						n[0]++;
						speed[0] = speed[0] - ( (2 * speed[0]) / (4 * n[0] + 1) );
					}else if ((!speed_up[0]) && ((speed[0] < target_speed[0]) || target_speed[0] == 0)){ //enter this loop if slowing down
						n[0]--;
						speed[0] = (speed[0] * (4 * n[0] + 1) / (4 * n[0] - 1));
					}

				}else{ //only enters this loop if current direction is different to desired direction

					if(speed[0] == 65000){ //if speed is super slow then toggle dir pin and set speed_up boolean to speed up
						HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_1);
						old_dir[0] = new_dir[0];
						speed_up[0] = 1;

					}else if(n[0] < 2){ //if speed is slower than acceleration speed then set speed to super slow
						speed[0] = 65000;

					}else{ //onyl enters this loop if going fast to just decelerate
						n[0]--;
						speed[0] = (speed[0] * (4 * n[0] + 1) / (4 * n[0] - 1));
					}
				}

				if(speed[0] > 65535){ //check to make sure speed won't over the ARR
					speed[0] = 65535;
				}

				if (speed[0] != 0){ //if speed isn't zero then update ARR
					TIM3->ARR = (uint32_t)speed[0];//update ARR
				}
			}
			second_time[0] = !second_time[0]; //flip second_time boolean

		}
	}
}


void TIM4_IRQHandler(void){
if(TIM4->SR & TIM_SR_UIF){ // if UIF flag is set
  TIM4->SR &= ~TIM_SR_UIF; // clear UIF flag

  if((speed[1] != 0) || (target_speed[1] != 0)){ //only enters this loop if not at stand still
  			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_2); //toggle the step pin, it requires this to happen twice to complet 1 step i.e high/low

  			if(second_time[1]){ //only enters this loop every second time that way the toggle happens twice to every 1 loop

  				if(new_dir[1] == old_dir[1]){ //only enters this loop if current direction is same as desired direction

  					if(((target_speed[1] > init_speed - 100) || (target_speed[1] == 0)) && (n[1] < 2)){
  						speed[1] = target_speed[1]; //if desired speed is slow and current speed is slow then goto striahgt to it
  					}else if ((speed[1] > init_speed - 100) && (target_speed[1] < init_speed - 100) && (speed_up[1]) && (target_speed[1] !=0)){
  						speed[1] = init_speed; //if current speed is slow and desired speed is fast then goto start acceleration speed
  						n[1] = 0;
  					}

  					if ((speed_up[1]) && (speed[1] > target_speed[1])){ //enter this loop if speeding up
  						n[1]++;
  						speed[1] = speed[1] - ( (2 * speed[1]) / (4 * n[1] + 1) );
  					}else if ((!speed_up[1]) && ((speed[1] < target_speed[1]) || target_speed[1] == 0)){ //enter this loop if slowing down
  						n[1]--;
  						speed[1] = (speed[1] * (4 * n[1] + 1) / (4 * n[1] - 1));
  					}

  				}else{ //only enters this loop if current direction is different to desired direction

  					if(speed[1] == 65000){ //if speed is super slow then toggle dir pin and set speed_up boolean to speed up
  						HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_3);
  						old_dir[1] = new_dir[1];
  						speed_up[1] = 1;

  					}else if(n[1] < 2){ //if speed is slower than acceleration speed then set speed to super slow
  						speed[1] = 65000;

  					}else{ //onyl enters this loop if going fast to just decelerate
  						n[1]--;
  						speed[1] = (speed[1] * (4 * n[1] + 1) / (4 * n[1] - 1));
  					}
  				}

  				if(speed[1] > 65535){ //check to make sure speed won't over the ARR
  					speed[1] = 65535;
  				}

  				if (speed[1] != 0){ //if speed isn't zero then update ARR
  					TIM4->ARR = (uint32_t)speed[1];//update ARR
  				}
  			}
  			second_time[1] = !second_time[1]; //flip second_time boolean

  		}

  }
}


void TIM5_IRQHandler(void){
if(TIM5->SR & TIM_SR_UIF){ // if UIF flag is set
  TIM5->SR &= ~TIM_SR_UIF; // clear UIF flag

  if((speed[2] != 0) || (target_speed[2] != 0)){ //only enters this loop if not at stand still
  			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_4); //toggle the step pin, it requires this to happen twice to complet 1 step i.e high/low

  			if(second_time[2]){ //only enters this loop every second time that way the toggle happens twice to every 1 loop

  				if(new_dir[2] == old_dir[2]){ //only enters this loop if current direction is same as desired direction

  					if(((target_speed[2] > init_speed - 100) || (target_speed[2] == 0)) && (n[2] < 2)){
  						speed[2] = target_speed[2]; //if desired speed is slow and current speed is slow then goto straight to it
  					}else if ((speed[2] > init_speed - 100) && (target_speed[2] < init_speed - 100) && (speed_up[2]) && (target_speed[2] !=0)){
  						speed[2] = init_speed; //if current speed is slow and desired speed is fast then goto start acceleration speed
  						n[2] = 0;
  					}

  					if ((speed_up[2]) && (speed[2] > target_speed[2])){ //enter this loop if speeding up
  						n[2]++;
  						speed[2] = speed[2] - ( (2 * speed[2]) / (4 * n[2] + 1) );
  					}else if ((!speed_up[2]) && ((speed[2] < target_speed[2]) || target_speed[2] == 0)){ //enter this loop if slowing down
  						n[2]--;
  						speed[2] = (speed[2] * (4 * n[2] + 1) / (4 * n[2] - 1));
  					}

  				}else{ //only enters this loop if current direction is different to desired direction

  					if(speed[2] == 65000){ //if speed is super slow then toggle dir pin and set speed_up boolean to speed up
  						HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_5);
  						old_dir[2] = new_dir[2];
  						speed_up[2] = 1;

  					}else if(n[2] < 2){ //if speed is slower than acceleration speed then set speed to super slow
  						speed[2] = 65000;

  					}else{ //onyl enters this loop if going fast to just decelerate
  						n[2]--;
  						speed[2] = (speed[2] * (4 * n[2] + 1) / (4 * n[2] - 1));
  					}
  				}

  				if(speed[2] > 65535){ //check to make sure speed won't over the ARR
  					speed[2] = 65535;
  				}

  				if (speed[2] != 0){ //if speed isn't zero then update ARR
  					TIM5->ARR = (uint32_t)speed[2];//update ARR
  				}
  			}
  			second_time[2] = !second_time[2]; //flip second_time boolean

  		}

  }
}


void TIM7_IRQHandler(void){
if(TIM7->SR & TIM_SR_UIF){ // if UIF flag is set
  TIM7->SR &= ~TIM_SR_UIF; // clear UIF flag

  if((speed[3] != 0) || (target_speed[3] != 0)){ //only enters this loop if not at stand still
  			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_6); //toggle the step pin, it requires this to happen twice to complet 1 step i.e high/low

  			if(second_time[3]){ //only enters this loop every second time that way the toggle happens twice to every 1 loop

  				if(new_dir[3] == old_dir[3]){ //only enters this loop if current direction is same as desired direction

  					if(((target_speed[3] > init_speed - 100) || (target_speed[3] == 0)) && (n[3] < 2)){
  						speed[3] = target_speed[3]; //if desired speed is slow and current speed is slow then goto striahgt to it
  					}else if ((speed[3] > init_speed - 100) && (target_speed[3] < init_speed - 100) && (speed_up[3]) && (target_speed[3] !=0)){
  						speed[3] = init_speed; //if current speed is slow and desired speed is fast then goto start acceleration speed
  						n[3] = 0;
  					}

  					if ((speed_up[3]) && (speed[3] > target_speed[3])){ //enter this loop if speeding up
  						n[3]++;
  						speed[3] = speed[3] - ( (2 * speed[3]) / (4 * n[3] + 1) );
  					}else if ((!speed_up[3]) && ((speed[3] < target_speed[3]) || target_speed[3] == 0)){ //enter this loop if slowing down
  						n[3]--;
  						speed[3] = (speed[3] * (4 * n[3] + 1) / (4 * n[3] - 1));
  					}

  				}else{ //only enters this loop if current direction is different to desired direction

  					if(speed[3] == 65000){ //if speed is super slow then toggle dir pin and set speed_up boolean to speed up
  						HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_7);
  						old_dir[3] = new_dir[3];
  						speed_up[3] = 1;

  					}else if(n[3] < 2){ //if speed is slower than acceleration speed then set speed to super slow
  						speed[3] = 65000;

  					}else{ //onyl enters this loop if going fast to just decelerate
  						n[3]--;
  						speed[3] = (speed[3] * (4 * n[3] + 1) / (4 * n[3] - 1));
  					}
  				}

  				if(speed[3] > 65535){ //check to make sure speed won't over the ARR
  					speed[3] = 65535;
  				}

  				if (speed[3] != 0){ //if speed isn't zero then update ARR
  					TIM7->ARR = (uint32_t)speed[3];//update ARR
  				}
  			}
  			second_time[3] = !second_time[3]; //flip second_time boolean

  		}
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
