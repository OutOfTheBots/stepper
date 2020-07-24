#include "main.h"
#include "stm32f4xx_hal.h"

void SystemClock_Config(void);
static void MX_GPIO_Init(void);

#define freq_source 80000000 //internal clock source freq

uint16_t prescaler = 40; //prescaler used by timer
uint32_t freq_counter; //the freq of the timer calculated from freq_source and prescaler
uint16_t init_speed = 10000; //this sets the acceleration by setting the timing of first step, the smaller the number the faster the acceleration
uint16_t SPR = 3200; //steps per revolution of the stepper motor
float tick_freq; //the freq that the steps need to be calculated from frq_counter RPM and SPR
float speed; //the current speed measured by timer ticks in ARR value to count up to
float target_speed; //the target speed that speed is accelerating towards

int32_t n;
int8_t curret_dir, target_dir, RPM_zero;


void stepper_setup(void){

	freq_counter = freq_source / (prescaler + 1);


	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN; //enable port D clock
	GPIOD->MODER |= GPIO_MODER_MODER12_1; //setup pin 12 on port d to AF mode
	GPIOD->AFR[1] = (GPIOD->AFR[1] & (0b1111<<(4*(12-8))) | 0b0010<<(4*(12-8))); //setup pin 12 on port D to AF timer 2-5
	GPIOD->MODER |= GPIO_MODER_MODE11_0 | GPIO_MODER_MODE10_0; //setup pin 11 and 13 as output for dir and EN pins
	GPIOD->ODR &= ~GPIO_ODR_OD11; //set direction pin

	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN; //enable the timer4 clock
	TIM4->CR1 &= ~TIM_CR1_CEN; //disable channel 1.
	TIM4->PSC = prescaler;   //set prescale
	TIM4->CCMR1 = (TIM4->CCMR1 & ~(0b110<<4)) | (0b110<<4); //set PWM mode 110
	TIM4->CCR1 = 10; //set to min rise time
	TIM4->ARR = init_speed; //set to timing
	TIM4->CCER |= TIM_CCER_CC1E; //enable output to pin.
	TIM4->CR1 |= TIM_CR1_ARPE; //buffer ARR
	TIM4->DIER |= TIM_DIER_UIE; //enable interupt

	NVIC_EnableIRQ(TIM4_IRQn); // Enable interrupt(NVIC level)

	//initialize variables
	speed = init_speed;
	RPM_zero = 1;
	curret_dir = 1;
	target_dir =1;
}

void disable_steppers(void){
	GPIOD->ODR &= ~GPIO_ODR_OD10; //set enable pin low
}

void enable_steppers(void){
	GPIOD->ODR |= GPIO_ODR_OD10; //set enable pin high
}

void set_speed(float RPM){

	if(RPM==0){
		RPM_zero = 1;
		target_speed = init_speed;
	}else{
		RPM_zero = 0;
		if(RPM>0)target_dir = 1;
		else{
			target_dir = 0;
			RPM = RPM *-1;
		}
		tick_freq = SPR * RPM / 60;
		target_speed = freq_counter / tick_freq;
	}
	TIM4->CR1 |= TIM_CR1_CEN; //enable channel 1.
}





int main(void){

  HAL_Init();
  SystemClock_Config();

  stepper_setup();



  enable_steppers();


  set_speed(400);
  HAL_Delay(3000);

  set_speed(-210);
  HAL_Delay(3000);

  set_speed(1);
  HAL_Delay(3000);

  set_speed(300);
  HAL_Delay(3000);


  disable_steppers();


  while (1){


  }
}


void TIM4_IRQHandler(void){

	TIM4->SR &= ~TIM_SR_UIF; // clear UIF flag


	if (RPM_zero && (speed >= init_speed))TIM4->CR1 &= ~TIM_CR1_CEN; //disable channel 1.

	if(target_dir == curret_dir){
		if (speed>=init_speed){
			speed = init_speed;
			n=0;
		}
		if((target_speed >= init_speed) && (speed >= init_speed)){
			speed = target_speed;
			n=0;
		}else if(speed>target_speed){
					n++;
					speed = speed - ( (2 * speed) / (4 * n + 1) );
		  	  }else{
		  		  n--;
		  		  speed = (speed * (4 * n + 1) / (4 * n - 1));
		  	  }

	}else{
		if(speed > init_speed-100){
			if(target_dir)GPIOD->ODR &= ~GPIO_ODR_OD11; //set direction pin
			else GPIOD->ODR |= GPIO_ODR_OD11; //set direction pin
			curret_dir = target_dir;
			speed = init_speed;
		}else{
			n--;
			speed = (speed * (4 * n + 1) / (4 * n - 1));
		}
	}
	TIM4->ARR = (uint32_t)speed;//update ARR
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
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

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();

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
