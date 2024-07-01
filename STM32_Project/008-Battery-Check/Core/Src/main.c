/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "M8N.h"
#include "FS-iA10B.h"
/* extern
 * bu, main.c'nin stm32f4xx_it.c dosyasında bildirilen iki global değişkene erişmesine izin verecektir.
 */
//telemetri
extern uint8_t uart6_rx_flag;
extern uint8_t uart6_rx_data;
//GPS
extern uint8_t m8n_rxbuff[36];
extern uint8_t m8n_rx_cplt_flag;
// kumanda receiver
extern uint8_t uart5_rx_flag;
extern uint8_t uart5_rx_data;
// kumanda receiver
extern uint8_t ibus_rxbuff[32];
extern uint8_t ibus_rx_cplt_flag;
/* extern
 * bu, main.c'nin stm32f4xx_it.c dosyasında bildirilen iki global değişkene erişmesine izin verecektir.
 */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
int _write(int file, char* p,int len)
{
	for(int i=0;i<len;i++)
	{
		while(!LL_USART_IsActiveFlag_TXE(USART6));
		LL_USART_TransmitData8(USART6, *(p+i));
	}
}
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int Is_iBus_Throttle_Min(void);
void ESC_Calibration(void);
int Is_iBus_Received(void);


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	unsigned short adcVal;
	float batVolt;


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
  LL_TIM_EnableCounter(TIM3);

//  LL_TIM_CC_EnableChannel(TIM3,LL_TIM_CHANNEL_CH4); // buzzer enable
//  TIM3->PSC = 2000;
//  HAL_Delay(100);
//  TIM3->PSC = 1500; 							//burayı kopyaladı nedenini anlamadım
//  HAL_Delay(100);
//  TIM3->PSC = 1000;
//  HAL_Delay(100);
//
//  LL_TIM_CC_DisableChannel(TIM3,LL_TIM_CHANNEL_CH4);  // buzzer disable

  LL_USART_EnableIT_RXNE(USART6); //interrupt olarak cagiracagiz
  LL_USART_EnableIT_RXNE(UART4); //interrupt olarak cagiracagiz
  LL_USART_EnableIT_RXNE(UART5); //interrupt olarak cagiracagiz

  M8N_Initialization();

  LL_TIM_EnableCounter(TIM5);
  LL_TIM_CC_EnableChannel(TIM5,LL_TIM_CHANNEL_CH1);
  LL_TIM_CC_EnableChannel(TIM5,LL_TIM_CHANNEL_CH2);
  LL_TIM_CC_EnableChannel(TIM5,LL_TIM_CHANNEL_CH3);
  LL_TIM_CC_EnableChannel(TIM5,LL_TIM_CHANNEL_CH4);

  HAL_ADC_Start_DMA(&hadc1, &adcVal, 1);
//
//  adcVal= ADC1->DR;

  while(Is_iBus_Received() == 0){
	  LL_TIM_CC_EnableChannel(TIM3,LL_TIM_CHANNEL_CH4); // buzzer enable
	     TIM3->PSC = 3000;
	     HAL_Delay(200);
	     LL_TIM_CC_DisableChannel(TIM3,LL_TIM_CHANNEL_CH4);  // b
	     HAL_Delay(200);
  }

  if(iBus.SwC == 2000){
	 LL_TIM_CC_EnableChannel(TIM3,LL_TIM_CHANNEL_CH4); // buzzer enable
	 TIM3->PSC = 1500;
	 HAL_Delay(200);
	 TIM3->PSC = 2000;
	 HAL_Delay(200);
	 TIM3->PSC = 1500;
	 HAL_Delay(200);
	 TIM3->PSC = 2000;
	 HAL_Delay(200);
	 LL_TIM_CC_DisableChannel(TIM3,LL_TIM_CHANNEL_CH4);  // b
	 HAL_Delay(200);

  ESC_Calibration();
  while(iBus.SwC != 1000){
	  Is_iBus_Received();

	  LL_TIM_CC_EnableChannel(TIM3,LL_TIM_CHANNEL_CH4); // buzzer enable

	 	 TIM3->PSC = 1500;
	 	 HAL_Delay(200);
	 	 TIM3->PSC = 2000;
	 	 HAL_Delay(200);

	 	 LL_TIM_CC_DisableChannel(TIM3,LL_TIM_CHANNEL_CH4);  // b

  	  }
  }
   while(Is_iBus_Throttle_Min() == 0){
	   LL_TIM_CC_EnableChannel(TIM3,LL_TIM_CHANNEL_CH4); // buzzer enable
	 	     TIM3->PSC = 1000;
	 	     HAL_Delay(70);
	 	     LL_TIM_CC_DisableChannel(TIM3,LL_TIM_CHANNEL_CH4);  // b
	 	     HAL_Delay(70);
   }





   LL_TIM_CC_EnableChannel(TIM3,LL_TIM_CHANNEL_CH4); // buzzer enable
   TIM3->PSC = 2000;
   HAL_Delay(100);
   TIM3->PSC = 1500; 							//burayı kopyaladı nedenini anlamadım
   HAL_Delay(100);
   TIM3->PSC = 1000;
   HAL_Delay(100);

   LL_TIM_CC_DisableChannel(TIM3,LL_TIM_CHANNEL_CH4);  // buzzer disable

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  batVolt=adcVal * 0.003619f;
	  printf("%d\t%.2f\n", adcVal,batVolt);
	  if(batVolt <10.0f){
		  TIM3->PSC = 1000;
		  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
	  }
	  else{
		   LL_TIM_CC_DisableChannel(TIM3,LL_TIM_CHANNEL_CH4);

	  }
	  HAL_Delay(100);

	 // LL_GPIO_TogglePin(GPIOD, LL_GPIO_PIN_14 | LL_GPIO_PIN_15);
	 // LL_USART_TransmitData8(USART6, 'A');
	  //HAL_Delay(1000);
/*
	  if(uart5_rx_flag == 1)
	 	  {
	 		  uart5_rx_flag = 0;
	 		  LL_USART_TransmitData8(USART6, uart5_rx_data); //alınan verilerin doğru olup olmadığını kontrol etmek için pc'ye geri gönderelim
	 	  }
	 	  */
	  /*

	  if(uart6_rx_flag == 1)
	  {
		  uart6_rx_flag = 0;
		  LL_USART_TransmitData8(USART6, uart6_rx_data); //alınan verilerin doğru olup olmadığını kontrol etmek için pc'ye geri gönderelim
	  }


	  */

	  /*
	  if(m8n_rx_cplt_flag == 1)
	  {
		  m8n_rx_cplt_flag = 0;

		  if(M8N_UBX_CHKSUM_Check(&m8n_rxbuff[0],36)==1)
		  {
			  LL_GPIO_TogglePin(GPIOD, LL_GPIO_PIN_14);
			  M8N_UBX_NAV_POSLLH_Parsing(&m8n_rxbuff[0],&posllh);
			  //printf(" LAT: %d\t\n LON:%d\t\n Height: %d\n",posllh.lat,posllh.lon,posllh.height);
			  printf(" LAT: %d\t\n LON:%d\t\n Height: %d\n",posllh.lat,posllh.lon,posllh.height);
			  //printf("LAT: %d\t",posllh.lat);

		  }

	  }

	  */

	  if(ibus_rx_cplt_flag == 1)
	  	  {
	  		  ibus_rx_cplt_flag = 0;
	  		  if(iBus_Check_CHKSUM(&ibus_rxbuff[0],32)==1)
	  		  {
	  			LL_GPIO_TogglePin(GPIOD, LL_GPIO_PIN_14);
	  			iBus_Parsing(&ibus_rxbuff[0], &iBus);
	  			if(iBus_isActiveFailSafe(&iBus) ==1)
	  			{
	  				LL_TIM_CC_EnableChannel(TIM3,LL_TIM_CHANNEL_CH4);
	  			}
	  			else
	  			{
	  				LL_TIM_CC_DisableChannel(TIM3,LL_TIM_CHANNEL_CH4);
	  			}
//	  			printf("%d\t %d\t %d\t %d\t %d\t %d\n",
//	  					iBus.RH,iBus.RV,iBus.LV,iBus.LH,iBus.SwA,iBus.SwC);
//	  			HAL_Delay(100);
	  		  }

	  	  }

	  TIM5->CCR1 = 10500 +(iBus.LV -1000)*10.5;
	  TIM5->CCR2 = 10500 +(iBus.LV -1000)*10.5;
	  TIM5->CCR3 = 10500 +(iBus.LV -1000)*10.5;
	  TIM5->CCR4 = 10500 +(iBus.LV -1000)*10.5;

//	  switch (uart6_rx_data) {
//		case '0': LL_GPIO_TogglePin(GPIOD, LL_GPIO_PIN_14 | LL_GPIO_PIN_15); break;
//		case '1':  LL_TIM_CC_EnableChannel(TIM3,LL_TIM_CHANNEL_CH4); break;
//		case '2':LL_TIM_CC_DisableChannel(TIM3,LL_TIM_CHANNEL_CH4); break;
//		default:
//			break;
//	}

//	  tim5_ccr4 +=10;
//	  	  	  if(tim5_ccr4>21000)tim5_ccr4 = 10500;      //siliyoruz calibrasyon yapmak için
//	  	  	  TIM5->CCR4 =tim5_ccr4;
//	  	  	  HAL_Delay(1);

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int Is_iBus_Throttle_Min(void){

	  if(ibus_rx_cplt_flag == 1)
		  	  {
		  		  ibus_rx_cplt_flag = 0;
		  		  if(iBus_Check_CHKSUM(&ibus_rxbuff[0],32)==1)
		  		  {
		  				iBus_Parsing(&ibus_rxbuff[0], &iBus);
		  				if(iBus.LV < 1010) return 1;

		  		  }

		  	  }
	  return 0;
}

void ESC_Calibration(void){

	  TIM5->CCR1 = 21000;
	  TIM5->CCR2 = 21000;
	  TIM5->CCR3 = 21000;
	  TIM5->CCR4 = 21000;
	  HAL_Delay(7000);

	  TIM5->CCR1 = 10500;
	  TIM5->CCR2 = 10500;
	  TIM5->CCR3 = 10500;
	  TIM5->CCR4 = 10500;
	   HAL_Delay(8000);
}

int Is_iBus_Received(void){
	if(ibus_rx_cplt_flag == 1)
			  	  {
			  		  ibus_rx_cplt_flag = 0;
			  		  if(iBus_Check_CHKSUM(&ibus_rxbuff[0],32)==1)
			  		  {
			  				iBus_Parsing(&ibus_rxbuff[0], &iBus);
			  				return 1;
			  		  }

			  	  }
		  return 0;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

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
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
