/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_PORT 		GPIOA         //GPIOC
#define LED_PIN 		GPIO_PIN_5    //GPIO_PIN_1

#define BTN_IRQ_PORT    GPIOC
#define BTN_IRQ_PIN     GPIO_PIN_13

#define BTN_USER_PORT	 GPIOD
#define BTN_USER_PIN 	 GPIO_PIN_2

#define PIN_GLOBAL_POWER_PORT  GPIOC
#define PIN_GLOBAL_POWER_PIN   GPIO_PIN_0

#define size_rx_uart  12

typedef struct 
{
	uint32_t command;
	uint32_t length;
	int8_t* buff;
}tcp_packet_t;

typedef struct
{
	uint32_t hz;
	float current_mA;
	float power_W;
	float bus_voltage_V;
	float shunt_voltage_V;
	int16_t current_reg;
	uint16_t config;
	uint16_t calib;
	uint16_t die_id;
	uint8_t addr;
	uint8_t rsrvd_byte;
}INA_230;

#define SLAVE_SUCCESS_ANSWER        0
#define SLAVE_BAD_ANSWER           -1

#define CMD_ANSWER            1
#define CMD_GET_INAINFO       2
#define CMD_SHUTDOWN        3

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void get_sleep();
void blink(uint8_t, uint32_t);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *);
void send_message (tcp_packet_t*);
void get_data_ina();

int8_t flag_btn_irq = 0;
int8_t flag_btn_user_pressed = 0;
int8_t flag_get_UART=0;
int8_t flag_need_sleep = 1;

int8_t flag_uart_getBlink=0;

//int8_t MSV[sizeMSV]={10, };
//
//int8_t counters = 0;

float current_mA = 0.0f;
int16_t current_reg = 0;
uint32_t hz = 0;
uint8_t addr = 0;
float power_W = 0.0f;
float bus_voltage_V = 0.0f;
float shunt_voltage_V = 0.0f;
uint16_t config=0;
uint16_t calib=0;
uint16_t die_id=0;

uint8_t rx_uart[12]={0, };
//uint8_t tx_uart[10]={10, 11, 12, 13, 14, 15, 16, 17, 18, 19};

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */

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
	MX_USART2_UART_Init();
	MX_USART1_UART_Init();

	/* USER CODE BEGIN 2 */

	//для отладки
	blink(6, 100);

	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		//отправляем в сон
		if(flag_need_sleep){
			get_sleep();
			//выход после режима
			flag_need_sleep=0;
		}

		//произошло короткое нажатие кнопки - вызывает прерывание
		//если произошло прерывание по кнопке BTN_IRQ
		if(flag_btn_irq){
			//необходимо проверить состояние кнопки на удержание BTN_USER
			//(BTN_USER и BTN_IRQ - физически это одна и та же кнопка, выводы дублируются на ещё один пин, так как на ней и режим прерывания и обычный )
			HAL_Delay(1900);
			if(HAL_GPIO_ReadPin(BTN_USER_PORT, BTN_USER_PIN) == GPIO_PIN_RESET){
				//кнопка удерживается какой то период времени
				flag_btn_user_pressed=1;
			}

			//кнопку удерживали, необходимо вкл прибор
			if(flag_btn_user_pressed){
				//включаем пин, отвечающий за глобальное управление питанием
				HAL_GPIO_WritePin(PIN_GLOBAL_POWER_PORT, PIN_GLOBAL_POWER_PIN, GPIO_PIN_SET);
				blink(6,50);  //для отладки
				//LED сигнализирующий, что питание подано
				//+++HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
				flag_btn_user_pressed = 0;

				//разрешаем работу по UART`у
				flag_get_UART=1;

				while(flag_get_UART){
					HAL_UART_Receive_IT (&huart1, (uint8_t*)rx_uart, size_rx_uart);

					//данный буффер собирается в обработчике прерываний
					int8_t action=rx_uart[0];

					switch (action)
					{
					//команда получения данных с INA
					case 0x02:
						blink(6,50);
						get_data_ina();
						break;
					//команда выключения
					case 0x03:
						blink(6, 1000);

						flag_get_UART = 0;
						flag_btn_user_pressed = 0;
						flag_btn_irq=0;
						flag_need_sleep=1;

						HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET); // для отладки - удалить

						//перед выключением питания необходим таймаут
						//для корректного выключения master`a
						// HAL_Delay(60000);
						HAL_Delay(6000);

						//пин управления питанием в LOW
						HAL_GPIO_WritePin(PIN_GLOBAL_POWER_PORT, PIN_GLOBAL_POWER_PIN, GPIO_PIN_RESET);
						//светодиод индикации питания выкл
						HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
						break;

					default:
						//другие команды игнорируются
						break;
					}

					//обнуление массива rx_uart
					memset(&rx_uart[0], 0x00, 12);
				}

			}else{
				flag_need_sleep=1;
				flag_btn_irq=0;
			}

		}

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
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
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 100;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
//	huart1.Init.BaudRate = 9600;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 9600;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : PC14 */
	GPIO_InitStruct.Pin = GPIO_PIN_14;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : PC0 */
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : LD2_Pin */
	GPIO_InitStruct.Pin = LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : PD2 */
	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin== GPIO_PIN_13) {
		//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
		flag_btn_irq=1;
		flag_need_sleep = 0;
	}else{
		__NOP();
	}
	//	else if(GPIO_Pin== GPIO_PIN_2){
	//		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	//	} else{
	//	__NOP();
	//	}
}

void get_sleep()
{
	HAL_SuspendTick();
	HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
	HAL_ResumeTick();
}

void blink(uint8_t counter_blink, uint32_t t_ms)
{
	for(int8_t i=0; i<counter_blink; ++i){
		HAL_GPIO_TogglePin(LED_PORT, LED_PIN); //Toggle LED
		HAL_Delay (t_ms);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//	__NOP();
	if(huart == &huart1){
		flag_uart_getBlink =1;
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	__NOP();
}

// void send_message (tcp_packet_t* ans_struct, RawSerial* _pc){
void send_message (tcp_packet_t* ans_struct){
	uint16_t len_send=sizeof(ans_struct->command)+sizeof(ans_struct->length) + ans_struct->length;
	// pc.putc(len_send);  //для отладки
	const uint8_t sizeTempMSV = 200;
	uint8_t TempMSV[sizeTempMSV];
	memset(&TempMSV[0], 0x00, 200);
	//собираем сообщение
	uint8_t pos=0;
	memcpy(&TempMSV[pos], &ans_struct->command, sizeof(ans_struct->command));
	 pos+=sizeof(ans_struct->command);
	memcpy(&TempMSV[pos], &ans_struct->length, sizeof(ans_struct->length));
	 pos+=sizeof(ans_struct->length);
	memcpy(&TempMSV[pos], &ans_struct->buff[0], ans_struct->length);

	//send message
	HAL_UART_Transmit_IT (&huart1, (uint8_t*)TempMSV, len_send-13);
//	HAL_UART_Transmit(&huart1, (uint8_t*)&TempMSV[0], len_send-13, 100);
}

void get_data_ina(){
	INA_230 ina;
	memset(&ina, 0x00, sizeof(INA_230));
	//================================================================================
	ina.hz = 10;                    //для отладки
	ina.current_mA = 10.7;          //для отладки
	ina.power_W = 11.8;
	ina.bus_voltage_V = 12.9;
	ina.shunt_voltage_V = 13.10;
	ina.current_reg = 14;
	ina.config = 15;
	ina.calib = 16;
	ina.die_id = 17;
	ina.addr = 18;

	//работаем с INA-----------------------------------------------
	// time_t seconds = time(NULL);
	// read the data
	// INA230 *ina230 =new  INA230(i2c);
	// delete ina230;

	// // void frequency(int hz);
	// current_mA = read_current();
	// current_reg = read_current_reg();
	// // float read_current_by_shuntvolt(void);
	// power_W = read_power();
	// bus_voltage_V = read_bus_voltage();
	// shunt_voltage_V = read_shunt_voltage();
	// config = read_config();
	// // uint16_t set_config(uint16_t cfg);
	// calib = read_calb(void);
	// die_id = read_die_id();
	// uint16_t set_calb(uint16_t clb);
	// uint8_t write_reg(uint8_t addr, uint8_t data);
	// uint8_t read_reg(uint8_t addr);

	// uint16_t read_mask_enable();

	// uint16_t read_alert_limit();

	// uint16_t set_mask_enable(uint16_t cfg);

	// uint16_t set_alert_limit(uint16_t cfg);

	// int16_t  get_shunt_res();
	//--------------------------------------------------------------------------
	//==========================================================================
	//собрать сообщение для отправки
	//==========================================================================
	const uint8_t sizeBuffer = 100;
	int8_t Buffer[sizeBuffer];
	//заслать ответ
	uint32_t res = SLAVE_SUCCESS_ANSWER;
	tcp_packet_t ans;
	uint8_t pos = 0;
	uint32_t packet_command = 2;
	memset(&ans, 0x00, sizeof(tcp_packet_t));
	memset(&Buffer[0], 0x00, sizeBuffer );
	ans.command = CMD_ANSWER;
	ans.length = sizeof(ans.command)+sizeof(res)
                		+sizeof(ina.hz)+sizeof(ina.current_mA)
						+sizeof(ina.power_W)+sizeof(ina.bus_voltage_V)+sizeof(ina.shunt_voltage_V)
						+sizeof(ina.current_reg)+sizeof(ina.config)+sizeof(ina.calib)
						+sizeof(ina.die_id)+sizeof(ina.addr);  //+всё в пакете
						// ans.length = sizeof(uint32_t)+sizeof(uint8_t)                        //sizeof(ans.command)+sizeof(res)
	//             +sizeof(uint32_t)+sizeof(float)                         //sizeof(ina.hz)+sizeof(ina.current_mA)
	//             +sizeof(float)+sizeof(float)+sizeof(float)              //sizeof(ina.power_W)+sizeof(ina.bus_voltage_V)+sizeof(ina.shunt_voltage_V)
	//             +sizeof(int16_t)+sizeof(uint16_t)+sizeof(uint16_t)      //sizeof(ina.current_reg)+sizeof(ina.config)+sizeof(ina.calib)
	//             +sizeof(uint16_t)+sizeof(uint8_t);  //+всё в пакете     //sizeof(ina.die_id)+sizeof(ina.addr);
	// ans.length = sizeof(ans.command)+sizeof(res) + sizeof(INA_230);  //+всё в пакете

	// pc.putc(ans.length);  //для отладки
	// memcpy(&Buffer[pos], (char*)&packet.command, sizeof(packet.command));
	memcpy(&Buffer[pos], (char*)&packet_command, sizeof(packet_command));
	// pos+= sizeof(packet.command);
	pos+= sizeof(packet_command);
	memcpy(&Buffer[pos], (char*)&res, sizeof(res));
	pos+=sizeof(res);
	//+всё в пакете
	memcpy(&Buffer[pos], (char*)&ina.hz, sizeof(ina.hz));
	pos+=sizeof(ina.hz);
	memcpy(&Buffer[pos], (char*)&ina.current_mA, sizeof(ina.current_mA));
	pos+=sizeof(ina.current_mA);
	memcpy(&Buffer[pos], (char*)&ina.power_W, sizeof(ina.power_W));
	pos+=sizeof(ina.power_W);
	memcpy(&Buffer[pos], (char*)&ina.bus_voltage_V, sizeof(ina.bus_voltage_V));
	pos+=sizeof(ina.bus_voltage_V);
	memcpy(&Buffer[pos], (char*)&ina.shunt_voltage_V, sizeof(ina.shunt_voltage_V));
	pos+=sizeof(ina.shunt_voltage_V);
	memcpy(&Buffer[pos], (char*)&ina.current_reg, sizeof(ina.current_reg));
	pos+=sizeof(ina.current_reg);
	memcpy(&Buffer[pos], (char*)&ina.config, sizeof(ina.config));
	pos+=sizeof(ina.config);
	memcpy(&Buffer[pos], (char*)&ina.calib, sizeof(ina.calib));
	pos+=sizeof(ina.calib);
	memcpy(&Buffer[pos], (char*)&ina.die_id, sizeof(ina.die_id));
	pos+=sizeof(ina.die_id);
	memcpy(&Buffer[pos], (char*)&ina.addr, sizeof(ina.addr));
	pos+=sizeof(ina.addr);

	pos=0;
	ans.buff = &Buffer[0];
	send_message(&ans);
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
 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
