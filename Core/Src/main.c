/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "queue.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef  int32_t  BME280_S32_t;
typedef  uint32_t BME280_U32_t;
typedef  uint16_t BME280_U16_t;
typedef enum{
	RTC_IDLE,
	RTC_INIT_WR,
	RTC_INIT_RD,
	RTC_INIT_CMPLT,
	RTC_DATA_RD,
	RTC_DATA_WR
}rtc_state_t;

typedef enum{
	BME_IDLE,
	BME_INIT_WR,
	BME_INIT_RD,
	BME_INIT_CMPLT
}bme_state_t;

/*typedef enum{
	CMD_MODE_IDLE,
	CMD_MODE_ENTER,
	CMD_MODE_EXIT,
}cmd_state_t;*/


typedef struct {

	int32_t temperature;
	uint32_t pressure;
	uint32_t humidity;
}wData;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_CMD_SIZE 50
#define RTC_INIT_DONE  (1UL << 0)
#define BME_INIT_DONE  (1UL << 1)
#define ALL_INIT_DONE (RTC_INIT_DONE | BME_INIT_DONE)

#define CMD_RCV_DONE (1UL << 0)

#define ULONG_MAX      0xFFFFFFFFUL
#define CMD_MODE_IDLE  (1UL << 0)
#define CMD_MODE_ENTER (1UL << 1)
//#define CMD_MODE_EXIT  (1UL << 1)

#define RTC_DATA_READY (1UL << 2)
#define BME_DATA_READY (1UL << 3)
#define ALL_DATA_READY (RTC_DATA_READY | BME_DATA_READY)

#define SLEEP  0
#define FORCED 1
#define NORMAL 3

#define OVRS_RATE_0 	0
#define OVRS_RATE_1 	1
#define OVRS_RATE_2 	2
#define OVRS_RATE_4 	3
#define OVRS_RATE_8 	4
#define OVRS_RATE_16 	5

#define IIR_FILTER_DISABLE	0
#define IIR_FILTER_2		1
#define IIR_FILTER_4		2
#define IIR_FILTER_8		3
#define IIR_FILTER_16		4

#define BME280_ADDR_ID 			0xD0
#define BME280_I2C_ADDRESS      (0x77<<1)
#define BME280_CHIP_ID			0x60
#define BME280_VAL_RESET 		0xB6
#define BME280_ADDR_RESET 		0xE0
#define BME280_ADDR_CTRL_HUM	0xF2
#define BME280_ADDR_STATUS		0xF3
#define BME280_ADDR_CTRL_MEAS	0xF4
#define BME280_ADDR_CONFIG		0xF5
#define BME280_ADDR_PRESS_MSB	0xF7
#define BME280_ADDR_PRESS_LSB	0xF8
#define BME280_ADDR_PRESS_XLSB	0xF9
#define BME280_ADDR_TEMP_MSB	0xFA
#define BME280_ADDR_TEMP_LSB	0xFB
#define BME280_ADDR_TEMP_XLSB	0xFC
#define BME280_ADDR_HUM_MSB		0xFD
#define BME280_ADDR_HUM_LSB		0xFE

#define CALIB_REG_BASEADDR_TMPR 0x88
#define CALIB_REG_COUNT_PT		25
#define CALIB_REG_COUNT_H		6
#define CALIB_REG_BASEADDR_HUM 	0xE1
#define PRESSURE_DATA_BASEADDR	0xF7
#define SENSOR_DATA_REG_COUNT	7

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

uint8_t usart_tx_en =1;

BaseType_t status;
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart2;

TaskHandle_t RTC_init_task_handle;
TaskHandle_t BME_init_task_handle;
TaskHandle_t RTC_task_handle;
TaskHandle_t UART_task_handle;
TaskHandle_t BME_task_handle;
TaskHandle_t CMD_task_handle;

HAL_StatusTypeDef bme_i2c_status;

volatile rtc_state_t DS1307_state = RTC_IDLE;
volatile bme_state_t BME280_state = BME_IDLE;
//volatile cmd_state_t CMD_state = CMD_MODE_IDLE;
uint32_t cmd_mode_flag = 0;
uint32_t CMD_state = CMD_MODE_IDLE;
const uint8_t rtc_init_val_WR=0x00;
uint8_t rtc_init_val_RD;

QueueHandle_t bme_data_queue;
QueueHandle_t rtc_data_queue;
QueueHandle_t rtc_cmd_queue;

BME280_U32_t pressure_pa;
uint8_t bme_chip_ID;
static uint8_t burst_read_arr[2048];

uint8_t rtc_data_bufRD[3];
uint8_t rtc_data_bufWR[3];
static uint8_t data_register_list [8]={0};
static uint16_t calibration_register_list_PT [12]={0};
static uint8_t calibration_register_list_H [8]={0};
uint16_t div[4]={1000,100,10,1};
static uint16_t dig_T1;
static int16_t dig_T2;
static int16_t dig_T3;

static uint16_t dig_P1;
static int16_t dig_P2;
static int16_t dig_P3;
static int16_t dig_P4;
static int16_t dig_P5;
static int16_t dig_P6;
static int16_t dig_P7;
static int16_t dig_P8;
static int16_t dig_P9;

static uint8_t dig_H1;
static int16_t dig_H2;
static uint8_t dig_H3;
static int16_t dig_H4;
static int16_t dig_H5;
static int8_t dig_H6;

static int32_t adc_P;
static int32_t adc_T;
static int32_t adc_H;

volatile BME280_S32_t t_fine;

uint8_t time_stamp[14];
uint8_t envStr[18];
wData data;

//UART-Tx Buffers
uint8_t time_stamp_qRx[14];
uint8_t envStr_qRx[18];

uint8_t uart_tx_buf[33];
uint8_t cmd_rx_buf[MAX_CMD_SIZE];

//UART-Rx Buffers
volatile uint8_t irx;
volatile uint8_t rxByte;
volatile uint8_t cmd_buf[MAX_CMD_SIZE];
uint8_t cmd_rx_buf[MAX_CMD_SIZE];
volatile uint8_t uart_rx_buf[MAX_CMD_SIZE];
uint8_t RTC_init_success = 0;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void RTC_init_task_handler(void*);
static void BME_init_task_handler(void*);
static void RTC_task_handler(void*);
static void UART_task_handler(void*);
static void BME_task_handler(void*);
static void CMD_task_handler(void*);

static void calibration_variables_init();
static void sensor_data_variables_init();
static void bme280_forced_mode_trigger(void);

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
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  //Initialize the DS1307 module
   //1.Reset the CH bit to 0, write 0x00 at sec_reg_addr(0x00)

  status = xTaskCreate(RTC_init_task_handler, "RTC-Init_task",300,NULL,4, &RTC_init_task_handle);
  configASSERT(status == pdPASS);

  status = xTaskCreate(BME_init_task_handler, "BME-Init_task",300,NULL,4, &BME_init_task_handle);
  configASSERT(status == pdPASS);

  //HAL_GPIO_TogglePin(GPIOD, LD4_Pin);
  status = xTaskCreate(UART_task_handler, "UART-Handling_task", 200,NULL,3, &UART_task_handle);
  configASSERT(status == pdPASS);

   //HAL_GPIO_TogglePin(GPIOD, LD5_Pin);
  status = xTaskCreate(RTC_task_handler, "RTC-Handling_task", 200,NULL,3, &RTC_task_handle);
  configASSERT(status == pdPASS);

  status = xTaskCreate(BME_task_handler, "BME-Handling_task", 200,NULL,3, &BME_task_handle);
  configASSERT(status == pdPASS);

  status = xTaskCreate(CMD_task_handler, "CMD-Handling_task", 200,NULL,3, &CMD_task_handle);
  configASSERT(status == pdPASS);




  // status = xTaskCreate(BME_task_handler,"BME-Handling_task",200,NULL,3,&BME_task_handle);
   //configASSERT(status == pdPASS);
  		//HAL_GPIO_TogglePin(GPIOD, LD6_Pin);


  rtc_cmd_queue = xQueueCreate(5,MAX_CMD_SIZE);
  configASSERT(rtc_cmd_queue != NULL);

  HAL_UART_Receive_IT(&huart2,&rxByte,1);

  vTaskStartScheduler();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_WS_Pin */
  GPIO_InitStruct.Pin = I2S3_WS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_WS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_SCK_Pin SPI1_MISO_Pin SPI1_MOSI_Pin */
  GPIO_InitStruct.Pin = SPI1_SCK_Pin|SPI1_MISO_Pin|SPI1_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S3_MCK_Pin I2S3_SCK_Pin I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_MCK_Pin|I2S3_SCK_Pin|I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_FS_Pin */
  GPIO_InitStruct.Pin = VBUS_FS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_FS_ID_Pin|OTG_FS_DM_Pin|OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */



void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;
    taskDISABLE_INTERRUPTS();
    while(1);   // breakpoint here → pcTaskName tells you WHICH task overflowed
}

void create_timestamp(){

	//Seconds
	uint8_t s = rtc_data_bufRD[0];

	//Minutes
	uint8_t m = rtc_data_bufRD[1];
	//Hour
	uint8_t h = rtc_data_bufRD[2];


	time_stamp[1] = (h & 0x0F) + '0';
	time_stamp[2] = ':';
	time_stamp[3] = ((m>>4)&0x07) + '0';
	time_stamp[4] = (m & 0x0F) + '0';
	time_stamp[5] = ':';
	time_stamp[6] = ((s>>4) & 0x07) + '0';
	time_stamp[7] = (s & 0x0F) + '0';

	if(h&(1<<6)){			//12 hrs format
		if(h&(1<<5)){ //PM
			time_stamp[9] = 'P';
			time_stamp[10] = 'M';
		}
		else{ // AM
			time_stamp[9] = 'A';
			time_stamp[10] = 'M';

		}
		time_stamp[0] = ((h>>4) & 0x01) + '0';
		time_stamp[8] = ' ';
		time_stamp[11]= '\r';
		time_stamp[12]= '\n';
		time_stamp[13]='\0';
	}

	else{					//24 Hrs format

		time_stamp[0] = ((h>>4) & 0x03) + '0';
		time_stamp[8] = '\r';
		time_stamp[9] = '\n';
		time_stamp[10] = '\0';
	}
}

static void calibration_variables_init()
{
	dig_T1 = (uint16_t)calibration_register_list_PT[0];
	dig_T2 = (int16_t)calibration_register_list_PT[1];
	dig_T3 = (int16_t)calibration_register_list_PT[2];

	dig_P1 = (uint16_t)calibration_register_list_PT[3];
	dig_P2 = (int16_t)calibration_register_list_PT[4];
	dig_P3 = (int16_t)calibration_register_list_PT[5];
	dig_P4 = (int16_t)calibration_register_list_PT[6];
	dig_P5 = (int16_t)calibration_register_list_PT[7];
	dig_P6 = (int16_t)calibration_register_list_PT[8];
	dig_P7 = (int16_t)calibration_register_list_PT[9];
	dig_P8 = (int16_t)calibration_register_list_PT[10];
	dig_P9 = (int16_t)calibration_register_list_PT[11];

	dig_H1 = (uint8_t)calibration_register_list_H[0];
	dig_H2 = (int16_t)((calibration_register_list_H[2]<<8) | (calibration_register_list_H[1]));
	dig_H3 = (uint8_t)(calibration_register_list_H[3]);
	dig_H4 = (int16_t)(((int16_t)calibration_register_list_H[4] << 4) |
	                   (calibration_register_list_H[5] & 0x0F));
	dig_H5 = (int16_t)(((int16_t)calibration_register_list_H[6] << 4) |
	                   ((calibration_register_list_H[5] & 0xF0) >> 4));
	dig_H6 = (int8_t)calibration_register_list_H[7];

}

static void bme280_forced_mode_trigger(void){

	/*Configures ctrl_hum register 0xF2*/
	uint8_t hum_ovrs= OVRS_RATE_4;
	//data_write(OVRS_RATE_4,BME280_ADDR_CTRL_HUM);
	HAL_I2C_Mem_Write_IT(&hi2c2, BME280_I2C_ADDRESS, BME280_ADDR_CTRL_HUM, I2C_MEMADD_SIZE_8BIT,&hum_ovrs,1);
	//HAL_I2C_Mem_Write(&hi2c2, BME280_I2C_ADDRESS, BME280_ADDR_CTRL_HUM, I2C_MEMADD_SIZE_8BIT, &hum_ovrs, 1, uint32_t Timeout);
	ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
	/*Configures ctrl_meas register 0xF4*/
	uint8_t tempreg = 0x00;
	tempreg |= (FORCED << 0);
	tempreg |= (OVRS_RATE_2 << 2);
	tempreg |= (OVRS_RATE_2 << 5);
	//data_write(tempreg,BME280_ADDR_CTRL_MEAS);
	HAL_I2C_Mem_Write_IT(&hi2c2, BME280_I2C_ADDRESS, BME280_ADDR_CTRL_MEAS, I2C_MEMADD_SIZE_8BIT,&tempreg,1);
	ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
}

static void sensor_data_variables_init(){
	adc_P =( ((uint32_t)data_register_list[0] << 12) | ((uint32_t)data_register_list[1] << 4) | ((uint32_t)data_register_list[2] >> 4));
	adc_T =( ((uint32_t)data_register_list[3] << 12) | ((uint32_t)data_register_list[4] << 4) | ((uint32_t)data_register_list[5] >> 4));
	adc_H = ( ((uint16_t)data_register_list[6] << 8)  | ((uint16_t)data_register_list[7])) ;

}

static BME280_S32_t BME280_compensate_T_int32(BME280_S32_t adc_T)
{
	BME280_S32_t var1, var2, T;
	//var1  = ((((adc_T>>3) – ((BME280_S32_t)dig_T1<<1))) * ((BME280_S32_t)dig_T2)) >> 11;
	var1 = (((adc_T >> 3) - ((BME280_S32_t)dig_T1<<1)) * ((BME280_S32_t)dig_T2)) >> 11  ;
	var2 = (((((adc_T >> 4) - ((BME280_S32_t)dig_T1)) * ((adc_T >> 4) - ((BME280_S32_t)dig_T1))) >> 12) *  ((BME280_S32_t)dig_T3)) >> 14;

	var2 =
	t_fine = var1 + var2;
	T  = (t_fine * 5 + 128) >> 8;
	return T;
}

BME280_U32_t BME280_compensate_P_int32(BME280_S32_t adc_P)
{
	BME280_S32_t var1, var2;
	BME280_U32_t p;
    var1 = ((BME280_S32_t)t_fine >> 1) - (BME280_S32_t)64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * (BME280_S32_t)dig_P6;
    var2 = var2 + ((var1 * (BME280_S32_t)dig_P5) << 1);
    var2 = (var2 >> 2) + ((BME280_S32_t)dig_P4 << 16);
    var1 = (((BME280_S32_t)dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) +
           (((BME280_S32_t)dig_P2 * var1) >> 1);
    var1 = (var1 >> 18);
    var1 = (((BME280_S32_t)32768 + var1) * (BME280_S32_t)dig_P1) >> 15;

    if (var1 == 0)
    {
        return 0;   // avoid exception caused by division by zero
    }

    p = (((BME280_U32_t)((BME280_S32_t)1048576 - adc_P) - (BME280_U32_t)(var2 >> 12)) * 3125U);

    if (p < 0x80000000U)
    {
        p = (p << 1) / (BME280_U32_t)var1;
    }
    else
    {
        p = (p / (BME280_U32_t)var1) * 2U;
    }

    var1 = ((BME280_S32_t)dig_P9 * (BME280_S32_t)(((p >> 3) * (p >> 3)) >> 13)) >> 12;
    var2 = ((BME280_S32_t)(p >> 2) * (BME280_S32_t)dig_P8) >> 13;

    p = (BME280_U32_t)((BME280_S32_t)p + ((var1 + var2 + (BME280_S32_t)dig_P7) >> 4));

    return p;   // pressure in Pa
}


BME280_U32_t BME280_compensate_H_int32(BME280_S32_t adc_H)
{
	BME280_S32_t v_x1_u32r;
	v_x1_u32r = (t_fine - ((BME280_S32_t)76800));

	v_x1_u32r = (((((adc_H << 14) - (((BME280_S32_t)dig_H4) << 20) - (((BME280_S32_t)dig_H5) *
	v_x1_u32r)) + ((BME280_S32_t)16384)) >> 15) * (((((((v_x1_u32r *
	((BME280_S32_t)dig_H6)) >> 10) * (((v_x1_u32r * ((BME280_S32_t)dig_H3)) >> 11) +
	((BME280_S32_t)32768))) >> 10) + ((BME280_S32_t)2097152)) * ((BME280_S32_t)dig_H2) +
	8192) >> 14));
	v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
	((BME280_S32_t)dig_H1)) >> 4));
	v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
	v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
	return (BME280_U32_t)(v_x1_u32r>>12);
}

void envData_to_string(wData* env_data){

	uint8_t digit;
	uint8_t fl=0;
	int32_t temperature = env_data->temperature;
	uint32_t pressure = env_data->pressure;
	uint16_t humidity = env_data->humidity;

	/*Temperature: Integer to string*/

	if(temperature<0) {
		envStr[0]='-';
		uint32_t temperature_abs = (~((uint32_t)temperature -1U));
		digit = temperature_abs/10;
		if(!digit){
			envStr[1]=digit+'0';
			envStr[2] = 'C';
			envStr[3] = ' ';
			envStr[4] = ' ';
		}

		else{
			envStr[1]=digit+'0';
			digit = temperature_abs -(10*digit);
			envStr[2]='0'+digit;
			envStr[3]='C';
			envStr[4] = ' ';
		}
	}

	else if(temperature == 0){
		envStr[0]='0';
		envStr[1] = 'C';
		envStr[2]=' ';
		envStr[3]=' ';
		envStr[4]=' ';
	}

	else{
		digit = temperature / 10;

		if(!digit){
			envStr[0]=digit+'0';
			envStr[1] = 'C';
			envStr[2]=' ';
			envStr[3]=' ';
			envStr[4]=' ';
		}

		else{
			envStr[0]=digit+'0';
			digit = temperature -(10*digit);
			envStr[1]='0'+digit;
			envStr[2]='C';
			envStr[3]=' ';
			envStr[4]=' ';
		}
	}


	/*Pressure: Integer to string*/

	for(uint8_t i =0; i<4;i++){
		digit = (pressure /div[i]);
		if(digit){
			fl=1;
			envStr[i+5] = digit +'0';
			pressure = pressure - (digit * div[i]);
		}
		else{
			if(fl) envStr[i+5]='0';
			else envStr[i+5] = ' ';
		}
	}

	envStr[9] = 'h';
	envStr[10] = 'P';
	envStr[11] = 'a';
	envStr[12] = ' ';

	/*Humidity: Integer to string*/
	fl = 0;
	for(uint8_t i =2; i<4;i++){
		digit = (humidity /div[i]);
		if(digit){
			fl=1;
			envStr[i+11] = digit +'0';
			humidity = humidity - (digit * div[i]);
		}

		else{
			if(fl) envStr[i+11]='0';
			else envStr[i+11] = ' ';
		}
	}
	envStr[15] = '%';
	envStr[16] = '\r';
	envStr[17] = '\n';

}

static void BME_init_task_handler(void* parameters){


	//HAL_StatusTypeDef ret;
	//ret = HAL_I2C_IsDeviceReady(&hi2c2,BME280_I2C_ADDRESS,3,100);
	//configASSERT(ret == HAL_OK);
	//Read BME280 chip-ID from 0xD0

	BME280_state = BME_INIT_RD;
	bme_i2c_status = HAL_I2C_Mem_Read_IT(&hi2c2,BME280_I2C_ADDRESS, BME280_ADDR_ID, I2C_MEMADD_SIZE_8BIT, &bme_chip_ID,1);
	ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
	if(BME280_state == BME_INIT_WR){
		//Device soft-reset
		uint8_t resetVal = BME280_VAL_RESET;
		HAL_I2C_Mem_Write_IT(&hi2c2, BME280_I2C_ADDRESS, BME280_ADDR_RESET, I2C_MEMADD_SIZE_8BIT, &resetVal,1);
		ulTaskNotifyTake(pdTRUE,portMAX_DELAY);

		/*Configures ctrl_hum register 0xF2*/
		uint8_t hum_OVSRt = OVRS_RATE_4;
		HAL_I2C_Mem_Write_IT(&hi2c2, BME280_I2C_ADDRESS, BME280_ADDR_CTRL_HUM, I2C_MEMADD_SIZE_8BIT, &hum_OVSRt,1);
		ulTaskNotifyTake(pdTRUE,portMAX_DELAY);

		//Configure IIR filter OFF
		uint8_t IIRfl_st = IIR_FILTER_DISABLE;
		HAL_I2C_Mem_Write_IT(&hi2c2, BME280_I2C_ADDRESS, BME280_ADDR_CONFIG, I2C_MEMADD_SIZE_8BIT, &IIRfl_st,1);
		ulTaskNotifyTake(pdTRUE,portMAX_DELAY);

		//Configure ctrl_meas register 0xF4
		uint8_t tempreg = 0x00;
		tempreg |= (FORCED << 0);
		tempreg |= (OVRS_RATE_2 << 2);
		tempreg |= (OVRS_RATE_2 << 5);
		HAL_I2C_Mem_Write_IT(&hi2c2, BME280_I2C_ADDRESS, BME280_ADDR_CTRL_MEAS, I2C_MEMADD_SIZE_8BIT, &tempreg,1);
		ulTaskNotifyTake(pdTRUE,portMAX_DELAY);

		/*Reset the array - calibration_register_list[]*/
		memset(burst_read_arr,0,sizeof(burst_read_arr));

		//For dig_Tx , dig_Px calibration registers
		HAL_I2C_Mem_Read_IT(&hi2c2,BME280_I2C_ADDRESS, CALIB_REG_BASEADDR_TMPR, I2C_MEMADD_SIZE_8BIT,burst_read_arr,CALIB_REG_COUNT_PT);
		ulTaskNotifyTake(pdTRUE,portMAX_DELAY);

		calibration_register_list_H[0] =  burst_read_arr[25];
		for(uint8_t i=0;i<=11;i++){
			calibration_register_list_PT[i] = (uint16_t)((burst_read_arr[2*i]) | (burst_read_arr[2*i +1]<<8));
		}

		//For  dig_Hx calibration registers
		memset(burst_read_arr,0,2048);
		HAL_I2C_Mem_Read_IT(&hi2c2,BME280_I2C_ADDRESS, CALIB_REG_BASEADDR_HUM, I2C_MEMADD_SIZE_8BIT,burst_read_arr,CALIB_REG_COUNT_H);
		ulTaskNotifyTake(pdTRUE,portMAX_DELAY);

		for(uint8_t i=1;i<=7;i++){

			calibration_register_list_H[i] = (uint8_t)burst_read_arr[i-1];
		}

		calibration_variables_init();

		BME280_state = BME_INIT_CMPLT;

		//create queue for bme280 data
		bme_data_queue = xQueueCreate(5,sizeof(envStr));
		configASSERT(rtc_data_queue != NULL);


		HAL_GPIO_WritePin(GPIOD, LD6_Pin,GPIO_PIN_SET); // BLUE LED

		xTaskNotify(RTC_task_handle,BME_INIT_DONE,eSetBits);
		xTaskNotify(BME_task_handle,BME_INIT_DONE,eSetBits);
		xTaskNotify(UART_task_handle,BME_INIT_DONE,eSetBits);

		vTaskDelete(NULL);
	}


}

static void BME_task_handler(void* parameters){

	uint32_t flag = 0;

	while((flag & ALL_INIT_DONE) != ALL_INIT_DONE){
		xTaskNotifyWait(0,0,&flag,portMAX_DELAY);
	}
	//ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
	TickType_t xLastWakeTime = xTaskGetTickCount();
	uint8_t status_reg;

	while(1){

#if FORCED == 1
		bme280_forced_mode_trigger();
#endif
		//Read status register bit3
		do{
			HAL_I2C_Mem_Read_IT(&hi2c2,BME280_I2C_ADDRESS, BME280_ADDR_STATUS, I2C_MEMADD_SIZE_8BIT,&status_reg,1);
			ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
		}while(status_reg &(0x01<<3));

		//Read raw data register
		HAL_I2C_Mem_Read_IT(&hi2c2,BME280_I2C_ADDRESS, PRESSURE_DATA_BASEADDR, I2C_MEMADD_SIZE_8BIT,burst_read_arr,SENSOR_DATA_REG_COUNT);
		ulTaskNotifyTake(pdTRUE,portMAX_DELAY);

		for(uint8_t i=0;i<8;i++){
			data_register_list[i] = burst_read_arr[i];
		}

		sensor_data_variables_init();
		data.temperature = BME280_compensate_T_int32((int32_t)adc_T) / 100;
		pressure_pa = BME280_compensate_P_int32((int32_t)adc_P);
		data.pressure = pressure_pa/ 100;
		data.humidity = BME280_compensate_H_int32((int32_t)adc_H) /1024 ;

		envData_to_string(&data);
		//send data(timestamp[])
		xQueueSend(bme_data_queue,envStr,pdMS_TO_TICKS(100));

		//notify the USART task
		//xTaskNotify(UART_task_handle,BME_DATA_READY,eSetBits);
		xTaskNotifyIndexed(UART_task_handle,0,BME_DATA_READY,eSetBits);
		//xTaskNotifyGive(UART_task_handle);
		vTaskDelayUntil(&xLastWakeTime,pdMS_TO_TICKS(1000));
	}

}

static void RTC_init_task_handler(void* parameters){

	HAL_I2C_Mem_Write_IT(&hi2c1,DS1307_I2C_ADDRESS,DS1307_ADDR_SEC,I2C_MEMADD_SIZE_8BIT,&rtc_init_val_WR,1);
	DS1307_state = RTC_INIT_WR;

	ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
	if(RTC_init_success){
		HAL_GPIO_WritePin(GPIOD, LD5_Pin,GPIO_PIN_SET); //RED LED
		//create queue
		rtc_data_queue = xQueueCreate(5,sizeof(time_stamp));
		configASSERT(rtc_data_queue != NULL);
		xTaskNotify(RTC_task_handle,RTC_INIT_DONE,eSetBits);
		xTaskNotify(BME_task_handle,RTC_INIT_DONE,eSetBits);
		xTaskNotify(UART_task_handle,RTC_INIT_DONE,eSetBits);
		//xTaskNotifyGive(RTC_task_handle);
		//xTaskNotifyGive(UART_task_handle);
		//DS1307_state = RTC_DATA_RD;
		vTaskDelete(NULL);
	}

	else{
		while(1){
			HAL_GPIO_TogglePin(GPIOD, LD3_Pin);
			vTaskDelay(pdMS_TO_TICKS(200));
		}
	}
}
uint8_t time_stamp_chk(uint8_t* cmd_rx_buf){
	for (uint8_t i=0; i<8; i++){
		if((i==2) || (i==5)){continue;}
		if((cmd_rx_buf[i] < '0') || (cmd_rx_buf[i] > '9')){
			return 0;
		}
	}
	return 1;
}

uint8_t process_user_data(){
	uint8_t s, m, h;
	uint32_t cmd_rx_buf_sz;
	cmd_rx_buf_sz = strlen(cmd_rx_buf);
	s = 0x00;
	m = 0x00;
	h = 0x00;

	if((cmd_rx_buf_sz!=8) && (cmd_rx_buf_sz!=11)){return 0;}
	if((cmd_rx_buf[2]!=':') || (cmd_rx_buf[5]!=':')){return 0;}
	if(!time_stamp_chk(cmd_rx_buf)){return 0;}

	else if(cmd_rx_buf[8] == '\0'){		//24 Hrs format
		h &= ~(1<<6);
		if(cmd_rx_buf[0] == '0') h &= ~(3<<5);
		else if(cmd_rx_buf[0] == '1') h |=(1<<4);
		else if(cmd_rx_buf[0] == '2') h |=(2<<4);
	}

	else if(cmd_rx_buf[8] == ' '){ //12Hrs format
		if(!strcmp(cmd_rx_buf+9,"AM")){ 
			h |= (1<<6);
			h &= ~(1<<5);
		}
		else if(!strcmp(cmd_rx_buf+9,"PM")){ 
			h |= (1<<6);
			h |= (1<<5);
		}

		else{
			return 0;
		}

		if(cmd_rx_buf[0]=='0'){
			h &= ~(1<<4);
		}
		else if(cmd_rx_buf[0]=='1'){
			h |= (1<<4);
		}
	}

	else{
		return 0;
	}

	h = ((h & 0xF0) | ((cmd_rx_buf[1]-'0') & 0x0F));

	m = (m & 0x00) | (((cmd_rx_buf[3] - '0')<<4) | (cmd_rx_buf[4]-'0'));

	s = (s & 0x80) | (((cmd_rx_buf[6] - '0')<<4) | (cmd_rx_buf[7]-'0'));

	rtc_data_bufWR[0] = s;
	rtc_data_bufWR[1] = m;
	rtc_data_bufWR[2] = h;

	return 1;
}

static void CMD_task_handler(void* parameters){

	while(1){
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		if(xQueueReceive(rtc_cmd_queue,cmd_rx_buf,0) == pdTRUE){
			//ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
			//memset(uart_rx_buf,0,MAX_CMD_SIZE);
			if(process_user_data()){
				xTaskNotify(RTC_task_handle,CMD_RCV_DONE,eSetBits);
			}
		}
	}
}

static void RTC_task_handler(void* parameters){
	uint32_t flag = 0;
	uint32_t cmd_flag = 0;
	while((flag & ALL_INIT_DONE) != ALL_INIT_DONE){
		xTaskNotifyWait(0,0,&flag,portMAX_DELAY);
	}
	//ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
	TickType_t xLastWakeTime = xTaskGetTickCount();
	while(1){

		if(xTaskNotifyWait(0,CMD_RCV_DONE,&cmd_flag,0) == pdTRUE){

			if(cmd_flag & CMD_RCV_DONE){
				cmd_flag=0;
				HAL_I2C_Mem_Write_IT(&hi2c1,DS1307_I2C_ADDRESS,DS1307_ADDR_SEC,I2C_MEMADD_SIZE_8BIT,rtc_data_bufWR,3);
				ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
			}
		}
			HAL_I2C_Mem_Read_IT(&hi2c1,DS1307_I2C_ADDRESS,DS1307_ADDR_SEC,I2C_MEMADD_SIZE_8BIT,rtc_data_bufRD,3);
			ulTaskNotifyTake(pdTRUE,portMAX_DELAY);

			//process the byte data to time-stamp
			create_timestamp();

			//send data(timestamp[])
			xQueueSend(rtc_data_queue,time_stamp,pdMS_TO_TICKS(100));

			//notify the USART task
			//xTaskNotify(UART_task_handle,RTC_DATA_READY,eSetBits);
			xTaskNotifyIndexed(UART_task_handle,0,RTC_DATA_READY,eSetBits);
			vTaskDelayUntil(&xLastWakeTime,pdMS_TO_TICKS(1000));
	}
}


void HAL_I2C_MemRxCpltCallback (I2C_HandleTypeDef *hi2c){

	BaseType_t ret = pdFALSE;
	//If I2C1 is selected -
	if(hi2c->Instance == I2C1){ //RTC - DS1307

		if(DS1307_state == RTC_INIT_RD){
			//Read the 7th bit of received data at 'rtc_init_val_RD'
			if(!((rtc_init_val_RD>>7) & 0x01)){ //CH bit is 0, successful DS1307 initialization
				RTC_init_success = 1;
				DS1307_state = RTC_INIT_CMPLT;
				vTaskNotifyGiveFromISR(RTC_init_task_handle,&ret);
				portYIELD_FROM_ISR(ret);
			}
		}

		else if(DS1307_state == RTC_INIT_CMPLT){
			//DS1307_state = RTC_DATA_RD;
			vTaskNotifyGiveFromISR(RTC_task_handle,&ret);
			portYIELD_FROM_ISR(ret);
		}
	}

	else if(hi2c->Instance == I2C2){
		if(BME280_state == BME_INIT_RD){
			if(bme_chip_ID == BME280_CHIP_ID){
				BME280_state = BME_INIT_WR;
				vTaskNotifyGiveFromISR(BME_init_task_handle,&ret);
				portYIELD_FROM_ISR(ret);
			}
		}
		else if(BME280_state == BME_INIT_WR){
			vTaskNotifyGiveFromISR(BME_init_task_handle,&ret);
			portYIELD_FROM_ISR(ret);
		}

		else if(BME280_state == BME_INIT_CMPLT){
			vTaskNotifyGiveFromISR(BME_task_handle,&ret);
			portYIELD_FROM_ISR(ret);
		}

	}
}


void HAL_I2C_MemTxCpltCallback (I2C_HandleTypeDef *hi2c){
	BaseType_t ret = pdFALSE;
	if(hi2c->Instance == I2C1){
		if(DS1307_state == RTC_INIT_WR){
			HAL_I2C_Mem_Read_IT(&hi2c1,DS1307_I2C_ADDRESS,DS1307_ADDR_SEC,I2C_MEMADD_SIZE_8BIT,&rtc_init_val_RD,1);
			DS1307_state = RTC_INIT_RD;
		}

		else if(DS1307_state == RTC_INIT_CMPLT){
			vTaskNotifyGiveFromISR(RTC_task_handle,&ret);
			portYIELD_FROM_ISR(ret);
		}

	}
	else if(hi2c->Instance == I2C2){
		if(BME280_state == BME_INIT_WR){
			vTaskNotifyGiveFromISR(BME_init_task_handle,&ret);
			portYIELD_FROM_ISR(ret);
		}

		else if(BME280_state == BME_INIT_CMPLT){
			vTaskNotifyGiveFromISR(BME_task_handle,&ret);
			portYIELD_FROM_ISR(ret);
		}
	}
}

static void UART_task_handler(void* parameters){
	uint32_t flag = 0;
	uint8_t time_format = 0;
	uint16_t tx_size=0;
	while((flag & ALL_INIT_DONE) != ALL_INIT_DONE){
		xTaskNotifyWait(0,0,&flag,portMAX_DELAY);
	}
	while(1){
		uint32_t log_ready_flag=0;
		//ulTaskNotifyTake(pdTRUE,portMAX_DELAY); //wait until next time-stamp is ready in the queue
		while((log_ready_flag & ALL_DATA_READY) != ALL_DATA_READY){
			xTaskNotifyWaitIndexed(0,0,0,&log_ready_flag,portMAX_DELAY);
		}
		log_ready_flag = 0;
		if(xQueueReceive(rtc_data_queue,time_stamp_qRx,portMAX_DELAY) == pdTRUE){
			if(time_stamp_qRx[10] == '\0'){ //24 Hrs format
				time_format=0;
			}
			else if(time_stamp_qRx[13] == '\0'){ // 12 Hrs format;

				time_format=1;
			}
		}
		//ulTaskNotifyTake(pdTRUE,portMAX_DELAY); //wait until next envStr is ready in the queue
		if(xQueueReceive(bme_data_queue,envStr_qRx,portMAX_DELAY) == pdTRUE){

			if(!time_format){ //24 Hrs
				tx_size=30;
				for(uint8_t i =0;i<30;i++){
					if(i<8){
						uart_tx_buf[i] = time_stamp_qRx[i];
					}
					else if(i>11){
						uart_tx_buf[i] = envStr_qRx[i-12];
					}

					else if (i==8)uart_tx_buf[i]=' ';
					else if (i==9)uart_tx_buf[i]='-';
					else if (i==10)uart_tx_buf[i]='-';
					else if (i==11)uart_tx_buf[i]=' ';
				}
			}
			else if(time_format){ //12 Hrs
				tx_size=33;
				for(uint8_t i=0;i<33;i++){
					if(i<11){
						uart_tx_buf[i] = time_stamp_qRx[i];
					}
					else if(i>14){
						uart_tx_buf[i] = envStr_qRx[i-15];
					}
					else if (i==11)uart_tx_buf[i]=' ';
					else if (i==12)uart_tx_buf[i]='-';
					else if (i==13)uart_tx_buf[i]='-';
					else if (i==14)uart_tx_buf[i]=' ';
				}
			}
			//if(xTaskNotifyWait(0,0,&cmd_mode_flag,0) == pdTRUE){
			if(xTaskNotifyWaitIndexed(1,0,ULONG_MAX,&cmd_mode_flag,0)== pdTRUE){
				if(cmd_mode_flag & CMD_MODE_ENTER){
					usart_tx_en=0;
				}
				else if (cmd_mode_flag & CMD_MODE_IDLE){
					HAL_GPIO_WritePin(GPIOD, LD3_Pin,GPIO_PIN_SET); // Orange LED
					usart_tx_en=1;
				}
			}
			if(usart_tx_en){
				HAL_UART_Transmit_IT(&huart2,uart_tx_buf,tx_size);
				ulTaskNotifyTake(pdTRUE,portMAX_DELAY);//  wait until the the next byte UART module is ready for next byte transmit
				//memset(uart_tx_buf,0,sizeof(uart_tx_buf));
			}
		}
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	BaseType_t ret = pdFALSE;

	if(huart->Instance == USART2){
		vTaskNotifyGiveFromISR(UART_task_handle,&ret);
		portYIELD_FROM_ISR(ret);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

	HAL_GPIO_WritePin(GPIOD, LD4_Pin,GPIO_PIN_SET); // GREEN LED
	BaseType_t ret = pdFALSE;

	if(huart->Instance == USART2){

		if((rxByte == '>')&&(CMD_state == CMD_MODE_IDLE)){ //User command prompt
			irx = 0;
			CMD_state = CMD_MODE_ENTER;
			xTaskNotifyIndexedFromISR(UART_task_handle,1,CMD_MODE_ENTER,eSetBits,&ret); // NOTIFY THE usart to stop transmit
		}
		else if((CMD_state == CMD_MODE_ENTER)){
			if(rxByte == '#'){
				CMD_state = CMD_MODE_IDLE;
				xTaskNotifyIndexedFromISR(UART_task_handle,1,CMD_MODE_IDLE,eSetBits,&ret);
				cmd_buf[irx]='\0';
				xQueueSendFromISR(rtc_cmd_queue,cmd_buf,NULL);
				vTaskNotifyGiveFromISR(CMD_task_handle,&ret);
				portYIELD_FROM_ISR(ret);
				irx = 0;
				memset(cmd_buf,0,MAX_CMD_SIZE);
			}

			else{
				cmd_buf[irx++] = rxByte;
			}
		}

		HAL_UART_Receive_IT(&huart2,&rxByte,1);
		portYIELD_FROM_ISR(ret);
	}
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
#ifdef USE_FULL_ASSERT
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
