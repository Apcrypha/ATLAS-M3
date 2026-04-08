/* USER CODE BEGIN Header */
/* Codes that might be reused later
 *
 *
	uint32_t joystickMax = 4095;				//Max timit of the joystick determined by the bit value of the joystick
	uint32_t lowerDeadZone = 2045;				//upper limit of the joystick when scrolling below
	uint32_t higherDeadZone = 2050;				//lower limit of the joystick when scrolling above

	//This is called ternary operation to check simple conditions
	int8_t direction = (joystick > higherDeadZone) ? 1 : (joystick < lowerDeadZone) ? -1 : 0;

	float V_target;
	if (direction == 1){	//forward
		V_target = ((joystick - higherDeadZone) * maxTick) / (joystickMax - higherDeadZone);
	}
	else if (direction == -1){	//backward
		V_target = ((lowerDeadZone - joystick) * maxTick) / lowerDeadZone;
	}
	else{	//when direction is 0 it means the joystick is within the deadzone so its centered
		velocity = 1;	//goes error when timer becomes 0
		__HAL_TIM_SET_COMPARE(htim,channel,velocity);
		return;
	}
 *
 *
 *
 *
 *
 */


/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// #include <> is used for system/language specific includes, while "" is used for project specific
#include <math.h>
#include "mpu6500.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct{//MPU struct
	float aX;		//Accelerometer. Units (g)gravity
	float aY;
	float aZ;

	float gX;		//Gyroscope. Units (dps)degrees per second
	float gY;
	float gZ;

}MPU_data;

typedef struct{ //Joystick values for the Ground vehicle
	//Joystick values must already be normalized from 0-8399

	int16_t leftMotor;
	int16_t rightMotor;

	int8_t leftMotor_Dir;	//+1 means Right turn; -1 means Left turn; 0 means not moving
	int8_t rightMotor_Dir;	//+1 means Forward; -1 means Backward; 0 means not moving

	uint8_t cameraAngle_X;
	uint8_t cameraAngle_Y;

	uint8_t cameraMove;		//Functions as a software interrupt. Only when this becomes 1 does the moveServo() run

}UGV_controls;

typedef struct{	//Parameters to be sent by the ELRS
	int16_t batteryPercentage;

}ELRS_data;



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
DMA_HandleTypeDef hdma_tim5_ch1;
DMA_HandleTypeDef hdma_tim5_ch2;
DMA_HandleTypeDef hdma_tim5_ch3_up;
DMA_HandleTypeDef hdma_tim5_ch4_trig;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */
MPU_data MPU_Data;
UGV_controls UGV_Controls;
ELRS_data ELRS_Data;
HAL_StatusTypeDef status;


float batPercentage = 0.00f;

volatile uint8_t mpuStatus = 0;
volatile uint16_t ADC_reading = 0;
int16_t leftMotor = 0;
int16_t rightMotor = 0;

uint16_t UGV_rightVelocity = 0;						//velocity is in pulse length for PWM
uint16_t UGV_leftVelocity = 0;


uint16_t count;
uint16_t Angle;

#define maxUGV_Tick 1679								//set by the 20Khz counter
#define bufferSize 4096	// The amount of ADC reading to store
uint16_t ADC_buffer[bufferSize];	//Array to temporarily store ADC readings

uint8_t ELRS_buffer[26];	//Received ELRS raw bits will be saved here
uint8_t ELRS_packet[26];	//Sent ELRS raw bits will be saved here

uint8_t rx_byte;              // Holds the single byte currently arriving
uint8_t rx_index = 0;         // Tracks where we are in the packet

uint16_t channels[16];		//Temporary stores channel values here when receiving

const uint8_t crsf_crc8_table[256] = {//Look up table for the CRC
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
    0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
    0x4D, 0x98, 0x32, 0xE7, 0xB3, 0x66, 0xCC, 0x19, 0x64, 0xB1, 0x1B, 0xCE, 0x9A, 0x4F, 0xE5, 0x30,
    0x1F, 0xCA, 0x60, 0xB5, 0xE1, 0x34, 0x9E, 0x4B, 0x36, 0xE3, 0x49, 0x9C, 0xC8, 0x1D, 0xB7, 0x62,
    0xE9, 0x3C, 0x96, 0x43, 0x17, 0xC2, 0x68, 0xBD, 0xC0, 0x15, 0xBF, 0x6A, 0x3E, 0xEB, 0x41, 0x94,
    0xBB, 0x6E, 0xC4, 0x11, 0x45, 0x90, 0x3A, 0xEF, 0x92, 0x47, 0xED, 0x38, 0x6C, 0xB9, 0x13, 0xC6,
    0x9A, 0x4F, 0xE5, 0x30, 0x64, 0xB1, 0x1B, 0xCE, 0xB3, 0x66, 0xCC, 0x19, 0x4D, 0x98, 0x32, 0xE7,
    0xC8, 0x1D, 0xB7, 0x62, 0x36, 0xE3, 0x49, 0x9C, 0xE1, 0x34, 0x9E, 0x4B, 0x1F, 0xCA, 0x60, 0xB5,
    0x3E, 0xEB, 0x41, 0x94, 0xC0, 0x15, 0xBF, 0x6A, 0x17, 0xC2, 0x68, 0xBD, 0xE9, 0x3C, 0x96, 0x43,
    0x6C, 0xB9, 0x13, 0xC6, 0x92, 0x47, 0xED, 0x38, 0x45, 0x90, 0x3A, 0xEF, 0xBB, 0x6E, 0xC4, 0x11,
    0xD7, 0x02, 0xA8, 0x7D, 0x29, 0xFC, 0x56, 0x83, 0xFE, 0x2B, 0x81, 0x54, 0x00, 0xD5, 0x7F, 0xAA,
    0x85, 0x50, 0xFA, 0x2F, 0x7B, 0xAE, 0x04, 0xD1, 0xDB, 0x0E, 0xA4, 0x71, 0x25, 0xF0, 0x5A, 0x8F,
    0x73, 0xA6, 0x0C, 0xD9, 0x8D, 0x58, 0xF2, 0x27, 0x5A, 0x8F, 0x25, 0xF0, 0xA4, 0x71, 0xDB, 0x0E,
    0x21, 0xF4, 0x5E, 0x8B, 0xDF, 0x0A, 0xA0, 0x75, 0x08, 0xDD, 0x77, 0xA2, 0xF6, 0x23, 0x89, 0x5C
};


/*---------------------------------------------------------------------Modular settings------------------------------------------------*/
/*All these Variables are changed only depending on the modular configuration*/

//For the controller
uint16_t rangeJoystick = 2047;	//Normalized ± range. Only supports up to 16bit but can be changed in the future

//For UGV
float UGV_k = 30.0f;	//Multiplier constant of the UGV








/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM5_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

void readMPU();

void moveServo();
void setServoAngle(TIM_HandleTypeDef *htim, uint32_t channel, uint8_t angle);

void UGV_setSpeed(TIM_HandleTypeDef *htim, uint32_t channel,uint16_t motor, int16_t *V_target, uint16_t *V_current);
void UGV_setDirection(GPIO_TypeDef* Port_A, uint16_t Pin_A, GPIO_TypeDef* Port_B, uint16_t Pin_B, int8_t *direction);

void CRSF_Parser(uint8_t* packet, ELRS_data* input);
void extractELRS(uint8_t* buf, uint16_t* ch);
void bitMask(uint16_t ch1);
int ELRS_mapper(int Input, int minInput, int maxInput);
uint8_t crsf_crc8(uint8_t *ptr, uint8_t len);




void readBattery();

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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_TIM5_Init();
  MX_UART4_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_RTC_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&ADC_buffer, bufferSize);	//Start the ADC and tell the DMA to where to store the data

  HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 10240, RTC_WAKEUPCLOCK_RTCCLK_DIV16);	//Start the LSE Timer

  //Start timer 4
  HAL_TIM_Base_Start_IT(&htim4);

  //Starts timer for motor driver
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  //Starts timer for servo
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  //Start UART for ELRS
  HAL_UART_Receive_IT(&huart4, &rx_byte, 1);


	//disable this when MPU is disconnected because Error_Handler() causes a loop.
  status = MPU6500_Init();	//Initialize MPU6500
  if(status != HAL_OK){
      Error_Handler();
  }
  status = MPU6500_EnableDataReadyInterrupts();	//Enable Interrupt pin
  if(status != HAL_OK){
      Error_Handler();
  }


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	  for (uint8_t angle = 0; angle <= 180; angle += 10)
      {
          setServoAngle(&htim3, TIM_CHANNEL_3, angle);
          setServoAngle(&htim3, TIM_CHANNEL_4, angle);
          Angle = angle;
          HAL_Delay(100);
      }

      // Sweep back from 180 to 0 degrees
      for (uint8_t angle = 180; angle > 0; angle -= 10)
      {
          setServoAngle(&htim3, TIM_CHANNEL_3, angle);
          setServoAngle(&htim3, TIM_CHANNEL_4, angle);
          Angle = angle;
          HAL_Delay(100);
      }
      if(mpuStatus == 1){ mpuStatus = 0;  readMPU();  }

      CRSF_Parser(ELRS_packet, &ELRS_Data);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
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

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */
/* The hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV6;
 * -> this determines the sampling rate where the APB2 is divided by the prescaler
 *
 *  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
 *  -> this determins how many cycles the ADC looks before storeing the data for more stable reading
 */


  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the WakeUp
  */
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 10240, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 4;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1679;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 168-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 168-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 5000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 168-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 420000;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PE8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PE9 PE11 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void moveServo(){

}

void setServoAngle(TIM_HandleTypeDef *htim, uint32_t channel, uint8_t angle){
	/*Convert servo pulse time to timer counts
	 *MCU Frequency = 168MHz
	 *Prescaler = 167
	 *Clock tick = 168,000,000 / 167 + 1
	 *Clock tick = 1uS
	 *			From Datasheet
	 *Minimum Counts =  300uS/1uS	  = 300		//0   deg
	 *Maximum Counts =  1,300uS/1uS	  = 1300	//180 deg
	*/

	//Map angle (0-180) via linear interpolation
	uint32_t pulse_length = 300 + (angle * (1300-300)/180);

	__HAL_TIM_SET_COMPARE(htim,channel,pulse_length);	//write duty cycle to PWM
}

void readMPU(){
	uint8_t int_status;
	int16_t accel_x, accel_y, accel_z;
	int16_t gyro_x, gyro_y, gyro_z;

	//Acknowledge that the INP pin was received.
	HAL_I2C_Mem_Read(&hi2c1, (0x68 << 1), 0x3A, I2C_MEMADD_SIZE_8BIT, &int_status, 1, 100);

	// Read raw sensor data
	status = MPU6500_ReadAccel(&accel_x, &accel_y, &accel_z);
	if(status != HAL_OK){ Error_Handler();	}

	status = MPU6500_ReadGyro(&gyro_x, &gyro_y, &gyro_z);
	if(status != HAL_OK){ Error_Handler();	}

	// Convert raw data to physical units
	// For ±8g range: 1g = 4096 LSB
	MPU_Data.aX = accel_x / 4096.0f;
	MPU_Data.aY = accel_y / 4096.0f;
	MPU_Data.aZ = accel_z / 4096.0f;

	// For ±2000°/s range: 1°/s = 16.4 LSB
	// In  degrees per second
	MPU_Data.gX = gyro_x / 16.4f;
	MPU_Data.gY = gyro_y / 16.4f;
	MPU_Data.gZ = gyro_z / 16.4f;

}

void readBattery(){//Converts ADC to battery percentage
	float maxVoltage = 3.029508;	//Max voltage of the Voltage divider
	float minVoltage = 2.163934;

	//This can change depending on the ADC reading of the Voltage Divider
	uint16_t maxADC = 4095;
	uint16_t minADC = 0;

	//linear Interpolation to convert ADC to Voltage reading
	float readingVoltage = ( ((float) (ADC_reading - minADC) * (maxVoltage - minVoltage) ) / (maxADC - minADC) ) + minVoltage;

	//Convert to battery percentage
	batPercentage = ( (readingVoltage - minVoltage) * 100 ) / (maxVoltage - minVoltage);

}

void UGV_setSpeed(TIM_HandleTypeDef *htim, uint32_t channel,uint16_t motor, int16_t *V_target, uint16_t *V_current){

	/*
	  The equation used is discrete proportional (P) controller where:
	  c(t) = k * e(t) * dt + b

	*/

	float dt = 0.005f; // 5ms. This is how fast the speed changes

	float error = *V_target - *V_current;

	//UGV_k = 30.0f -> determines how fast the correction changes

	float acceleration = UGV_k * error;

	*V_current += acceleration * dt + 2; //2 is used as a bias to prevent a zero timer

	__HAL_TIM_SET_COMPARE(htim,channel, *V_current);	//write duty cycle to PWM
}

void UGV_setDirection(GPIO_TypeDef *Port_A, uint16_t Pin_A, GPIO_TypeDef *Port_B, uint16_t Pin_B, int8_t *direction)
{
    if (*direction == 1) {
        HAL_GPIO_WritePin(Port_A, Pin_A, GPIO_PIN_SET);
        HAL_GPIO_WritePin(Port_B, Pin_B, GPIO_PIN_RESET);

    } else if (*direction == -1) {
        HAL_GPIO_WritePin(Port_A, Pin_A, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Port_B, Pin_B, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(Port_A, Pin_A, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Port_B, Pin_B, GPIO_PIN_RESET);
    }

}

int ELRS_mapper(int Input, int minInput, int maxInput){	//Maps values into ELRS ready
	return ((Input - minInput) * (1811 - 172) / (maxInput - minInput)) + 172;
}

void CRSF_Parser(uint8_t* packet, ELRS_data* input) {//CRSF parser
    uint16_t ch[16];

    // --- MAPPING ---
    // Directly assign if they are already in 172-1811 range
    ch[0] = input->batteryPercentage;


    // Fill unused channels with center value (992)
    for(int i = 5; i < 16; i++) ch[i] = 992;

    // --- STEP 2: PACKET HEADER ---
    packet[0] = 0xEE; // Address
    packet[1] = 24;   // Length
    packet[2] = 0x16; // Type (RC Channels)

    // --- BIT-PACKING (The 11-bit Squeeze) ---
    // This part never changes, even if struct changes
    packet[3]  = (uint8_t)(ch[0] & 0x07FF);
    packet[4]  = (uint8_t)((ch[0] >> 8) | (ch[1] << 3));
    packet[5]  = (uint8_t)((ch[1] >> 5) | (ch[2] << 6));
    packet[6]  = (uint8_t)(ch[2] >> 2);
    packet[7]  = (uint8_t)((ch[2] >> 10) | (ch[3] << 1));
    packet[8]  = (uint8_t)((ch[3] >> 7) | (ch[4] << 4));
    packet[9]  = (uint8_t)((ch[4] >> 4) | (ch[5] << 7));
    packet[10] = (uint8_t)(ch[5] >> 1);
    packet[11] = (uint8_t)((ch[5] >> 9) | (ch[6] << 2));
    packet[12] = (uint8_t)((ch[6] >> 6) | (ch[7] << 5));
    packet[13] = (uint8_t)(ch[7] >> 3);
    packet[14] = (uint8_t)(ch[8] & 0x07FF);
    packet[15] = (uint8_t)((ch[8] >> 8) | (ch[9] << 3));
    packet[16] = (uint8_t)((ch[9] >> 5) | (ch[10] << 6));
    packet[17] = (uint8_t)(ch[10] >> 2);
    packet[18] = (uint8_t)((ch[10] >> 10) | (ch[11] << 1));
    packet[19] = (uint8_t)((ch[11] >> 7) | (ch[12] << 4));
    packet[20] = (uint8_t)((ch[12] >> 4) | (ch[13] << 7));
    packet[21] = (uint8_t)(ch[13] >> 1);
    packet[22] = (uint8_t)((ch[13] >> 9) | (ch[14] << 2));
    packet[23] = (uint8_t)((ch[14] >> 6) | (ch[15] << 5));
    packet[24] = (uint8_t)(ch[15] >> 3);

    // --- CRC Calculation---
    packet[25] = crsf_crc8(&packet[2], 23);
}

void extractELRS(uint8_t *buf, uint16_t *ch) {//Extracts the raw ELRS bits to values
    // buf[0] is Address, buf[1] is Length, buf[2] is Type
    // The payload starts at buf[3]

    ch[0]  = ((uint16_t)buf[3]       | (uint16_t)buf[4] << 8)                       & 0x07FF;
    ch[1]  = ((uint16_t)buf[4] >> 3  | (uint16_t)buf[5] << 5)                       & 0x07FF;
    ch[2]  = ((uint16_t)buf[5] >> 6  | (uint16_t)buf[6] << 2 | (uint16_t)buf[7] << 10) & 0x07FF;
    ch[3]  = ((uint16_t)buf[7] >> 1  | (uint16_t)buf[8] << 7)                       & 0x07FF;
    ch[4]  = ((uint16_t)buf[8] >> 4  | (uint16_t)buf[9] << 4)                       & 0x07FF;
    ch[5]  = ((uint16_t)buf[9] >> 7  | (uint16_t)buf[10] << 1 | (uint16_t)buf[11] << 9) & 0x07FF;
    ch[6]  = ((uint16_t)buf[11] >> 2 | (uint16_t)buf[12] << 6)                      & 0x07FF;
    ch[7]  = ((uint16_t)buf[12] >> 5 | (uint16_t)buf[13] << 3)                      & 0x07FF;

    ch[8]  = ((uint16_t)buf[14]      | (uint16_t)buf[15] << 8)                      & 0x07FF;
    ch[9]  = ((uint16_t)buf[15] >> 3 | (uint16_t)buf[16] << 5)                      & 0x07FF;
    ch[10] = ((uint16_t)buf[16] >> 6 | (uint16_t)buf[17] << 2 | (uint16_t)buf[18] << 10) & 0x07FF;
    ch[11] = ((uint16_t)buf[18] >> 1 | (uint16_t)buf[19] << 7)                      & 0x07FF;
    ch[12] = ((uint16_t)buf[19] >> 4 | (uint16_t)buf[20] << 4)                      & 0x07FF;
    ch[13] = ((uint16_t)buf[20] >> 7 | (uint16_t)buf[21] << 1 | (uint16_t)buf[22] << 9) & 0x07FF;
    ch[14] = ((uint16_t)buf[22] >> 2 | (uint16_t)buf[23] << 6)                      & 0x07FF;
    ch[15] = ((uint16_t)buf[23] >> 5 | (uint16_t)buf[24] << 3)                      & 0x07FF;
}

uint8_t crsf_crc8(uint8_t *ptr, uint8_t len) {//Compute CRC for ELRS
	uint8_t crc = 0;
	    while (len--) {
	        crc = crsf_crc8_table[crc ^ *ptr++];
	    }
	    return crc;
}

void bitMask(uint16_t ch1){//decomposes the bits of channel 1 via bit masking
	UGV_Controls.leftMotor_Dir 	 = (ch1 >> 0) & 1;
	UGV_Controls.rightMotor_Dir  = (ch1 >> 1) & 1;
	UGV_Controls.cameraMove		 = (ch1 >> 2) & 1;
}
//--------------------------------------------- ISR Functions---------------------------------
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {//ISR when UART receives something
    if (huart->Instance == UART4) {//checks if UART4
        ELRS_buffer[rx_index++] = rx_byte;

        // 1. Check for Sync Byte (Address)
        // ELRS Receiver typically sends 0xC8 to the Flight Controller
        if (ELRS_buffer[0] != 0xC8 && ELRS_buffer[0] != 0xEE) {
            rx_index = 0;
        }
        // 2. Check if we have a full packet (Header 2 bytes + Payload 24 bytes)
        else if (rx_index >= 26) {
            // Verify CRC
            uint8_t computed_crc = crsf_crc8(&ELRS_buffer[2], 23);

            if (computed_crc == ELRS_buffer[25]) {
                // SUCCESS! Unpack the bits into your struct
            	extractELRS(ELRS_buffer, channels);

                // Map to variables
            	bitMask(channels[0]);
                UGV_Controls.cameraAngle_X = channels[1];
                UGV_Controls.cameraAngle_Y = channels[2];
                UGV_Controls.rightMotor    = channels[3];
                UGV_Controls.leftMotor	   = channels[4];

            }

            // Reset index for next packet
            rx_index = 0;
        }

        // IMPORTANT: Re-enable the interrupt for the next byte
        HAL_UART_Receive_IT(&huart4, &rx_byte, 1);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){	// Gets called whenever there is an overflow on any timer

	//change this use the millis() equivalent of esp32
	if (htim->Instance == TIM4){	//Checks if timer overflow is Timer 4
       //ISR every 5ms

    	UGV_setDirection(GPIOC, GPIO_PIN_6, GPIOA, GPIO_PIN_7, &UGV_Controls.leftMotor_Dir);
    	UGV_setDirection(GPIOE, GPIO_PIN_11, GPIOE, GPIO_PIN_9, &UGV_Controls.rightMotor_Dir);

    	UGV_setSpeed(&htim1, TIM_CHANNEL_3, leftMotor, &UGV_Controls.leftMotor, &UGV_leftVelocity);
    	UGV_setSpeed(&htim1, TIM_CHANNEL_4, rightMotor, &UGV_Controls.rightMotor, &UGV_rightVelocity);

    }
}

void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc){	//LSE Timer ISR
	readBattery();

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {	// Called when ADC buffer is completely filled}
/*
 * Sampling time is calculated by:
 *
 * ADC clock = APB2 / prescaler		--> 84MHz / 4 = 21MHz
 * Conversion time = Sampling time + Conversion time for bits(12.5 cycles for 12bit)	--> 144 Cycles + 12.5 Cycles = 156.5 Cycles
 * Cycle = 1 / ADC clock	--> 1 / 21MHz = 47.619ns
 * Sampling time = Conversion time * Cycle		-->	47.619ns * 156.5 cycles	= 7.452us per sample
 *
 * Make sure that an ISR is always below 50% time than the interrupt
 */

	uint32_t sum;
	for (uint16_t i = 0; i <= 4095; i ++)
	      {
		sum += ADC_buffer[i];
	      }
	ADC_reading = sum/4096;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){	//EXTI ISR
	if(GPIO_Pin == GPIO_PIN_8){
		mpuStatus = 1;	//Set to a volatile variable instead of reading MPU directly to prevent a blocking
	}
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
