/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "stdlib.h"
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CNT_ERROR_TOLERANCE 10
//#define PWM_MAX 1500//4000 //2500 //1500 task 2
#define SERVO_RIGHT_MAX 180
#define SERVO_LEFT_MAX 45
//#define SERVO_RIGHT_MAX 150
//#define SERVO_LEFT_MAX 80
#define SERVO_STRAIGHT 100
#define INTEGRAL_MAX 1000000
#define GREATER_TURN_PWM 1500
#define LESSER_TURN_PWM 500

// for ultrasonic sensor
#define TRIG_PIN GPIO_PIN_5
#define TRIG_PORT GPIOB
#define ECHO_PIN GPIO_PIN_12
#define ECHO_PORT GPIOD
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for DCMotor */
osThreadId_t DCMotorHandle;
const osThreadAttr_t DCMotor_attributes = {
  .name = "DCMotor",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for RefreshOLED */
osThreadId_t RefreshOLEDHandle;
const osThreadAttr_t RefreshOLED_attributes = {
  .name = "RefreshOLED",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for EncoderMotorA */
osThreadId_t EncoderMotorAHandle;
const osThreadAttr_t EncoderMotorA_attributes = {
  .name = "EncoderMotorA",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for EncoderMotorB */
osThreadId_t EncoderMotorBHandle;
const osThreadAttr_t EncoderMotorB_attributes = {
  .name = "EncoderMotorB",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UART_Command_RX */
osThreadId_t UART_Command_RXHandle;
const osThreadAttr_t UART_Command_RX_attributes = {
  .name = "UART_Command_RX",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Gyrohandle */
osThreadId_t GyrohandleHandle;
const osThreadAttr_t Gyrohandle_attributes = {
  .name = "Gyrohandle",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for IRsensortask */
osThreadId_t IRsensortaskHandle;
const osThreadAttr_t IRsensortask_attributes = {
  .name = "IRsensortask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TEST */
osThreadId_t TESTHandle;
const osThreadAttr_t TEST_attributes = {
  .name = "TEST",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task2 */
osThreadId_t Task2Handle;
const osThreadAttr_t Task2_attributes = {
  .name = "Task2",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UltrasoundQueue */
osMessageQueueId_t UltrasoundQueueHandle;
const osMessageQueueAttr_t UltrasoundQueue_attributes = {
  .name = "UltrasoundQueue"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM5_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM4_Init(void);
static void MX_IWDG_Init(void);
void StartDefaultTask(void *argument);
void DCMotor_task(void *argument);
void RefreshOLED_task(void *argument);
void EncoderMotorA_task(void *argument);
void EncoderMotorB_task(void *argument);
void UART_Command_RX_task(void *argument);
void Gyro(void *argument);
void IRsensor(void *argument);
void Testing(void *argument);
void task2(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// New commands from Rpi board will be received and stored in this buffer
//ICM20948 imu;
//uint8_t aRxBuffer[1];
uint8_t aRxBuffer[4];
uint8_t oledRow0[20];
uint8_t oledRow1[20];
uint8_t oledRow2[20];
uint8_t oledRow3[20];
uint8_t oledRow4[20];
uint8_t oledRow5[20];

uint8_t buff[20];
double turningAngle = 0;
double GlobalAngle = 0;
double tempAngle= 0;
uint8_t ICMAddr = 0x68;
uint32_t prevTick;
int16_t angularSpeed;
double oldturningAngle = 0;
double correction;
int PWM_MAX =4000; //2000;//4000 //2500 //1500 task 2
//task2
int stop=0;
int totalDistA = 0;
int totalDist = 900;//1100 //900 + 100 extra + 190 distance from block
int MoveCount =0;
///

char cmd[3];
char hello[7];
int received = 0;
int RA5 =0;
int turning = -1;
int dir = -2;
float input = 0.0;
int32_t diffA = 0;
int32_t diffB = 0;
int32_t pwmValB;
int32_t pwmValA;
int32_t millisOldA = 0;
int32_t errorOldA = 0;
int32_t errorAreaA = 0;
int32_t millisOldB = 0;
int32_t errorOldB = 0;
int32_t errorAreaB = 0;
float targetCount = 0.0;
double targetAngle = 0;

//ultraSonic
uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t Difference = 0;
uint8_t Is_First_Captured = 0;  // is the first value captured ?
uint8_t Distance  = 100; // previously 0
//
//for IR sensor
// TODO only one sensor is in use right now.
uint32_t right_adc;
uint32_t left_adc;
double right_sensor;
double left_sensor;
uint32_t right_sensor_int;
uint32_t left_sensor_int;
uint32_t LPF_SUM_right = 0;
uint32_t LPF_SUM_left = 0;
uint32_t counter = 0;

int8_t start = 0;
int8_t isDone = 0;

//checklist A5
int countA5 =1;

void SendFeedBack(int done){
	if(done == 1){
		HAL_UART_Transmit(&huart3, "ACK\n", 4, 0xFFFF);

	}
	else if(done == -1){
		HAL_UART_Transmit(&huart3, "-1\n", 3, 0xFFFF);
	}
	else if(done == 2){
		HAL_UART_Transmit(&huart3, hello, 7, 0xFFFF);
		}

}

void HAL_GPIO_EXTI_Callback( uint16_t GPIO_Pin ) {

	if ( GPIO_Pin == USER_PB_Pin) {
		// toggle LED
		if (start == 0){
			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_10);
			start = 1;
		}
		else
			start = 0;
 	    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	char hello[20];
	BaseType_t xHigherPriorityTaskWoken = 0; //If 0 => higher priority task is not woken, else woken
	HAL_IWDG_Refresh(&hiwdg); //If this doesn't execute, system resets
	if(htim->Instance == TIM4)
	{
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // if the interrupt source is channel1
		{
			if (Is_First_Captured==0) // if the first value is not captured
			{
				IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
				Is_First_Captured = 1;  // set the first captured as true
				// Now change the polarity to falling edge
				__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
			}
			else if (Is_First_Captured==1)   // if the first is already captured
			{
				IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // read second value
				__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter
				if (IC_Val2 > IC_Val1)
				{
					Difference = IC_Val2-IC_Val1;
				}
				else if (IC_Val1 > IC_Val2)
				{
					Difference = (65534 - IC_Val1) + IC_Val2;
				}

//				Distance = Difference * 0.034 / 2 * 4.9;
				Distance = Difference * 0.034 / 2 * 4.9;
				Is_First_Captured = 0; // set it back to false
				//xQueueSendFromISR(UltrasoundQueueHandle, &Distance, &xHigherPriorityTaskWoken);
				// set polarity to rising edge
				__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
				__HAL_TIM_DISABLE_IT(&htim4, TIM_IT_CC1);
				portEND_SWITCHING_ISR(xHigherPriorityTaskWoken); //Causes context switch to highest priority task if xHigherPriorityTaskWoken, else nothing happens

			}
			sprintf(hello, "Dist: %5dcm\0", Distance);
			OLED_ShowString(10,10,hello);

			 // Enable the interrupt again for the next measurement
			//__HAL_TIM_ENABLE_IT(&htim4, TIM_IT_CC1);
		}
	}
}

void delay (uint16_t delay){
	__HAL_TIM_SET_COUNTER(&htim4, 0);
	while (__HAL_TIM_GET_COUNTER(&htim4) < delay);
}

void HCSR04_Read (void)
{
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	delay(10);  // wait for 10 us
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low

	__HAL_TIM_ENABLE_IT(&htim4, TIM_IT_CC1);
}


void readByte(uint8_t addr, uint8_t* data){
	buff[0] = addr;
	HAL_I2C_Master_Transmit(&hi2c1, ICMAddr<<1, buff, 1, 10);
	HAL_I2C_Master_Receive(&hi2c1, ICMAddr<<1, data, 2, 20);
}

void writeByte(uint8_t addr, uint8_t data){
	buff[0] = addr;
	buff[1] = data;
	HAL_I2C_Master_Transmit(&hi2c1, ICMAddr << 1, buff, 2, 20);
}

void gyroStart(){
	writeByte(0x07, 0x07);
	osDelay(10);
	writeByte(0x07, 0x00);
	osDelay(10);
}

void gyroInit(){

	writeByte(0x06, 0x00);
	osDelay(10);
	writeByte(0x03, 0x80);
	osDelay(10);
	writeByte(0x07, 0x07);
	osDelay(10);
	writeByte(0x06, 0x01);
	osDelay(10);
	writeByte(0x7F, 0x20);
	osDelay(10);
	writeByte(0x01, 0x2F);
	osDelay(10);
	writeByte(0x0, 0x00);
	osDelay(10);
	writeByte(0x7F, 0x00);
	osDelay(10);
	writeByte(0x07, 0x00);
	osDelay(10);
}

/* Start PWM for DC and servor motors
	 * htim8 channel 1: PWMA - DC motor A
	 * htim8 channel 2: PWMB - DC motor B
	 * htim1 channel 4: Servo motor*/


void StartPMWVal(void){
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}

void SetPinReverse(void){
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, DIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, DIN2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
}

void SetPinForward(void){
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, DIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, DIN1_Pin, GPIO_PIN_SET);
}

void SetMotorAForward(void){
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
}
void SetMotorAReverse(void){
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
}

void SetMotorAStop(void){
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
}

void SetMotorBForward(void){
	HAL_GPIO_WritePin(GPIOB, DIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, DIN1_Pin, GPIO_PIN_SET);
}
void SetMotorBReverse(void){
	HAL_GPIO_WritePin(GPIOB, DIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, DIN2_Pin, GPIO_PIN_SET);
}
void SetMotorBStop(void){
	HAL_GPIO_WritePin(GPIOB, DIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, DIN2_Pin, GPIO_PIN_RESET);
}

void SetPinStop(void){
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, DIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, DIN1_Pin, GPIO_PIN_RESET);
}

void TurnFullLeft(void){
	turning = 1;
	htim1.Instance -> CCR4 = SERVO_LEFT_MAX;
}
void TurnFullRight(void){
	turning = 1;
	htim1.Instance -> CCR4 = SERVO_RIGHT_MAX;
}
// assume 90 deg.
void TurnStraighten(void){
	turning = 1;
	htim1.Instance -> CCR4 = SERVO_STRAIGHT;
}

void ResetValues(void)
{
	errorOldA = 0;
	errorOldB = 0;
	errorAreaA = 0;
	errorAreaB = 0;
	pwmValA = 0;
	pwmValB = 0;
	turningAngle = 0;
	//targetCount=0;
//	TempAngle =0;
	isDone = 0;
	//Distance =100;
	__HAL_TIM_SET_COUNTER(&htim2, 0);
	__HAL_TIM_SET_COUNTER(&htim5, 0);
}

void ResetFT(void){
	dir=1;
	turning =1;
	isDone=0;
	turningAngle=0;
}

void ResetBT(void){
	dir=-1;
	turning =1;
	isDone=0;
	turningAngle=0;
}
int CalculateDist(int encoderTick){
	//targetCount = ((800 - 28) / 207.345) * 1560;
	double dist= ((double)encoderTick/1560) * 207.345;

	return dist;
}

int CalcPIDA_Dist(void){
	int32_t errorA = 0;
	int32_t errorRate = 0;

	float_t kp = 2;//.2; //before 0.5 max 2500
	float_t ki = 0.001;
	float_t kd = 35;

	int32_t dt = 0;

	int32_t millisNow;
	int32_t pwmValA_temp;
	float count = abs(__HAL_TIM_GET_COUNTER(&htim2));
	if(targetCount - count > 0)
	{
		millisNow = HAL_GetTick();
		dt = millisNow - millisOldA;
		millisOldA = millisNow;
		errorA = targetCount - count;
		errorRate = (errorA - errorOldA)/dt;
		errorOldA = errorA;
		errorAreaA = errorAreaA + errorA*dt;

		if(errorAreaA > INTEGRAL_MAX)
			errorAreaA = INTEGRAL_MAX;
		else if (errorAreaA < -INTEGRAL_MAX)
			errorAreaA = -INTEGRAL_MAX;

		pwmValA_temp = kp*errorA + kd*errorRate + ki*errorAreaA + 110;

		if(pwmValA_temp > PWM_MAX){
			pwmValA_temp = PWM_MAX;
		}
		else if(pwmValA_temp < -PWM_MAX){
			pwmValA_temp = -PWM_MAX;
		}
		return pwmValA_temp;
	}
	else
	{

	}
	return 0;
}

int CalcPIDB_Dist(void){
	int32_t errorB = 0;
	int32_t errorRate = 0;
	float_t kp = 2;
	float_t ki = 0.001;
	float_t kd = 30;

	int32_t dt = 0;
	int32_t millisNow;
	int32_t pwmValB_temp;
	float count = abs(__HAL_TIM_GET_COUNTER(&htim5));
	if(targetCount - count-20 > 0)
	{
		millisNow = HAL_GetTick();
		dt = millisNow - millisOldB;
		millisOldB = millisNow;
		errorB = targetCount - count;
		errorRate = (errorB - errorOldB)/dt;
		errorOldB = errorB;
		errorAreaB = errorAreaB + errorB*dt;
		if(errorAreaB > INTEGRAL_MAX)
			errorAreaB = INTEGRAL_MAX;
		else if (errorAreaB < -INTEGRAL_MAX)
			errorAreaB = -INTEGRAL_MAX;

		pwmValB_temp = kp*errorB + kd*errorRate + ki*errorAreaB;

		if(pwmValB_temp > PWM_MAX){
			pwmValB_temp = PWM_MAX;
		}
		else if(pwmValB_temp < -PWM_MAX){
			pwmValB_temp = -PWM_MAX;
		}
		return pwmValB_temp;
	}
	else
	{
		if(dir==2 || dir ==3){
		//OLED_ShowString(0,0,"done");
			isDone=1;
			if(strcmp(cmd,"FA")==0){
				SendFeedBack(2);
			}

		}
		else if(turning == 0 && (dir == 1 || dir == -1)){
			dir=0;
			targetCount=0;
		}

	}
	return 0;
}



int ServoCorrection(void){
	int corrMultiplier = 3;

	if(dir == 1 || dir ==2 || dir==4){
		correction = (double)(SERVO_STRAIGHT + turningAngle*corrMultiplier);

	}
	else if(dir == -1 || dir ==3)
		correction = (double)(SERVO_STRAIGHT - turningAngle*corrMultiplier);

//	else if(dir==2 || 3)
//		correction = (double)(SERVO_STRAIGHT - turningAngle*corrMultiplier);

	if(correction > SERVO_RIGHT_MAX){
		correction = SERVO_RIGHT_MAX;
	}
	else if(correction < SERVO_LEFT_MAX){
		correction = SERVO_LEFT_MAX;
	}
	return correction;
}

int ServoCorrectionTurn(void){
	int corrMultiplier = 2;

	if(dir == 1)
		correction = (double)(SERVO_STRAIGHT + (tempAngle-targetAngle)*corrMultiplier);
	else if(dir == -1)
		correction = (double)(SERVO_STRAIGHT - (tempAngle-targetAngle)*corrMultiplier);

	if(correction > SERVO_RIGHT_MAX){
		correction = SERVO_RIGHT_MAX;
	}
	else if(correction < SERVO_LEFT_MAX){
		correction = SERVO_LEFT_MAX;
	}
	return correction;
}


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
  MX_TIM8_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_TIM5_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_TIM4_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */

  // Delay to allow proper initialization
  HAL_Delay(1000);  // Wait 1 second before starting UART reception

  // Initialize OLED
  OLED_Init();

  // Start UART reception for USART3 (since you're using huart3 for communication)
  HAL_UART_Receive_IT(&huart3, (uint8_t *)aRxBuffer, 4);  // Start UART reception

  // Other initializations
  millisOldA = HAL_GetTick();
  millisOldB = HAL_GetTick();
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  __HAL_TIM_SET_COUNTER(&htim5, 0);
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);  // for ultra sonic sensor

  // Optionally add another delay here if needed
  HAL_Delay(1000);  // Wait 1 second to ensure everything is stable

  // If IMU initialization is required
  // IMU_Initialise(&imu, &hi2c1, &huart3);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of UltrasoundQueue */
  UltrasoundQueueHandle = osMessageQueueNew (16, sizeof(uint16_t), &UltrasoundQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of DCMotor */
  DCMotorHandle = osThreadNew(DCMotor_task, NULL, &DCMotor_attributes);

  /* creation of RefreshOLED */
  RefreshOLEDHandle = osThreadNew(RefreshOLED_task, NULL, &RefreshOLED_attributes);

  /* creation of EncoderMotorA */
  EncoderMotorAHandle = osThreadNew(EncoderMotorA_task, NULL, &EncoderMotorA_attributes);

  /* creation of EncoderMotorB */
  EncoderMotorBHandle = osThreadNew(EncoderMotorB_task, NULL, &EncoderMotorB_attributes);

  /* creation of UART_Command_RX */
  UART_Command_RXHandle = osThreadNew(UART_Command_RX_task, NULL, &UART_Command_RX_attributes);

  /* creation of Gyrohandle */
  GyrohandleHandle = osThreadNew(Gyro, NULL, &Gyrohandle_attributes);

  /* creation of IRsensortask */
  IRsensortaskHandle = osThreadNew(IRsensor, NULL, &IRsensortask_attributes);

  /* creation of TEST */
  TESTHandle = osThreadNew(Testing, NULL, &TEST_attributes);

  /* creation of Task2 */
  Task2Handle = osThreadNew(task2, NULL, &Task2_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

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
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_128;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

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
  htim1.Init.Prescaler = 160;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 72-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
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
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 7199;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DC_Pin
                          |LED3_Pin|Gyro_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, AIN2_Pin|AIN1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DIN1_Pin|DIN2_Pin|TRIG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : OLED_SCL_Pin OLED_SDA_Pin OLED_RST_Pin OLED_DC_Pin
                           LED3_Pin Gyro_Pin */
  GPIO_InitStruct.Pin = OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DC_Pin
                          |LED3_Pin|Gyro_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : AIN2_Pin AIN1_Pin */
  GPIO_InitStruct.Pin = AIN2_Pin|AIN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DIN1_Pin DIN2_Pin */
  GPIO_InitStruct.Pin = DIN1_Pin|DIN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_PB_Pin */
  GPIO_InitStruct.Pin = USER_PB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_PB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TRIG_Pin */
  GPIO_InitStruct.Pin = TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TRIG_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* When ever there is a message received, this function interrupt will be called*/

void OLED_ShowNum(uint8_t x, uint8_t y, int num, uint8_t len, uint8_t size)
{
    char str[16];  // Buffer to hold the string representation of the number
    snprintf(str, sizeof(str), "%d", num);  // Convert the number to a string
    OLED_ShowString(x, y, (uint8_t *)str);  // Display the string on the OLED
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    UNUSED(huart);
    // Copy command and terminate with null character

    if(received ==0){
    strncpy(cmd, (char *)aRxBuffer, 2);  // Copy the first two characters
    cmd[2] = '\0';  // Null-terminate the string

    // Parse input value from the buffer
    input = (int)(aRxBuffer[2] - '0') * 100 + (int)(aRxBuffer[3] - '0') * 10;
    osDelay(500);

    // Initialize target count
    targetCount = 0.0;
    // Debugging: Show the received command and input on the [;OLED
    OLED_ShowString(20, 0, (uint8_t *)cmd);  // Show label for command
//    OLED_ShowString(0, 30, (uint8_t *)aRxBuffer);    // Show actual command

    // Check if command is FW or BW
    if (strcmp(cmd, "FW") == 0)
    {
    	//indoor
    	targetCount = ((input - 58) / 207.345) * 1560; //78
//outdoor
//        targetCount = ((input - 26) / 207.345) * 1560;

        //SendFeedBack(1);
    }
    else if (strcmp(cmd, "BW") == 0)
      {

         targetCount = ((input - 38) / 207.345) * 1560;// indoorv //138
    	//targetCount = ((input - 68) / 207.345) * 1560; //outdoor
          //SendFeedBack(1);
      }

    else if(strcmp(cmd,"RS")==0){
//    	osDelayUntil(2000);
//    	SendFeedBack(1);
    }
//    else if((strcmp(cmd,"GO")==0)|| (strcmp(cmd,"RW")==0)){
//    	SendFeedBack(1);
//    }
    else if(strcmp(cmd, "RW")==0 || strcmp(cmd, "PW")==0 || strcmp(cmd, "KW")==0){ //image recon fail
    	sprintf(hello, "SNAP_%d\n", (int)(aRxBuffer[3] - '0'));

    }

    // Restart UART reception (4 bytes expected: 2 for command, 2 for input)
    else if(strcmp(cmd, "FA")==0){ //image recon fail
    	sprintf(hello, "SNAP_%d\n", (int)(aRxBuffer[3] - '0'));
    }

    received =1;
    __HAL_UART_FLUSH_DRREGISTER(&huart3);
    memset(aRxBuffer, '0', sizeof(aRxBuffer));  // Clear buffer after processing
    }

    HAL_UART_Receive_IT(&huart3, (uint8_t *)aRxBuffer, 4);  // Restart UART reception
}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
//	  sprintf(oledRow1, "eCnt: %5d", __HAL_TIM_GET_COUNTER(&htim2)); // left wheel
//
//	  sprintf(oledRow2, "Rd:%3d Ld:%3d", (int)right_sensor_int, (int)left_sensor_int);
	  sprintf(oledRow3, "Tang : %3d", (long)turningAngle);//
//	  sprintf(oledRow3, "DistT:%d", totalDist);//
	  sprintf(oledRow4, "G: %3d", (long)GlobalAngle);
//	  sprintf(oledRow3, "OLDTang : %5d", (long)oldturningAngle);//
//	  sprintf(oledRow3, "Rd : %3d", (int)right_sensor_int);
//	  sprintf(oledRow2, "recvd: %d", received); //
//	  sprintf(oledRow3, "Turn: %d", turning);
//	  sprintf(oledRow3, "Tcount: %5d", (long)targetCount);
//	  sprintf(oledRow4, "Dist: %5dcm\0", Distance);
////	  sprintf(oledRow5, "IsDone: %d", isDone);
////	  OLED_ShowString(0,50,(long)input);
	  OLED_ShowString(0,20, cmd);
//	  sprintf(oledRow4, "Rx : %s", aRxBuffer);
//	  OLED_ShowString(0, 50, (char *)aRxBuffer);

//	  sprintf(oledRow5, "cnt: %5d", (long)targetCount);
	  sprintf(oledRow5, "cnt: %2d", (long)targetAngle);
//	  sprintf(oledRow2, "MC: %d", MoveCount);
//	  sprintf(oledRow1, "PB: %5d", pwmValB);
//	  sprintf(oledRow0, "PA: %5d", pwmValA);
//	  sprintf(oledRow1, "IRL: %5d", left_sensor_int);
//	  sprintf(oledRow0, "IRR: %5d", right_sensor_int);
////	  HAL_IWDG_Refresh(&hiwdg);
//	  sprintf(oledRow2, "ID: %d", isDone);
//	  sprintf(oledRow1, "DIR: %d", dir);

	  osDelay(1);

  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_DCMotor_task */
/**
* @brief Function implementing the DCMotor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DCMotor_task */
void DCMotor_task(void *argument)
{
  /* USER CODE BEGIN DCMotor_task */
	StartPMWVal();
	int angleOffset = 6;
	double error;
	int baseSpeed = 3000; // Base speed for motors during normal movement 1500 prev
	int minSpeed = 600;   // Minimum speed during slow down near the target angle
	while(1)
	{

		switch (dir)
		{
			case 0:
				// send completed cmd back to RPi, ready to execute new command
				SetPinStop();
				htim1.Instance -> CCR4 = SERVO_STRAIGHT;
				//ResetValues();
				//osDelay(100);
				OLED_ShowString(0,50,"Sending Feedback");
				//osDelay(100);
				if(strcmp(cmd,"RW")==0 || strcmp(cmd,"PW")==0 || strcmp(cmd,"KM")==0){
					Distance=100;
					SendFeedBack(2);
				}
				else
					SendFeedBack(1);
				ResetValues();

				dir = -2; // should go to default
				//osDelay(100);
				break;
			case 1:
//				__HAL_TIM_SET_COUNTER(&htim2, 0);
//				__HAL_TIM_SET_COUNTER(&htim5, 0);
				//SetPinForward();
				SetPinForward();
				if(turning == 0){
					SetPinForward();
					htim1.Instance -> CCR4 = ServoCorrection();
//					if(strcmp(cmd,"GO")==0){
//						pwmValA = 2000;
//						pwmValB = 2000;
//					}
				}
				else if(turning == 1 && (strcmp(cmd, "FL") == 0))// left turn
				{
					htim1.Instance -> CCR4 = SERVO_LEFT_MAX+8;
					//htim1.Instance -> CCR4 = SERVO_LEFT_MAX+15;
					//OLED_ShowString(30,30,"LEsdFT");
					//osDelay(150);
//					pwmValA = 300;
//					pwmValB = 1500;
					error = fabs(turningAngle - targetAngle);
					if (error < 30) {  // If close to the target, slow down
					      pwmValB = fmax(4000 * error /30.0, 800);
					      pwmValA = fmax(300 * error / 20.0, 300);
					     // pwmValA=0;
					} else {
						pwmValA = 300;
						pwmValB = 4000;
					}
				}

				else if(turning == 1 && strcmp(cmd, "FR") == 0){
					//OLED_ShowString(30,20,"FR");
//					htim1.Instance -> CCR4 = SERVO_RIGHT_MAX;
//					osDelay(150);
//					pwmValA = 1500;
//					pwmValB = 300;
			          error = fabs(turningAngle - targetAngle);
//						//my room
			          if (error < 30) {  // Slow down near the target
			        	  pwmValA = fmax(4000 * error / 30.0, 900);
			        	  //pwmValB = 0;
			        	  pwmValB = fmax(300 * error / 20.0, 600);
			          } else {
			        	  pwmValA = 4000;
			        	  pwmValB = 1000 	;

			          }

				}

				//TASK 2
				else if(turning ==1 && strcmp(cmd,"HR")==0){
					htim1.Instance -> CCR4 = SERVO_RIGHT_MAX;
						//osDelay(150);
						error = fabs(turningAngle - targetAngle);
					//						//my room
						if (error < 20) {  // Slow down near the target
							pwmValA = fmax(3000 * error / 30.0,800);
							pwmValB = fmax(300 * error / 20.0, 800);
						} else {
							pwmValA = 3000; //3000
							pwmValB = 300;
						}
				}
				else if(turning ==1 && strcmp(cmd,"HL")==0){
					htim1.Instance -> CCR4 = SERVO_LEFT_MAX;
					//osDelay(150);
					error = fabs(turningAngle - targetAngle);
									//						//my room
					if (error < 20) {  // Slow down near the target
						pwmValB = fmax(3000 * error / 30.0, 800);
						pwmValA = fmax(300 * error / 20.0, 800);
					} else {
						pwmValB = 3000;
						pwmValA = 300;
							}
								}
				else if(turning ==1 && strcmp(cmd,"RR")==0){
						htim1.Instance -> CCR4 = SERVO_RIGHT_MAX;
						//osDelay(150);
						error = fabs(turningAngle - targetAngle);
									//						//my room
						if (error < 30) {  // Slow down near the target
							pwmValA = fmax(4000 * error / 40.0, 800);
							pwmValB = fmax(300 * error / 20.0, 800);
										}
						else {
							pwmValA = 4000; // 3000
							pwmValB = 500;
										}
					}
				else if(turning ==1 && strcmp(cmd,"UR")==0){
										//htim1.Instance -> CCR4 = SERVO_RIGHT_MAX-10;
										htim1.Instance -> CCR4 = SERVO_RIGHT_MAX-10;
										//htim1.Instance -> CCR4 = SERVO_RIGHT_MAX-20;
										//osDelay(150);
										error = fabs(turningAngle - targetAngle);
													//						//my room
										if (error < 25) {  // Slow down near the target
											pwmValA = fmax(4000 * error / 30.0, 800);
											pwmValB = fmax(300 * error / 20.0, 800);
														}
										else {
											pwmValA = 4000; //3500
											pwmValB = 500;
														}
				}
				else if(turning == 1 && (strcmp(cmd, "LL") == 0))// left turn
								{
									htim1.Instance -> CCR4 = SERVO_LEFT_MAX+12;
									//OLED_ShowString(30,30,"LEsdFT");
									//osDelay(150);

									error = fabs(turningAngle - targetAngle);
									if (error < 35) {  // If close to the target, slow down
									      pwmValB = fmax(5000 * error /50.0, 800);
									      pwmValA = fmax(600 * error / 20.0, 800);
									} else {
										pwmValA = 500;
										pwmValB = 5000;
									}
								}
				else if(turning == 1 && (strcmp(cmd, "UL") == 0))// left turn
								{
									htim1.Instance -> CCR4 = SERVO_LEFT_MAX+15;
									//OLED_ShowString(30,30,"LEsdFT");
									//osDelay(150);
				//					pwmValA = 300;
				//					pwmValB = 1500;
									error = fabs(turningAngle - targetAngle);
									if (error < 25) {  // If close to the target, slow down
									      pwmValB = fmax(3500 * error /40.0, 800);
									      pwmValA = fmax(300 * error / 20.0, 800);
									} else {
										pwmValA = 500;
										pwmValB = 3500;
									}
								}
//				else if(turning == 1 && (strcmp(cmd, "CC") == 0))// left turn
//				{
//					if(targetAngle>0){
//									htim1.Instance -> CCR4 = SERVO_LEFT_MAX+12;
//									//OLED_ShowString(30,30,"LEsdFT");
//									//osDelay(150);
//				//					pwmValA = 300;
//				//					pwmValB = 1500;
//									error = fabs(turningAngle - targetAngle);
//									if (error < 25) {  // If close to the target, slow down
//									      pwmValB = fmax(baseSpeed * error /50.0, minSpeed);
//									      pwmValA = fmax(300 * error / 20.0, minSpeed);
//									} else {
//										pwmValA = 500;
//										pwmValB = baseSpeed;
//									}
//								}
//					else if(targetAngle<0){
//						htim1.Instance -> CCR4 = SERVO_RIGHT_MAX;
//						osDelay(150);
//						error = fabs(turningAngle - targetAngle);
//									//						//my room
//						if (error < 20) {  // Slow down near the target
//							pwmValA = fmax(baseSpeed * error / 30.0, minSpeed);
//							pwmValB = fmax(300 * error / 20.0, minSpeed);
//										}
//						else {
//							pwmValA = baseSpeed;
//							pwmValB = 500;
//										}
//					}}

				break;

			case 2: //forward
				//OLED_ShowString(0,50,"dir 222222");
				SetPinForward();
				htim1.Instance -> CCR4 = ServoCorrection();
				if(isDone){
					if(strcmp(cmd,"FR")==0){
						targetAngle=-90;
						htim1.Instance -> CCR4 = SERVO_RIGHT_MAX;
						SetPinStop();
						osDelay(100);
						ResetFT();
						osDelay(100);

					}
					if(strcmp(cmd,"FL")==0){
						targetAngle=90;
						SetPinStop();
						osDelay(100);
						ResetFT();
						osDelay(100);
					}
					if(strcmp(cmd,"BR")==0)	{
						targetAngle=90;
						SetPinStop();
						osDelay(100);
						ResetBT();
						osDelay(100);
					}
					if(strcmp(cmd,"BL")==0){
						targetAngle=-90;
						SetPinStop();
						osDelay(100);
						ResetBT();
						osDelay(100);
					}


				}
				break;
			case 3: //reverse
				OLED_ShowString(0,50,"dir 33333");
				SetPinReverse();
				htim1.Instance -> CCR4 = ServoCorrection();
				if(isDone){
					if(strcmp(cmd,"FR")==0){
						targetAngle=-90;
						htim1.Instance -> CCR4 = SERVO_RIGHT_MAX;
						SetPinStop();
						//osDelay(100);
						ResetFT();
						osDelay(100);

					}
					if(strcmp(cmd,"FL")==0){
						targetAngle=90;
						SetPinStop();
						//osDelay(100);
						ResetFT();
						osDelay(100);
					}
					if(strcmp(cmd,"BR")==0)	{
						targetAngle=90;
						SetPinStop();
						//osDelay(100);
						ResetBT();
						osDelay(100);
					}
					if(strcmp(cmd,"BL")==0){
						targetAngle=-90;
						SetPinStop();
						//osDelay(100);
						ResetBT();
						osDelay(100);
					}
					if(strcmp(cmd,"FA")==0){
						SetPinStop();
						isDone=0;
						dir=-2;
						osDelay(100);

					}

				}
				break;
			case 4:
				SetPinForward();
				htim1.Instance -> CCR4 = ServoCorrection();
				pwmValA=1500;
				pwmValB=1500;
//
				break;

			case -1:
//				__HAL_TIM_SET_COUNTER(&htim2, 0);
//				__HAL_TIM_SET_COUNTER(&htim5, 0);
				SetPinReverse();
				if(turning == 0)
					htim1.Instance -> CCR4 = ServoCorrection();

				else if(turning == 1 && (strcmp(cmd, "BL") == 0))// reverse left turn
				{
//					pwmValB = GREATER_TURN_PWM;
//					pwmValA = LESSER_TURN_PWM;
					//htim1.Instance -> CCR4 = SERVO_LEFT_MAX+5;
					//osDelay(100)
					htim1.Instance -> CCR4 = SERVO_LEFT_MAX;
					///INDOOR
					error = fabs(turningAngle - targetAngle);
					if (error < 40) {  // If close to the target, slow down
						pwmValB = fmax(4000 * error /40.0, 800);
						pwmValA = fmax(1400 * error / 20.0, 800);

					} else {
						pwmValA = 1400;
						pwmValB =4000;

					}

					//my room
//					error = fabs(turningAngle - targetAngle);
//					if (error < 30) {  // If close to the target, slow down
//						pwmValB = fmax(4000 * error /40.0, 800);
//						pwmValA = fmax(300 * error / 20.0, 800);
//
//					} else {
//						pwmValA = 800;
//						pwmValB =4000;
//
//					}



//					if (error < 30) {  // If close to the target, slow down
//											pwmValB = fmax(3000 * error /40.0, 600);
//											pwmValA = fmax(300 * error / 20.0, 600);
//										} else {
//											pwmValA = 600;
//											pwmValB =3000;
//
//										}



				}
				else if(turning == 1 && (strcmp(cmd, "BR") == 0))// reverse right turn
				{

					//htim1.Instance -> CCR4 = SERVO_RIGHT_MAX;
					//osDelay(150);
					htim1.Instance -> CCR4 = SERVO_RIGHT_MAX;
					//indoor
					error = fabs(turningAngle - targetAngle);
					 if (error < 40) {  // Slow down near the target
						 pwmValA = fmax(4200 * error / 40.0, 600);
						 pwmValB = fmax(600 * error / 20.0, 600);
					 } else {
						 pwmValA = 4200;
						 pwmValB = 400;
					 }

					 //my room
//					error = fabs(turningAngle - targetAngle);
//					 if (error < 30) {  // Slow down near the target
//						 pwmValA = fmax(4500 * error / 40.0, 600);
//						 pwmValB = fmax(300 * error / 20.0, 600);
//					 } else {
//						 pwmValA = 4500;
//						 pwmValB = 300;
//					 }
				}

				break;
			default:
				SetPinStop();
		}
		// Reverse if sensor detects car is <= 100mm (10cm)
//		if(cmd == 'a' && right_sensor_int <= 10){
////			cmd = '-';
////			sprintf(oledRow4, "Obstacle!");
////			ResetValues();
////			TurnStraighten();
////			targetCount = ((190 - 30)/207.345) * 1560;
////			turning = 0;
////			dir = -1;
////			cmd = '-';
////			continue;
//		}


		//30x30 indoor
			    if(turning == 1)
			    {
			    	//if((strcmp(cmd, "FL") == 0) && turningAngle >= targetAngle-5)//my rm
			    	if((strcmp(cmd, "FL") == 0) && turningAngle >= targetAngle-2) // outdoor
			    	//if((strcmp(cmd, "FL") == 0) && turningAngle >= targetAngle)//-1){//-4) // indoor
			    	{
			    				   SetPinStop();
			    				   turning = 0;
			    				   ResetValues();
			    				   osDelay(100);
			    				   htim1.Instance -> CCR4 = SERVO_STRAIGHT;
			    				   osDelay(200);

					   	 	   	  // targetCount = ((55-28)/207.345) * 1560; //my rm

			    				   targetCount=0; //indoor
			    				   //targetCount = ((40-28)/207.345) * 1560; //outdoor
					   	 	   	   //dir = 1; //outdoor
					   	 	   	   dir=-1;
			    			   }
			    	else if((strcmp(cmd, "BR") == 0) && turningAngle >= targetAngle-2) // indoor task 1
			    	//else if((strcmp(cmd, "BR") == 0) && turningAngle >= targetAngle-1)//outdoor//-3)
			    				{
			    				   SetPinStop();
			    				   turning = 0;
			    				   ResetValues();
			    				   osDelay(100);
			    				   htim1.Instance -> CCR4 = SERVO_STRAIGHT;
			    				   osDelay(200);


			    				   //targetCount = ((70-28)/207.345) * 1560;
			    				   targetCount=0;
			    				   //dir =-1;
			    				   dir=1;

			    				   }
			    			   else if((strcmp(cmd, "BL") == 0) && turningAngle <= targetAngle+1 ) //indoor
			    			 //  else if((strcmp(cmd, "BL") == 0) && turningAngle <= targetAngle+1)// ) // // my rm
			    			   {
			    				   SetPinStop();
			    				   turning = 0;
			    				   ResetValues();
			    				   osDelay(100);
			    				   htim1.Instance -> CCR4 = SERVO_STRAIGHT;
			    				   osDelay(200);

			    				   targetCount = ((60-28)/207.345) * 1560;
			    				   //targetCount= ((65-28)/207.345) * 1560;
			    				   //targetCount=0;
			    				   dir = -1;

			    			   }

			    			   else if((strcmp(cmd, "FR") == 0) && turningAngle <= targetAngle+1){ // task 1 indoor
			    			   //else if((strcmp(cmd, "FR") == 0) && turningAngle <= targetAngle+1){ //outdoor task2 //+ angleOffset+2 indoor
			    				   SetPinStop();
			    				   turning = 0;
			    				   ResetValues();
			    				   osDelay(100);
			    				   htim1.Instance -> CCR4 = SERVO_STRAIGHT;
			    				   targetCount = ((90-28)/207.345) * 1560;
			    				   	 	   	   osDelay(100);

			    				   	 	   	   //my rm

			    				   	 	   	   dir = -1;
			    				   //	 	   	   dir = 0;

			    			   }
			    	/////////////////////////////TASK 2///////////////////////////////////////////////////
			    			   else if((strcmp(cmd, "RR") == 0) && turningAngle <= targetAngle+3){
			    				   SetPinStop();

			    				   turning = 0;
			    				   osDelay(100);
			    				   htim1.Instance -> CCR4 = SERVO_STRAIGHT;
			    				   osDelay(100);
			    				   ResetValues();
//			    				   targetCount = ((85-28)/207.345) * 1560; //my rm
			    				   dir = 0;

			    			   }
			    			   else if((strcmp(cmd, "HR") == 0) && turningAngle <= targetAngle+2){//indoor
			    				//   else if((strcmp(cmd, "HR") == 0) && turningAngle <= targetAngle+2){//outdoor
					    				   SetPinStop();
					    				   //SendFeedBack(1);
					    				   turning = 0;
					    				   osDelay(100);
					    				   htim1.Instance -> CCR4 = SERVO_STRAIGHT;
					    				   osDelay(100);
					    				   ResetValues();
		//			    				   targetCount = ((85-28)/207.345) * 1560; //my rm
					    				   dir = 0;

					    			   }
			    			   else if((strcmp(cmd, "LL") == 0) && turningAngle >= targetAngle-2){//indoor
			    			//else if((strcmp(cmd, "LL") == 0) && turningAngle >= targetAngle-2){//outdoor
			    					SetPinStop();
			    					//SendFeedBack(1);
			    					turning =0;
			    					ResetValues();
			    					osDelay(100);
				    				htim1.Instance -> CCR4 = SERVO_STRAIGHT;
				    				osDelay(100);

				    				dir = 0;
			    			   }
			    			   else if((strcmp(cmd, "HL") == 0) && turningAngle >= targetAngle +3){ //indoor
			    				   //else if((strcmp(cmd, "HL") == 0) && turningAngle >= targetAngle){ //outdoor
			    				   SetPinStop();
			    				   //SendFeedBack(1);
			    				   turning =0;
			    				   osDelay(100);
			    					htim1.Instance -> CCR4 = SERVO_STRAIGHT;
			    					osDelay(100);
			    					ResetValues();
			    					dir = 0;
			    			   }
			    			   else if((strcmp(cmd, "UL") == 0) && turningAngle >= targetAngle-3){//indoor
			    				 //  else if((strcmp(cmd, "UL") == 0) && turningAngle >= targetAngle-3){//outdoor
			    			  		SetPinStop();
			    			  		//SendFeedBack(1);
			    			  		turning =0;
			    			  		osDelay(100);
			    			  		htim1.Instance -> CCR4 = SERVO_STRAIGHT;
			    			  		osDelay(100);
			    			  		ResetValues();
			    			  		dir = 0;
			    			   }
			    			   else if((strcmp(cmd, "UR") == 0) && turningAngle <= targetAngle+3){
			    				   SetPinStop();
			    				   //SendFeedBack(1);
			    				   turning = 0;
			    				   osDelay(100);
			    				   htim1.Instance -> CCR4 = SERVO_STRAIGHT;
			    				   osDelay(100);
			    				   ResetValues();
			    				   dir = 0;

			    			   }
			    			   else if(strcmp(cmd,"CC")==0){
			    				   if(targetAngle>0){
			    					   if(turningAngle >= targetAngle-2){
			    						   SetPinStop();
			    						   turning = 0;
			    						   osDelay(100);
			    						   htim1.Instance -> CCR4 = SERVO_STRAIGHT;
			    						   osDelay(100);
			    						   ResetValues();
			    						   dir = 0;
			    					   }
			    				   }
			    				   else if(targetAngle<0){
			    					   if(turningAngle <= targetAngle+2){
					    				   SetPinStop();
					    				   turning = 0;
					    				   osDelay(100);
					    				   htim1.Instance -> CCR4 = SERVO_STRAIGHT;
					    				   osDelay(100);
					    				   ResetValues();
		//			    				   targetCount = ((85-28)/207.345) * 1560; //my rm
					    				   dir = 0;
			    				   }

			    			   }


			    }

	    }
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, pwmValB);
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmValA);
		osDelay(1);
	}
  /* USER CODE END DCMotor_task */
}

/* USER CODE BEGIN Header_RefreshOLED_task */
/**
* @brief Function implementing the RefreshOLED thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RefreshOLED_task */
void RefreshOLED_task(void *argument)
{
  /* USER CODE BEGIN RefreshOLED_task */
  /* Infinite loop */
  for(;;)
  {
	OLED_ShowString(0,0,oledRow0);
	OLED_ShowString(0,10,oledRow1);
	OLED_ShowString(0,20,oledRow2);
	OLED_ShowString(0,30,oledRow3);
	OLED_ShowString(0,40,oledRow4);
	OLED_ShowString(0,50,oledRow5);

	  OLED_Refresh_Gram();
	  osDelay(100);

	  HAL_IWDG_Refresh(&hiwdg);
	  osDelay(1);
  }
  /* USER CODE END RefreshOLED_task */
}

/* USER CODE BEGIN Header_EncoderMotorA_task */
/**
* @brief Function implementing the EncoderMotorA thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_EncoderMotorA_task */
void EncoderMotorA_task(void *argument)
{
  /* USER CODE BEGIN EncoderMotorA_task */
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	int cnt1, cnt2;
	uint32_t tick;
	cnt1 = __HAL_TIM_GET_COUNTER(&htim2);
	tick = HAL_GetTick();
  /* Infinite loop */
  for(;;)
  {
	  // Get the difference between the cnt1 and cnt2 channels every 1000 ticks
	  if(HAL_GetTick() - tick > 10L)
	  {
		  cnt2 = __HAL_TIM_GET_COUNTER(&htim2);
		  if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2))
		  {
			  if(cnt2 < cnt1)
				  diffA = cnt1 - cnt2;
			  else
				  diffA = (65535 - cnt2) + cnt1;
		  }
		  else
		  {
			  if(cnt2 > cnt1)
				  diffA = cnt2 - cnt1;
			  else
				  diffA = (65535 - cnt1) + cnt2;
		  }
		  // Ensure diffA is reasonable and filter out incorrect large jumps

		  if(diffA > 10000)  // Example threshold (adjust based on your system)
			  diffA = 0;  // Discard large invalid jumps
		             // Reset the counter if overflow is suspected
		  if(diffA == 65535)
			  diffA = 0;
		  //OLED_ShowString(0,0,diffA);
		  cnt1 = __HAL_TIM_GET_COUNTER(&htim2);
		  if((strcmp(cmd,"GF")==0)){
			  double distanceTraveled = CalculateDist(diffA);
			  totalDist += distanceTraveled; //reset before every "GO" command
			  totalDistA += diffA;
		  }
		  tick = HAL_GetTick();
	  }
	  if(turning == 0 && (dir == 1 || dir == -1 || dir==2 || dir==3))
			pwmValA = CalcPIDA_Dist();
  }
  /* USER CODE END EncoderMotorA_task */
}

/* USER CODE BEGIN Header_EncoderMotorB_task */
/**
* @brief Function implementing the EncoderMotorB thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_EncoderMotorB_task */
void EncoderMotorB_task(void *argument)
{
  /* USER CODE BEGIN EncoderMotorB_task */
	HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
	int cnt1, cnt2;
	uint32_t tick;
	cnt1 = __HAL_TIM_GET_COUNTER(&htim5);
	tick = HAL_GetTick();
  /* Infinite loop */
  for(;;)
  {
	  if(HAL_GetTick() - tick > 1000L)
	  {
		  cnt2 = __HAL_TIM_GET_COUNTER(&htim5);
		  if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim5))
		  {
			  if(cnt2 < cnt1)
				  diffB = cnt1 - cnt2;
			  else
				  diffB = (65535 - cnt2) + cnt1;
		  }
		  else
		  {
			  if(cnt2 > cnt1)
				  diffB = cnt2 - cnt1;
			  else
				  diffB = (65535 - cnt1) + cnt2;
		  }
		  if(diffB == 65535)
			  diffB = 0;
		  cnt1 = __HAL_TIM_GET_COUNTER(&htim5);
		  tick = HAL_GetTick();
	  }
	  if(turning == 0 && (dir == 1 || dir == -1 || dir==2 || dir==3))
			pwmValB = CalcPIDB_Dist();
  }
  /* USER CODE END EncoderMotorB_task */
}

/* USER CODE BEGIN Header_UART_Command_RX_task */
/**
* @brief Function implementing the UART_Command_RX thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UART_Command_RX_task */
void UART_Command_RX_task(void *argument)
{
  /* USER CODE BEGIN UART_Command_RX_task */
  /* Infinite loop */
  for(;;)
  {
	  /* Do a switch case here to inteprete the different commands
	   * Idea is to associate different alphabets to different sets of instructions
	   * i.e: 'a' = go straight
	   * 	  'b' = reverse,
	   * 	  'c' = turn fully left
	   * 	  etc...
	   * 	  split the operational range of the servo motor into equal intervals
	   * 	  and associate it (and othecr actions) with a particular alphabet.*/

	  if(received == 1)
	  {

		  received = 0;

		  	  if(strcmp(cmd, "FW") == 0){ // move Forward
		  		  OLED_ShowString(0,0, "FW TOH");
		  		  htim1.Instance -> CCR4 = SERVO_STRAIGHT;
				  osDelay(100);
				  turning = 0;
				  dir = 1;
				  //SendFeedBack(1);
		  	  }
		  	  else if(strcmp(cmd, "BW") == 0){ // move Backward
		  		  htim1.Instance -> CCR4 = SERVO_STRAIGHT;
				  OLED_ShowString(0,0, "bw BACK");
				  osDelay(100);
				  turning = 0;
				  dir = -1;
				  //SendFeedBack(1);
		  	}
//
		  	  //30x30
		  	  else if(strcmp(cmd, "FL") == 0){ // turn Forward Left
		  		  //TurnFullLeft();
		  		  htim1.Instance -> CCR4 = SERVO_LEFT_MAX+8;
		  		  osDelay(150);
		  		  turning = 1;
		  		  targetAngle=90;
//		  		  turning=1;
//		  		  targetAngle = 90;
		  		 // targetCount = ((65 - 28) / 207.345) * 1560;// my rm

		  		//targetCount = ((55 - 28) / 207.345) * 1560; //outdoor


		  		  dir = 1;
		  		osDelay(100);
		  			  	}

		  	  else if(strcmp(cmd, "FR") == 0){ // turn Forward Right
		  		  OLED_ShowString(0,0, "FR RIGHT");


		  		  //osDelay(100);
		  		  //targetAngle=-90;
		  		  turning = 0;

		  		 // targetCount = ((105- 28) / 207.345) * 1560; //indoor
		  		  targetCount = ((72-28) / 207.345) * 1560; //outdoor
		  		  //targetCount=0;
		  		  osDelay(100);
		  		 // dir = 1;
		  		  dir=3;
		  		  osDelay(100);
		  			  	}

		  	  else if(strcmp(cmd, "BL") == 0){// turn Backward Left
		  		  OLED_ShowString(0,0, "bL");
		  		//htim1.Instance -> CCR4 = SERVO_LEFT_MAX+5; //outdoor i think
		  		htim1.Instance -> CCR4 = SERVO_LEFT_MAX;

		  		 osDelay(150);

		  	//	TurnFullLeft();
//		  		 turning = 0;
//		  		 targetCount=0; //my rm
//		  		 osDelay(150);
//		  		 dir=3; // my rm
		  		 //////
		  		  //targetCount = ((62 - 28) / 207.345) * 1560;
		  		  //targetCount=0;
		  		  //osDelay(150);
		  		  turning =1;
		  		  targetAngle=-90;
		  		  dir=-1; // my rm
		  		  osDelay(100);

		  			  	}
		  	  else if(strcmp(cmd, "BR") == 0){// turn Backward Right

//		  	//				  TurnFullRight();
		  		htim1.Instance -> CCR4 = SERVO_RIGHT_MAX;
		  		osDelay(150);
//		  		  turning=1;
		  		  turning = 1;
		  		  targetAngle=90;
		  		  //osDelay(100);
		  		  dir=-1;
		  		  //targetCount = ((80 - 28) / 207.345) * 1560; //indoor
		  		//targetCount = ((80 - 28) / 207.345) * 1560; //outdoor
		  		  //targetCount=0;
		  		  osDelay(100);
		  		 // dir = 2;

	  					  }
		  	  ///START///
		  	  //image reg fail
		  	  else if(strcmp(cmd, "FA") == 0){

		  		  turning =0;
		  		  targetCount = ((100 - 28) / 207.345) * 1560;
		  		  osDelay(100);
		  		  dir = 3;
		  	  }

		  	  //task 2

		  	  else if(strcmp(cmd, "GO") ==0){
//		  		  htim1.Instance -> CCR4 = SERVO_STRAIGHT;
//				  osDelay(150);
//				  dir=4;
		  		PWM_MAX=4000;
		  		stop=0;
		  		Distance = 100;
		  		//totalDistA=0; //reset travel dist
		  		targetCount = ((2000 - 28) / 207.345) * 1560;
		  		htim1.Instance -> CCR4 = SERVO_STRAIGHT;
		  		osDelay(100);
		  		turning = 0;
		  		dir = 1;
		  	  }
		  	  else if(strcmp(cmd, "GF") ==0){
//		  		  htim1.Instance -> CCR4 = SERVO_STRAIGHT;
//				  osDelay(150);
//				  dir=4;
		  		//PWM_MAX=1500;
		  		  PWM_MAX=4000;
			  	Distance = 100;
		  		targetCount = ((2000 - 28) / 207.345) * 1560;
		  		htim1.Instance -> CCR4 = SERVO_STRAIGHT;
		  		osDelay(100);
		  		turning = 0;
		  		dir = 1;
		  	  }
		  	  else if(strcmp(cmd, "GG") ==0){
//		  		  htim1.Instance -> CCR4 = SERVO_STRAIGHT;
//				  osDelay(150);
//				  dir=4;
		  		PWM_MAX=4000;
		  		Distance =100;
		  		//totalDistA=0; //reset travel dist
		  		targetCount = ((2000 - 28) / 207.345) * 1560;
		  		htim1.Instance -> CCR4 = SERVO_STRAIGHT;
		  		osDelay(100);
		  		turning = 0;
		  		dir = 1;
		  	  }
		  	  //LEFT side
		  	  else if(strcmp(cmd, "HR")==0){ //45 right
		  		  targetAngle=-45;
		  		Distance =100;
		  		  //TurnFullRight();

		  		  turning=1;
		  		  osDelay(100);
		  		  dir=1;
		  	  }
		  	  else if(strcmp(cmd, "HL")==0){ //45 left
		  		  targetAngle=40;
		  		Distance =100;
		  		  //TurnFullLeft();
		  		  turning=1;
		  		  osDelay(100);
		  		  dir=1;
		  	  }
		  	  else if(strcmp(cmd, "RR")==0){ //90 right
		  		  //PWM_MAX=1500;
		  		  targetAngle=-90;
		  		  //TurnFullRight();
		  		Distance =100;
		  		  turning=1;
		  		  osDelay(100);

		  		  dir=1;

		  	  }
		  	 else if(strcmp(cmd, "LL")==0){ //90 left
		  		 	 //PWM_MAX=1500;
		  		 	 //TurnFullLeft();
		  		 	Distance =100;
		  		 	 targetAngle=90;
		  		 	 turning=1;
		  			 osDelay(100);
		  			 dir=1;
		  	 }
		  	 else if(strcmp(cmd, "UR")==0){ //90 left
		  		 	 //TurnFullLeft();
		  		 	 targetAngle=-180;
		  		 	 turning=1;
		  			 osDelay(100);
		  			 dir=1;
		  	 }
		  	 else if(strcmp(cmd, "UL")==0){ //90 left
		  		 	 //TurnFullLeft();
		  		 	 targetAngle=180;
		  		 	 turning=1;
		  			 osDelay(100);
		  			 dir=1;
		  	 }
		  	 else if(strcmp(cmd, "RW")==0){ //1st left
		  		 	 targetCount = ((270-(Distance*10)- 28) / 207.345) * 1560;
		  		 	 osDelay(50);
			  		htim1.Instance -> CCR4 = SERVO_STRAIGHT;

			  		osDelay(50);
			  		turning = 0;
			  		dir = -1;
		  	 }
		  	 else if(strcmp(cmd, "KM")==0){ //1st right
		  		 	 targetCount = ((200-(Distance*10)- 28) / 207.345) * 1560;
		  		 	 osDelay(50);
			  		htim1.Instance -> CCR4 = SERVO_STRAIGHT;

			  		osDelay(50);
			  		turning = 0;
			  		dir = -1;
		  	 }

		  	 else if(strcmp(cmd, "PW")==0){
		  		 	 targetCount = ((300-(Distance*10)- 28) / 207.345) * 1560;
		  		 	 osDelay(50);
			  		htim1.Instance -> CCR4 = SERVO_STRAIGHT;

			  		osDelay(50);
			  		turning = 0;
			  		dir = -1;
		  	 }
		  	 else if(strcmp(cmd,"CC")==0){
		  		 targetAngle= 180-GlobalAngle; //face the center
		  		 osDelay(100);
		  		 turning = 1;
		  		 dir = 1;
		  	 }
		  	 else if(strcmp(cmd, "EN")==0){
		  		htim1.Instance -> CCR4 = SERVO_STRAIGHT;
		  		PWM_MAX=6000;
		  		Distance =100;
		  		targetCount = ((totalDist- 28) / 207.345) * 1560;
		  		osDelay(100);
		  		turning = 0;
		  		dir = 1;
		  	 }


		  	  else{
				  //cmd = '--';
		  		  turning=-1;
				  osDelay(10);
		  	  }
		  }
  }
  /* USER CODE END UART_Command_RX_task */
}

/* USER CODE BEGIN Header_Gyro */
/**
* @brief Function implementing the Gyrohandle thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Gyro */
void Gyro(void *argument)
{
  /* USER CODE BEGIN Gyro */
	 uint8_t val[2] = {0,0};
	 uint32_t currTick;
	 int16_t offset = 0;//-2; //edit to tune
	 gyroInit();
	 prevTick = HAL_GetTick();
	   while (1)
	   {
		   currTick = HAL_GetTick();
		   if(currTick - prevTick >= 10L)
		   {
			   readByte(0x37, val);
			   angularSpeed = (val[0] << 8) | val[1];

			   if((angularSpeed >= -3 && angularSpeed <= 3) || (dir == 0 || dir == -2 ||dir==2||dir==3)){
				   turningAngle += 0;
				   GlobalAngle += (double)(angularSpeed + offset)*((currTick - prevTick)/16400.0);
			   }
			   else{
				   turningAngle += (double)(angularSpeed + offset)*((currTick - prevTick)/16400.0);
				   //tempAngle += (double)(angularSpeed + offset)*((currTick - prevTick)/16400.0);
				   GlobalAngle += (double)(angularSpeed + offset)*((currTick - prevTick)/16400.0);
			   }
			   if(turningAngle >= 720)
			       turningAngle = 0;
			   else if(turningAngle <= -720)
				   turningAngle = 0;
			   prevTick = HAL_GetTick();


		   }
	   }
  /* USER CODE END Gyro */
}

/* USER CODE BEGIN Header_IRsensor */
/**
* @brief Function implementing the IRsensortask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_IRsensor */
void IRsensor(void *argument)
{
  /* USER CODE BEGIN IRsensor */
//
////	char buffer[100];
//  /* Infinite loop */
  for(;;)
  {
//	  HAL_ADC_Start(&hadc2);
//	  HAL_ADC_Start(&hadc3);
//	  HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
//	  HAL_ADC_PollForConversion(&hadc3,HAL_MAX_DELAY);
//	  left_adc = HAL_ADC_GetValue(&hadc2);
//	  right_adc = HAL_ADC_GetValue(&hadc3);
//
//	  LPF_SUM_right = LPF_SUM_right+right_adc;
//	  LPF_SUM_left = LPF_SUM_left+left_adc;
//	  counter++;
//	  if(counter>=100)
//	  	right_sensor = LPF_SUM_right/counter;
//	  	left_sensor = LPF_SUM_left/counter;
//
//	  	right_sensor_int = (0.0000074673 *pow(right_sensor,2))-(0.042958* right_sensor)+70.9308;
//	  	left_sensor_int = (0.0000074673 *pow(left_sensor,2))-(0.042958* left_sensor)+70.9308;
//
//	  	LPF_SUM_right = 0;
//	  	LPF_SUM_left = 0;
//	  	counter = 0;
	  }
	  osDelay(1);

  /* USER CODE END IRsensor */
}

/* USER CODE BEGIN Header_Testing */
/**
* @brief Function implementing the TEST thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Testing */
void Testing(void *argument)
{
  /* USER CODE BEGIN Testing */
//  /* Infinite loop */
	int test =0;
//	RA5=1; // comment for A5
	  for(;;)
	  {
		//HCSR04_Read();
////
		if(test==0){
//			strcpy(cmd,"GO");
//			received =1;
//			//SendFeedBack(1);
//		  if(MoveCount<=1){
			htim1.Instance -> CCR4 = SERVO_STRAIGHT;
			osDelay(1000);
//						strcpy(cmd,"BW");
//						targetCount= ((1000 - 18) / 207.345) * 1560;
//						received=1;
//						osDelay(4000);
//			strcpy(cmd,"FL");
//			received=1;
//			osDelay(5000);
//			strcpy(cmd,"RW");
//			received=1;
//			osDelay(5000);

//			strcpy(cmd,"GF");
//			received=1;
//			osDelay(5000);
//			strcpy(cmd,"GG");
//			received=1;
//			osDelay(5000);
//			dir=1;
//			turning=2;
//			pwmValA=4000;
//			pwmValB=5000;

//						received =1;
//			strcpy(cmd,"GF");
////			targetCount= ((500 - 28) / 207.345) * 1560;
//			received=1;
//			osDelay(8000);
//			strcpy(cmd,"RW");
////			targetCount= ((500 - 28) / 207.345) * 1560;
//			received=1;
//			osDelay(4000);




//			1st///LEFT
//			  OLED_ShowString(0,30,"Turning");
//			strcpy(cmd,"FW");
//			targetCount= ((100 - 28) / 207.345) * 1560;
//			received=1;
//			osDelay(4000);
//						strcpy(cmd,"HL");
//						received=1;
//						osDelay(4000);
//						strcpy(cmd,"FW");
//						targetCount= ((100 - 28) / 207.345) * 1560;
//						received=1;
//						osDelay(4000);
////
//						strcpy(cmd,"RR");
//						received=1;
//						osDelay(4000);
//						strcpy(cmd,"FW");
//						targetCount= ((100 - 28) / 207.345) * 1560;
//						received=1;
//						osDelay(4000);
//
//						strcpy(cmd,"HL");
//						received=1;
//						osDelay(4000);


			//1st///////////// right
//			strcpy(cmd,"HR");
//			received=1;
//			osDelay(4000);
////			strcpy(cmd,"FW");
////			targetCount= ((130 - 28) / 207.345) * 1560;
////			received=1;
////			osDelay(4000);
//			strcpy(cmd,"LL");
//			received=1;
//			osDelay(4000);
////									strcpy(cmd,"FW");
////									targetCount= ((100 - 28) / 207.345) * 1560;
////									received=1;
////									osDelay(4000);
//			strcpy(cmd,"HR");
//			received=1;
//			osDelay(5000);

//						strcpy(cmd,"GO");
//			//			targetCount= ((500 - 28) / 207.345) * 1560;
//						received=1;
//						osDelay(8000);
//						strcpy(cmd,"RW");
//			//			targetCount= ((500 - 28) / 207.345) * 1560;
//						received=1;
//						osDelay(4000);

			//2nd obs right turn
//			strcpy(cmd,"RR");
//			received=1;
//			osDelay(4000);
//			strcpy(cmd,"BW");
//			targetCount= ((130 - 28) / 207.345) * 1560;
//			received=1;
//			osDelay(4000);
//			strcpy(cmd,"UL");
//			received=1;
//			osDelay(8000);
//			strcpy(cmd,"FW");
//			targetCount= ((300 - 28) / 207.345) * 1560;
//			received=1;
//			osDelay(4000);
//			strcpy(cmd,"LL");
//			received=1;
//			osDelay(4000);
//			strcpy(cmd,"EN");
//			received=1;
//			osDelay(15000);
//			strcpy(cmd,"LL");
//			received=1;
//			osDelay(4000);
//			strcpy(cmd,"BW");
//			targetCount= ((90 - 28) / 207.345) * 1560;
//			received=1;
//			osDelay(4000);
//			strcpy(cmd,"RR");
//			received=1;
//			osDelay(4000);
//			strcpy(cmd,"GG");
//			received=1;
//			osDelay(4000);
////////////////////////////////////////////////////////////


			//2nd obs left turn
//			strcpy(cmd,"LL");
//			received=1;
//			osDelay(4000);
//			strcpy(cmd,"UR");
//			received=1;
//			osDelay(8000);
//			strcpy(cmd,"FW");
//			targetCount= ((450 - 28) / 207.345) * 1560;
//			received=1;
//			osDelay(4000);
//			strcpy(cmd,"RR");
//			received=1;
//			osDelay(4000);
//						strcpy(cmd,"EN");
//						received=1;
//						osDelay(15000);
//						strcpy(cmd,"RR");
//						received=1;
//						osDelay(4000);
//						strcpy(cmd,"LL");
//						received=1;
//						osDelay(4000);
//						strcpy(cmd,"GG");
//						received=1;
//						osDelay(4000);


			//MoveCount=-1;
//		  }



////////////////////


//
//
//			strcpy(cmd,"BL");
//		//	targetCount = ((800 - 28) / 207.345) * 1560;
//			received=1;
//////			osDelay(11000);
//			strcpy(cmd,"FW");
//			targetCount = ((800 - 28) / 207.345) * 1560;
//			received=1;
//			osDelay(10000);
//			strcpy(cmd,"BW");
//			targetCount = ((800 - 28) / 207.345) * 1560;
//			received=1;
////			osDelay(10000);
//						strcpy(cmd,"FR");
//						received=1;
//						osDelay(7000);
//						strcpy(cmd,"BR");
//						received=1;
//						osDelay(7000);
//						strcpy(cmd,"FL");
//						received=1;
//						osDelay(7000);
//						strcpy(cmd,"BL");
//						received=1;
//						osDelay(7000);


//
////
			test=1;
//	//		osDelay(2000);
		}
    osDelay(10);
  }
  /* USER CODE END Testing */
}

/* USER CODE BEGIN Header_task2 */
/**
* @brief Function implementing the Task2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_task2 */
void task2(void *argument)
{
  /* USER CODE BEGIN task2 */

	  TickType_t lastWakeTime = xTaskGetTickCount();  // Get current tick count

	  const int checkInterval = pdMS_TO_TICKS(50);  // 50 ms interval for reading
  /* Infinite loop */
  for(;;)
  {
//	  //HCSR04_Read();
	  if(stop){
		  stop=0;
		  dir=0;
		  osDelay(10);

	  }
	  	if(strcmp(cmd,"GO")==0 || strcmp(cmd,"GG")==0 || strcmp(cmd,"GF")==0 ){ //29
	  		HCSR04_Read();
	  		if(strcmp(cmd,"GO")==0 && Distance<=35){
	  			PWM_MAX = 1000;
	  			if(strcmp(cmd,"GO")==0 && Distance<=20)
	  		  					{
	  				  						targetCount=0;
	  				  						SetPinStop();
	  				  						ResetValues();
	  				  						Distance=100;
	  				  						PWM_MAX = 4000;
	  				  						osDelay(20);
	  				  						//SendFeedBack(1);
	  				  						turning =-1;
	  				  						dir=0;

	  				  						//OLED_ShowString(0,50,"END OF T2");

	  				  						strcpy(cmd,"-");
	  				  						osDelay(100);
	  				  					}
	  			}
	  		else if(strcmp(cmd,"GF")==0 && Distance<=35){
	  			PWM_MAX = 1000;
	  			if(strcmp(cmd,"GF")==0 && Distance<=20)
	  		  					{
						targetCount=0;
	  						SetPinStop();
	  						ResetValues();
	  						Distance=100;
	  						PWM_MAX = 4000;
	  						osDelay(20);
	  						//SendFeedBack(1);
	  						turning =-1;
	  						dir=0;

	  						//OLED_ShowString(0,50,"END OF T2");

	  						strcpy(cmd,"-");
	  						osDelay(100);
	  				  					}
	  					}
	  		else if(strcmp(cmd,"GG")==0 && Distance<=45){
	  			PWM_MAX = 1000;
	  			if(strcmp(cmd,"GG")==0 && Distance<=30)
	  					{
	  						targetCount=0;
	  						SetPinStop();
	  						osDelay(20);
	  						//SendFeedBack(1);
	  						turning =-1;
	  						//dir=0;
	  						//OLED_ShowString(0,50,"END OF T2");
	  						ResetValues();
	  						strcpy(cmd,"-");
	  						osDelay(100);
	  					}
	  		}
	  		else{
	  			PWM_MAX=4000;
	  		}
	  		osDelay(80);
	    }
	  	//osDelay(80);

//	  	//osDelayUntil(5);
	    }
	    /* USER CODE END task2 */
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
  while (1){
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
