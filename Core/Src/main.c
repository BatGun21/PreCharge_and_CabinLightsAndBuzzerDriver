/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <stdio.h>
#include <stdlib.h>
#include "stm32f0xx.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define R_PRECHARGE 250 // in ohms
#define C_PRECHARGE 1000e-3 // in farads
#define RC_TIME_CONSTANT (R_PRECHARGE * C_PRECHARGE)// in ms
#define DEBOUNCE_DELAY 200 // in ms
#define MAX_SUPPLY_VOLTAGE 61 // in Volts
#define MIN_SUPPLY_VOLTAGE 30 // in Volts
#define MAX_SENSE_VOLTAGE 61 // in Volts
#define MIN_SENSE_VOLTAGE 30 // in Volts
#define Number_of_Samples 50
#define Variance 2 // in Volts
#define Clock_Frequency 8000 //KHz
#define ADC_CHANNEL 1
#define GPIO_PORT_PCHG GPIOC
#define GPIO_PORT_ADC GPIOB
#define GPIO_PIN_PCHG_RELAY GPIO_ODR_4
#define GPIO_PIN_CONTACTOR_RELAY GPIO_ODR_3

#define GPIO_PORT_LEDS GPIOC
#define GPIO_PORT_CABINLEDS GPIOB
#define GPIO_PORT_BUZZER GPIOB
#define GPIO_PORT_SWITCH GPIOA
#define GPIO_PORT_DOORSWITCHES GPIOA
#define GPIO_PORT_DICKEYSWITCH GPIOB
#define GPIO_PORT_MOTORDRIVE GPIOB
#define GPIO_PORT_CAB_ON_DOOR GPIOB
#define GPIO_RED_LED_PIN GPIO_PIN_6
#define GPIO_BLUE_LED_PIN GPIO_PIN_7
#define GPIO_ORANGE_LED_PIN GPIO_PIN_8
#define GPIO_GREEN_LED_PIN GPIO_PIN_9
#define GPIO_PIN_BUZZER_CTRL GPIO_PIN_2
#define GPIO_PIN_CAB_LIGHT_FRNT_CTRL GPIO_PIN_3
#define GPIO_PIN_CAB_LIGHT_REAR_CTRL GPIO_PIN_4
#define GPIO_PIN_CAB_LIGHT_DICKEY_CTRL GPIO_PIN_5
#define GPIO_PIN_CAB_DOOR_SW_DICKEY GPIO_PIN_6
#define GPIO_PIN_CAB_DOOR_SW_REAR_L GPIO_PIN_15
#define GPIO_PIN_CAB_DOOR_SW_REAR_R GPIO_PIN_8
#define GPIO_PIN_CAB_DOOR_SW_FRNT_L GPIO_PIN_5
#define GPIO_PIN_CAB_DOOR_SW_FRNT_R GPIO_PIN_4
#define GPIO_PIN_MOTORDRIVE_SIGNAL GPIO_PIN_7
#define GPIO_PIN_CAB_ON_DOOR_SIGNAL GPIO_PIN_8
#define KillSwitch_PIN GPIO_PIN_0
#define ON 1
#define OFF 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint16_t adc_data;
char errMsg[100];
char succMsg[100];
int switchon = 0;
char msg2 [50];
char msg [20];
int counter = 0;
int delayActiveFlag = 0;
int current = 0;
int V_threshold = 0;
float avg_v_supply = 0;
float avg_V_in = 0;
float V_supply_arr [Number_of_Samples] = {0.0};
float V_in_arr [Number_of_Samples] = {0.0};
char supplyError [50] = "Critical: Supply is out of Range ";
char noiseError [50] = "Noise error / Check Connection ";
char timeOut[50] = "7RC timeout has reached";
char killSwitch[50] =  "kill switch was pressed";
volatile uint32_t debounceTimer = 0;
int killSwitchFlagRE = 0;
int relaystate = 0;
struct Wait {
	int currentTime;
	int delayTime ;
	int activeFlag;
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void SysTick_Init(uint32_t ticks);
uint16_t adc_rx(void);
void adc_init(void);
void SysTick_Handler(void);
uint16_t read_adc(uint8_t channel);
void Precharge(void);
void ConfigureVoltageSourcePin();
float adcValtoVolts (uint16_t adcVal);
void LED_init(void);
void switchpressed(void);
void Voltage_Print(void);
void printTimestamp(void);
void DelayMSW(unsigned int time);
float Average(float array[], int size);
void sense_V_supply(void);
void sense_V_in(void);
float Avg_V_in (void);
float Avg_V_Supply (void);
void EXTI_Init(void);
void supplySenseLoop (void);
int time_expired (struct Wait *WaitTime);
uint16_t debounceSwitch(uint16_t pin, struct Wait *WaitTimeOneMilli, struct Wait *WaitTimeTwoMilli, struct Wait *WaitTimeThreeMilli);
void PreChargeRelayCTRL(int state);
void ContactorRelayCTRL(int state);
int VoltageInRange(float Vin);
int PreChargeRelayIsON(void);
void FrontLightRelayCTRL(int state);
void BackLightRelayCTRL(int state);
void DickeyLightRelayCTRL(int state);
void BuzzerCTRL(int state);
void ConfigureOutputPins(void);
void ConfigureInputPins(void);
void killSwitch_Handler(void);
int Check_Front_Door_Switches(void);
int Check_Rear_Door_Switches(void);
int Check_Dickey_Door_Switch(void);
int Check_Motor_Drive_Signal(void);
int Check_Cab_On_Door_Signal(void);
void BuzzerDriver(void);
void CabinLights_and_BuzzerDriver(void);
void SetWait (struct Wait *WaitTime);
void ResetWait (struct Wait *WaitTime);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

struct Wait Three_RC = {0,3*RC_TIME_CONSTANT,0};
struct Wait Seven_RC = {0,7*RC_TIME_CONSTANT,0};
struct Wait OneSec = {0, 1000, 0};
struct Wait TwoSec = {0, 2000, 0};

struct Wait killSWDebounceOneMilliSec = {0,1,0};
struct Wait killSWDebounceTwoMilliSec = {0,2,0};
struct Wait killSWDebounceThreeMilliSec = {0,3,0};

struct Wait FrontRightSWDebounceOneMilliSec = {0,1,0};
struct Wait FrontRightSWDebounceTwoMilliSec = {0,2,0};
struct Wait FrontRightSWDebounceThreeMilliSec = {0,3,0};

struct Wait FrontLeftSWDebounceOneMilliSec = {0,1,0};
struct Wait FrontLeftSWDebounceTwoMilliSec = {0,2,0};
struct Wait FrontLeftSWDebounceThreeMilliSec = {0,3,0};

struct Wait RearRightSWDebounceOneMilliSec = {0,1,0};
struct Wait RearRightSWDebounceTwoMilliSec = {0,2,0};
struct Wait RearRightSWDebounceThreeMilliSec = {0,3,0};

struct Wait RearLeftSWDebounceOneMilliSec = {0,1,0};
struct Wait RearLeftSWDebounceTwoMilliSec = {0,2,0};
struct Wait RearLeftSWDebounceThreeMilliSec = {0,3,0};

struct Wait DickeySWDebounceOneMilliSec = {0,1,0};
struct Wait DickeySWDebounceTwoMilliSec = {0,2,0};
struct Wait DickeySWDebounceThreeMilliSec = {0,3,0};

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
  SysTick_Init(Clock_Frequency);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  ConfigureOutputPins();
  ConfigureInputPins();
  ConfigureVoltageSourcePin();
  adc_init();
  LED_init();
  EXTI_Init();
  ContactorRelayCTRL(OFF);
  PreChargeRelayCTRL(OFF);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (killSwitchFlagRE){
		  killSwitch_Handler();
	  }else{
		  supplySenseLoop();
		  while(PreChargeRelayIsON()){
			  Precharge();
			  CabinLights_and_BuzzerDriver();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void killSwitch_Handler(void){
	  uint16_t pin = 0;
	  uint16_t killPin = GPIO_PORT_SWITCH->IDR & KillSwitch_PIN;
	  pin  = debounceSwitch(killPin, &killSWDebounceOneMilliSec, &killSWDebounceTwoMilliSec, &killSWDebounceThreeMilliSec);
	  if (pin!=0){
		  //Turn of all Relays
		  PreChargeRelayCTRL(OFF);
		  ContactorRelayCTRL(OFF);
		  FrontLightRelayCTRL(OFF);
		  BackLightRelayCTRL(OFF);
		  DickeyLightRelayCTRL(OFF);
		  BuzzerCTRL(OFF);
		  while(1){
			//Halt Operation
			  SetWait(&OneSec);
			  if (time_expired(&OneSec)){
				  GPIO_PORT_LEDS->ODR ^= GPIO_RED_LED_PIN; //Error Blink
				  ResetWait(&OneSec);
			  }
		  }
	  }
}

void CabinLights_and_BuzzerDriver(void){

	  if(Check_Motor_Drive_Signal()){
		  BuzzerDriver();
	  }

	  if(Check_Cab_On_Door_Signal()){

		  if (Check_Front_Door_Switches()){
			  FrontLightRelayCTRL(ON);
		  }else{
			  FrontLightRelayCTRL(OFF);
		  }

		  if (Check_Rear_Door_Switches()){
			  BackLightRelayCTRL(ON);
		  }else{
			  BackLightRelayCTRL(OFF);
		  }

		  if(Check_Dickey_Door_Switch()){
			  DickeyLightRelayCTRL(ON);
		  }else{
			  DickeyLightRelayCTRL(OFF);
		  }
	  }else{
		  FrontLightRelayCTRL(OFF);
		  BackLightRelayCTRL(OFF);
		  DickeyLightRelayCTRL(OFF);
	  }
}

void SetWait (struct Wait *WaitTime){
	  if (!(WaitTime->activeFlag)){
		  WaitTime->currentTime = counter;
		  WaitTime->activeFlag = 1;
	  }
}

void ResetWait (struct Wait *WaitTime1){
	WaitTime1->activeFlag = 0;
}

void EXTI0_1_IRQHandler(void) {
    if (EXTI->PR & EXTI_PR_PR0) { // Check if EXTI Line 0 triggered the interrupt
    	killSwitchFlagRE = 1;
    }
    EXTI->PR = EXTI_PR_PR0; // Clear the interrupt pending bit by writing '1' to it
}

uint16_t debounceSwitch(uint16_t pin, struct Wait *WaitTimeOneMilli, struct Wait *WaitTimeTwoMilli, struct Wait *WaitTimeThreeMilli){

	uint16_t currPin = 0;
	uint16_t temp = 0;
	temp = pin;

	SetWait(WaitTimeOneMilli);
	SetWait(WaitTimeTwoMilli);
	SetWait(WaitTimeThreeMilli);

	if(time_expired(WaitTimeOneMilli)){
		ResetWait(WaitTimeOneMilli);
		if (pin==temp){
			if (time_expired(WaitTimeTwoMilli)){
				ResetWait(WaitTimeTwoMilli);
				if (pin==temp){
					if (time_expired(WaitTimeThreeMilli)){
						ResetWait(WaitTimeThreeMilli);
						currPin = temp;
					}
				}
			}

		}else{
			currPin = pin;
		}
	}

	return currPin;
}

void EXTI_Init(void) {
	  // Enabling Clock for Port A
	  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	  // Enabling Input for PA0
	  GPIOA->MODER &= 0xfffffffc;
	  // Setting the speed of the pin PA0 to High Speed
	  GPIOA->OSPEEDR |= 0x00000003;
	  // Enabling Pull Down for PA0
	  GPIOA->PUPDR |= 0x00000002;

	  // Configure EXTI Line 0 for PA0 with a rising edge trigger
	  EXTI->IMR |= EXTI_IMR_MR0; // Enable interrupt on line 0
	  EXTI->RTSR |= EXTI_RTSR_TR0; // Trigger on rising edge

	  // Enable EXTI0_1_IRQn (EXTI Line 0 and 1) in the NVIC
	  NVIC_EnableIRQ(EXTI0_1_IRQn);
	  NVIC_SetPriority(EXTI0_1_IRQn, 0);
}

float Avg_and_remove_outliers_V_Supply (void){

	float V_supply = 0;
	float V_mean = 0;

	V_mean = Average(V_supply_arr, Number_of_Samples);

	for (int i = 0; i < Number_of_Samples; i++){
		if (!(abs(V_mean - V_supply_arr[i]) < Variance)){
			V_supply_arr[i] = 0.0;
		}
	}

	V_supply = Average(V_supply_arr, Number_of_Samples);
	return V_supply;
}

float Avg_and_remove_outliers_V_in (void){

	float V_in = 0;
	float V_mean = 0;

	V_mean = Average(V_in_arr, Number_of_Samples);

	for (int i = 0; i < Number_of_Samples; i++){
		if (!(abs(V_mean - V_in_arr[i]) < Variance)){
			V_in_arr[i] = 0.0;
		}
	}

	V_in = Average(V_in_arr, Number_of_Samples);
	return V_in;
}

void sense_V_supply(void){

	int i = 0;

	while (i<Number_of_Samples){

		uint16_t adcVal = read_adc(ADC_CHANNEL);
		float V_supply = adcValtoVolts(adcVal);

		if ((V_supply >= MIN_SUPPLY_VOLTAGE) && (V_supply <= MAX_SUPPLY_VOLTAGE)){
			V_supply_arr[i] = V_supply;
			i++;
		}else{
			SetWait(&OneSec);
			if (time_expired(&OneSec)){
				HAL_UART_Transmit(&huart2, (uint8_t*)supplyError, strlen(supplyError), 100); //supply out of range
				Voltage_Print();
				ResetWait(&OneSec);
			}
		}
	}
}

void sense_V_in(void){

	int i = 0;

	while (i<Number_of_Samples){

		uint16_t adcVal = read_adc(ADC_CHANNEL);
		float V_in = adcValtoVolts(adcVal);

		if (VoltageInRange(V_in)){
			V_in_arr[i] = V_in;

		}else{
			V_in_arr[i] = V_in;
			SetWait(&OneSec);
			if (time_expired(&OneSec)){
				HAL_UART_Transmit(&huart2, (uint8_t*)noiseError, strlen(noiseError), 100); //For Noise
				Voltage_Print();
				SetWait(&OneSec);			}
		}
		i++;

	}
}

float adcValtoVolts (uint16_t adcVal){
	float Vin = (adcVal/4096.0)*2.9;
	Vin = Vin*(48.0/2.70);
	Vin += Vin*(0.06); //Correction
	return Vin;
}

void LED_init(void){
	  // Enabling Clock for Port C
	  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

	  GPIOC->MODER |= GPIO_MODER_MODER6_0|GPIO_MODER_MODER7_0|GPIO_MODER_MODER8_0|GPIO_MODER_MODER9_0;
}

int time_expired (struct Wait *WaitTime){
	int timeExpiredFlag = 0;
	if (counter> (WaitTime->currentTime) + (WaitTime->delayTime)){
		timeExpiredFlag = 1;
	}else{
		timeExpiredFlag = 0;
	}
	return timeExpiredFlag;
}

void Precharge(void) {

	SetWait(&Three_RC);
	SetWait(&Seven_RC);

	if (time_expired(&Three_RC)){

		sense_V_in();
		avg_V_in = Avg_and_remove_outliers_V_in();

	    if (avg_V_in <= V_threshold) {

	    	ContactorRelayCTRL(OFF); // Turn off the contactor relay // This turn off is essential to turn of when the supply is cut off
	        GPIO_PORT_LEDS->ODR |= (GPIO_RED_LED_PIN);
	        GPIO_PORT_LEDS->ODR &= ~GPIO_BLUE_LED_PIN;


	        if (time_expired(&Seven_RC)){

	        	ResetWait(&Seven_RC);
	        	PreChargeRelayCTRL(OFF);
	        }

	    }else {

	        ContactorRelayCTRL(ON);
	        GPIO_PORT_LEDS->ODR |= GPIO_BLUE_LED_PIN;
	        GPIO_PORT_LEDS->ODR &= ~GPIO_RED_LED_PIN;

	    }
	    ResetWait(&Three_RC);
	}
}

void ConfigureVoltageSourcePin(void) {
    // Enable the GPIO port clock
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

    // Configure PC3 as general purpose output
    GPIO_PORT_PCHG->MODER |= GPIO_MODER_MODER3_0;

    // Configure PC3 as push-pull
    GPIO_PORT_PCHG->OTYPER &= ~(GPIO_OTYPER_OT_3);

    // Configure PC1 to high speed
    GPIO_PORT_PCHG->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR3;

    // Configure PC4 as general purpose output
    GPIO_PORT_PCHG->MODER |= (GPIO_MODER_MODER4_0);

    // Configure PC4 as push-pull
    GPIO_PORT_PCHG->OTYPER &= ~(GPIO_OTYPER_OT_4);
    // Using a Pull Up Resistor

    // Configure PC4 to high speed
    GPIO_PORT_PCHG->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR4;
}

void adc_init(void) {
    // Enable the ADC clock
    RCC->APB2ENR |= RCC_APB2ENR_ADCEN;

    // Enable the GPIOA clock (assuming you are using PA1 for ADC)
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    // Configure PA1 as analog input
    GPIOA->MODER |= GPIO_MODER_MODER1; // Analog mode

    // Configure ADC settings
    ADC1->CFGR1 &= ~ADC_CFGR1_RES; // Clear the RES bits for 12-bit resolution
    ADC1->CFGR1 &= ~ADC_CFGR1_ALIGN; // Data right-aligned
    ADC1->CFGR1 |= ADC_CFGR1_CONT; // Continuous conversion mode
    ADC1->CHSELR &= ~ADC_CHSELR_CHSEL1; // Clear the CHSEL1 bits
    ADC1->CHSELR |= ADC_CHSELR_CHSEL1; // Set the channel number in CHSEL1 bits (Channel 1 for PA1)

    // Enable the ADC
    ADC1->CR |= ADC_CR_ADEN;

    // Wait for ADC to be ready
    while (!(ADC1->ISR & ADC_ISR_ADRDY));

    // Start the conversion
    ADC1->CR |= ADC_CR_ADSTART;
}

uint16_t read_adc(uint8_t channel) {
    // Set the channel in the sequence register
    ADC1->CHSELR = (1 << channel);  // Assuming channel is less than 16

    // Start the conversion
    ADC1->CR |= ADC_CR_ADSTART;

    // Wait for the end of conversion
    while (!(ADC1->ISR & ADC_ISR_EOC));

    // Read the converted value
    uint16_t result = ADC1->DR;

    return result;
}

void SysTick_Init(uint32_t ticks){

	SysTick->CTRL = 0; // Disable SysTick

	SysTick->LOAD = ticks-1; // Set Reload Register

	// Setting Interrupt Priority to the highest
	NVIC_SetPriority(SysTick_IRQn, (1<<__NVIC_PRIO_BITS)-1);

	SysTick->VAL = 0; // Reset the SysTick counter value

	SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk; // Selecting internal clock source
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk; // Enabling SysTick exception Request when 0


	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; // Enable SysTick
}

void DelayMSW(unsigned int time){
	for(int i=0; i<=time; i++){
		while ((SysTick->CTRL & 0x00010000) == 0){
				//Wait for 1 millisec.
		}
	}
}

void SysTick_Handler(void) {

	if (counter == 0xffffffff) {
        counter = 0; // Reset the counter if the maximum value is reached
    } else {
        counter++; // Increment the counter
    }

    if (debounceTimer > 0) {
        debounceTimer--;
    }

}

void Voltage_Print(void){ // Debug
	  uint16_t adcVal = read_adc(ADC_CHANNEL);
	  float Vin = adcValtoVolts(adcVal);
	  sprintf(msg2, " Vol = %.3f V ", Vin);
	  printTimestamp();
	  HAL_UART_Transmit(&huart2, (uint8_t*)(msg2), strlen(msg2), 200);
}

void printTimestamp(void) { // Debug
	sprintf(msg, " Time = %d ms:", counter);
	HAL_UART_Transmit(&huart2, (uint8_t*)(msg), strlen(msg), 200);
}

float Average(float array[], int size) {
    float sum = 0.0;
    float count = 0;
    for (int i = 0; i < size; i++) {
        if (array[i] != 0) {
            sum += array[i];
            count++;
        }
    }
    // Avoid division by zero
    if (count == 0) {
        return 0.0;
    }
    return sum / count;
}

void supplySenseLoop (void){
	  do{
		  sense_V_supply();
		  avg_v_supply = Avg_and_remove_outliers_V_Supply();

		  V_threshold = 0.9 * avg_v_supply;
		  DelayMSW(50); // Wait for connection to stable

	  }while (!(VoltageInRange(avg_v_supply)));
	  PreChargeRelayCTRL(ON);
}

void PreChargeRelayCTRL(int state){
	if (state){
		GPIO_PORT_PCHG->ODR |= GPIO_PIN_PCHG_RELAY; // PreCharge Relay is ON
	}else{
		GPIO_PORT_PCHG->ODR &= ~(GPIO_PIN_PCHG_RELAY); // Turn off the precharge relay
	}
}

void ContactorRelayCTRL(int state){
	if (state){
		GPIO_PORT_PCHG->ODR |= GPIO_PIN_CONTACTOR_RELAY;  // Turn on the contactor relay
	}else{
		GPIO_PORT_PCHG->ODR &= ~(GPIO_PIN_CONTACTOR_RELAY); // Turn off the Contactor relay
	}
}

int PreChargeRelayIsON(void){
	int isItON = 0;
	if ((GPIO_PORT_PCHG->ODR & GPIO_PIN_PCHG_RELAY)== GPIO_PIN_PCHG_RELAY){
		isItON = 1;
	}else{
		isItON = 0;
	}
	return isItON;
}

int VoltageInRange(float Vin){
	int isIt = 0;
	if ((Vin >= MIN_SUPPLY_VOLTAGE) && (Vin <= MAX_SUPPLY_VOLTAGE)){
		isIt = 1;
	}else{
		isIt = 0;
	}
	return isIt;
}

void FrontLightRelayCTRL(int state){
	if (state){
		GPIO_PORT_CABINLEDS->ODR |= GPIO_PIN_CAB_LIGHT_FRNT_CTRL; // Front CabinLight is ON
	}else{
		GPIO_PORT_CABINLEDS->ODR &= ~(GPIO_PIN_CAB_LIGHT_FRNT_CTRL); // Front CabinLight is OFF
	}
}

void BackLightRelayCTRL(int state){
	if (state){
		GPIO_PORT_CABINLEDS->ODR |= GPIO_PIN_CAB_LIGHT_REAR_CTRL;  // Back CabinLight is ON
	}else{
		GPIO_PORT_CABINLEDS->ODR &= ~(GPIO_PIN_CAB_LIGHT_REAR_CTRL); // Back CabinLight is OFF
	}
}

void DickeyLightRelayCTRL(int state){
	if (state){
		GPIO_PORT_CABINLEDS->ODR |= GPIO_PIN_CAB_LIGHT_DICKEY_CTRL;  // Dickey CabinLight is ON
	}else{
		GPIO_PORT_CABINLEDS->ODR &= ~(GPIO_PIN_CAB_LIGHT_DICKEY_CTRL); // Dickey CabinLight is OFF
	}
}

void BuzzerCTRL(int state){
	if (state){
		GPIO_PORT_BUZZER->ODR |= GPIO_PIN_BUZZER_CTRL;  // Buzzer is ON
	}else{
		GPIO_PORT_BUZZER->ODR &= ~(GPIO_PIN_BUZZER_CTRL); // Buzzer is OFF
	}
}

void BuzzerDriver(void){
	if (Check_Front_Door_Switches()||Check_Rear_Door_Switches()||Check_Dickey_Door_Switch()){
		BuzzerCTRL(ON);
	}else{
		BuzzerCTRL(OFF);
	}
}

void ConfigureOutputPins(void){

	// Enable clock for GPIO Port B and C
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN;

    // Configure pins PB3, PB4, PB5 output
    GPIO_PORT_CABINLEDS->MODER |= GPIO_MODER_MODER3_0 | GPIO_MODER_MODER4_0 | GPIO_MODER_MODER5_0;

    // Configure pins PC3, PC4, PC15 as high-speed output
    GPIO_PORT_CABINLEDS->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR3 | GPIO_OSPEEDER_OSPEEDR4 | GPIO_OSPEEDER_OSPEEDR5;

    // Set pins PB3, PB4, PB5 as push-pull
    GPIO_PORT_CABINLEDS->OTYPER &= ~(GPIO_OTYPER_OT_3 | GPIO_OTYPER_OT_4 | GPIO_OTYPER_OT_5);

    // Configure pin PB2 output
    GPIO_PORT_BUZZER->MODER |= GPIO_MODER_MODER2_0;

    // Configure pin PB2 as high-speed output
    GPIO_PORT_BUZZER->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR2;

    // Set pin PB2 as push-pull
    GPIO_PORT_BUZZER->OTYPER &= ~GPIO_OTYPER_OT_2;
}

void ConfigureInputPins(void){
    // Enable clock for GPIO Port A and Port B
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;

    // Configure PA4, PA5, PA8, PA15 as digital input with internal pull-up for Front Right/Left and Back  Right/Left Switch
    GPIOA->MODER &= ~(GPIO_MODER_MODER4 | GPIO_MODER_MODER5 | GPIO_MODER_MODER8 | GPIO_MODER_MODER15); // Clear bits
    GPIOA->PUPDR |= GPIO_PUPDR_PUPDR4_0 | GPIO_PUPDR_PUPDR5_0 | GPIO_PUPDR_PUPDR8_0 | GPIO_PUPDR_PUPDR15_0; // Set pull-up

    // Configure PB6 and PB8 as digital input with internal pull-up for Dickey Switch and Cab_ON_Signal
    GPIOB->MODER &= ~(GPIO_MODER_MODER6 | GPIO_MODER_MODER8); // Clear bits
    GPIOB->PUPDR |= (GPIO_PUPDR_PUPDR6_0 | GPIO_PUPDR_PUPDR8_0); // Set pull-up

    // Configure PB7  as digital input with internal pull-up for Motor Drive Input
    GPIOB->MODER &= ~GPIO_MODER_MODER7; // Clear bits
    GPIOB->PUPDR |= GPIO_PUPDR_PUPDR7_0 ; // Set pull-up
}

int Check_Front_Door_Switches(void){
    int state = 0;
    int frontLeftSwitch = GPIO_PORT_DOORSWITCHES->IDR & GPIO_PIN_CAB_DOOR_SW_FRNT_L;
    int frontRightSwitch = GPIO_PORT_DOORSWITCHES->IDR & GPIO_PIN_CAB_DOOR_SW_FRNT_R;

    if (frontLeftSwitch == 0 || frontRightSwitch == 0) {
        int frontLeftDebounced = debounceSwitch(frontLeftSwitch, &FrontLeftSWDebounceOneMilliSec, &FrontLeftSWDebounceTwoMilliSec, &FrontLeftSWDebounceThreeMilliSec);
        int frontRightDebounced = debounceSwitch(frontRightSwitch, &FrontRightSWDebounceOneMilliSec, &FrontRightSWDebounceTwoMilliSec, &FrontLeftSWDebounceThreeMilliSec);

        if (frontLeftDebounced == 0 || frontRightDebounced == 0) {
            state = 1;
        }
    }

    return state;
}

int Check_Rear_Door_Switches(void){
    int state = 0;
    int rearLeftSwitch = GPIO_PORT_DOORSWITCHES->IDR & GPIO_PIN_CAB_DOOR_SW_REAR_L;
    int rearRightSwitch = GPIO_PORT_DOORSWITCHES->IDR & GPIO_PIN_CAB_DOOR_SW_REAR_R;

    if (rearLeftSwitch == 0 || rearRightSwitch == 0) {
        int rearLeftDebounced = debounceSwitch(rearLeftSwitch, &RearLeftSWDebounceOneMilliSec, &RearLeftSWDebounceTwoMilliSec, &RearLeftSWDebounceThreeMilliSec);
        int rearRightDebounced = debounceSwitch(rearRightSwitch, &RearRightSWDebounceOneMilliSec, &RearRightSWDebounceTwoMilliSec, &RearRightSWDebounceThreeMilliSec);

        if (rearLeftDebounced == 0 || rearRightDebounced == 0) {
            state = 1;
        }
    }
    return state;
}

int Check_Dickey_Door_Switch(void){
    int state = 0;
    int dickeySwitch = GPIO_PORT_DICKEYSWITCH->IDR & GPIO_PIN_CAB_DOOR_SW_DICKEY;

    if (dickeySwitch == 0) {
        int dickeyDebounced = debounceSwitch(dickeySwitch, &DickeySWDebounceOneMilliSec, &DickeySWDebounceTwoMilliSec, &DickeySWDebounceThreeMilliSec);
        if (dickeyDebounced == 0) {
            state = 1;
        }
    }
    return state;
}

int Check_Motor_Drive_Signal(void){
	if (((GPIO_PORT_MOTORDRIVE->IDR & GPIO_PIN_MOTORDRIVE_SIGNAL) != 0)){
		return 0;
	}else{
		return 1;
	}
}

int Check_Cab_On_Door_Signal(void){
	if (((GPIO_PORT_CAB_ON_DOOR->IDR & GPIO_PIN_CAB_ON_DOOR_SIGNAL) != 0)){
		return 0;
	}else{
		return 1;
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
