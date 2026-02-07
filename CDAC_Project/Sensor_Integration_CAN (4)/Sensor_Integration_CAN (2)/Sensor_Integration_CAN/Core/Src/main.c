/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : CAN2 Automotive Dashboard Transmitter with FreeRTOS
  * @attention      : ACS712 Current Calculation Fixed
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>
#include <math.h> // Added for fabsf if needed

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
// #include "SEGGER_SYSVIEW.h" // Uncomment if using Segger
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* CAN Message Queue Item Structure */
typedef struct {
    uint32_t msg_id;
    uint8_t data[8];
    uint8_t dlc;
} CAN_QueueMessage_t;

/* Vehicle Data Structure (Protected by Mutex) */
typedef struct {
    uint16_t rpm;
    uint16_t vehicle_speed;
    float battery_voltage;
    uint8_t battery_soc;
    int16_t battery_current;
    float battery_temperature;
    uint8_t door_status;
    uint8_t seatbelt_status;
} VehicleData_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define STACK_SIZE 256
#define CAN_QUEUE_SIZE 20

/* LED Pin Definitions - Adjust for your board */
#define LED_GREEN_PIN   GPIO_PIN_12
#define LED_ORANGE_PIN  GPIO_PIN_13
#define LED_RED_PIN     GPIO_PIN_14
#define LED_BLUE_PIN    GPIO_PIN_15
#define LED_GPIO_PORT   GPIOD

/* --- SENSOR CONFIGURATION --- */
#define VOLTAGE_DIVIDER_RATIO   5.0f    // (30k/7.5k)

/* ACS712 Configuration */
#define ACS712_SENSITIVITY      0.066f  // 30A Model (66mV/A)
// Standard ACS712 Zero Point is 2.5V (5V / 2).
// IMPORTANT: If your Multimeter reads 2.48V at PA1 when 0A, change this to 2.48f.
#define ACS712_ZERO_POINT       2.50f

#define ADC_BUF_LEN 4
#define WHEEL_MARKS             1

/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
CAN_HandleTypeDef hcan2;
TIM_HandleTypeDef htim1; // Motor PWM
TIM_HandleTypeDef htim3; // Speed Sensor

/* Speed measurement variables */
volatile uint32_t speed_pulse_count = 0;
volatile uint32_t last_capture_value = 0;
volatile uint32_t motor_rpm_calculated = 0;

/* Smoothed RPM variable */
volatile float motor_rpm_smoothed = 0.0f;
#define RPM_ALPHA 0.2f

/* Smoothed Voltage variable */
volatile float battery_voltage_smoothed = 0.0f;
#define VOLT_ALPHA 0.1f

/* Motor control */
volatile uint16_t motor_pwm_duty = 0;

/* ADC Buffer: [0]=Voltage, [1]=Current, [2]=Temp, [3]=Potentiometer */
uint32_t adc_buffer[ADC_BUF_LEN];

/* Task Handles */
TaskHandle_t SensorReadTaskHandle = NULL;
TaskHandle_t CANTransmitTaskHandle = NULL;
TaskHandle_t MonitorTaskHandle = NULL;

/* Queue & Mutex Handles */
QueueHandle_t CANMessageQueue = NULL;
SemaphoreHandle_t VehicleDataMutex = NULL;

/* Shared Vehicle Data */
VehicleData_t vehicle_data = {0};

/* Counters */
volatile uint32_t can_tx_errors = 0;
volatile uint32_t can_tx_count = 0;
volatile uint32_t sensor_read_count = 0;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);

/* Task Functions */
void Task_SensorRead(void *argument);
void Task_CANTransmit(void *argument);
void Task_Monitor(void *argument);

/* Sensor Helper Functions */
uint16_t Read_MotorRPM(void);
uint16_t Read_VehicleSpeed(void);
void Read_BatteryStatus(float *voltage, uint8_t *soc, int16_t *current, float *temperature);
uint8_t Read_DoorStatus(void);
uint8_t Read_SeatbeltStatus(void);
void Update_MotorSpeed_FromPot(void);

/* Message Helper Functions */
void Queue_CANMessage(uint32_t msg_id, uint8_t *data, uint8_t dlc);
void PrepareAndQueue_MotorSpeed(uint16_t rpm);
void PrepareAndQueue_VehicleSpeed(uint16_t speed);
void PrepareAndQueue_BatteryStatus(float voltage, uint8_t soc, int16_t current, float temperature);
void PrepareAndQueue_DoorStatus(uint8_t doors, uint8_t belts);

/* LED Helpers */
void LED_Green_Toggle(void)  { HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_GREEN_PIN); }
void LED_Orange_Toggle(void) { HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_ORANGE_PIN); }
void LED_Red_Toggle(void)    { HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_RED_PIN); }
void LED_Blue_Toggle(void)   { HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_BLUE_PIN); }
void LED_Red_On(void)        { HAL_GPIO_WritePin(LED_GPIO_PORT, LED_RED_PIN, GPIO_PIN_SET); }
void LED_Red_Off(void)       { HAL_GPIO_WritePin(LED_GPIO_PORT, LED_RED_PIN, GPIO_PIN_RESET); }

/* Main ----------------------------------------------------------------------*/
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN2_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();

  /* Start PWM for motor control */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  /* Start ADC DMA */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_BUF_LEN);

  /* Create Mutex */
  VehicleDataMutex = xSemaphoreCreateMutex();

  /* Create Queue */
  CANMessageQueue = xQueueCreate(CAN_QUEUE_SIZE, sizeof(CAN_QueueMessage_t));

  /* Create Tasks */
  xTaskCreate(Task_SensorRead, "SensorRead", STACK_SIZE, NULL, tskIDLE_PRIORITY + 3, &SensorReadTaskHandle);
  xTaskCreate(Task_CANTransmit, "CANTransmit", STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, &CANTransmitTaskHandle);
  xTaskCreate(Task_Monitor, "Monitor", STACK_SIZE / 2, NULL, tskIDLE_PRIORITY + 1, &MonitorTaskHandle);

  /* Start Scheduler */
  vTaskStartScheduler();

  while (1) {}
}

/* ---------------------------------------------------------------------------
 * TASK IMPLEMENTATIONS
 * --------------------------------------------------------------------------- */

void Task_SensorRead(void *argument)
{
    while(1)
    {
        LED_Green_Toggle();

        uint16_t rpm = Read_MotorRPM();
        uint16_t speed = Read_VehicleSpeed();
        float voltage, temperature;
        uint8_t soc;
        int16_t current;

        Read_BatteryStatus(&voltage, &soc, &current, &temperature);

        uint8_t doors = Read_DoorStatus();
        uint8_t belts = Read_SeatbeltStatus();

        Update_MotorSpeed_FromPot();

        if(xSemaphoreTake(VehicleDataMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            vehicle_data.rpm = rpm;
            vehicle_data.vehicle_speed = speed;
            vehicle_data.battery_voltage = voltage;
            vehicle_data.battery_soc = soc;
            vehicle_data.battery_current = current;
            vehicle_data.battery_temperature = temperature;
            vehicle_data.door_status = doors;
            vehicle_data.seatbelt_status = belts;
            xSemaphoreGive(VehicleDataMutex);
            sensor_read_count++;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void Task_CANTransmit(void *argument)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    uint8_t cycle_count = 0;

    while(1)
    {
        LED_Orange_Toggle();

        if(xSemaphoreTake(VehicleDataMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            PrepareAndQueue_MotorSpeed(vehicle_data.rpm);
            PrepareAndQueue_VehicleSpeed(vehicle_data.vehicle_speed);
            PrepareAndQueue_BatteryStatus(vehicle_data.battery_voltage,
                                         vehicle_data.battery_soc,
                                         vehicle_data.battery_current,
                                         vehicle_data.battery_temperature);

            if(cycle_count % 2 == 0) {
                PrepareAndQueue_DoorStatus(vehicle_data.door_status, vehicle_data.seatbelt_status);
            }
            cycle_count++;
            xSemaphoreGive(VehicleDataMutex);
        }

        CAN_QueueMessage_t msg;
        while(xQueueReceive(CANMessageQueue, &msg, 0) == pdTRUE)
        {
            TxHeader.StdId = msg.msg_id;
            TxHeader.ExtId = 0x00;
            TxHeader.RTR = CAN_RTR_DATA;
            TxHeader.IDE = CAN_ID_STD;
            TxHeader.DLC = msg.dlc;
            TxHeader.TransmitGlobalTime = DISABLE;

            uint32_t timeout = HAL_GetTick() + 10;
            while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0) {
                if(HAL_GetTick() > timeout) {
                    can_tx_errors++;
                    LED_Red_On();
                    break;
                }
                taskYIELD();
            }

            if(HAL_CAN_AddTxMessage(&hcan2, &TxHeader, msg.data, &TxMailbox) == HAL_OK) {
                can_tx_count++;
                LED_Red_Off();
            } else {
                can_tx_errors++;
                LED_Red_On();
            }
            vTaskDelay(pdMS_TO_TICKS(5));
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void Task_Monitor(void *argument)
{
    while(1)
    {
        LED_Blue_Toggle();
        uint32_t error_code = HAL_CAN_GetError(&hcan2);

        if(error_code != HAL_CAN_ERROR_NONE) {
            LED_Red_On();
            if(error_code & HAL_CAN_ERROR_BOF) HAL_CAN_ResetError(&hcan2);
        } else {
            LED_Red_Off();
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* ---------------------------------------------------------------------------
 * SENSOR HELPER FUNCTIONS
 * --------------------------------------------------------------------------- */

void Read_BatteryStatus(float *voltage, uint8_t *soc, int16_t *current, float *temperature)
{
    // --- 1. BATTERY VOLTAGE (PA0) ---
    float v_pin = (adc_buffer[0] * 3.3f) / 4095.0f;
    *voltage = v_pin * VOLTAGE_DIVIDER_RATIO;

    // --- 2. BATTERY CURRENT (PA1 - ACS712) ---
    // [FIX APPLIED HERE]
    float c_pin = (adc_buffer[1] * 3.3f) / 4095.0f;
    
    // Calculate raw current (Direct connection assumes no voltage divider on ACS712 output)
    float amps = (c_pin - ACS712_ZERO_POINT) / ACS712_SENSITIVITY;

    // Deadband for noise rejection (+/- 0.4A)
    if(amps > -0.4f && amps < 0.4f) amps = 0.0f;

    *current = (int16_t)(amps * 10.0f);

    // --- 3. BATTERY TEMPERATURE (PA7 - Inverted NTC) ---
    float raw_val = (adc_buffer[2] * 330.0f) / 4095.0f; 
    *temperature = 25.0f + (49.0f - raw_val) * 2.0f;
    if (*temperature < 0.0f) *temperature = 0.0f;
    if (*temperature > 120.0f) *temperature = 120.0f;

    // --- 4. STATE OF CHARGE (SOC) ---
    battery_voltage_smoothed = (VOLT_ALPHA * (*voltage)) + ((1.0f - VOLT_ALPHA) * battery_voltage_smoothed);
    
    if (battery_voltage_smoothed >= 9.0f) *soc = 100;
    else if (battery_voltage_smoothed <= 6.0f) *soc = 0;
    else *soc = (uint8_t)((battery_voltage_smoothed - 6.0f) / 3.0f * 100.0f);
}

uint16_t Read_MotorRPM(void) {
    static uint32_t last_update = 0;
    if (HAL_GetTick() - last_update > 4000) {
        static uint32_t last_pulse_count = 0;
        if (speed_pulse_count == last_pulse_count) motor_rpm_calculated = 0;
        last_pulse_count = speed_pulse_count;
        last_update = HAL_GetTick();
    }
    return (uint16_t)motor_rpm_calculated;
}

uint16_t Read_VehicleSpeed(void) {
    return (uint16_t)(motor_rpm_calculated / 60.0f);
}

uint8_t Read_DoorStatus(void) {
    return (uint8_t)HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1);
}

uint8_t Read_SeatbeltStatus(void) {
    return (uint8_t)HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5);
}

void Update_MotorSpeed_FromPot(void) {
    uint32_t pot_value = adc_buffer[3];
    motor_pwm_duty = (pot_value * 1000) / 4095;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, motor_pwm_duty);
}

/* ---------------------------------------------------------------------------
 * CAN QUEUE HELPERS
 * --------------------------------------------------------------------------- */

void Queue_CANMessage(uint32_t msg_id, uint8_t *data, uint8_t dlc) {
    CAN_QueueMessage_t msg;
    msg.msg_id = msg_id;
    msg.dlc = dlc;
    memcpy(msg.data, data, dlc);
    xQueueSend(CANMessageQueue, &msg, 0);
}

void PrepareAndQueue_MotorSpeed(uint16_t rpm) {
    uint8_t data[2] = {(rpm >> 8) & 0xFF, rpm & 0xFF};
    Queue_CANMessage(0x100, data, 2);
}

void PrepareAndQueue_VehicleSpeed(uint16_t speed) {
    uint8_t data[2] = {(speed >> 8) & 0xFF, speed & 0xFF};
    Queue_CANMessage(0x105, data, 2);
}

void PrepareAndQueue_BatteryStatus(float voltage, uint8_t soc, int16_t current, float temperature) {
    uint8_t data[6];
    uint16_t voltage_mv = (uint16_t)(voltage * 100);
    data[0] = (voltage_mv >> 8) & 0xFF;
    data[1] = voltage_mv & 0xFF;
    data[2] = soc;
    data[3] = (current >> 8) & 0xFF;
    data[4] = current & 0xFF;
    data[5] = (uint8_t)temperature;
    Queue_CANMessage(0x101, data, 6);
}

void PrepareAndQueue_DoorStatus(uint8_t doors, uint8_t belts) {
    uint8_t data[2] = {doors, belts};
    Queue_CANMessage(0x102, data, 2);
}

/* ---------------------------------------------------------------------------
 * HARDWARE INIT
 * --------------------------------------------------------------------------- */

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}

static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  HAL_ADC_Init(&hadc1);

  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 3;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 4;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}

static void MX_CAN2_Init(void)
{
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 6;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = ENABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = ENABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  HAL_CAN_Init(&hcan2);

  CAN_FilterTypeDef sFilterConfig;
  sFilterConfig.FilterBank = 14;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;
  HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig);
  HAL_CAN_Start(&hcan2);
}

static void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 83;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  HAL_TIM_Base_Init(&htim1);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);
}

static void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 8399;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim3);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);
  HAL_TIM_IC_Init(&htim3);

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 8;
  HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1);

  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
}

static void MX_DMA_Init(void)
{
  __HAL_RCC_DMA2_CLK_ENABLE();
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /* Door & Seatbelt Inputs */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* LED Outputs */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
        uint32_t current_capture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        uint32_t diff = (uint16_t)(current_capture - last_capture_value);
        last_capture_value = current_capture;

        if (diff > 50) {
            float frequency_hz = 10000.0f / diff;
            uint32_t instant_rpm = (uint32_t)((frequency_hz * 60.0f) / WHEEL_MARKS);
            
            // EMA Smoothing
            motor_rpm_smoothed = (RPM_ALPHA * instant_rpm) + ((1.0f - RPM_ALPHA) * motor_rpm_smoothed);
            motor_rpm_calculated = (uint32_t)motor_rpm_smoothed;
        }
        speed_pulse_count++;
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2) HAL_IncTick();
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}
