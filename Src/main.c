/*
* This file is part of the hoverboard-firmware-hack project.
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f1xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "config.h"
#include "util.h"
#include "BLDC_controller.h"
#include "rtwtypes.h"
#include "comms.h"

#if defined(DEBUG_I2C_LCD) || defined(SUPPORT_LCD)
#include "hd44780.h"
#endif

void SystemClock_Config(void);

//------------------------------------------------------------------------
// Global variables set externally
//------------------------------------------------------------------------
extern TIM_HandleTypeDef htim_left;
extern TIM_HandleTypeDef htim_right;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern volatile adc_buf_t adc_buffer;

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

// Matlab defines
extern P    rtP_Left;
extern P    rtP_Right;
extern ExtY rtY_Left;
extern ExtY rtY_Right;
extern ExtU rtU_Left;
extern ExtU rtU_Right;

extern uint8_t     inIdx;
extern uint8_t     inIdx_prev;
extern InputStruct input1[];
extern InputStruct input2[];

extern int16_t speedAvg;
extern int16_t speedAvgAbs;
extern volatile uint32_t timeoutCntGen;
extern volatile uint8_t  timeoutFlgGen;
extern uint8_t timeoutFlgADC;
extern uint8_t timeoutFlgSerial;

extern volatile int pwml;
extern volatile int pwmr;
extern uint8_t enable;
extern int16_t batVoltage;

//------------------------------------------------------------------------
// Global variables set here in main.c
//------------------------------------------------------------------------
uint8_t backwardDrive;
extern volatile uint32_t buzzerTimer;
volatile uint32_t main_loop_counter;
int16_t batVoltageCalib;
int16_t board_temp_deg_c;
int16_t left_dc_curr;
int16_t right_dc_curr;
int16_t dc_curr;
int16_t cmdL;
int16_t cmdR;

// ############################### KATAMARAN İÇİN EKLENEN DEĞİŞKENLER ###############################
volatile uint8_t operation_mode = MODE_MANUAL;
volatile uint32_t last_manual_activity = 0;
volatile uint8_t autopilot_requested = 0;
static int16_t prev_left_raw = 0, prev_right_raw = 0;

// ESP32 iletişim için
#define ESP32_BUFFER_SIZE 32
volatile uint8_t esp32_buffer[ESP32_BUFFER_SIZE];
volatile uint8_t esp32_index = 0;

// Katamaran özel ayarları
#define KATAMARAN_MAX_SPEED 800
#define KATAMARAN_FILTER 16384
#define KATAMARAN_RATE 150
// ########################### END OF KATAMARAN İÇİN EKLENEN DEĞİŞKENLER ############################

// Local variables
#if defined(FEEDBACK_SERIAL_USART2) || defined(FEEDBACK_SERIAL_USART3)
typedef struct{
  uint16_t  start;
  int16_t   cmd1;
  int16_t   cmd2;
  int16_t   speedR_meas;
  int16_t   speedL_meas;
  int16_t   batVoltage;
  int16_t   boardTemp;
  uint16_t  cmdLed;
  uint16_t  checksum;
} SerialFeedback;
static SerialFeedback Feedback;
#endif

static int16_t    speed;
static int16_t    steer;
static int16_t    steerRateFixdt;
static int16_t    speedRateFixdt;
static int32_t    steerFixdt;
static int32_t    speedFixdt;

static uint32_t    buzzerTimer_prev = 0;
static uint32_t    inactivity_timeout_counter;

// ############################### KATAMARAN FONKSİYONLARI ###############################
void apply_motor_limits(void) {
    input1[inIdx].cmd = CLAMP(input1[inIdx].cmd, -KATAMARAN_MAX_SPEED, KATAMARAN_MAX_SPEED);
    input2[inIdx].cmd = CLAMP(input2[inIdx].cmd, -KATAMARAN_MAX_SPEED, KATAMARAN_MAX_SPEED);
}

void check_battery_safety(void) {
    if (batVoltage < BAT_LVL1) {
        input1[inIdx].cmd = input1[inIdx].cmd / 2;
        input2[inIdx].cmd = input2[inIdx].cmd / 2;
    }
    
    if (batVoltage < BAT_DEAD) {
        enable = 0;
        operation_mode = MODE_MANUAL;
        autopilot_requested = 0;
    }
}

void paddle_wheel_compensation(void) {
    static int16_t prev_cmdL = 0, prev_cmdR = 0;
    const int16_t max_change = 50;
    
    if (abs(input1[inIdx].cmd - prev_cmdL) > max_change) {
        input1[inIdx].cmd = prev_cmdL + ((input1[inIdx].cmd > prev_cmdL) ? max_change : -max_change);
    }
    
    if (abs(input2[inIdx].cmd - prev_cmdR) > max_change) {
        input2[inIdx].cmd = prev_cmdR + ((input2[inIdx].cmd > prev_cmdR) ? max_change : -max_change);
    }
    
    prev_cmdL = input1[inIdx].cmd;
    prev_cmdR = input2[inIdx].cmd;
}

uint8_t calculate_battery_percentage(void) {
    float voltage_ratio = (batVoltageCalib / 100.0f - BATTERY_MIN_VOLTAGE) / 
                         (BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE);
    return (uint8_t)(voltage_ratio * 100);
}

#ifdef CONTROL_SERIAL_USART3
void process_esp32_commands(void) {
    if (esp32_index > 0) {
        esp32_buffer[esp32_index] = '\0';
        
        if (strncmp((char*)esp32_buffer, "MOTOR,", 6) == 0) {
            int left_cmd, right_cmd;
            if (sscanf((char*)esp32_buffer + 6, "%d,%d", &left_cmd, &right_cmd) == 2) {
                if (operation_mode == MODE_AUTOPILOT) {
                    input1[inIdx].cmd = left_cmd;
                    input2[inIdx].cmd = right_cmd;
                }
            }
        }
        else if (strncmp((char*)esp32_buffer, "MODE,", 5) == 0) {
            uint8_t mode;
            if (sscanf((char*)esp32_buffer + 5, "%d", &mode) == 1) {
                if (mode == MODE_AUTOPILOT) {
                    autopilot_requested = 1;
                } else if (mode == MODE_MANUAL) {
                    operation_mode = MODE_MANUAL;
                    autopilot_requested = 0;
                }
                last_manual_activity = HAL_GetTick();
            }
        }
        else if (strncmp((char*)esp32_buffer, "RESET_AUTO", 10) == 0) {
            autopilot_requested = 0;
            operation_mode = MODE_MANUAL;
        }
        
        esp32_index = 0;
    }
}

void send_katamaran_telemetry(void) {
    char telemetry_data[64];
    snprintf(telemetry_data, sizeof(telemetry_data), 
             "BAT,%d,%d,%d,%d\n",
             calculate_battery_percentage(),
             batVoltageCalib,
             input1[inIdx].cmd,
             input2[inIdx].cmd);
    
    HAL_UART_Transmit(&huart3, (uint8_t*)telemetry_data, strlen(telemetry_data), 100);
}
#endif

void USART3_IRQHandler(void) {
    if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_RXNE)) {
        uint8_t data = (uint8_t)(huart3.Instance->DR & 0xFF);
        
        if (data == '\n' || esp32_index >= ESP32_BUFFER_SIZE - 1) {
            process_esp32_commands();
        } else {
            esp32_buffer[esp32_index++] = data;
        }
    }
}
// ########################### END OF KATAMARAN FONKSİYONLARI ############################

int main(void) {
    HAL_Init();
    __HAL_RCC_AFIO_CLK_ENABLE();
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
    HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
    HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
    HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
    HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);
    HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
    HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

    SystemClock_Config();

    __HAL_RCC_DMA1_CLK_DISABLE();
    MX_GPIO_Init();
    MX_TIM_Init();
    MX_ADC1_Init();
    MX_ADC2_Init();
    BLDC_Init();

    HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, GPIO_PIN_SET);
    Input_Lim_Init();
    Input_Init();

    HAL_ADC_Start(&hadc1);
    HAL_ADC_Start(&hadc2);

    poweronMelody();
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
  
    int32_t board_temp_adcFixdt = adc_buffer.temp << 16;
    int16_t board_temp_adcFilt  = adc_buffer.temp;

    while(HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) { HAL_Delay(10); }

    while(1) {
        if (buzzerTimer - buzzerTimer_prev > 16*DELAY_IN_MAIN_LOOP) {

            // ############################### KATAMARAN KONTROL SİSTEMİ ###############################
            apply_motor_limits();
            check_battery_safety();
            paddle_wheel_compensation();

            // Mod değişim kontrolü
            if (abs(input1[inIdx].raw - prev_left_raw) > 10 || abs(input2[inIdx].raw - prev_right_raw) > 10) {
                operation_mode = MODE_MANUAL;
                last_manual_activity = HAL_GetTick();
                autopilot_requested = 0;
            }
            
            if (autopilot_requested && operation_mode != MODE_AUTOPILOT) {
                operation_mode = MODE_AUTOPILOT;
                last_manual_activity = HAL_GetTick();
            }

            prev_left_raw = input1[inIdx].raw;
            prev_right_raw = input2[inIdx].raw;
            // ########################### END OF KATAMARAN KONTROL SİSTEMİ ############################

            readCommand();
            calcAvgSpeed();

            #ifndef VARIANT_TRANSPOTTER
            if (enable == 0 && !rtY_Left.z_errCode && !rtY_Right.z_errCode && 
                ABS(input1[inIdx].cmd) < 50 && ABS(input2[inIdx].cmd) < 50){
                beepShort(6);
                beepShort(4); HAL_Delay(100);
                steerFixdt = speedFixdt = 0;
                enable = 1;
            }
            #endif

            rateLimiter16(input1[inIdx].cmd, KATAMARAN_RATE, &steerRateFixdt);
            rateLimiter16(input2[inIdx].cmd, KATAMARAN_RATE, &speedRateFixdt);
            filtLowPass32(steerRateFixdt >> 4, KATAMARAN_FILTER, &steerFixdt);
            filtLowPass32(speedRateFixdt >> 4, KATAMARAN_FILTER, &speedFixdt);
            steer = (int16_t)(steerFixdt >> 16);
            speed = (int16_t)(speedFixdt >> 16);

            mixerFcn(speed << 4, steer << 4, &cmdR, &cmdL);

            #ifdef INVERT_R_DIRECTION
                pwmr = cmdR;
            #else
                pwmr = -cmdR;
            #endif
            #ifdef INVERT_L_DIRECTION
                pwml = -cmdL;
            #else
                pwml = cmdL;
            #endif

            filtLowPass32(adc_buffer.temp, TEMP_FILT_COEF, &board_temp_adcFixdt);
            board_temp_adcFilt  = (int16_t)(board_temp_adcFixdt >> 16);
            board_temp_deg_c    = (TEMP_CAL_HIGH_DEG_C - TEMP_CAL_LOW_DEG_C) * (board_temp_adcFilt - TEMP_CAL_LOW_ADC) / (TEMP_CAL_HIGH_ADC - TEMP_CAL_LOW_ADC) + TEMP_CAL_LOW_DEG_C;

            batVoltageCalib = batVoltage * BAT_CALIB_REAL_VOLTAGE / BAT_CALIB_ADC;

            left_dc_curr  = -(rtU_Left.i_DCLink * 100) / A2BIT_CONV;
            right_dc_curr = -(rtU_Right.i_DCLink * 100) / A2BIT_CONV;
            dc_curr       = left_dc_curr + right_dc_curr;

            // ESP32 telemetri gönderimi
            #if defined(CONTROL_SERIAL_USART3)
            if (main_loop_counter % (TELEMETRY_INTERVAL_MS / DELAY_IN_MAIN_LOOP) == 0) {
                send_katamaran_telemetry();
            }
            #endif

            poweroffPressCheck();

            if (TEMP_POWEROFF_ENABLE && board_temp_deg_c >= TEMP_POWEROFF && speedAvgAbs < 20){
                poweroff();
            } else if (BAT_DEAD_ENABLE && batVoltage < BAT_DEAD && speedAvgAbs < 20){
                poweroff();
            }

            inactivity_timeout_counter++;
            if (abs(cmdL) > 50 || abs(cmdR) > 50) {
                inactivity_timeout_counter = 0;
            }

            if (inactivity_timeout_counter > (INACTIVITY_TIMEOUT * 60 * 1000) / (DELAY_IN_MAIN_LOOP + 1)) {
                poweroff();
            }

            inIdx_prev = inIdx;
            buzzerTimer_prev = buzzerTimer;
            main_loop_counter++;
        }
    }
}

void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInit;

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = 16;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI_DIV2;
    RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLL_MUL16;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType           = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource        = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider       = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider      = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider      = RCC_HCLK_DIV1;

    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

    PeriphClkInit.PeriphClockSelection    = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection       = RCC_ADCPCLK2_DIV4;
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}
