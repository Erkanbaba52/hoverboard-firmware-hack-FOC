// Define to prevent recursive inclusion
#ifndef CONFIG_H
#define CONFIG_H

#include "stm32f1xx_hal.h"

// ############################### VARIANT SELECTION ###############################
#define VARIANT_ADC         // Manuel kontrol ADC (KY-024 hall sensörleri) ← BURAYA EKLEYİN!
#define DUAL_INPUTS         // ADC (Primary) + UART (Auxiliary - Autopilot)
// ########################### END OF VARIANT SELECTION ############################

// ############################### PROJE ÖZEL AYARLAR ###############################
// KY024 Hall Effect Sensör Pin Tanımlamaları
#define HALL_LEFT_PIN    GPIO_PIN_2
#define HALL_LEFT_PORT   GPIOA
#define HALL_RIGHT_PIN   GPIO_PIN_3
#define HALL_RIGHT_PORT  GPIOA

// KY024 Sensör Değer Aralıkları (0-20 aralığı için ölçeklendirilmiş)
#define HALL_MIN_BACKWARD  0     // Manyetik uzak - Tam geri
#define HALL_MAX_BACKWARD  8     // Manyetik uzak - Yavaş geri
#define HALL_MIN_STOP      9     // Orta - Dur
#define HALL_MAX_STOP      11    // Orta - Dur  
#define HALL_MIN_FORWARD   12    // Manyetik yakın - Yavaş ileri
#define HALL_MAX_FORWARD   20    // Manyetik yakın - Tam ileri

// ESP32 İletişim için UART Ayarları
#define UART_ESP32        USART3
#define UART_ESP32_TX_PIN GPIO_PIN_10
#define UART_ESP32_TX_PORT GPIOB
#define UART_ESP32_RX_PIN GPIO_PIN_11
#define UART_ESP32_RX_PORT GPIOB
#define UART_ESP32_BAUD   115200

// Çalışma Modları
#define MODE_MANUAL       0
#define MODE_AUTOPILOT    1

// Protokol Ayarları
#define TELEMETRY_INTERVAL_MS 1000

// 12S LiFePO4 Batarya Ayarları
#define BATTERY_MAX_VOLTAGE   43.8f
#define BATTERY_MIN_VOLTAGE   30.0f
#define BATTERY_NOMINAL_VOLTAGE 38.4f

// Hata Kodları
#define ERROR_NONE          0
#define ERROR_ESP32_COMM    1
#define ERROR_SENSOR_LEFT   2
#define ERROR_SENSOR_RIGHT  3

// ADC Tanımlamaları
#define ADC_LEFT_CHANNEL    ADC_CHANNEL_2
#define ADC_RIGHT_CHANNEL   ADC_CHANNEL_3

// 50cm ÇARK ve KATAMARAN AYARLARI
#define PADDLE_WHEEL_DIAMETER   50
#define CATAMARAN_WEIGHT        100
#define CATAMARAN_CAPACITY      350

// SERİAL PROTOKOL
#define SERIAL_START_FRAME      0xABCD
#define SERIAL_BUFFER_SIZE      64
#define SERIAL_TIMEOUT          160
#define USART3_BAUD             115200
#define USART3_WORDLENGTH       UART_WORDLENGTH_8B

// MOD DEĞİŞİM AYARLARI
#define MANUAL_OVERRIDE_ENABLE  1
#define AUTO_TIMEOUT_MS         5000
// ########################### END OF PROJE ÖZEL AYARLAR ############################

// ############################### ORİJİNAL AYARLAR ###############################
#define PWM_FREQ            16000
#define DEAD_TIME              48
#define DELAY_IN_MAIN_LOOP    5
#define TIMEOUT                20
#define A2BIT_CONV             50
#define ADC_CONV_TIME_7C5      20
#define ADC_CONV_CLOCK_CYCLES  20
#define ADC_CLOCK_DIV          4
#define ADC_TOTAL_CONV_TIME    80
#define BOARD_VARIANT           0
// ########################### END OF ORİJİNAL AYARLAR ############################

// ############################### MOTOR CONTROL #########################
#define CTRL_TYP_SEL    FOC_CTRL
#define CTRL_MOD_REQ    VLT_MODE
#define I_MOT_MAX       16
#define I_DC_MAX        18
#define N_MOT_MAX       1500
#define FIELD_WEAK_ENA  0
// ########################### END OF MOTOR CONTROL ########################

// ############################### BATTERY ###############################
#define BAT_FILT_COEF           655
#define BAT_CALIB_REAL_VOLTAGE  4380
#define BAT_CALIB_ADC           1492
#define BAT_CELLS               12
#define BAT_LVL5                (390 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE
#define BAT_LVL4                (380 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE
#define BAT_LVL3                (370 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE
#define BAT_LVL2                (360 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE
#define BAT_LVL1                (350 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE
#define BAT_DEAD                (320 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE
#define BAT_LVL2_ENABLE         1
#define BAT_LVL1_ENABLE         1
#define BAT_DEAD_ENABLE         1
#define BAT_BLINK_INTERVAL      80
// ######################## END OF BATTERY ###############################

// ############################### TEMPERATURE ###############################
#define TEMP_FILT_COEF          655
#define TEMP_CAL_LOW_ADC        1655
#define TEMP_CAL_LOW_DEG_C      358
#define TEMP_CAL_HIGH_ADC       1588
#define TEMP_CAL_HIGH_DEG_C     489
#define TEMP_WARNING_ENABLE     1
#define TEMP_WARNING            600
#define TEMP_POWEROFF_ENABLE    0
// ######################## END OF TEMPERATURE ###############################

// ############################## DEFAULT SETTINGS ############################
#define INACTIVITY_TIMEOUT        15
#define BEEPS_BACKWARD            1
#define ADC_MARGIN                100
#define ADC_PROTECT_TIMEOUT       100
#define ADC_PROTECT_THRESH        200
#define AUTO_CALIBRATION_ENA
#define DEFAULT_RATE                200
#define DEFAULT_FILTER              8192
#define DEFAULT_SPEED_COEFFICIENT   16384
#define DEFAULT_STEER_COEFFICIENT   8192
// ######################### END OF DEFAULT SETTINGS ##########################

// ############################## INPUT FORMAT ############################
#define ADC_INPUT_LEFT_CH      2
#define ADC_INPUT_RIGHT_CH     3
// ############################## END OF INPUT FORMAT ############################

// ################################# VARIANT_ADC SETTINGS ############################
#ifdef VARIANT_ADC
  #define CONTROL_ADC           0
  #define DUAL_INPUTS
  #define PRI_INPUT1            3, 0, 1024, 4095, 120
  #define PRI_INPUT2            3, 0, 1024, 4095, 120
  #define FLASH_WRITE_KEY       0x1101
  #define CONTROL_SERIAL_USART3 1
  #define FEEDBACK_SERIAL_USART3
  #define AUX_INPUT1            3, -1000, 0, 1000, 0
  #define AUX_INPUT2            3, -1000, 0, 1000, 0
#endif
// ############################# END OF VARIANT_ADC SETTINGS #########################

// ############################### APPLY DEFAULT SETTINGS ###############################
#ifndef RATE
  #define RATE DEFAULT_RATE
#endif
#ifndef FILTER
  #define FILTER DEFAULT_FILTER
#endif
#ifndef SPEED_COEFFICIENT
  #define SPEED_COEFFICIENT DEFAULT_SPEED_COEFFICIENT
#endif
#ifndef STEER_COEFFICIENT
  #define STEER_COEFFICIENT DEFAULT_STEER_COEFFICIENT
#endif
// ########################### END OF APPLY DEFAULT SETTING ############################

// ############################### VALIDATE SETTINGS ###############################
#if !defined(VARIANT_ADC)
  #error VARIANT_ADC tanımlanmalı!
#endif

#endif /* CONFIG_H */
