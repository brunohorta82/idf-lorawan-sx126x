#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "sx126x-board.h"
#include "radio.h"
#include "sx126x.h"
#include "timer.h"
#include <esp_log.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// SX126x chip type
#define SX1261_CHIP 1
#define SX1262_CHIP 2
#define SX1268_CHIP 2

// Microcontroller - SX126x pin configuration
struct hw_config
{
  int CHIP_TYPE = SX1262_CHIP; // Module type, see defines above
  int PIN_LORA_RESET;          // LORA RESET
  int PIN_LORA_NSS;            // LORA SPI CS
  int PIN_LORA_SCLK;           // LORA SPI CLK
  int PIN_LORA_MISO;           // LORA SPI MISO
  int PIN_LORA_DIO_1;          // LORA DIO_1
  int PIN_LORA_BUSY;           // LORA SPI BUSY
  int PIN_LORA_MOSI;           // LORA SPI MOSI
  int RADIO_TXEN = -1;         // LORA ANTENNA TX ENABLE (eByte E22 module only)
  int RADIO_RXEN = -1;         // LORA ANTENNA RX ENABLE (eByte E22 module only)
  bool USE_DIO2_ANT_SWITCH =
      false;                  // Whether DIO2 is used to control the antenna
  bool USE_DIO3_TCXO = false; // Whether DIO3 is used to control the oscillator
  bool USE_DIO3_ANT_SWITCH =
      false;                     // Whether DIO2 is used to control the antenna
  bool USE_LDO = true;          // Whether SX126x uses LDO or DCDC power regulator
  bool USE_RXEN_ANT_PWR = false; // Whether RX_EN is used as antenna power
  RadioTcxoCtrlVoltage_t TCXO_CTRL_VOLTAGE = TCXO_CTRL_3_3V;
};

extern hw_config _hwConfig;

uint32_t lora_hardware_init();

uint32_t lora_hardware_re_init(hw_config hwConfig);

void lora_hardware_uninit(void);

void BoardGetUniqueId(uint8_t *id);

uint8_t BoardGetBatteryLevel(void);

void BoardDisableIrq(void);

void BoardEnableIrq(void);

bool start_lora_task(void);
