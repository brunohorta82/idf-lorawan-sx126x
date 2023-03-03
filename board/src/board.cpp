#include "board.h"

/** Semaphore used by SX126x IRQ handler to wake up LoRaWAN task */
SemaphoreHandle_t _lora_sem = NULL;

/** LoRa task handle */
TaskHandle_t _loraTaskHandle;
/** GPS reading task */
void _lora_task(void *pvParameters);

hw_config _hwConfig;

uint8_t BoardGetBatteryLevel(void)
{
  uint8_t batteryLevel = 0;
  // TODO BE IMPLEMENTED
  return batteryLevel;
}

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR BoardDisableIrq(void) { }

void IRAM_ATTR BoardEnableIrq(void) {  }
uint32_t lora_hardware_init()
{
 // Define the HW configuration between MCU and SX126x
  _hwConfig.CHIP_TYPE = SX1262_CHIP;           // Chip type, SX1261 or SX1262
  _hwConfig.PIN_LORA_RESET = CONFIG_LORA_RESET; // LORA RESET
  _hwConfig.PIN_LORA_NSS = CONFIG_LORA_NSS;     // LORA SPI CS
  _hwConfig.PIN_LORA_SCLK = CONFIG_SPI_SCLK;   // LORA SPI CLK
  _hwConfig.PIN_LORA_MISO = CONFIG_SPI_MISO;   // LORA SPI MISO
  _hwConfig.PIN_LORA_DIO_1 = CONFIG_LORA_DIO1; // LORA DIO_1
  _hwConfig.PIN_LORA_BUSY = CONFIG_LORA_BUSY;   // LORA SPI BUSY
  _hwConfig.PIN_LORA_MOSI = CONFIG_SPI_MOSI;   // LORA SPI MOSI
  TimerConfig();

  SX126xIoInit();

  // After power on the sync word should be 2414. 4434 could be possible on a
  // restart If we got something else, something is wrong.
  uint16_t readSyncWord = 0;
  SX126xReadRegisters(REG_LR_SYNCWORD, (uint8_t *)&readSyncWord, 2);

  ESP_LOGI("BRD", "SyncWord = %04X", readSyncWord);

  if ((readSyncWord == 0x2414) || (readSyncWord == 0x4434))
  {

    if (start_lora_task())
    {
      return 0;
    }
    else
    {
      return 1;
    }
  }
  return 1;
}

uint32_t lora_hardware_re_init(hw_config hwConfig)
{
  _hwConfig.CHIP_TYPE = hwConfig.CHIP_TYPE;           // Chip type, SX1261 or SX1262
  _hwConfig.PIN_LORA_RESET = hwConfig.PIN_LORA_RESET; // LORA RESET
  _hwConfig.PIN_LORA_NSS = hwConfig.PIN_LORA_NSS;     // LORA SPI CS
  _hwConfig.PIN_LORA_SCLK = hwConfig.PIN_LORA_SCLK;   // LORA SPI CLK
  _hwConfig.PIN_LORA_MISO = hwConfig.PIN_LORA_MISO;   // LORA SPI MISO
  _hwConfig.PIN_LORA_DIO_1 = hwConfig.PIN_LORA_DIO_1; // LORA DIO_1
  _hwConfig.PIN_LORA_BUSY = hwConfig.PIN_LORA_BUSY;   // LORA SPI BUSY
  _hwConfig.PIN_LORA_MOSI = hwConfig.PIN_LORA_MOSI;   // LORA SPI MOSI
  _hwConfig.RADIO_TXEN =
      hwConfig.RADIO_TXEN; // LORA ANTENNA TX ENABLE (e.g. eByte E22 module)
  _hwConfig.RADIO_RXEN =
      hwConfig.RADIO_RXEN; // LORA ANTENNA RX ENABLE (e.g. eByte E22 module)
  _hwConfig.USE_DIO2_ANT_SWITCH =
      hwConfig.USE_DIO2_ANT_SWITCH; // LORA DIO2 controls antenna
  _hwConfig.USE_DIO3_TCXO =
      hwConfig.USE_DIO3_TCXO; // LORA DIO3 controls oscillator voltage (e.g.
                              // eByte E22 module)
  _hwConfig.USE_DIO3_ANT_SWITCH =
      hwConfig.USE_DIO3_ANT_SWITCH; // LORA DIO3 controls antenna (e.g. Insight
                                    // SIP ISP4520 module)
  _hwConfig.USE_RXEN_ANT_PWR =
      hwConfig.USE_RXEN_ANT_PWR; // RXEN used as power for antenna switch

  TimerConfig();

  // TODO SX126xIoReInit();

  // After power on the sync word should be 2414. 4434 could be possible on a
  // restart If we got something else, something is wrong.
  uint16_t readSyncWord = 0;
  SX126xReadRegisters(REG_LR_SYNCWORD, (uint8_t *)&readSyncWord, 2);

  ESP_LOGI("BRD", "SyncWord = %04X", readSyncWord);

  if ((readSyncWord == 0x2414) || (readSyncWord == 0x4434))
  {

    if (start_lora_task())
    {
      return 0;
    }
    else
    {
      return 1;
    }
  }
  return 1;
}

void _lora_task(void *pvParameters)
{
  ESP_LOGI("BRD", "LoRa Task started");

  while (1)
  {
    if (xSemaphoreTake(_lora_sem, portMAX_DELAY) == pdTRUE)
    {
      Radio.BgIrqProcess();
    }
  }
}

bool start_lora_task(void)
{
  _lora_sem = xSemaphoreCreateBinary();

  xSemaphoreGive(_lora_sem);

  xSemaphoreTake(_lora_sem, 10);

  if (!xTaskCreate(_lora_task, "LORA", 4096, NULL, 1, &_loraTaskHandle))

  {
    return false;
  }
  return true;
}

void lora_hardware_uninit(void)
{

  vTaskSuspend(_loraTaskHandle);
  SX126xIoDeInit();
}
