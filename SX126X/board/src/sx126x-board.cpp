#include "board.h"
#include "sx126x.h"
#include "sx126x-board.h"
#include <driver/spi_master.h>
#include "driver/gpio.h"
#define LOW_LORA 0
#define HIGH_LORA 1
#define SPI_HOST SPI2_HOST
spi_device_handle_t SpiHandle;
void initSPI(void)
{
 esp_err_t ret;
  spi_bus_config_t spi_bus_config = {
      .mosi_io_num = _hwConfig.PIN_LORA_MOSI,
      .miso_io_num = _hwConfig.PIN_LORA_MISO,
      .sclk_io_num = _hwConfig.PIN_LORA_SCLK,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
	  .max_transfer_sz = 0};
  ret = spi_bus_initialize(SPI_HOST, &spi_bus_config, SPI_DMA_CH_AUTO);
  assert(ret == ESP_OK);

  spi_device_interface_config_t devcfg;
  memset(&devcfg, 0, sizeof(spi_device_interface_config_t));
  devcfg.clock_speed_hz = 2000000;
  devcfg.spics_io_num = -1;
  devcfg.queue_size = 7;
  devcfg.mode = 0;
  devcfg.flags = SPI_DEVICE_NO_DUMMY;

  ret = spi_bus_add_device(SPI_HOST, &devcfg, &SpiHandle);
  assert(ret == ESP_OK);
	
}

// No need to initialize DIO3 as output everytime, do it once and remember it
bool dio3IsOutput = false;
bool spi_write_byte(uint8_t *Dataout, size_t DataLength)
{
  spi_transaction_t SPITransaction;

  if (DataLength > 0)
  {
    memset(&SPITransaction, 0, sizeof(spi_transaction_t));
    SPITransaction.length = DataLength * 8;
    SPITransaction.tx_buffer = Dataout;
    SPITransaction.rx_buffer = NULL;
    spi_device_transmit(SpiHandle, &SPITransaction);
  }

  return true;
}

bool spi_read_byte(uint8_t *Datain, uint8_t *Dataout, size_t DataLength)
{
  spi_transaction_t SPITransaction;

  if (DataLength > 0)
  {
    memset(&SPITransaction, 0, sizeof(spi_transaction_t));
    SPITransaction.length = DataLength * 8;
    SPITransaction.tx_buffer = Dataout;
    SPITransaction.rx_buffer = Datain;
    spi_device_transmit(SpiHandle, &SPITransaction);
  }

  return true;
}

uint8_t spi_transfer(uint8_t address)
{
  uint8_t datain[1];
  uint8_t dataout[1];
  dataout[0] = address;
  spi_read_byte(datain, dataout, 1);
  return datain[0];
}

void SX126xIoInit(void)
{

	initSPI();
	dio3IsOutput = false;
  gpio_reset_pin((gpio_num_t)_hwConfig.PIN_LORA_NSS);
  gpio_set_direction((gpio_num_t)_hwConfig.PIN_LORA_NSS, GPIO_MODE_OUTPUT);
  gpio_set_level((gpio_num_t)_hwConfig.PIN_LORA_NSS, HIGH_LORA);

  gpio_reset_pin((gpio_num_t)_hwConfig.PIN_LORA_RESET);
  gpio_set_direction((gpio_num_t)_hwConfig.PIN_LORA_RESET, GPIO_MODE_OUTPUT);

  gpio_reset_pin((gpio_num_t)_hwConfig.PIN_LORA_BUSY);
  gpio_set_direction((gpio_num_t)_hwConfig.PIN_LORA_BUSY, GPIO_MODE_INPUT);
  if (_hwConfig.USE_RXEN_ANT_PWR)
  {ESP_LOGI("LORA", "NA5O");
    if (_hwConfig.RADIO_TXEN != -1)
    {
      ESP_LOGI("LORA", "N4AO");
      gpio_reset_pin((gpio_num_t)_hwConfig.RADIO_TXEN);
      gpio_set_direction((gpio_num_t)_hwConfig.RADIO_TXEN, GPIO_MODE_INPUT);
    }
    gpio_reset_pin((gpio_num_t)_hwConfig.RADIO_RXEN);
    gpio_set_direction((gpio_num_t)_hwConfig.RADIO_RXEN, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)_hwConfig.PIN_LORA_NSS, LOW_LORA);
  }
  else if ((_hwConfig.RADIO_TXEN != -1) && (_hwConfig.RADIO_RXEN != -1))
  {
    ESP_LOGI("LORA", "NAO2");
    gpio_reset_pin((gpio_num_t)_hwConfig.RADIO_TXEN);
    gpio_set_direction((gpio_num_t)_hwConfig.RADIO_TXEN, GPIO_MODE_OUTPUT);
    gpio_reset_pin((gpio_num_t)_hwConfig.RADIO_RXEN);
    gpio_set_direction((gpio_num_t)_hwConfig.RADIO_RXEN, GPIO_MODE_OUTPUT);
    SX126xRXena();
  }

	SX126xReset();
}

void SX126xIoReInit(void)
{
	ESP_LOGW("FUNCTION NOT IMPLEMENTED","SX126xIoReInit");
}

void SX126xIoIrqInit(DioIrqHandler dioIrq)
{
	//attachInterrupt(_hwConfig.PIN_LORA_DIO_1, dioIrq, RISING);
	gpio_install_isr_service(0);
	 gpio_set_direction((gpio_num_t)_hwConfig.PIN_LORA_DIO_1, GPIO_MODE_INPUT);
	gpio_pulldown_en((gpio_num_t)_hwConfig.PIN_LORA_DIO_1);
    gpio_pullup_dis((gpio_num_t)_hwConfig.PIN_LORA_DIO_1);
	gpio_set_intr_type((gpio_num_t)_hwConfig.PIN_LORA_DIO_1, GPIO_INTR_POSEDGE);
    gpio_isr_handler_add((gpio_num_t)_hwConfig.PIN_LORA_DIO_1, dioIrq,(void *)_hwConfig.PIN_LORA_DIO_1);
}

void SX126xIoDeInit(void)
{
	dio3IsOutput = false;
	//detachInterrupt(_hwConfig.PIN_LORA_DIO_1);	
	gpio_reset_pin((gpio_num_t)_hwConfig.PIN_LORA_NSS);
	gpio_reset_pin((gpio_num_t)_hwConfig.PIN_LORA_BUSY);
  	gpio_reset_pin((gpio_num_t)_hwConfig.PIN_LORA_DIO_1);
  	gpio_reset_pin((gpio_num_t)_hwConfig.PIN_LORA_RESET);
}

void SX126xReset(void)
{
    gpio_set_direction((gpio_num_t)_hwConfig.PIN_LORA_RESET, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)_hwConfig.PIN_LORA_RESET, LOW_LORA);
	delay(10);
	gpio_set_direction((gpio_num_t)_hwConfig.PIN_LORA_RESET, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)_hwConfig.PIN_LORA_RESET, HIGH_LORA);
	delay(20);
	dio3IsOutput = false;
}

void SX126xWaitOnBusy()
{
	int timeout = 1000;
	while (gpio_get_level((gpio_num_t)_hwConfig.PIN_LORA_BUSY))
	{
		delay(1);
		timeout -= 1;
		if (timeout < 0)
		{
			/// \todo This error should be reported to the main app
			ESP_LOGI("LORA", "[SX126xWaitOnBusy] Timeout waiting for BUSY low");
			return;
		}
	}
}

void SX126xWakeup(void)
{
  dio3IsOutput = false;
  gpio_set_level((gpio_num_t)_hwConfig.PIN_LORA_NSS, LOW_LORA);
  spi_transfer((uint8_t)RADIO_GET_STATUS);
  spi_transfer(RADIO_RESET_STATS);
  gpio_set_level((gpio_num_t)_hwConfig.PIN_LORA_NSS, HIGH_LORA);
  SX126xWaitOnBusy();
}

void SX126xWriteCommand(RadioCommands_t command, uint8_t *buffer, uint16_t size)
{
  SX126xCheckDeviceReady();
  gpio_set_level((gpio_num_t)_hwConfig.PIN_LORA_NSS, LOW_LORA);
  spi_transfer((uint8_t)command);
  for (uint16_t n = 0; n < size; n++)
  {
    spi_transfer(buffer[n]);
  }
 gpio_set_level((gpio_num_t)_hwConfig.PIN_LORA_NSS, HIGH_LORA);
 if (command != RADIO_SET_SLEEP)
	{
		SX126xWaitOnBusy();
	}
  
}

void SX126xReadCommand(RadioCommands_t command, uint8_t *buffer, uint16_t size)
{
  SX126xCheckDeviceReady();
  gpio_set_level((gpio_num_t)_hwConfig.PIN_LORA_NSS, LOW_LORA);
  spi_transfer((uint8_t)command);
  spi_transfer(0x00);
  for (uint16_t n = 0; n < size; n++)
  {
    buffer[n] = spi_transfer(RADIO_RESET_STATS);
  }
  gpio_set_level((gpio_num_t)_hwConfig.PIN_LORA_NSS, HIGH_LORA);
  SX126xWaitOnBusy();
}

void SX126xWriteRegisters(uint16_t address, uint8_t *buffer, uint16_t size)
{
	SX126xCheckDeviceReady();
    gpio_set_level((gpio_num_t)_hwConfig.PIN_LORA_NSS, LOW_LORA);
	spi_transfer(RADIO_WRITE_REGISTER); // 0x0D
	spi_transfer((address & 0xFF00) >> 8);
	spi_transfer(address & 0xff);
	for (uint16_t n = 0; n < size; n++)
	{
	spi_transfer(buffer[n]);
	}
	gpio_set_level((gpio_num_t)_hwConfig.PIN_LORA_NSS, HIGH_LORA);
	SX126xWaitOnBusy();
}

void SX126xWriteRegister(uint16_t address, uint8_t value)
{
	SX126xWriteRegisters(address, &value, 1);
}

void SX126xReadRegisters(uint16_t address, uint8_t *buffer, uint16_t size)
{
  SX126xCheckDeviceReady();
  gpio_set_level((gpio_num_t)_hwConfig.PIN_LORA_NSS, LOW_LORA);
  spi_transfer(RADIO_READ_REGISTER); // 0x1D
  spi_transfer((address & 0xFF00) >> 8);
  spi_transfer(address & 0xff);
  spi_transfer(RADIO_RESET_STATS);

  for (uint16_t n = 0; n < size; n++)
  {
    buffer[n] = spi_transfer(RADIO_RESET_STATS);
  }
  gpio_set_level((gpio_num_t)_hwConfig.PIN_LORA_NSS, HIGH_LORA);
  SX126xWaitOnBusy();
}

uint8_t SX126xReadRegister(uint16_t address)
{
	uint8_t data;
	SX126xReadRegisters(address, &data, 1);
	return data;
}

void SX126xWriteBuffer(uint8_t offset, uint8_t *buffer, uint8_t size)
{

  SX126xCheckDeviceReady();
  gpio_set_level((gpio_num_t)_hwConfig.PIN_LORA_NSS, LOW_LORA);

  spi_transfer(RADIO_WRITE_BUFFER); // 0x0E
  spi_transfer(offset);                  // offset in tx fifo
  for (uint8_t i = 0; i < size; i++)
  {
    spi_transfer(buffer[i]);
  }
  gpio_set_level((gpio_num_t)_hwConfig.PIN_LORA_NSS, HIGH_LORA);
  SX126xWaitOnBusy();
}

void SX126xReadBuffer(uint8_t offset, uint8_t *buffer, uint8_t size)
{
  SX126xCheckDeviceReady();;
  SX126xWaitOnBusy();
  gpio_set_level((gpio_num_t)_hwConfig.PIN_LORA_NSS, LOW_LORA);
  spi_transfer(RADIO_READ_BUFFER); // 0x1E
  spi_transfer(offset);
  spi_transfer(0x00);
  for (uint16_t i = 0; i < size; i++)
  {
    buffer[i] = spi_transfer(RADIO_RESET_STATS);
  }
  gpio_set_level((gpio_num_t)_hwConfig.PIN_LORA_NSS, HIGH_LORA);
  SX126xWaitOnBusy();
}

void SX126xSetRfTxPower(int8_t power)
{
	SX126xSetTxParams(power, RADIO_RAMP_40_US);
}

uint8_t SX126xGetPaSelect(uint32_t channel)
{
	if (_hwConfig.CHIP_TYPE == SX1262_CHIP)
	{
		return SX1262;
	}
	else
	{
		return SX1261;
	}
}

static void SX126xDio3Control(bool state)
{
	ESP_LOGW("FUNCTION NOT IMPLEMENTED","SX126xIoDeInit");
}

void SX126xAntSwOn(void)
{
	// Use if DIO3 is used as antenna switch power control
	if (_hwConfig.USE_DIO3_ANT_SWITCH)
	{
		SX126xDio3Control(true);
	}

	// Use if RADIO_RXEN is used as antenna switch power control
	if (_hwConfig.USE_RXEN_ANT_PWR)
	{
		gpio_set_level((gpio_num_t)_hwConfig.RADIO_RXEN, HIGH_LORA);
	}
}

void SX126xAntSwOff(void)
{
	// Use if DIO3 is used as antenna switch power control
	if (_hwConfig.USE_DIO3_ANT_SWITCH)
	{
		SX126xDio3Control(false);
	}
	// Use if RADIO_RXEN is used as antenna switch power control
	if (_hwConfig.USE_RXEN_ANT_PWR)
	{
		gpio_set_level((gpio_num_t)_hwConfig.RADIO_RXEN, LOW_LORA);
	}
}

void SX126xRXena(void)
{
  if (!_hwConfig.USE_RXEN_ANT_PWR)
  {
    if ((_hwConfig.RADIO_RXEN != -1) && (_hwConfig.RADIO_TXEN != -1))
    {
      gpio_set_level((gpio_num_t)_hwConfig.RADIO_RXEN, HIGH_LORA);
      gpio_set_level((gpio_num_t)_hwConfig.RADIO_TXEN, LOW_LORA);
    }
  }
  else
  {
    gpio_set_level((gpio_num_t)_hwConfig.RADIO_RXEN, HIGH_LORA);
  }
}

void SX126xTXena(void)
{
  if (!_hwConfig.USE_RXEN_ANT_PWR)
  {
    if ((_hwConfig.RADIO_RXEN != -1) && (_hwConfig.RADIO_TXEN != -1))
    {
      gpio_set_level((gpio_num_t)_hwConfig.RADIO_RXEN, LOW_LORA);
      gpio_set_level((gpio_num_t)_hwConfig.RADIO_TXEN, HIGH_LORA);
    }
  }
  else
  {
    gpio_set_level((gpio_num_t)_hwConfig.RADIO_RXEN, HIGH_LORA);
  }
}
bool SX126xCheckRfFrequency(uint32_t frequency)
{
	// Implement check. Currently all frequencies are supported
	return true;
}

void SX126xGetStats(uint16_t *nb_pkt_received, uint16_t *nb_pkt_crc_error, uint16_t *nb_pkt_length_error)
{
	uint8_t buf[6];

	SX126xReadCommand(RADIO_GET_STATS, buf, 6);

	*nb_pkt_received = (buf[0] << 8) | buf[1];
	*nb_pkt_crc_error = (buf[2] << 8) | buf[3];
	*nb_pkt_length_error = (buf[4] << 8) | buf[5];
}

void SX126xResetStats(void)
{
	uint8_t buf[6] = {0x00};

	SX126xWriteCommand(RADIO_RESET_STATS, buf, 6);
}
