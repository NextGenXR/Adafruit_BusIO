
#ifndef Adafruit_SPIDevice_h
#define Adafruit_SPIDevice_h

#ifdef HAL_SPI_MODULE_ENABLED

#include <stdio.h>

#ifdef ARDUINO_NUCLEO_F767ZI
#include <Arduino.h>
#include <wiring_constants.h>
#include VARIANT_H
#endif

#include <Adafruit_def.h>


#if !defined(SPI_INTERFACES_COUNT) ||                                          \
    (defined(SPI_INTERFACES_COUNT) && (SPI_INTERFACES_COUNT > 0))

#ifdef USE_HAL_DRIVER

#if __has_include(<main.h>)
#include <main.h>
#endif

#include <stm32yyxx_hal_def.h>
#include <stm32yyxx_hal_conf.h>

#include <stm32yyxx_hal_spi.h>
#include <stm32yyxx_ll_spi.h>
#include <STM32duino_SPI.h>

#include <stm32yyxx_ll_gpio.h>
#include <stm32yyxx_hal_gpio_ex.h>
#endif

// some modern SPI definitions don't have BitOrder enum
#if (defined(__AVR__) && !defined(ARDUINO_ARCH_MEGAAVR)) ||                    \
    defined(ESP8266) || defined(TEENSYDUINO) || defined(SPARK) ||              \
    defined(ARDUINO_ARCH_SPRESENSE) || defined(MEGATINYCORE) ||                \
    defined(DXCORE) || defined(ARDUINO_AVR_ATmega4809) ||                      \
    defined(ARDUINO_AVR_ATmega4808) || defined(ARDUINO_AVR_ATmega3209) ||      \
    defined(ARDUINO_AVR_ATmega3208) || defined(ARDUINO_AVR_ATmega1609) ||      \
    defined(ARDUINO_AVR_ATmega1608) || defined(ARDUINO_AVR_ATmega809) ||       \
    defined(ARDUINO_AVR_ATmega808) || defined(ARDUINO_ARCH_ARC32)

typedef enum _BitOrder {
  SPI_BITORDER_MSBFIRST = MSBFIRST,
  SPI_BITORDER_LSBFIRST = LSBFIRST,
} BusIOBitOrder;

#elif defined(ESP32) || defined(__ASR6501__) || defined(__ASR6502__)

// some modern SPI definitions don't have BitOrder enum and have different SPI
// mode defines
typedef enum _BitOrder {
  SPI_BITORDER_MSBFIRST = SPI_MSBFIRST,
  SPI_BITORDER_LSBFIRST = SPI_LSBFIRST,
} BusIOBitOrder;

#else
// Some platforms have a BitOrder enum but its named MSBFIRST/LSBFIRST
#define SPI_BITORDER_MSBFIRST MSBFIRST
#define SPI_BITORDER_LSBFIRST LSBFIRST
typedef BitOrder BusIOBitOrder;
#endif

#ifndef STM
#define STM
#endif

#if defined(__AVR__) || defined(TEENSYDUINO)
typedef volatile uint8_t BusIO_PortReg;
typedef uint8_t BusIO_PortMask;
#define BUSIO_USE_FAST_PINIO

#elif defined(ESP8266) || defined(ESP32) || defined(__SAM3X8E__) ||            \
    defined(ARDUINO_ARCH_SAMD)
typedef volatile uint32_t BusIO_PortReg;
typedef uint32_t BusIO_PortMask;
#define BUSIO_USE_FAST_PINIO

#elif (defined(__arm__) || defined(ARDUINO_FEATHER52)) &&                      \
    !defined(ARDUINO_ARCH_MBED) && !defined(ARDUINO_ARCH_RP2040)
typedef volatile uint32_t BusIO_PortReg;
typedef uint32_t BusIO_PortMask;
#if !defined(__ASR6501__) && !defined(__ASR6502__)
#define BUSIO_USE_FAST_PINIO
#endif

#else
#undef BUSIO_USE_FAST_PINIO
#endif

#ifndef LOW
#define LOW 0
#define HIGH 1
#endif

#ifdef ARDUINO
enum GPIO_TypeDef* GPIOx
{
	PORT_A = GPIOA,
	PORT_B = GPIOB,
	PORT_C = GPIOC,
	PORT_D = GPIOD,
	PORT_E = GPIOE,
	PORT_F = GPIOF,
	PORT_G = GPIOG,
	PORT_H = GPIOH
};
#endif

#define SPI_TIMEOUT 1000

typedef struct SPI_Device_t {
	uint16_t SS_Pin;
	GPIO_TypeDef*  SS_Port;
	SPI_HandleTypeDef *hspi;
} SPI_Device;

/**! The class which defines how we will talk to this device over SPI **/
class Adafruit_SPIDevice {
public:

#ifdef ARDUINO
  Adafruit_SPIDevice(int8_t cspin, uint32_t freq = 1000000,
                     BusIOBitOrder dataOrder = SPI_BITORDER_MSBFIRST,
                     uint8_t dataMode = SPI_MODE0, SPIClass *theSPI = &SPI);

  Adafruit_SPIDevice(int8_t cspin, int8_t sck, int8_t miso, int8_t mosi,
                     uint32_t freq = 1000000,
                     BusIOBitOrder dataOrder = SPI_BITORDER_MSBFIRST,
                     uint8_t dataMode = SPI_MODE0);
#endif

#ifdef USE_HAL_DRIVER
  Adafruit_SPIDevice(SPI_HandleTypeDef* Handle, GPIO_TypeDef* csGPIO, uint16_t csGPIO_Pin);
  Adafruit_SPIDevice(SPI_Device_t device);
  Adafruit_SPIDevice(SPI_InitTypeDef* spiInit, SPI_HandleTypeDef* Handle, GPIO_TypeDef* csGPIO, uint16_t csGPIO_Pin);
#endif

  ~Adafruit_SPIDevice();

  bool begin(void);
  bool read(BufferTypeDef buffer, LengthTypeDef len, uint8_t sendvalue = 0xFF);
  bool write(BufferTypeDef buffer, LengthTypeDef len, BufferTypeDef prefix_buffer = NULL,
		  LengthTypeDef prefix_len = 0);
  bool write_then_read(BufferTypeDef write_buffer, LengthTypeDef write_len,
		  	  	  	  BufferTypeDef read_buffer, LengthTypeDef read_len,
                       uint8_t sendvalue = 0xFF);
  bool write_and_read(BufferTypeDef buffer, LengthTypeDef len);

#ifdef ARDUINO
  uint8_t transfer(uint8_t send);
  void transfer(uint8_t *buffer, size_t len);
#endif
  void beginTransaction(void);
  void endTransaction(void);

private:
#ifdef USE_HAL_DRIVER
  SPI_InitTypeDef *_spiInit;
  SPI_HandleTypeDef* _spiHandle;
  HAL_SPI_StateTypeDef spiStatus = HAL_SPI_STATE_RESET;

  GPIO_TypeDef* _csGPIO;
  uint16_t _csGPIO_Pin;
#endif

#ifdef ARDUINO
  SPIClass *_spi;
  SPISettings *_spiSetting;
#endif
  BusIOBitOrder _dataOrder;
  uint8_t _dataMode;

  uint32_t _freq;

  void setChipSelect(int value);
  int8_t _cs = 0;

  // For HAL, use the internal controllers configured in CubeMX or other code.
#ifdef ARDUINO
  int8_t _sck;
  int8_t _mosi;
  int8_t _miso;

 #ifdef BUSIO_USE_FAST_PINIO
  BusIO_PortReg *mosiPort, *clkPort, *misoPort, *csPort;
  BusIO_PortMask mosiPinMask, misoPinMask, clkPinMask, csPinMask;
 #endif
#endif

#endif

  bool _begun;
};

#endif // has SPI defined

#endif /* HAL_SPI_MODULE_ENABLED */




