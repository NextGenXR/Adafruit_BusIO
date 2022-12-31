/*
 * Adafruit_I2CDevice.h
 *
 *  Created on: Oct 25, 2022
 *      Author: joconnor
 */

#ifndef BSP_ADAFRUIT_BUSIO_ADAFRUIT_I2CDEVICE_H_
#define BSP_ADAFRUIT_BUSIO_ADAFRUIT_I2CDEVICE_H_

#include <stm32yyxx_hal_def.h>
#include <stm32yyxx_hal_i2c.h>
#include <Adafruit_def.h>

#ifdef __cplusplus

#include <cstdbool>

#define MASTER_BOARD
#define I2C_ADDRESS        0x47	// 71
#define LED_STATUS_TIMEOUT  1000 /* 1 Second */
#define I2C_OWN_ADDRESS 0xFE
#define MAX_BUFFER_SIZE 32

/**
 * @brief Defines related to I2C clock speed
 */

#define I2C_TIMING        0x00D00E28  /* (Rise time = 120ns, Fall time = 25ns) */
// #define I2C_TIMING 0x20404768;	// From CubeMX

typedef struct
{
	I2C_AddressTypeDef Address;
	I2C_HandleTypeDef *Handle;
	I2C_TypeDef *Type;
	bool Master = true;
} I2C_Device;

class Adafruit_I2CDevice
{
public:
	Adafruit_I2CDevice(I2C_AddressTypeDef Address, I2C_HandleTypeDef *Handle, bool Master = true);
	virtual ~Adafruit_I2CDevice();

	uint8_t scan(BufferTypeDef address_list);

	I2C_AddressTypeDef address(void);
	bool begin(bool addr_detect = true);
	void end(void);
	bool detected(void);
	uint8_t scan_addresses(I2C_AddressTypeDef *buffer);

	bool read(BufferTypeDef buffer, LengthTypeDef len = 1, bool stop = true);
	bool write(BufferTypeDef buffer, LengthTypeDef len = 1, bool stop = true, BufferTypeDef prefix_buffer = NULL,
			LengthTypeDef prefix_len = 0);
	bool write_then_read(BufferTypeDef write_buffer, LengthTypeDef write_len, BufferTypeDef read_buffer,
			LengthTypeDef read_len, bool stop = false);
	bool setSpeed(uint32_t desiredclk);

private:
	bool _begun;
	size_t _maxBufferSize;
	bool _read(BufferTypeDef *buffer, LengthTypeDef len, bool stop);

	I2C_Device i2c_device;
	HAL_StatusTypeDef transmit_hal_i2c(BufferTypeDef pData, LengthTypeDef Size);
	HAL_StatusTypeDef receive_hal_i2c(BufferTypeDef pData, LengthTypeDef Size);
	HAL_StatusTypeDef init_i2c(I2C_HandleTypeDef *_I2cHandle);

};

#endif

#endif /* BSP_ADAFRUIT_BUSIO_ADAFRUIT_I2CDEVICE_H_ */
