/*
 * Adafruit_I2CDevice.cpp
 *
 * Each class object controls one hardware device.
 *
 *  Created on: Oct 25, 2022
 *      Author: joconnor
 */


#include <stm32yyxx_hal_def.h>
#include <stm32yyxx_hal_i2c.h>

#include <Adafruit_I2CDevice.h>

#ifdef __cplusplus
extern "C" {
#endif
	extern I2C_HandleTypeDef hi2c1;
	extern I2C_HandleTypeDef hi2c2;
	extern I2C_HandleTypeDef hi2c4;
#ifdef __cplusplus
}
#endif

constexpr unsigned long int I2C_TIMEOUT = 1000;
constexpr unsigned long int I2C_TRIALS = 5;
#define ADDR_BUFFER_SZ 32

HAL_StatusTypeDef I2c_status = HAL_ERROR;

/**
  * @brief Variables related to Master process
  */
/* aCommandCode declaration array    */
/* [CommandCode][RequestSlaveAnswer] */
/* {CODE, YES/NO}                    */
const char* aCommandCode[4][4] = {
  {"CHIP_NAME", "YES"},
  {"CHIP_REVISION", "YES"},
  {"LOW_POWER", "NO"},
  {"WAKE_UP", "NO"}};

#define BUFFER_SIZE 0xF
uint8_t *pMasterTransmitBuffer = (uint8_t*)((&aCommandCode[0]));
uint8_t      ubMasterNbCommandCode     = sizeof(aCommandCode[0][0]);
uint8_t      aMasterReceiveBuffer[BUFFER_SIZE] = {0};
__IO uint8_t ubMasterNbDataToReceive   = sizeof(aMasterReceiveBuffer);
__IO uint8_t ubMasterNbDataToTransmit  = 0;
uint8_t      ubMasterCommandIndex      = 0;
__IO uint8_t ubMasterReceiveIndex      = 0;

/**
  * @brief Variables related to Slave process
  */
const char* aSlaveInfo[]      = {
                  "STM32F767xx",
                  "1.2.3"};

uint8_t       aSlaveReceiveBuffer[BUFFER_SIZE]  = {0};
uint8_t*      pSlaveTransmitBuffer      = 0;
__IO uint8_t  ubSlaveNbDataToTransmit   = 0;
uint8_t       ubSlaveInfoIndex          = 0xFF;
__IO uint8_t  ubSlaveReceiveIndex       = 0;
uint32_t      uwTransferDirection       = 0;
__IO uint32_t uwTransferInitiated       = 0;
__IO uint32_t uwTransferEnded           = 0;


Adafruit_I2CDevice::Adafruit_I2CDevice(I2C_AddressTypeDef Address, I2C_HandleTypeDef * Handle, bool Master)
{
	// TODO Auto-generated constructor stub
	_begun = false;
	_maxBufferSize = MAX_BUFFER_SIZE;
	i2c_device.Address = (Address<<1);
	i2c_device.Handle = Handle;
	i2c_device.Type = Handle->Instance;
	i2c_device.Master = Master;
}


Adafruit_I2CDevice::~Adafruit_I2CDevice()
{
	// TODO Auto-generated destructor stub
}

I2C_AddressTypeDef Adafruit_I2CDevice::address(void)
{
	return (i2c_device.Address>>1);
}

bool Adafruit_I2CDevice::begin(bool addr_detect)
{
	_begun = true;
	return (true);
}

void Adafruit_I2CDevice::end(void)
{
	_begun = false;
}

uint8_t Adafruit_I2CDevice::scan_addresses(I2C_AddressTypeDef* addressbuffer)
{
#define MAX_I2C_ADDRESS 0x7F
#define START_I2C_ADDRESS 0x0E

	for(int x=0; x<ADDR_BUFFER_SZ; x++)
	{
		addressbuffer[x] = 0;
	}

	uint8_t count = 0;
	for(uint16_t addr = START_I2C_ADDRESS; addr< MAX_I2C_ADDRESS; addr++)
	{
		if(HAL_I2C_IsDeviceReady(i2c_device.Handle, (addr<<1), 1, I2C_TIMEOUT) == HAL_OK)
		{
			addressbuffer[count++] = addr;
		}
	}
	return (count);
}


bool Adafruit_I2CDevice::detected(void)
{
	return (HAL_I2C_IsDeviceReady(i2c_device.Handle, i2c_device.Address, I2C_TRIALS, I2C_TIMEOUT) == HAL_OK);
}


bool Adafruit_I2CDevice::read(BufferTypeDef buffer, LengthTypeDef len, bool stop)
{
	return (receive_hal_i2c(buffer, len) == HAL_OK);
}


/********************/
/* Adafruit Methods */
/********************/

bool Adafruit_I2CDevice::write(BufferTypeDef buffer, LengthTypeDef len, bool stop,
		BufferTypeDef prefix_buffer, LengthTypeDef prefix_len)
{
	if(prefix_len != 0)
	{
		transmit_hal_i2c(prefix_buffer, prefix_len);
	}

	return (transmit_hal_i2c(buffer, len) == HAL_OK);
}

bool Adafruit_I2CDevice::write_then_read(BufferTypeDef write_buffer, LengthTypeDef write_len,
		BufferTypeDef read_buffer, LengthTypeDef read_len, bool stop)
{
	/* TODO: Any Data Validation needed? */

	auto result1 = write(write_buffer, write_len);
	auto result2 = read(read_buffer, read_len);

	return (result1 == true && result2 == true);
}

bool Adafruit_I2CDevice::setSpeed(uint32_t desiredclk)
{
	/* TODO Required? */

	return (true);
}


HAL_StatusTypeDef Adafruit_I2CDevice::transmit_hal_i2c(BufferTypeDef pData, LengthTypeDef Size)
{
	/* TODO: Slave doesn't need address, needs a default */

	/* Address is already left-shifted for HAL driver */

	if(i2c_device.Master)
		return(HAL_I2C_Master_Transmit(i2c_device.Handle, i2c_device.Address, pData, Size, I2C_TIMEOUT));

	return (HAL_I2C_Slave_Transmit(i2c_device.Handle, pData, Size, I2C_TIMEOUT));
}

HAL_StatusTypeDef Adafruit_I2CDevice::receive_hal_i2c(BufferTypeDef pData, LengthTypeDef Size)
{
	/* Address is already left-shifted for HAL driver */

	if(i2c_device.Master)
		return (HAL_I2C_Master_Receive(i2c_device.Handle, i2c_device.Address, pData, Size, I2C_TIMEOUT));
	else
		return (HAL_I2C_Slave_Receive(i2c_device.Handle, pData, Size, I2C_TIMEOUT));
}

/*
 * Initialize the I2C Interface if necessary
 *
 *
 * */
HAL_StatusTypeDef Adafruit_I2CDevice::init_i2c(I2C_HandleTypeDef* _I2cHandle) //, I2C_TypeDef* _I2Cx )
{
	HAL_StatusTypeDef result = HAL_OK;

	if(HAL_I2C_IsDeviceReady(i2c_device.Handle, i2c_device.Address, I2C_TRIALS, I2C_TIMEOUT) == HAL_OK)
		return (result);

	// TODO: This overrides the settings from the CubeMX
	/*##-1- Configure the I2C peripheral ######################################*/
	//_I2cHandle->Instance             = _I2Cx;
	_I2cHandle->Init.Timing          = I2C_TIMING;
	_I2cHandle->Init.OwnAddress1     = I2C_ADDRESS;
	_I2cHandle->Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
	_I2cHandle->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	_I2cHandle->Init.OwnAddress2     = I2C_OWN_ADDRESS;
	_I2cHandle->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	_I2cHandle->Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;

	result = HAL_I2C_Init(_I2cHandle);
	if(result != HAL_OK)
	{
	/* Initialization Error */
	Error_Handler();
	}

	return (result);
}
