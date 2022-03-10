//----------------------------------------------------------------------------------------------------
//библиотеки
//----------------------------------------------------------------------------------------------------
#include "i2c.h"
#include "stm32f4xx_hal.h"

//----------------------------------------------------------------------------------------------------
//макроопределения
//----------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------
//глобальные переменные
//----------------------------------------------------------------------------------------------------

//частота I2C
static const uint32_t F_I2C=100000UL;
static I2C_HandleTypeDef hi2c3;

//----------------------------------------------------------------------------------------------------
//прототипы функций
//----------------------------------------------------------------------------------------------------


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//функции
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//----------------------------------------------------------------------------------------------------
//инициализация
//----------------------------------------------------------------------------------------------------
bool I2C_Init(void)
{
 __HAL_RCC_GPIOA_CLK_ENABLE(); 	
 __HAL_RCC_GPIOC_CLK_ENABLE(); 	
 hi2c3.Instance=I2C3;
 hi2c3.Init.ClockSpeed=F_I2C;
 hi2c3.Init.DutyCycle=I2C_DUTYCYCLE_2;
 hi2c3.Init.OwnAddress1=0;
 hi2c3.Init.AddressingMode=I2C_ADDRESSINGMODE_7BIT;
 hi2c3.Init.DualAddressMode=I2C_DUALADDRESS_DISABLE;
 hi2c3.Init.OwnAddress2=0;
 hi2c3.Init.GeneralCallMode=I2C_GENERALCALL_DISABLE;
 hi2c3.Init.NoStretchMode=I2C_NOSTRETCH_DISABLE;
 if (HAL_I2C_Init(&hi2c3)!= HAL_OK) return(false);
 return(true);
}
//----------------------------------------------------------------------------------------------------
//передать данные
//----------------------------------------------------------------------------------------------------
bool I2C_WriteBuffer(uint8_t addr,uint8_t *buffer,uint8_t buffer_size)
{
 static const uint32_t delay_ms=1000;	
 while(HAL_I2C_Master_Transmit(&hi2c3,(uint16_t)(addr<<1),(uint8_t*)buffer,(uint16_t)buffer_size, delay_ms)!=HAL_OK)
 {
  if (HAL_I2C_GetError(&hi2c3)!=HAL_I2C_ERROR_AF) return(false);
 }
 while(HAL_I2C_GetState(&hi2c3)!=HAL_I2C_STATE_READY)
 {
 }
 return(true);
}
//----------------------------------------------------------------------------------------------------
//принять данные
//----------------------------------------------------------------------------------------------------
bool I2C_ReadBuffer(uint8_t addr,uint8_t reg_addr,uint8_t *buffer,uint8_t buffer_size)
{
 static const uint32_t delay_ms=1000;	
 I2C_WriteBuffer(addr,&reg_addr,1);
 while(HAL_I2C_Master_Receive(&hi2c3,(uint16_t)(addr<<1),buffer,(uint16_t)buffer_size,delay_ms)!=HAL_OK)
 {
  if (HAL_I2C_GetError(&hi2c3)!=HAL_I2C_ERROR_AF) return(false);
 }
 while(HAL_I2C_GetState(&hi2c3)!=HAL_I2C_STATE_READY)
 {
 }
 return(true);
}
