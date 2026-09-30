#ifndef __IIC_H
#define __IIC_H

#include "stm32f10x.h"
#include <stdio.h>

#define I2C_RCC         RCC_APB2Periph_GPIOB
#define I2C_PORT        GPIOB
#define I2C_SCL_PIN     GPIO_Pin_6
#define I2C_SDA_PIN     GPIO_Pin_7

void I2C_Init_All(void);

#endif