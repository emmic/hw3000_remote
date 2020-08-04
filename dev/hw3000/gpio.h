/******************************************************************************
 * @file     gpio.h
 * @brief    gpio funcations
 *           
 *
 * @version  1.0
 * @date     2016-01-14
 *
 * @author   Sundy
 *
 * @note
 * @Copyright (C) 2016 Shanghai Eastsoft Microelectronics C0., Ltd.
 ******************************************************************************/
#ifndef __M928_GPIO_H
#define __M928_GPIO_H

#include "stm8s.h"

#define  HW3000_CSN_PORT        GPIOA
#define  HW3000_CSN_IO          GPIO_PIN_3

#define  HW3000_CLK_PORT        GPIOC
#define  HW3000_CLK_IO          GPIO_PIN_5

#define  HW3000_SDO_PORT        GPIOC
#define  HW3000_SDO_IO          GPIO_PIN_7

#define  HW3000_SDI_PORT        GPIOC
#define  HW3000_SDI_IO          GPIO_PIN_6

#define  HW3000_IRQ_PORT        GPIOC
#define  HW3000_IRQ_IO          GPIO_PIN_4

#define  HW3000_PDN_PORT        GPIOC
#define  HW3000_PDN_IO          GPIO_PIN_3

//************************************************   

#define  LED1_PORT              GPIOD
#define  LED1_IO                GPIO_PIN_4   
   
#define  LED2_PORT              GPIOD
#define  LED2_IO                GPIO_PIN_5
   
#define  KEY1_PORT              GPIOD
#define  KEY1_IO                GPIO_PIN_2
   
#define  KEY2_PORT              GPIOD
#define  KEY2_IO                GPIO_PIN_1
   
#define  KEY3_PORT              GPIOD
#define  KEY3_IO                GPIO_PIN_3

#define  KEY4_PORT              GPIOA
#define  KEY4_IO                GPIO_PIN_2
   
#define  KEY5_PORT              GPIOA
#define  KEY5_IO                GPIO_PIN_1
   
#define  KEY6_PORT              GPIOB
#define  KEY6_IO                GPIO_PIN_5
   
#define  KEY7_PORT              GPIOD
#define  KEY7_IO                GPIO_PIN_6
   
#define  KEY8_PORT              GPIOB
#define  KEY8_IO                GPIO_PIN_4

void MCU_gpio_init(void);
void SPI_CS_Enable();
void SPI_CS_Disable();
void SPI_Write(uint8_t data);
uint8_t SPI_Read();

void SPI_Pwr_ON();
void SPI_Pwr_OFF();



#endif


