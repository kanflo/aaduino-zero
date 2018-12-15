/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 Johan Kanflo (github.com/kanflo)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef __PINDEFS_H__
#define __PINDEFS_H__

#define BUTTON_SEL_PORT      GPIOA
#define BUTTON_SEL_PIN       GIO2
#define BUTTON_SEL_EXTI      EXTI2
#define BUTTON_SEL_isr       exti2_isr
#define BUTTON_SEL_NVIC      NVIC_EXTI2_IRQ

#define LED_PORT            GPIOA
#define LED_PIN             GPIO6

#define AUX_PORT            GPIOB
#define AUX_PIN             GPIO15

#define USART1_TXO_PORT     GPIOA
#define USART1_TXO_PIN      GPIO9
#define USART1_TXO_AF       GPIO_AF4
#define USART1_RXI_PORT     GPIOA
#define USART1_RXI_PIN      GPIO10
#define USART1_RXI_AF       GPIO_AF4

#define I2C1_SCL_PORT       GPIOB
#define I2C1_SCL_PIN        GPIO6
#define I2C1_SCL_AF         GPIO_AF1
#define I2C1_SDA_PORT       GPIOB
#define I2C1_SDA_PIN        GPIO7
#define I2C1_SDA_AF         GPIO_AF1

#define SPI1_MOSI_PORT      GPIOB
#define SPI1_MOSI_PIN       GPIO5
#define SPI1_MOSI_AF        GPIO_AF0
#define SPI1_MISO_PORT      GPIOB
#define SPI1_MISO_PIN       GPIO4
#define SPI1_MISO_AF        GPIO_AF0
#define SPI1_SCK_PORT       GPIOB
#define SPI1_SCK_PIN        GPIO3
#define SPI1_SCK_AF         GPIO_AF0
#define SPI1_RFM_CS_PORT    GPIOA
#define SPI1_RFM_CS_PIN     GPIO15
#define SPI1_FLASH_CS_PORT  GPIOA
#define SPI1_FLASH_CS_PIN   GPIO12

#define TEMP_ALERT_PORT     GPIOA
#define TEMP_ALERT_PIN      GPIO0
#define RFM_RESET_PORT      GPIOA
#define RFM_RESET_PIN       GPIO3
#define RFM_IRQ_PORT        GPIOA
#define RFM_IRQ_PIN         GPIO7

#endif // __PINDEFS_H__
