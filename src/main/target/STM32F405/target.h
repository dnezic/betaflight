/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

// OUR TARGET

#pragma once

#define TARGET_BOARD_IDENTIFIER "S405"

#define USBD_PRODUCT_STRING     "Betaflight STM32F405"

#ifndef STM32F405
//#define STM32F405
#endif

#ifndef STM32F407
#define STM32F407
#endif

#ifndef STM32F407xx
#define STM32F407xx
#endif

//#define STM32F407xx
//#define STM32F40_41xxx

#define FC_TARGET_MCU     STM32F405

#undef USE_RX_SX1280

#define USE_I2C_DEVICE_1
#define USE_I2C_DEVICE_2
#define USE_I2C_DEVICE_3

#define USE_UART1
#define USE_UART2
#define USE_UART3
#define USE_UART4
#define USE_UART5
#define USE_UART6

#define SERIAL_PORT_COUNT       (UNIFIED_SERIAL_PORT_COUNT + 6)

//#define UART1_RX_PIN PA10
//#define UART1_TX_PIN PA9

//#define UART2_RX_PIN PD6
//#define UART2_TX_PIN PD5

//#define UART3_RX_PIN PD9
//#define UART3_TX_PIN PD8

#define USE_INVERTER

#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2
#define USE_SPI_DEVICE_3

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff
#define TARGET_IO_PORTE 0xffff
#define TARGET_IO_PORTF 0xffff

#define USE_I2C
#define I2C_FULL_RECONFIGURABILITY

#define USE_DSHOT_BITBAND

#define USE_BEEPER
#define BEEPER_PIN      PC4

#define USE_SPI
#define SPI_FULL_RECONFIGURABILITY

#define USE_VCP

#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2

#define UNIFIED_SERIAL_PORT_COUNT       3

#define USE_GPS

#define USE_USB_DETECT

#define USE_ESCSERIAL

#define USE_ADC

#define USE_CUSTOM_DEFAULTS

#define USE_RX_V202
#define USE_RX_SPI
#undef USE_RX_BIND

#define LED0_PIN PA7
#define LED1_PIN NONE
#define RX_SPI_LED_PIN          PA6
#define RX_SPI_LED              PA6

#define USE_RX_NRF24
#define RX_SPI_DEFAULT_PROTOCOL RX_SPI_NRF24_V202_250K
#define RX_SPI_INSTANCE         SPI1
#define RX_CE_PIN               PB6
#define RX_NSS_PIN              PB7

/*
#define SPI1_NSS_PIN            PB7
#define SPI1_SCK_PIN            PB3
#define SPI1_MISO_PIN           PB4
#define SPI1_MOSI_PIN           PB5

#define SPI2_NSS_PIN            PB12
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15*/

#define USE_GYRO
#define USE_GYRO_MPU6500
#define USE_GYRO_SPI_MPU9250
#define USE_ACC_SPI_MPU9250
#define USE_ACC
#define USE_ACC_MPU6500
#define USE_BARO
#define USE_BARO_BMP280
#define USE_MAG
#define USE_MAG_HMC5883

#undef USE_BARO_SPI_BMP280

#define DEFAULT_RX_FEATURE      FEATURE_RX_SPI

// #define USE_MAG_SPI_AK8963

#define USE_SDCARD
#define USE_SDCARD_SDIO
#undef SDCARD_DETECT_PIN
#undef SDCARD_DETECT_INVERTED
#define SDCARD_SDIO_DMA_OPT 0
#define SDIO_DEVICE             SDIODEV_1
#define SDIO_USE_4BIT           1
#define SDIO_CK_PIN             PC12
#define SDIO_CMD_PIN            PD2
#define SDIO_D0_PIN             PC8
#define SDIO_D1_PIN             PC9
#define SDIO_D2_PIN             PC10
#define SDIO_D3_PIN             PC11

#undef USE_SDCARD_SPI

#undef USE_RX_FLYSKY
#undef USE_RX_SPEKTRUM
#undef USE_RX_EXPRESSLRS
#undef USE_RX_EXPRESSLRS_TELEMETRY
#undef USE_RX_SX1280
#undef USE_RX_SX127X
#undef USE_RX_CC2500

#define USE_FLASHFS
#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT 1

#define USE_FLASH_TOOLS
#define USE_FLASH
#define USE_FLASH_W25Q128FV

