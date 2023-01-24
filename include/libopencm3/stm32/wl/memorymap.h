/*
 * This file is part of the libopencm3 project.
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef LIBOPENCM3_MEMORYMAP_H
#define LIBOPENCM3_MEMORYMAP_H

#include <libopencm3/cm3/memorymap.h>

/* --- STM32 specific peripheral definitions ------------------------------- */

/* Memory map for all busses */
#define FLASH_BASE			(0x08000000U)
#define PERIPH_BASE			(0x40000000U)
#define FMC1_BANK_BASE			(0x60000000U)
#define FMC3_BANK_BASE			(0x80000000U)
#define INFO_BASE			(0x1fff0000U)
#define PERIPH_BASE_APB1		(PERIPH_BASE + 0x00000)
#define PERIPH_BASE_APB2		(PERIPH_BASE + 0x10000)
#define PERIPH_BASE_AHB1		(PERIPH_BASE + 0x20000)
#define PERIPH_BASE_AHB2		(0x48000000U)
#define PERIPH_BASE_AHB3		(0x58000000U)
#define PERIPH_BASE_APB3		(0x58010000U)

/* Register boundary addresses */

/* APB1 */
#define TIM2_BASE			(PERIPH_BASE_APB1 + 0x0000)
#define RTC_BASE			(PERIPH_BASE_APB1 + 0x2800)
#define WWDG_BASE			(PERIPH_BASE_APB1 + 0x2c00)
#define IWDG_BASE			(PERIPH_BASE_APB1 + 0x3000)
#define SPI2_BASE			(PERIPH_BASE_APB1 + 0x3800)
#define USART2_BASE			(PERIPH_BASE_APB1 + 0x4400)
#define I2C1_BASE			(PERIPH_BASE_APB1 + 0x5400)
#define I2C2_BASE			(PERIPH_BASE_APB1 + 0x5800)
#define I2C3_BASE			(PERIPH_BASE_APB1 + 0x5c00)
#define DAC_BASE			(PERIPH_BASE_APB1 + 0x7400)
#define LPTIM1_BASE			(PERIPH_BASE_APB1 + 0x7c00)
#define LPUART1_BASE			(PERIPH_BASE_APB1 + 0x8000)
#define LPTIM2_BASE			(PERIPH_BASE_APB1 + 0x9400)
#define LPTIM3_BASE			(PERIPH_BASE_APB1 + 0x9800)


/* APB2 */
#define SYSCFG_BASE			(PERIPH_BASE_APB2 + 0x0000)
#define VREFBUF_BASE			(PERIPH_BASE_APB2 + 0x0030)
/* TODO: handle SYSCFG */
#define SYSCFG_CONTINUE_BASE (PERIPH_BASE_APB2 + 0x0100)
#define COMP_BASE			(PERIPH_BASE_APB2 + 0x0200)
#define TIM1_BASE			(PERIPH_BASE_APB2 + 0x2C00)
#define SPI1_BASE			(PERIPH_BASE_APB2 + 0x3000)
#define USART1_BASE			(PERIPH_BASE_APB2 + 0x3800)
#define TIM16_BASE			(PERIPH_BASE_APB2 + 0x4400)
#define TIM17_BASE			(PERIPH_BASE_APB2 + 0x4800)

/* AHB1 */
#define DMA1_BASE			(PERIPH_BASE_AHB1 + 0x0000)
#define DMA2_BASE			(PERIPH_BASE_AHB1 + 0x0400)
#define CRC_BASE			(PERIPH_BASE_AHB1 + 0x3000)

/* AHB2 */
#define GPIO_PORT_A_BASE		(PERIPH_BASE_AHB2 + 0x0000)
#define GPIO_PORT_B_BASE		(PERIPH_BASE_AHB2 + 0x0400)
#define GPIO_PORT_C_BASE		(PERIPH_BASE_AHB2 + 0x0800)
#define GPIO_PORT_H_BASE		(PERIPH_BASE_AHB2 + 0x1c00)
/* Still AHB2, good job ST */
#define ADC1_BASE			(PERIPH_BASE_APB2 + 0x2400)

/* AHB3 */
#define RCC_BASE			            (PERIPH_BASE_AHB3 + 0x0000)
#define POWER_CONTROL_BASE	      (PERIPH_BASE_AHB3 + 0x0400)
#define EXTI_BASE			            (PERIPH_BASE_AHB3 + 0x0800)
#define RNG_BASE			            (PERIPH_BASE_AHB3 + 0x1000)
#define AES_BASE		            	(PERIPH_BASE_AHB3 + 0x1800)

/* APB3 */
#define SUBGHZSPI_BASE            (PERIPH_BASE_APB3 + 0x0000)

/* AHB3 is split in multiple memory ranges, thanks ST */
#define FLASH_MEM_INTERFACE_BASE	(0x1FFF7800)

/* Private peripherals */
#define DBGMCU_BASE			(PPBI_BASE + 0x00042000)

/* Device Electronic Signature */
#define DESIG_FLASH_SIZE_BASE		(INFO_BASE + 0x75e0)
#define DESIG_UNIQUE_ID_BASE		(INFO_BASE + 0x7590)
#define DESIG_UNIQUE_ID0		MMIO32(DESIG_UNIQUE_ID_BASE)
#define DESIG_UNIQUE_ID1		MMIO32(DESIG_UNIQUE_ID_BASE + 4)
#define DESIG_UNIQUE_ID2		MMIO32(DESIG_UNIQUE_ID_BASE + 0x14)
#define DESIG_PACKAGE			MMIO16((INFO_BASE + 0x7500))

/* ST provided factory calibration values @ 3.0V */
#define ST_VREFINT_CAL			MMIO16((INFO_BASE + 0x75aa))
#define ST_TSENSE_CAL1_30C		MMIO16((INFO_BASE + 0x75a8))
#define ST_TSENSE_CAL2_110C		MMIO16((INFO_BASE + 0x75ca))

#endif
