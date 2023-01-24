/** @defgroup rcc_defines RCC Defines
 *
 * @ingroup STM32L4xx_defines
 *
 * @brief <b>Defined Constants and Types for the STM32L4xx Reset and Clock
 * Control</b>
 *
 * @version 1.0.0
 *
 * @author @htmlonly &copy; @endhtmlonly 2015 Karl Palsson <karlp@tweak.net.au>
 *
 * @date 12 November 2015
 *
 * LGPL License Terms @ref lgpl_license
 *  */

/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2015 Karl Palsson <karlp@tweak.net.au>
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
 *
 */

/**@{*/

#ifndef LIBOPENCM3_RCC_H
#define LIBOPENCM3_RCC_H

#include <libopencm3/stm32/pwr.h>

/** @defgroup rcc_registers RCC Registers
 * @brief Reset / Clock Control Registers
 @{*/
 /** Clock control register */
#define RCC_CR				MMIO32(RCC_BASE + 0x00)
#define RCC_ICSCR			MMIO32(RCC_BASE + 0x04)
 /** Clock Configuration register */
#define RCC_CFGR			MMIO32(RCC_BASE + 0x08)
/** PLL Configuration register */
#define RCC_PLLCFGR			MMIO32(RCC_BASE + 0x0c)

/** Clock interrupt enable register */
#define RCC_CIER			MMIO32(RCC_BASE + 0x18)
/** Clock interrupt flag resiger */
#define RCC_CIFR			MMIO32(RCC_BASE + 0x1c)
/** Clock interrupt clear register */
#define RCC_CICR			MMIO32(RCC_BASE + 0x20)
#define RCC_AHB1RSTR_OFFSET		0x28
#define RCC_AHB1RSTR			MMIO32(RCC_BASE + RCC_AHB1RSTR_OFFSET)
#define RCC_AHB2RSTR_OFFSET		0x2c
#define RCC_AHB2RSTR			MMIO32(RCC_BASE + RCC_AHB2RSTR_OFFSET)
#define RCC_AHB3RSTR_OFFSET		0x30
#define RCC_AHB3RSTR			MMIO32(RCC_BASE + RCC_AHB3RSTR_OFFSET)
#define RCC_APB1RSTR1_OFFSET		0x38
#define RCC_APB1RSTR1			MMIO32(RCC_BASE + RCC_APB1RSTR1_OFFSET)
#define RCC_APB1RSTR2_OFFSET		0x3c
#define RCC_APB1RSTR2			MMIO32(RCC_BASE + RCC_APB1RSTR2_OFFSET)
#define RCC_APB2RSTR_OFFSET		0x40
#define RCC_APB2RSTR			MMIO32(RCC_BASE + RCC_APB2RSTR_OFFSET)
#define RCC_APB3RSTR_OFFSET		0x44
#define RCC_APB3RSTR			MMIO32(RCC_BASE + RCC_APB3RSTR_OFFSET)
#define RCC_AHB1ENR_OFFSET		0x48
#define RCC_AHB1ENR			MMIO32(RCC_BASE + RCC_AHB1ENR_OFFSET)
#define RCC_AHB2ENR_OFFSET		0x4c
#define RCC_AHB2ENR			MMIO32(RCC_BASE + RCC_AHB2ENR_OFFSET)
#define RCC_AHB3ENR_OFFSET		0x50
#define RCC_AHB3ENR			MMIO32(RCC_BASE + RCC_AHB3ENR_OFFSET)
#define RCC_APB1ENR1_OFFSET		0x58
#define RCC_APB1ENR1			MMIO32(RCC_BASE + RCC_APB1ENR1_OFFSET)
#define RCC_APB1ENR2_OFFSET		0x5c
#define RCC_APB1ENR2			MMIO32(RCC_BASE + RCC_APB1ENR2_OFFSET)
#define RCC_APB2ENR_OFFSET		0x60
#define RCC_APB2ENR			MMIO32(RCC_BASE + RCC_APB2ENR_OFFSET)
#define RCC_APB3ENR_OFFSET		0x64
#define RCC_APB3ENR			MMIO32(RCC_BASE + RCC_APB3ENR_OFFSET)
#define RCC_AHB1SMENR_OFFSET		0x68
#define RCC_AHB1SMENR			MMIO32(RCC_BASE + RCC_AHB1SMENR_OFFSET)
#define RCC_AHB2SMENR_OFFSET		0x6c
#define RCC_AHB2SMENR			MMIO32(RCC_BASE + RCC_AHB2SMENR_OFFSET)
#define RCC_AHB3SMENR_OFFSET		0x70
#define RCC_AHB3SMENR			MMIO32(RCC_BASE + RCC_AHB3SMENR_OFFSET)
#define RCC_APB1SMENR1_OFFSET		0x78
#define RCC_APB1SMENR1			MMIO32(RCC_BASE + RCC_APB1SMENR1_OFFSET)
#define RCC_APB1SMENR2_OFFSET		0x7c
#define RCC_APB1SMENR2			MMIO32(RCC_BASE + RCC_APB1SMENR2_OFFSET)
#define RCC_APB2SMENR_OFFSET		0x80
#define RCC_APB2SMENR			MMIO32(RCC_BASE + RCC_APB2SMENR_OFFSET)
#define RCC_APB3SMENR_OFFSET		0x84
#define RCC_APB3SMENR			MMIO32(RCC_BASE + RCC_APB3SMENR_OFFSET)
#define RCC_CCIPR			MMIO32(RCC_BASE + 0x88)
/** Backup Domain control register */
#define RCC_BDCR			MMIO32(RCC_BASE + 0x90)
/** Clock control and status register */
#define RCC_CSR				MMIO32(RCC_BASE + 0x94)
#define RCC_EXTCFGR   MMIO32(RCC_BASE + 0x108)
/** @}*/

/** @defgroup rcc_cr_values RCC_CR values
 * @ingroup rcc_registers
 * @brief Clock Control register values
 @{*/
#define RCC_CR_PLLRDY				(1 << 25)
#define RCC_CR_PLLON				(1 << 24)
#define RCC_CR_CSSON				(1 << 19)
#define RCC_CR_HSEBYP     	(1 << 21)
#define RCC_CR_HSERDY				(1 << 17)
#define RCC_CR_HSEPRE       (1 << 20)
#define RCC_CR_HSEON				(1 << 16)
#define RCC_CR_HSIKERDY   	(1 << 12)
#define RCC_CR_HSIASFS			(1 << 11)
#define RCC_CR_HSIRDY				(1 << 10)
#define RCC_CR_HSIKERON			(1 << 9)
#define RCC_CR_HSION				(1 << 8)
/** @}*/

/** @defgroup rcc_cr_msirange MSI Range
 * @ingroup STM32L4xx_rcc_defines
 * @brief Range of the MSI oscillator
Twelve frequency ranges are available: 100 kHz, 200 kHz, 400 kHz, 800 kHz,
1 MHz, 2 MHz, 4 MHz (default value), 8 MHz, 16 MHz, 24 MHz, 32 MHz and 48 MHz
@sa rcc_csr_msirange
@{*/
#define RCC_CR_MSIRANGE_SHIFT			4
#define RCC_CR_MSIRANGE_MASK			0xf
#define RCC_CR_MSIRANGE_100KHZ			0
#define RCC_CR_MSIRANGE_200KHZ			1
#define RCC_CR_MSIRANGE_400KHZ			2
#define RCC_CR_MSIRANGE_800KHZ			3
#define RCC_CR_MSIRANGE_1MHZ			4
#define RCC_CR_MSIRANGE_2MHZ			5
#define RCC_CR_MSIRANGE_4MHZ			6
#define RCC_CR_MSIRANGE_8MHZ			7
#define RCC_CR_MSIRANGE_16MHZ			8
#define RCC_CR_MSIRANGE_24MHZ			9
#define RCC_CR_MSIRANGE_32MHZ			10
#define RCC_CR_MSIRANGE_48MHZ			11
/**@}*/
#define RCC_CR_MSIRGSEL				(1 << 3)
#define RCC_CR_MSIPLLEN				(1 << 2)
#define RCC_CR_MSIRDY				(1 << 1)
#define RCC_CR_MSION				(1 << 0)


/* --- RCC_ICSCR values ---------------------------------------------------- */

#define RCC_ICSCR_HSITRIM_SHIFT		24
#define RCC_ICSCR_HSITRIM_MASK		0x7f
#define RCC_ICSCR_HSICAL_SHIFT		16
#define RCC_ICSCR_HSICAL_MASK		0xff

#define RCC_ICSCR_MSITRIM_SHIFT		8
#define RCC_ICSCR_MSITRIM_MASK		0xff
#define RCC_ICSCR_MSICAL_SHIFT		0
#define RCC_ICSCR_MSICAL_MASK		0xff

/* --- RCC_CFGR values ----------------------------------------------------- */

/* MCOPRE */
#define RCC_CFGR_MCOPRE_DIV1	    0
#define RCC_CFGR_MCOPRE_DIV2	    1
#define RCC_CFGR_MCOPRE_DIV4	    2
#define RCC_CFGR_MCOPRE_DIV8	    3
#define RCC_CFGR_MCOPRE_DIV16	    4
#define RCC_CFGR_MCOPRE_SHIFT	    28
#define RCC_CFGR_MCOPRE_MASK	    0x7

/* MCO: Microcontroller clock output */
#define RCC_CFGR_MCO_NOCLK			0x0
#define RCC_CFGR_MCO_SYSCLK			0x1
#define RCC_CFGR_MCO_MSI			0x2
#define RCC_CFGR_MCO_HSI16			0x3
#define RCC_CFGR_MCO_HSE			0x4
#define RCC_CFGR_MCO_PLL			0x5
#define RCC_CFGR_MCO_LSI			0x6
#define RCC_CFGR_MCO_LSE			0x8
#define RCC_CFGR_MCO_PLLP 		0xD
#define RCC_CFGR_MCO_PLLQ     0xE

#define RCC_CFGR_MCO_SHIFT		24
#define RCC_CFGR_MCO_MASK			0xf

/* Wakeup from stop clock selection */
#define RCC_CFGR_STOPWUCK_MSI		(0 << 15)
#define RCC_CFGR_STOPWUCK_HSI16		(1 << 15)

#define RCC_CFGR_PPRE1_SHIFT			8
#define RCC_CFGR_PPRE1_MASK			0x7
#define RCC_CFGR_PPRE2_SHIFT			11
#define RCC_CFGR_PPRE2_MASK			0x7
/** @defgroup rcc_cfgr_apbxpre RCC_CFGR APBx prescale factors
 * These can be used for both APB1 and APB2 prescaling
 * @{
 */
#define RCC_CFGR_PPRE_NODIV			0x0
#define RCC_CFGR_PPRE_DIV2			0x4
#define RCC_CFGR_PPRE_DIV4			0x5
#define RCC_CFGR_PPRE_DIV8			0x6
#define RCC_CFGR_PPRE_DIV16			0x7
/**@}*/

#define RCC_CFGR_HPRE_SHIFT			4
#define RCC_CFGR_HPRE_MASK			0xf
/** @defgroup rcc_cfgr_ahbpre RCC_CFGR AHB prescale factors
@{*/
#define RCC_CFGR_HPRE_NODIV			0x0
#define RCC_CFGR_HPRE_DIV3      0x1
#define RCC_CFGR_HPRE_DIV5      0x2
#define RCC_CFGR_HPRE_DIV6      0x5
#define RCC_CFGR_HPRE_DIV10     0x6
#define RCC_CFGR_HPRE_DIV32     0x7
#define RCC_CFGR_HPRE_DIV2			(0x8 + 0)
#define RCC_CFGR_HPRE_DIV4			(0x8 + 1)
#define RCC_CFGR_HPRE_DIV8			(0x8 + 2)
#define RCC_CFGR_HPRE_DIV16			(0x8 + 3)
#define RCC_CFGR_HPRE_DIV64			(0x8 + 4)
#define RCC_CFGR_HPRE_DIV128			(0x8 + 5)
#define RCC_CFGR_HPRE_DIV256			(0x8 + 6)
#define RCC_CFGR_HPRE_DIV512			(0x8 + 7)
/**@}*/

/* SWS: System clock switch status */
#define RCC_CFGR_SWS_MSI			0x0
#define RCC_CFGR_SWS_HSI16			0x1
#define RCC_CFGR_SWS_HSE			0x2
#define RCC_CFGR_SWS_PLL			0x3
#define RCC_CFGR_SWS_MASK			0x3
#define RCC_CFGR_SWS_SHIFT			2

/* SW: System clock switch */
#define RCC_CFGR_SW_MSI				0x0
#define RCC_CFGR_SW_HSI16			0x1
#define RCC_CFGR_SW_HSE				0x2
#define RCC_CFGR_SW_PLL				0x3
#define RCC_CFGR_SW_MASK			0x3
#define RCC_CFGR_SW_SHIFT			0

/* --- RCC_PLLCFGR - PLL Configuration Register */
#define RCC_PLLCFGR_PLLR_SHIFT		29
#define RCC_PLLCFGR_PLLR_MASK		0x7
#define RCC_PLLCFGR_PLLR_DIV2		1
#define RCC_PLLCFGR_PLLR_DIV3		2
#define RCC_PLLCFGR_PLLR_DIV4		3
#define RCC_PLLCFGR_PLLR_DIV5		4
#define RCC_PLLCFGR_PLLR_DIV6		5
#define RCC_PLLCFGR_PLLR_DIV7		6
#define RCC_PLLCFGR_PLLR_DIV8		7
#define RCC_PLLCFGR_PLLREN		(1<<28)

/* PLLQ configuration */
#define RCC_PLLCFGR_PLLQ_SHIFT		25
#define RCC_PLLCFGR_PLLQ_MASK		0x7
#define RCC_PLLCFGR_PLLQ_DIV2		1
#define RCC_PLLCFGR_PLLQ_DIV3		2
#define RCC_PLLCFGR_PLLQ_DIV4		3
#define RCC_PLLCFGR_PLLQ_DIV5		4
#define RCC_PLLCFGR_PLLQ_DIV6		5
#define RCC_PLLCFGR_PLLQ_DIV7		6
#define RCC_PLLCFGR_PLLQ_DIV8		7
#define RCC_PLLCFGR_PLLQEN		(1 << 24)

/* PLLP configuration */
#define RCC_PLLCFGR_PLLP_SHIFT		25
#define RCC_PLLCFGR_PLLP_MASK		0x1f
#define RCC_PLLCFGR_PLLP(x)		((x)-1)
#define RCC_PLLCFGR_PLLPEN		(1 << 16)

/** @defgroup rcc_pllcfgr_plln RCC_PLLCFGR PLLN values
@ingroup STM32L4xx_rcc_defines
 * Allowed values 8 <= n <= 86
@{*/
#define RCC_PLLCFGR_PLLN_SHIFT		0x8
#define RCC_PLLCFGR_PLLN_MASK		0x7f
/**@}*/

/** @defgroup rcc_pllcfgr_pllm RCC_PLLCFGR PLLM values
@ingroup STM32L4xx_rcc_defines
 * Allowed values 1 <= m <= 8
@{*/
#define RCC_PLLCFGR_PLLM_SHIFT		0x4
#define RCC_PLLCFGR_PLLM_MASK		0x7
#define RCC_PLLCFGR_PLLM(x)		((x)-1)
/**@}*/

#define RCC_PLLCFGR_PLLSRC_SHIFT	0
#define RCC_PLLCFGR_PLLSRC_MASK		0x3
#define RCC_PLLCFGR_PLLSRC_NONE		0
#define RCC_PLLCFGR_PLLSRC_MSI		1
#define RCC_PLLCFGR_PLLSRC_HSI16	2
#define RCC_PLLCFGR_PLLSRC_HSE		3


/* --- RCC_CIER - Clock interrupt enable register -------------------------- */

#define RCC_CIER_LSE_CSSIE			(1 << 9)
/* OSC ready interrupt enable bits */
#define RCC_CIER_PLLRDYIE			(1 << 5)
#define RCC_CIER_HSERDYIE			(1 << 4)
#define RCC_CIER_HSIRDYIE			(1 << 3)
#define RCC_CIER_MSIRDYIE			(1 << 2)
#define RCC_CIER_LSERDYIE			(1 << 1)
#define RCC_CIER_LSIRDYIE			(1 << 0)

/* --- RCC_CIFR - Clock interrupt flag register */

#define RCC_CIFR_LSECSSF			(1 << 9)
#define RCC_CIFR_CSSF				(1 << 8)
#define RCC_CIFR_PLLRDYF			(1 << 5)
#define RCC_CIFR_HSERDYF			(1 << 4)
#define RCC_CIFR_HSIRDYF			(1 << 3)
#define RCC_CIFR_MSIRDYF			(1 << 2)
#define RCC_CIFR_LSERDYF			(1 << 1)
#define RCC_CIFR_LSIRDYF			(1 << 0)

/* --- RCC_CICR - Clock interrupt clear register */

#define RCC_CICR_LSECSSC			(1 << 9)
#define RCC_CICR_CSSC				  (1 << 8)
#define RCC_CICR_PLLRDYC			(1 << 5)
#define RCC_CICR_HSERDYC			(1 << 4)
#define RCC_CICR_HSIRDYC			(1 << 3)
#define RCC_CICR_MSIRDYC			(1 << 2)
#define RCC_CICR_LSERDYC			(1 << 1)
#define RCC_CICR_LSIRDYC			(1 << 0)

/** @defgroup rcc_ahbrstr_rst RCC_AHBxRSTR reset values (full set)
@{*/
/** @defgroup rcc_ahb1rstr_rst RCC_AHB1RSTR reset values
@{*/
#define RCC_AHB1RSTR_CRCRST			  (1 << 12)
#define RCC_AHB1RSTR_DMAMUX1RST   (1 << 2)
#define RCC_AHB1RSTR_DMA2RST			(1 << 1)
#define RCC_AHB1RSTR_DMA1RST			(1 << 0)
/**@}*/

/** @defgroup rcc_ahb2rstr_rst RCC_AHB2RSTR reset values
@{*/
#define RCC_AHB2RSTR_GPIOHRST			(1 << 7)
#define RCC_AHB2RSTR_GPIOCRST			(1 << 2)
#define RCC_AHB2RSTR_GPIOBRST			(1 << 1)
#define RCC_AHB2RSTR_GPIOARST			(1 << 0)

/**@}*/

/** @defgroup rcc_ahb3rstr_rst RCC_AHB3RSTR reset values
@{*/
#define RCC_AHB3RSTR_FLASHRST     (1 << 25)
#define RCC_AHB3RSTR_HSEMRST      (1 << 19)
#define RCC_AHB3RSTR_RNGRST       (1 << 18)
#define RCC_AHB3RSTR_AESRST       (1 << 17)
#define RCC_AHB3RSTR_PKARST       (1 << 16)


/**@}*/
/**@}*/

/** @defgroup rcc_apb1rstr_rst RCC_APB1RSTRx reset values (full set)
@{*/
/** @defgroup rcc_apb1rstr1_rst RCC_APB1RSTR1 reset values
@{*/
#define RCC_APB1RSTR1_LPTIM1RST			(1 << 31)
#define RCC_APB1RSTR1_DAC1RST			(1 << 29)
#define RCC_APB1RSTR1_I2C3RST			(1 << 23)
#define RCC_APB1RSTR1_I2C2RST			(1 << 22)
#define RCC_APB1RSTR1_I2C1RST			(1 << 21)
#define RCC_APB1RSTR1_USART2RST			(1 << 17)
#define RCC_APB1RSTR1_SPI2S2RST		(1 << 14)
#define RCC_APB1RSTR1_TIM2RST			(1 << 0)
/**@}*/

/** @defgroup rcc_apb1rstr2_rst RCC_APB1RSTR2 reset values
@{*/
#define RCC_APB1RSTR2_LPTIM3RST			(1 << 6)
#define RCC_APB1RSTR2_LPTIM2RST			(1 << 5)
#define RCC_APB1RSTR2_LPUART1RST		(1 << 0)
/**@}*/
/**@}*/

/** @defgroup rcc_apb2rstr_rst RCC_APB2RSTR reset values
@{*/
#define RCC_APB2RSTR_DFSDMRST			(1 << 24)
#define RCC_APB2RSTR_SAI2RST			(1 << 22)
#define RCC_APB2RSTR_SAI1RST			(1 << 21)
#define RCC_APB2RSTR_TIM17RST			(1 << 18)
#define RCC_APB2RSTR_TIM16RST			(1 << 17)
#define RCC_APB2RSTR_USART1RST		(1 << 14)
#define RCC_APB2RSTR_SPI1RST			(1 << 12)
#define RCC_APB2RSTR_TIM1RST			(1 << 11)
#define RCC_APB2RSTR_ADCRST 			(1 << 9)
/**@}*/

/** @defgroup rcc_apb3rstr_rst RCC_APB3RSTR reset values
@{*/
#define RCC_APB3RSTR_SUBGHZSPIRST	(1 << 0)
/**@}*/

/* --- RCC_AHB1ENR values --------------------------------------------------- */

/** @defgroup rcc_ahbenr_en RCC_AHBxENR enable values (full set)
 *@{*/
/** @defgroup rcc_ahb1enr_en RCC_AHB1ENR enable values
@ingroup STM32L4xx_rcc_defines

@{*/
#define RCC_AHB1ENR_CRCEN			  (1 << 12)
#define RCC_AHB1ENR_DMAMUX1EN		(1 << 2)
#define RCC_AHB1ENR_DMA2EN			(1 << 1)
#define RCC_AHB1ENR_DMA1EN			(1 << 0)
/**@}*/

/* --- RCC_AHB2ENR values --------------------------------------------------- */

/** @defgroup rcc_ahb2enr_en RCC_AHB2ENR enable values
@ingroup STM32L4xx_rcc_defines

@{*/
#define RCC_AHB2ENR_GPIOHEN			(1 << 7)
#define RCC_AHB2ENR_GPIOCEN			(1 << 2)
#define RCC_AHB2ENR_GPIOBEN			(1 << 1)
#define RCC_AHB2ENR_GPIOAEN			(1 << 0)
/**@}*/

/* --- RCC_AHB3ENR values --------------------------------------------------- */

/** @defgroup rcc_ahb3enr_en RCC_AHB3ENR enable values
@ingroup STM32L4xx_rcc_defines

@{*/
#define RCC_AHB3ENR_FLASHEN		(1 << 25)
#define RCC_AHB3ENR_HSEMEN		(1 << 19)
#define RCC_AHB3ENR_RNGEN 		(1 << 18)
#define RCC_AHB3ENR_AESEN 		(1 << 17)
#define RCC_AHB3ENR_PKAEN			(1 << 16)
/**@}*/

/**@}*/

/* --- RCC_APB1ENR1 values -------------------------------------------------- */

/** @defgroup rcc_apb1enr_en RCC_APB1ENRx enable values (full set)
 *@{*/
/** @defgroup rcc_apb1enr1_en RCC_APB1ENR1 enable values
@ingroup STM32L4xx_rcc_defines

@{*/
#define RCC_APB1ENR1_LPTIM1EN			(1 << 31)
#define RCC_APB1ENR1_DAC1EN			(1 << 29)
#define RCC_APB1ENR1_I2C3EN			(1 << 23)
#define RCC_APB1ENR1_I2C2EN			(1 << 22)
#define RCC_APB1ENR1_I2C1EN			(1 << 21)
#define RCC_APB1ENR1_USART2EN			(1 << 17)
#define RCC_APB1ENR1_SPI2EN			(1 << 14)
#define RCC_APB1ENR1_WWDGEN			(1 << 11)
#define RCC_APB1ENR1_RTCAPBENEN	(1 << 10)
#define RCC_APB1ENR1_TIM2EN			(1 << 0)
/**@}*/

/* --- RCC_APB1ENR2 values -------------------------------------------------- */

/** @defgroup rcc_apb1enr2_en RCC_APB1ENR2 enable values
@ingroup STM32L4xx_rcc_defines

@{*/
#define RCC_APB1ENR2_LPTIM3EN			(1 << 6)
#define RCC_APB1ENR2_LPTIM2EN			(1 << 5)
#define RCC_APB1ENR2_LPUART1EN			(1 << 0)
/**@}*/
/**@}*/

/* --- RCC_APB2ENR values -------------------------------------------------- */

/** @defgroup rcc_apb2enr_en RCC_APB2ENR enable values
@ingroup STM32WLxx_rcc_defines

@{*/
#define RCC_APB2ENR_TIM17EN			(1 << 18)
#define RCC_APB2ENR_TIM16EN			(1 << 17)
#define RCC_APB2ENR_USART1EN		(1 << 14)
#define RCC_APB2ENR_SPI1EN			(1 << 12)
#define RCC_APB2ENR_TIM1EN			(1 << 11)
#define RCC_APB2ENR_ADCEN   		(1 << 9)
/**@}*/

/* --- RCC_APB3ENR values -------------------------------------------------- */

/** @defgroup rcc_apb2enr_en RCC_APB3ENR enable values
@ingroup STM32WLxx_rcc_defines

@{*/
#define RCC_APB3ENR_SUBGHZSPIEN   		(1 << 0)
/**@}*/

/* --- RCC_AHB1SMENR - AHB1 periph clock in sleep mode --------------------- */

#define RCC_AHB1SMENR_CRCSMEN			  (1 << 12)
#define RCC_AHB1SMENR_DMAMUX1SMEN		(1 << 1)
#define RCC_AHB1SMENR_DMA2SMEN			(1 << 1)
#define RCC_AHB1SMENR_DMA1SMEN			(1 << 0)

/* --- RCC_AHB2SMENR - AHB2 periph clock in sleep mode --------------------- */

#define RCC_AHB2SMENR_GPIOHSMEN			(1 << 7)
#define RCC_AHB2SMENR_GPIOCSMEN			(1 << 2)
#define RCC_AHB2SMENR_GPIOBSMEN			(1 << 1)
#define RCC_AHB2SMENR_GPIOASMEN			(1 << 0)

/* --- RCC_AHB3SMENR - AHB3 periph clock in sleep mode --------------------- */
#define RCC_AHB3SMENR_FLASHSMEN		(1 << 25)
#define RCC_AHB3SMENR_SRAM2SMEN		(1 << 24)
#define RCC_AHB3SMENR_SRAM1SMEN		(1 << 23)
#define RCC_AHB3SMENR_RNGSMEN			(1 << 18)
#define RCC_AHB3SMENR_AESSMEN			(1 << 17)
#define RCC_AHB3SMENR_PKASMEN			(1 << 16)

/* --- RCC_APB1SMENR1 - APB1 periph clock in sleep mode -------------------- */

#define RCC_APB1SMENR1_LPTIM1SMEN		(1 << 31)
#define RCC_APB1SMENR1_DAC1SMEN			(1 << 29)
#define RCC_APB1SMENR1_I2C3SMEN			(1 << 23)
#define RCC_APB1SMENR1_I2C2SMEN			(1 << 22)
#define RCC_APB1SMENR1_I2C1SMEN			(1 << 21)
#define RCC_APB1SMENR1_USART2SMEN		(1 << 17)
#define RCC_APB1SMENR1_SPI2S2MEN		(1 << 14)
#define RCC_APB1SMENR1_WWDGSMEN			(1 << 11)
#define RCC_APB1SMENR1_RTCAPBSMEN		(1 << 10)
#define RCC_APB1SMENR1_TIM2SMEN			(1 << 0)

/* --- RCC_APB1SMENR2 - APB1 periph clock in sleep mode -------------------- */

#define RCC_APB1SMENR2_LPTIM3SMEN		(1 << 6)
#define RCC_APB1SMENR2_LPTIM2SMEN		(1 << 5)
#define RCC_APB1SMENR2_LPUART1SMEN		(1 << 0)

/* --- RCC_APB2SMENR - APB2 periph clock in sleep mode --------------------- */

#define RCC_APB2SMENR_TIM17SMEN			(1 << 18)
#define RCC_APB2SMENR_TIM16SMEN			(1 << 17)
#define RCC_APB2SMENR_USART1SMEN		(1 << 14)
#define RCC_APB2SMENR_SPI1SMEN			(1 << 12)
#define RCC_APB2SMENR_TIM1SMEN			(1 << 11)
#define RCC_APB2SMENR_ADCSMEN		    (1 << 9)

/* --- RCC_APB3SMENR - APB3 periph clock in sleep mode --------------------- */

#define RCC_APB3SMENR_SUBGHZSPISMEN		(1 << 0)

/* --- RCC_CCIPR - Peripherals independent clock config register ----------- */

#define RCC_CCIPR_RNGSEL_PLLQ   0
#define RCC_CCIPR_RNGSEL_LSI    1
#define RCC_CCIPR_RNGSEL_LSE    2
#define RCC_CCIPR_RNGSEL_MSI    3
#define RCC_CCIPR_RNGSEL_MASK   0x3
#define RCC_CCIPR_RNGSEL_SHIFT  30

#define RCC_CCIPR_ADCSEL_NONE		  0
#define RCC_CCIPR_ADCSEL_HSI16  	1
#define RCC_CCIPR_ADCSEL_PLLP   	2
#define RCC_CCIPR_ADCSEL_SYS		  3
#define RCC_CCIPR_ADCSEL_MASK		  0x3
#define RCC_CCIPR_ADCSEL_SHIFT		28

#define RCC_CCIPR_LPTIMxSEL_APB		0
#define RCC_CCIPR_LPTIMxSEL_LSI		1
#define RCC_CCIPR_LPTIMxSEL_HSI16	2
#define RCC_CCIPR_LPTIMxSEL_LSE		3
#define RCC_CCIPR_LPTIMxSEL_MASK	0x3
#define RCC_CCIPR_LPTIM3SEL_SHIFT	22
#define RCC_CCIPR_LPTIM2SEL_SHIFT	20
#define RCC_CCIPR_LPTIM1SEL_SHIFT	18

#define RCC_CCIPR_I2CxSEL_APB		0
#define RCC_CCIPR_I2CxSEL_SYS		1
#define RCC_CCIPR_I2CxSEL_HSI16		2
#define RCC_CCIPR_I2CxSEL_MASK		0x3
#define RCC_CCIPR_I2C3SEL_SHIFT		16
#define RCC_CCIPR_I2C2SEL_SHIFT		14
#define RCC_CCIPR_I2C1SEL_SHIFT		12

#define RCC_CCIPR_LPUART1SEL_APB	0
#define RCC_CCIPR_LPUART1SEL_SYS	1
#define RCC_CCIPR_LPUART1SEL_HSI16	2
#define RCC_CCIPR_LPUART1SEL_LSE	3
#define RCC_CCIPR_LPUART1SEL_MASK	0x3
#define RCC_CCIPR_LPUART1SEL_SHIFT	10

#define RCC_CCIPR_USARTxSEL_APB		0
#define RCC_CCIPR_USARTxSEL_SYS		1
#define RCC_CCIPR_USARTxSEL_HSI16	2
#define RCC_CCIPR_USARTxSEL_LSE		3
#define RCC_CCIPR_USARTxSEL_MASK	0x3
#define RCC_CCIPR_UARTxSEL_APB		RCC_CCIPR_USARTxSEL_APB
#define RCC_CCIPR_UARTxSEL_SYS		RCC_CCIPR_USARTxSEL_SYS
#define RCC_CCIPR_UARTxSEL_HSI16	RCC_CCIPR_USARTxSEL_HSI16
#define RCC_CCIPR_UARTxSEL_LSE		RCC_CCIPR_USARTxSEL_LSE
#define RCC_CCIPR_UARTxSEL_MASK		RCC_CCIPR_USARTxSEL_MASK
#define RCC_CCIPR_USART2SEL_SHIFT	2
#define RCC_CCIPR_USART1SEL_SHIFT	0

/* --- RCC_BDCR - Backup domain control register --------------------------- */

#define RCC_BDCR_LSCOSEL		(1 << 25)
#define RCC_BDCR_LSCOEN			(1 << 24)
#define RCC_BDCR_BDRST			(1 << 16)
#define RCC_BDCR_RTCEN			(1 << 15)
#define RCC_BDCR_LSESYSRDY	(1 << 11)

#define RCC_BDCR_RTCSEL_NONE		  0
#define RCC_BDCR_RTCSEL_LSE		    1
#define RCC_BDCR_RTCSEL_LSI		    2
#define RCC_BDCR_RTCSEL_HSEDIV32	3
#define RCC_BDCR_RTCSEL_SHIFT		  8
#define RCC_BDCR_RTCSEL_MASK		  0x3

#define RCC_BDCR_LSESYSEN			(1 << 7)
#define RCC_BDCR_LSECSSD			(1 << 6)
#define RCC_BDCR_LSECSSON			(1 << 5)

#define RCC_BDCR_LSEDRV_LOW		  0
#define RCC_BDCR_LSEDRV_MEDLOW	1
#define RCC_BDCR_LSEDRV_MEDHIGH	2
#define RCC_BDCR_LSEDRV_HIGH		3
#define RCC_BDCR_LSEDRV_SHIFT		3
#define RCC_BDCR_LSEDRV_MASK		0x3

#define RCC_BDCR_LSEBYP				(1 << 2)
#define RCC_BDCR_LSERDY				(1 << 1)
#define RCC_BDCR_LSEON				(1 << 0)

/* --- RCC_CSR - Control/Status register ----------------------------------- */

#define RCC_CSR_LPWRRSTF			(1 << 31)
#define RCC_CSR_WWDGRSTF			(1 << 30)
#define RCC_CSR_IWDGRSTF			(1 << 29)
#define RCC_CSR_SFTRSTF				(1 << 28)
#define RCC_CSR_BORRSTF				(1 << 27)
#define RCC_CSR_PINRSTF				(1 << 26)
#define RCC_CSR_OBLRSTF				(1 << 25)
#define RCC_CSR_RFILARSTF			(1 << 24)
#define RCC_CSR_RMVF	  			(1 << 23)
#define RCC_CSR_RFRST   			(1 << 15)
#define RCC_CSR_RFRSTF   			(1 << 14)

#define RCC_CSR_RESET_FLAGS	(RCC_CSR_LPWRRSTF | RCC_CSR_WWDGRSTF |\
		RCC_CSR_IWDGRSTF | RCC_CSR_SFTRSTF | RCC_CSR_BORRSTF |\
		RCC_CSR_PINRSTF | RCC_CSR_OBLRSTF | RCC_CSR_RFILARSTF | RCC_CSR_RFRSTF)

/** @defgroup rcc_csr_msirange MSI Range after standby values
@brief Range of the MSI oscillator after returning from standby
@ingroup STM32L4xx_rcc_defines
@sa rcc_cr_msirange
@{*/
#define RCC_CSR_MSIRANGE_MASK			0xf
#define RCC_CSR_MSIRANGE_SHIFT		8
#define RCC_CSR_MSIRANGE_1MHZ			4
#define RCC_CSR_MSIRANGE_2MHZ			5
#define RCC_CSR_MSIRANGE_4MHZ			6
#define RCC_CSR_MSIRANGE_8MHZ			7
/**@}*/

#define RCC_CSR_LSIPRE   		(1 << 4)
#define RCC_CSR_LSIRDY			(1 << 1)
#define RCC_CSR_LSION				(1 << 0)

/* --- RCC_EXTCFGR - Control/Status register ----------------------------------- */

#define RCC_EXTCFGR_SHDHPREF  (1 << 16)

#define RCC_EXTCFGR_SHDHPRE_SHIFT   0
#define RCC_EXTCFGR_SHDHPRE_MASK    0xf
#define RCC_EXTCFGR_SHDHPRE_DIV3    1
#define RCC_EXTCFGR_SHDHPRE_DIV5    2
#define RCC_EXTCFGR_SHDHPRE_DIV6    5
#define RCC_EXTCFGR_SHDHPRE_DIV10   6
#define RCC_EXTCFGR_SHDHPRE_DIV32   7
#define RCC_EXTCFGR_SHDHPRE_DIV2    (0x8 + 0)
#define RCC_EXTCFGR_SHDHPRE_DIV4    (0x8 + 1)
#define RCC_EXTCFGR_SHDHPRE_DIV8    (0x8 + 2)
#define RCC_EXTCFGR_SHDHPRE_DIV16   (0x8 + 3)
#define RCC_EXTCFGR_SHDHPRE_DIV64   (0x8 + 4)
#define RCC_EXTCFGR_SHDHPRE_DIV128  (0x8 + 5)
#define RCC_EXTCFGR_SHDHPRE_DIV256  (0x8 + 6)
#define RCC_EXTCFGR_SHDHPRE_DIV512  (0x8 + 7)


struct rcc_clock_scale {
	uint8_t pllm;
	uint16_t plln;
	uint8_t pllp;
	uint8_t pllq;
	uint8_t pllr;
	uint8_t pll_source;
	uint32_t flash_config;
	uint8_t hpre;
	uint8_t ppre1;
	uint8_t ppre2;
	enum pwr_vos_scale voltage_scale;
	uint32_t ahb_frequency;
	uint32_t apb1_frequency;
	uint32_t apb2_frequency;
};

enum rcc_clock_config_entry {
	RCC_CLOCK_VRANGE1_80MHZ,
	RCC_CLOCK_CONFIG_END
};

extern const struct rcc_clock_scale rcc_hsi16_configs[RCC_CLOCK_CONFIG_END];

/* --- Variable definitions ------------------------------------------------ */

extern uint32_t rcc_ahb_frequency;
extern uint32_t rcc_apb1_frequency;
extern uint32_t rcc_apb2_frequency;

/* --- Function prototypes ------------------------------------------------- */

enum rcc_osc {
	RCC_PLL, RCC_HSE, RCC_HSI16, RCC_MSI, RCC_LSE, RCC_LSI,
};


#define _REG_BIT(base, bit)		(((base) << 5) + (bit))

enum rcc_periph_clken {

	/* AHB1 peripherals */
	RCC_CRC = _REG_BIT(RCC_AHB1ENR_OFFSET, 12),
	RCC_DMAMUX1 = _REG_BIT(RCC_AHB1ENR_OFFSET, 2),
	RCC_DMA2 = _REG_BIT(RCC_AHB1ENR_OFFSET, 1),
	RCC_DMA1 = _REG_BIT(RCC_AHB1ENR_OFFSET, 0),

	/* AHB2 peripherals */
	RCC_GPIOH = _REG_BIT(RCC_AHB2ENR_OFFSET, 7),
	RCC_GPIOC = _REG_BIT(RCC_AHB2ENR_OFFSET, 2),
	RCC_GPIOB = _REG_BIT(RCC_AHB2ENR_OFFSET, 1),
	RCC_GPIOA = _REG_BIT(RCC_AHB2ENR_OFFSET, 0),

	/* AHB3 peripherals */
  RCC_FLASH = _REG_BIT(RCC_AHB3ENR_OFFSET, 25),
  RCC_HSEM = _REG_BIT(RCC_AHB3ENR_OFFSET, 19),
  RCC_RNG = _REG_BIT(RCC_AHB3ENR_OFFSET, 18),
	RCC_AES = _REG_BIT(RCC_AHB3ENR_OFFSET, 17),
	RCC_PKA = _REG_BIT(RCC_AHB3ENR_OFFSET, 16),

	/* APB1 peripherals */
	RCC_LPTIM1 = _REG_BIT(RCC_APB1ENR1_OFFSET, 31),
	RCC_DAC1 = _REG_BIT(RCC_APB1ENR1_OFFSET, 29),
	RCC_I2C3 = _REG_BIT(RCC_APB1ENR1_OFFSET, 23),
	RCC_I2C2 = _REG_BIT(RCC_APB1ENR1_OFFSET, 22),
	RCC_I2C1 = _REG_BIT(RCC_APB1ENR1_OFFSET, 21),
	RCC_USART2 = _REG_BIT(RCC_APB1ENR1_OFFSET, 17),
	RCC_SPI2 = _REG_BIT(RCC_APB1ENR1_OFFSET, 14),
	RCC_TIM2 = _REG_BIT(RCC_APB1ENR1_OFFSET, 0),
	/* apb1-2 */
	RCC_LPTIM3 = _REG_BIT(RCC_APB1ENR2_OFFSET, 6),
	RCC_LPTIM2 = _REG_BIT(RCC_APB1ENR2_OFFSET, 5),
	RCC_LPUART1 = _REG_BIT(RCC_APB1ENR2_OFFSET, 0),

	/* APB2 peripherals */
	RCC_TIM17 = _REG_BIT(RCC_APB2ENR_OFFSET, 18),
	RCC_TIM16 = _REG_BIT(RCC_APB2ENR_OFFSET, 17),
	RCC_USART1 = _REG_BIT(RCC_APB2ENR_OFFSET, 14),
	RCC_SPI1 = _REG_BIT(RCC_APB2ENR_OFFSET, 12),
	RCC_TIM1 = _REG_BIT(RCC_APB2ENR_OFFSET, 11),
	RCC_ADC = _REG_BIT(RCC_APB2ENR_OFFSET, 9),
	
  /* APB3 peripherals */
  RCC_SUBGHZ = _REG_BIT(RCC_APB3ENR_OFFSET, 0),

	/* AHB1 peripherals in sleep mode */
	SCC_CRC = _REG_BIT(RCC_AHB1SMENR_OFFSET, 12),
	SCC_DMAMUX1 = _REG_BIT(RCC_AHB1SMENR_OFFSET, 2),
	SCC_DMA2 = _REG_BIT(RCC_AHB1SMENR_OFFSET, 1),
	SCC_DMA1 = _REG_BIT(RCC_AHB1SMENR_OFFSET, 0),

	/* AHB2 peripherals in sleep mode */
	SCC_GPIOH = _REG_BIT(RCC_AHB2SMENR_OFFSET, 7),
	SCC_GPIOC = _REG_BIT(RCC_AHB2SMENR_OFFSET, 2),
	SCC_GPIOB = _REG_BIT(RCC_AHB2SMENR_OFFSET, 1),
	SCC_GPIOA = _REG_BIT(RCC_AHB2SMENR_OFFSET, 0),

	/* AHB3 peripherals in sleep mode */
  SCC_FLASH = _REG_BIT(RCC_AHB3SMENR_OFFSET, 25),
  SCC_HSEM = _REG_BIT(RCC_AHB3SMENR_OFFSET, 19),
  SCC_RNG = _REG_BIT(RCC_AHB3SMENR_OFFSET, 18),
  SCC_AES = _REG_BIT(RCC_AHB3SMENR_OFFSET, 17),
	SCC_PKA = _REG_BIT(RCC_AHB3SMENR_OFFSET, 16),

	/* APB1 peripherals in sleep mode */
	SCC_LPTIM1 = _REG_BIT(RCC_APB1ENR1_OFFSET, 31),
	SCC_DAC1 = _REG_BIT(RCC_APB1ENR1_OFFSET, 29),
	SCC_I2C3 = _REG_BIT(RCC_APB1ENR1_OFFSET, 23),
	SCC_I2C2 = _REG_BIT(RCC_APB1ENR1_OFFSET, 22),
	SCC_I2C1 = _REG_BIT(RCC_APB1ENR1_OFFSET, 21),
	SCC_USART2 = _REG_BIT(RCC_APB1ENR1_OFFSET, 17),
	SCC_SPI2 = _REG_BIT(RCC_APB1ENR1_OFFSET, 14),
	SCC_WWDG = _REG_BIT(RCC_APB1ENR1_OFFSET, 11),
	SCC_TIM2 = _REG_BIT(RCC_APB1ENR1_OFFSET, 0),
	/* apb1-2 */
  SCC_LPTIM3 = _REG_BIT(RCC_APB1ENR2_OFFSET, 6),
	SCC_LPTIM2 = _REG_BIT(RCC_APB1ENR2_OFFSET, 5),
	SCC_LPUART1 = _REG_BIT(RCC_APB1ENR2_OFFSET, 0),

	/* APB2 peripherals in sleep mode */
	SCC_TIM17 = _REG_BIT(RCC_APB2ENR_OFFSET, 18),
	SCC_TIM16 = _REG_BIT(RCC_APB2ENR_OFFSET, 17),
	SCC_USART1 = _REG_BIT(RCC_APB2ENR_OFFSET, 14),
	SCC_SPI1 = _REG_BIT(RCC_APB2ENR_OFFSET, 12),
	SCC_TIM1 = _REG_BIT(RCC_APB2ENR_OFFSET, 11),
  SCC_ADC1 = _REG_BIT(RCC_APB2ENR_OFFSET, 9),

  /* APB3 peripherals in sleep mode */
  SCC_SUBGHZ = _REG_BIT(RCC_APB3ENR_OFFSET, 0),
};


enum rcc_periph_rst {

	/* AHB1 peripherals */
	RST_CRC = _REG_BIT(RCC_AHB1RSTR_OFFSET, 12),
	RST_DMAMUX1 = _REG_BIT(RCC_AHB1RSTR_OFFSET, 2),
	RST_DMA2 = _REG_BIT(RCC_AHB1RSTR_OFFSET, 1),
	RST_DMA1 = _REG_BIT(RCC_AHB1RSTR_OFFSET, 0),

	/* AHB2 peripherals */
	RST_GPIOH = _REG_BIT(RCC_AHB2RSTR_OFFSET, 7),
	RST_GPIOC = _REG_BIT(RCC_AHB2RSTR_OFFSET, 2),
	RST_GPIOB = _REG_BIT(RCC_AHB2RSTR_OFFSET, 1),
	RST_GPIOA = _REG_BIT(RCC_AHB2RSTR_OFFSET, 0),

	/* AHB3 peripherals */
  RST_FLASH = _REG_BIT(RCC_AHB3RSTR_OFFSET, 25),
  RST_HSEM = _REG_BIT(RCC_AHB3RSTR_OFFSET, 19),
  RST_RNG = _REG_BIT(RCC_AHB3RSTR_OFFSET, 18),
	RST_AES = _REG_BIT(RCC_AHB3RSTR_OFFSET, 17),
	RST_PKA = _REG_BIT(RCC_AHB3RSTR_OFFSET, 16),

	/* APB1 peripherals */
	RST_LPTIM1 = _REG_BIT(RCC_APB1RSTR1_OFFSET, 31),
	RST_DAC1 = _REG_BIT(RCC_APB1RSTR1_OFFSET, 29),
	RST_I2C3 = _REG_BIT(RCC_APB1RSTR1_OFFSET, 23),
	RST_I2C2 = _REG_BIT(RCC_APB1RSTR1_OFFSET, 22),
	RST_I2C1 = _REG_BIT(RCC_APB1RSTR1_OFFSET, 21),
	RST_USART2 = _REG_BIT(RCC_APB1RSTR1_OFFSET, 17),
	RST_SPI2 = _REG_BIT(RCC_APB1RSTR1_OFFSET, 14),
	RST_TIM2 = _REG_BIT(RCC_APB1RSTR1_OFFSET, 0),
	/* apb1-2 */
	RST_LPTIM3 = _REG_BIT(RCC_APB1RSTR2_OFFSET, 6),
	RST_LPTIM2 = _REG_BIT(RCC_APB1RSTR2_OFFSET, 5),
	RST_LPUART1 = _REG_BIT(RCC_APB1RSTR2_OFFSET, 0),

	/* APB2 peripherals */
	RST_TIM17 = _REG_BIT(RCC_APB2RSTR_OFFSET, 18),
	RST_TIM16 = _REG_BIT(RCC_APB2RSTR_OFFSET, 17),
	RST_USART1 = _REG_BIT(RCC_APB2RSTR_OFFSET, 14),
	RST_SPI1 = _REG_BIT(RCC_APB2RSTR_OFFSET, 12),
	RST_TIM1 = _REG_BIT(RCC_APB2RSTR_OFFSET, 11),
	RST_ADC = _REG_BIT(RCC_APB2RSTR_OFFSET, 9),
	
  /* APB3 peripherals */
  RST_SUBGHZ = _REG_BIT(RCC_APB3RSTR_OFFSET, 0),
};
#include <libopencm3/stm32/common/rcc_common_all.h>

BEGIN_DECLS

void rcc_osc_ready_int_clear(enum rcc_osc osc);
void rcc_osc_ready_int_enable(enum rcc_osc osc);
void rcc_osc_ready_int_disable(enum rcc_osc osc);
int rcc_osc_ready_int_flag(enum rcc_osc osc);
void rcc_css_int_clear(void);
int rcc_css_int_flag(void);
void rcc_wait_for_sysclk_status(enum rcc_osc osc);
void rcc_osc_on(enum rcc_osc osc);
void rcc_osc_off(enum rcc_osc osc);
void rcc_css_enable(void);
void rcc_css_disable(void);
void rcc_set_sysclk_source(uint32_t clk);
void rcc_set_pll_source(uint32_t pllsrc);
void rcc_set_ppre2(uint32_t ppre2);
void rcc_set_ppre1(uint32_t ppre1);
void rcc_set_hpre(uint32_t hpre);
void rcc_set_main_pll(uint32_t source, uint32_t pllm, uint32_t plln, uint32_t pllp, uint32_t pllq, uint32_t pllr);
uint32_t rcc_system_clock_source(void);
void rcc_clock_setup_pll(const struct rcc_clock_scale *clock);
void rcc_set_msi_range(uint32_t msi_range);
void rcc_set_msi_range_standby(uint32_t msi_range);
void rcc_pll_output_enable(uint32_t pllout);
void rcc_enable_rtc_clock(void);
void rcc_disable_rtc_clock(void);
void rcc_set_rtc_clock_source(enum rcc_osc clk);
uint32_t rcc_get_usart_clk_freq(uint32_t usart);
uint32_t rcc_get_timer_clk_freq(uint32_t timer);
uint32_t rcc_get_i2c_clk_freq(uint32_t i2c);
uint32_t rcc_get_spi_clk_freq(uint32_t spi);

void rcc_subghz_reset_release(void);
int rcc_subghz_running(void);

END_DECLS

/**@}*/

#endif
