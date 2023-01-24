/** @addtogroup exti_defines
 *
 * @author @htmlonly &copy; @endhtmlonly 2019 Guillaume Revaillot <g.revaillot@gmail.com>
 *
 */
/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2019 Guillaume Revaillot <g.revaillot@gmail.com>
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

/**@{*/

/** @cond */
#if defined(LIBOPENCM3_EXTI_H)
/** @endcond */
#ifndef LIBOPENCM3_EXTI_COMMON_WL_H
#define LIBOPENCM3_EXTI_COMMON_WL_H

/* --- EXTI registers ------------------------------------------------------ */

/** @defgroup exti_registers EXTI Registers
@{*/
/** EXTI Rising Trigger Selection Register 1 */
#define EXTI_RTSR1			MMIO32(EXTI_BASE + 0x00)
#define EXTI_RTSR			EXTI_RTSR1

/** EXTI Falling Trigger Selection Register 1 */
#define EXTI_FTSR1			MMIO32(EXTI_BASE + 0x04)
#define EXTI_FTSR			EXTI_FTSR1

/** EXTI Software Interrupt Event Register 1 */
#define EXTI_SWIER1			MMIO32(EXTI_BASE + 0x08)

/** EXTI Pending Register 1 */
#define EXTI_PR1        MMIO32(EXTI_BASE + 0x0C)

/** EXTI Rising Trigger Selection Register 2 */
#define EXTI_RTSR2      MMIO32(EXTI_BASE + 0x20)

/** EXTI Falling Trigger Selection Register 2 */
#define EXTI_FTSR2			MMIO32(EXTI_BASE + 0x24)

/** EXTI Software Interrupt Event Register 2 */
#define EXTI_SWIER2			MMIO32(EXTI_BASE + 0x28)

/** EXTI Pending Register 2 */
#define EXTI_PR2        MMIO32(EXTI_BASE + 0x2C)


/** EXTI Interrupt Mask Registers 1 */
#define EXTI_IMR1			MMIO32(EXTI_BASE + 0x80)
#define EXTI_IMR			EXTI_IMR1

/** EXTI Event Mask Registers 1 */
#define EXTI_EMR1			MMIO32(EXTI_BASE + 0x84)
#define EXTI_EMR			EXTI_EMR1

/** EXTI Interrupt Mask Registers 2 */
#define EXTI_IMR2			MMIO32(EXTI_BASE + 0x90)

/**@}*/

/* EXTI number definitions */
#define EXTI0				  0
#define EXTI1				  1
#define EXTI2				  2
#define EXTI3				  3
#define EXTI4				  4
#define EXTI5				  5
#define EXTI6				  6
#define EXTI7				  7
#define EXTI8				  8
#define EXTI9				  9
#define EXTI10				10
#define EXTI11				11
#define EXTI12				12
#define EXTI13				13
#define EXTI14				14
#define EXTI15				15
#define EXTI16				16
#define EXTI17				17
#define EXTI18				18
#define EXTI19				19
#define EXTI20				20
#define EXTI21				21
#define EXTI22				22
#define EXTI23				23
#define EXTI24				24
#define EXTI25				25
#define EXTI26				26
#define EXTI27				27
#define EXTI28				28
#define EXTI29				29
#define EXTI30				30
#define EXTI31				31
#define EXTI32				32
#define EXTI33				33
#define EXTI34				34
#define EXTI38				38
#define EXTI42				42
#define EXTI43				43
#define EXTI44				44
#define EXTI45				45
#define EXTI46				46

#define IS_EXTI_EXTENDED(x) ((x >= 32))
#define EXTI_MASK(x)  (1<<x)

/* --- EXTI_EXTICR Values -------------------------------------------------*/

#define EXTI_EXTICR_FIELDSIZE		8
#define EXTI_EXTICR_GPIOA		0
#define EXTI_EXTICR_GPIOB		1
#define EXTI_EXTICR_GPIOC		2
#define EXTI_EXTICR_GPIOH		7

/* Trigger types */
enum exti_trigger_type {
	EXTI_TRIGGER_RISING,
	EXTI_TRIGGER_FALLING,
	EXTI_TRIGGER_BOTH,
};

BEGIN_DECLS

void exti_set_trigger(uint32_t extis, enum exti_trigger_type trig);
void exti_enable_request(uint32_t extis);
void exti_disable_request(uint32_t extis);
void exti_reset_request(uint32_t extis);
void exti_select_source(uint32_t exti, uint32_t gpioport);
uint32_t exti_get_flag_status(uint32_t exti);

END_DECLS
/**@}*/

#endif
/** @cond */
#else
#warning "exti_common_wl.h should not be included directly, only via exti.h"
#endif
/** @endcond */

/**@}*/
