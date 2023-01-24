/** @addtogroup exti_file EXTI peripheral API
 * @ingroup peripheral_apis
 */
/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2010 Mark Butler <mbutler@physics.otago.ac.nz>
 * Copyright (C) 2012 Karl Palsson <karlp@tweak.net.au>
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
 * This provides the code for the "next gen" EXTI block provided in F2/F4/F7/L1
 * devices.  (differences only in the source selection)
 */
/**@{*/


#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/gpio.h>

#if defined(EXTI_EXTICR)
	#define EXTICR_SELECTION_FIELDSIZE	EXTI_EXTICR_FIELDSIZE
	#define EXTICR_SELECTION_REG(x)	EXTI_EXTICR(x)
#elif defined(AFIO_EXTICR)
	#define EXTICR_SELECTION_FIELDSIZE	AFIO_EXTICR_FIELDSIZE
	#define EXTICR_SELECTION_REG(x)	AFIO_EXTICR(x)
#else
	#include <libopencm3/stm32/syscfg.h>
	#define EXTICR_SELECTION_FIELDSIZE	SYSCFG_EXTICR_FIELDSIZE
	#define EXTICR_SELECTION_REG(x)	SYSCFG_EXTICR(x)
#endif


void exti_set_trigger(uint32_t extis, enum exti_trigger_type trig)
{
	switch (trig) {
	case EXTI_TRIGGER_RISING:
    if (IS_EXTI_EXTENDED(extis))
    {
      EXTI_RTSR2 |= EXTI_MASK(extis);
      EXTI_FTSR2 &= ~EXTI_MASK(extis);
    }
    else
    {
      EXTI_RTSR1 |= EXTI_MASK(extis);
      EXTI_FTSR1 &= ~EXTI_MASK(extis);
    }
		break;

	case EXTI_TRIGGER_FALLING:
		if (IS_EXTI_EXTENDED(extis))
    {
      EXTI_RTSR2 &= ~EXTI_MASK(extis);
      EXTI_FTSR2 |= EXTI_MASK(extis);
    }
    else
    {
      EXTI_RTSR1 &= ~EXTI_MASK(extis);
      EXTI_FTSR1 |= EXTI_MASK(extis);
    }
		break;

	case EXTI_TRIGGER_BOTH:
    if (IS_EXTI_EXTENDED(extis))
    {
      EXTI_RTSR2 |= EXTI_MASK(extis);
      EXTI_FTSR2 |= EXTI_MASK(extis);
    }
    else
    {
      EXTI_RTSR1 |= EXTI_MASK(extis);
      EXTI_FTSR1 |= EXTI_MASK(extis);
    }
		break;
	}
}

void exti_enable_request(uint32_t extis)
{
  if (IS_EXTI_EXTENDED(extis))
  {
    /* Enable interrupts. */
    EXTI_IMR2 |= EXTI_MASK(extis);

    /* No events for extended EXTIs */
  }
  else
  {
    /* Enable interrupts. */
    EXTI_IMR1 |= EXTI_MASK(extis);

    /* Enable events. */
    EXTI_EMR1 |= EXTI_MASK(extis);
  }
	
}

void exti_disable_request(uint32_t extis)
{
  if (IS_EXTI_EXTENDED(extis))
  {
    /* Disable interrupts. */
    EXTI_IMR2 &= ~EXTI_MASK(extis);
  }
  else
  {
    /* Disable interrupts. */
    EXTI_IMR1 &= ~EXTI_MASK(extis);

    /* Disable events. */
    EXTI_EMR1 &= ~EXTI_MASK(extis);
  }
}

/*
 * Reset the interrupt request by writing a 1 to the corresponding
 * pending bit register.
 */
void exti_reset_request(uint32_t extis)
{
  if (IS_EXTI_EXTENDED(extis))
  {
    EXTI_PR2 = EXTI_MASK(extis);
  }
  else
  {
	  EXTI_PR1 = EXTI_MASK(extis);
  }
}

/*
 * Check the flag of a given EXTI interrupt.
 * */
uint32_t exti_get_flag_status(uint32_t exti)
{
  if (IS_EXTI_EXTENDED(exti))
  {
    return EXTI_PR2 & EXTI_MASK(exti);
  }
  else
  {
	  return EXTI_PR1 & EXTI_MASK(exti);
  }
}

/*
 * Remap an external interrupt line to the corresponding pin on the
 * specified GPIO port.
 *
 * TODO: This could be rewritten in fewer lines of code.
 */
void exti_select_source(uint32_t exti, uint32_t gpioport)
{
	uint32_t line;
	for (line = 0; line < 16; line++) {
		if (!(exti & (1 << line))) {
			continue;
		}

		uint32_t bits = 0;

		switch (gpioport) {
		case GPIOA:
			bits = 0;
			break;
		case GPIOB:
			bits = 1;
			break;
		case GPIOC:
			bits = 2;
			break;
#if defined(GPIOH) && defined(GPIO_PORT_H_BASE)
		case GPIOH:
			bits = 7;
			break;
#endif
		}

		uint8_t shift = (uint8_t)(EXTICR_SELECTION_FIELDSIZE * (line % 4));
		uint32_t mask = ((1 << EXTICR_SELECTION_FIELDSIZE) - 1) << shift;
		uint32_t reg = line / 4;

		EXTICR_SELECTION_REG(reg) = (EXTICR_SELECTION_REG(reg) & ~mask) | (bits << shift);
	};
}
/**@}*/

