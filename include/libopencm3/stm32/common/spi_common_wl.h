/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2011 Fergus Noble <fergusnoble@gmail.com>
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

/* THIS FILE SHOULD NOT BE INCLUDED DIRECTLY, BUT ONLY VIA SPI.H
The order of header inclusion is important. spi.h includes the device
specific memorymap.h header before including this header file.*/

/** @cond */
#ifdef LIBOPENCM3_SPI_H
/** @endcond */
#pragma once

/**@{*/

#include <libopencm3/stm32/common/spi_common_v2.h>

/* FRF control. */
void spi_set_frf_ti(uint32_t spi);
void spi_set_frf_motorola(uint32_t spi);

/* NSSP control. */
void spi_enable_nssp(uint32_t spi);
void spi_disable_nssp(uint32_t spi);

/** @cond */
#else
#warning "spi_common_wl.h should not be included explicitly, only via spi.h"
#endif

/** @endcond */
/**@}*/