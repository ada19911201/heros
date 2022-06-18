/*
 * @Descripttion: 键盘文件
 * @version: 0.0.1
 * @Author: zoroada
 * @Date: 2022-03-08 19:20:54
 * @LastEditors: zoroada
 * @LastEditTime: 2022-03-10 19:01:34
 */
/* Copyright 2020 Christopher Courtney, aka Drashna Jael're  (@drashna) <drashna@live.com>
 * Copyright 2019 Sunjun Kim
 * Copyright 2020 Ploopy Corporation
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "report.h"
#include "config.h"
#include "nrf_gpio.h"

#ifndef HERO_CPI
#    define HERO_CPI 2400
#endif

#ifndef HERO_CLOCK_SPEED
#    define HERO_CLOCK_SPEED 2000000
#endif

#ifndef HERO_SPI_LSBFIRST
#    define HERO_SPI_LSBFIRST false
#endif

#ifndef HERO_SPI_MODE
#    define HERO_SPI_MODE 3
#endif

#ifndef HERO_SPI_DIVISOR
#    ifdef __AVR__
#        define HERO_SPI_DIVISOR (F_CPU / HERO_CLOCK_SPEED)
#    else
#        define HERO_SPI_DIVISOR 64
#    endif
#endif

#ifndef HERO_LIFTOFF_DISTANCE
#    define HERO_LIFTOFF_DISTANCE 0x02
#endif

#ifndef ROTATIONAL_TRANSFORM_ANGLE
#    define ROTATIONAL_TRANSFORM_ANGLE 0x00
#endif

#ifndef HERO_CS_PIN
#    error "No chip select pin defined -- missing HERO_CS_PIN"
#endif

/*
The pmw33660 and pmw3389 use the same registers and timing and such.
The only differences between the two is the firmware used, and the
range for the DPI. So add a semi-secret hack to allow use of the
pmw3389's firmware blob.  Also, can set the max cpi range too.
This should work for the 3390 and 3391 too, in theory.
*/
#ifndef HERO_FIRMWARE_H
#    define HERO_FIRMWARE_H "hero_firmware.h"
#endif

#ifdef CONSOLE_ENABLE
void print_byte(uint8_t byte);
#endif
typedef int16_t spi_status_t;
typedef struct {
    int16_t dx;           // displacement on x directions. Unit: Count. (CPI * Count = Inch value)
    int8_t  mdx;
    int16_t dy;           // displacement on y directions.
    int8_t  mdy;
} report_hero_t;
extern report_mouse_t g_mouse_report;
void             spi_write_adv(uint8_t reg_addr, uint8_t data);
uint8_t          spi_read_adv(uint8_t reg_addr);
bool             hero_init(void);
void             hero_set_cpi(uint32_t cpi);
// uint16_t         hero_get_cpi(void);
void             hero_upload_firmware(void);
bool             hero_check_signature(void);
report_hero_t hero_read_burst(void);
void             sers_task_thread (void *p_context);

void             spi_init(void);
void             spi_uninit(void);
void             spi_write(uint8_t Dat);
void             spi_read();

#define    HERO_CS_LOW    nrf_gpio_pin_clear(HERO_CS_PIN)  
#define    HERO_CS_HIGH   nrf_gpio_pin_set(HERO_CS_PIN)     
#define    HERO_RESET_HIGH   nrf_gpio_pin_set(HERO_RESET_PIN)   
#define degToRad(angleInDegrees) ((angleInDegrees)*M_PI / 180.0)
#define radToDeg(angleInRadians) ((angleInRadians)*180.0 / M_PI)
