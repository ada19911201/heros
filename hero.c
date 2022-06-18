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

#include "hero.h"
#include "wait.h"
#include "debug.h"
#include "print.h"
#include "wait_api.h"
#include "task_config.h"
#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "nrfx_spim.h"
#include "report.h"
#include "host.h"
// #include "xprintf.h"

#include HERO_FIRMWARE_H
//SPIM驱动程序实例ID,ID和外设编号对应，0:SPIM0 1:SPIM1 2:SPIM2 3:SPIM3
#define SPI_INSTANCE 3
//定义SPI驱动呢程序实例
static const nrfx_spim_t spi = NRFX_SPIM_INSTANCE(SPI_INSTANCE);
//SPI传输完成标志
static volatile bool spi_xfer_done;
//SPI发送缓存数组，使用EasyDMA时一定要定义为static类型
static uint8_t spi_tx_buf[HERO_LENGTH];
static uint8_t motion_tx_buf[5] ={0x85, 0x86, 0x87, 0x88, 0x80};
//SPI接收缓存数组，使用EasyDMA时一定要定义为static类型
static uint8_t spi_rx_buf[HERO_LENGTH];

//定义SPIM传输描述符结构体
static nrfx_spim_xfer_desc_t spim3_xfer;
report_mouse_t g_mouse_report = {};

// clang-format on

#define D_DPI   2400
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#define constrain_hid(amt) ((amt) < -127 ? -127 : ((amt) > 127 ? 127 : (amt)))

void spi_write_adv(uint8_t reg_addr, uint8_t data) {
    spi_write(reg_addr & ~0x80);
    spi_write(data);
}

uint8_t spi_read_adv(uint8_t reg_addr) {
    spi_write(reg_addr | 0x80);
    spi_write(0x80);
    return spi_rx_buf[0];
}
static void hero_83_80_03_x(const uint8_t x)
{
	(void)spi_read_adv(0x03);
	delay_us(1);
	spi_write_adv(0x03, x);
}
void hero_set_cpi(uint32_t cpi) {
    static uint8_t val = 0 ;
    HERO_CS_LOW;
    if(cpi > 12800){
        val = (uint8_t)((cpi - 50) / 100);
        hero_83_80_03_x(0x24);
    }else{
        val = (uint8_t)((cpi / 50) - 1);
        hero_83_80_03_x(0x20);
    }
	spi_write_adv(0x0C, val); // could also make x, y dpi independent
	spi_write_adv(0x0D, val);
	HERO_CS_HIGH;
}

// uint16_t hero_get_cpi(void) {
//     uint8_t cpival = spi_read_adv(REG_Config1);
//     return (uint16_t)((cpival + 1) & 0xFF) * 100;
// }
static void spi_write_motion()
{
    //设置SPI传输描述符
    spim3_xfer.p_tx_buffer = motion_tx_buf;
    spim3_xfer.tx_length = 5;
    spim3_xfer.p_rx_buffer = spi_rx_buf;
    spim3_xfer.rx_length = 5;
    //传输完成标志设置为false
    spi_xfer_done = false;
    	//拉低CS，使能W25Q128FV
    //启动数据传输
    ret_code_t  err_code =nrfx_spim_xfer(&spi, &spim3_xfer, 0);
    APP_ERROR_CHECK(err_code);
    //等待SPI传输完成
    while (!spi_xfer_done)
    {
    }
}
struct two_bytes { uint8_t val1, val2; };
static struct two_bytes hero_reg_read2(const uint8_t reg1, const uint8_t reg2)
{
	struct two_bytes ret;
	(void)spi_write(reg1 | 0x80);
	(void)spi_write(reg2 | 0x80);
    ret.val1= spi_rx_buf[0];
    (void)spi_write(0x80);
    ret.val2= spi_rx_buf[0];
	return ret;
}


bool hero_init(void) {

    spi_init();
    HERO_CS_LOW;
    spi_write_adv(0x0A, 0x40);  // Shutdown first
    HERO_CS_HIGH;
    delay_us(200);
//1 写入blob
    HERO_CS_LOW;
    spi_write_adv(0x2A, 0xCF); delay_us(20);
    spi_write_adv(0x2B, 0x06); delay_us(20);
    spi_write_adv(0x2C, 0x08); delay_us(20);
    spi_write_adv(0x2D, 0x00); delay_us(20);
    hero_upload_firmware();
    HERO_CS_HIGH;
    delay_us(1);
    //2
    HERO_CS_LOW;
    spi_write_adv(0x2A, 0xCF); delay_us(20);
    spi_write_adv(0x2B, 0x02); delay_us(20);
    spi_write_adv(0x2C, 0x08); delay_us(20);
    spi_write_adv(0x2D, 0x00); delay_us(20);
    for (size_t i = 0; i < FIRMWARE_LENGTH; i += 2) {
		const struct two_bytes val = hero_reg_read2(0x2E, 0x2F);
		if (val.val1 != firmware_data[i] || val.val2 != firmware_data[i + 1]) {
			HERO_CS_HIGH;
			return 1;
		}
	}
    HERO_CS_HIGH;
    delay_us(1);
    //3
    HERO_CS_LOW;
    spi_write_adv(0x2A, 0x00);
    HERO_CS_HIGH;
    delay_us(1);
    HERO_CS_LOW;
    spi_write_adv(0x76, 0x88);
    HERO_CS_HIGH;
    delay_us(1);
    //4
    HERO_CS_LOW;
    spi_write_adv(0x40, 0x82);
    spi_write_adv(0x43, 0x01);
    spi_write_adv(0x43, 0x00);
	(void)hero_reg_read2(0x4A, 0x4B);
	spi_write_adv(0x40, 0x83);
	spi_write_adv(0x43, 0x01);
	spi_write_adv(0x43, 0x00);
	(void)hero_reg_read2(0x4A, 0x4B);
	spi_write_adv(0x40, 0x84);
	spi_write_adv(0x43, 0x01);
	spi_write_adv(0x43, 0x00);
	(void)hero_reg_read2(0x4A, 0x4B);
	spi_write_adv(0x40, 0x85);
	spi_write_adv(0x43, 0x01);
	spi_write_adv(0x43, 0x00);
    (void)hero_reg_read2(0x4A, 0x4B);
	spi_write_adv(0x40, 0x86);
	spi_write_adv(0x43, 0x01);
	spi_write_adv(0x43, 0x00);
    (void)hero_reg_read2(0x4A, 0x4B);
	spi_write_adv(0x40, 0x87);
	spi_write_adv(0x43, 0x01);
	spi_write_adv(0x43, 0x00);
	(void)hero_reg_read2(0x4A, 0x4B);
    HERO_CS_HIGH;
    delay_us(1);
    //5
    // HERO_CS_LOW;
    // spi_write_adv(0x76, 0x00);
    // HERO_CS_HIGH;
    // delay_us(1);
    //6
    HERO_CS_LOW;
	spi_write_adv(0x30, 0x16);
	spi_write_adv(0x31, 0x40);
	spi_write_adv(0x32, 0x0A);
	spi_write_adv(0x33, 0x08);
	spi_write_adv(0x34, 0x0B);
	spi_write_adv(0x35, 0x08);
    HERO_CS_HIGH;
    delay_us(1);
    //7
    HERO_CS_LOW;
	spi_write_adv(0x03, 0x20); delay_us(20);
	spi_write_adv(0x0E, 0x11); delay_us(20);
    spi_write_adv(0x0F, 0x0D); delay_us(20);
    spi_write_adv(0x1F, 0xF2); delay_us(20);
	spi_write_adv(0x22, 0xC8); delay_us(20);
	spi_write_adv(0x23, 0x97); delay_us(20);
	spi_write_adv(0x24, 0xB6); delay_us(20);
	spi_write_adv(0x25, 0x36); delay_us(20);
	spi_write_adv(0x26, 0xDA); delay_us(20);
	spi_write_adv(0x27, 0x50); delay_us(20);
	spi_write_adv(0x28, 0xC4); delay_us(20);
	spi_write_adv(0x29, 0x00); delay_us(20);
	spi_write_adv(0x64, 0x06); delay_us(20);
    HERO_CS_HIGH;
    delay_us(1);
    //8
    HERO_CS_LOW;
    spi_write_adv(0x0B, 0x40);
    HERO_CS_HIGH;
    delay_us(1);
    //9
    HERO_CS_LOW;
    spi_write_adv(0x02, 0x80);
    HERO_CS_HIGH;
    delay_us(1800);  
    //10
    HERO_CS_LOW;    
    spi_write_motion();
    HERO_CS_HIGH;
    delay_us(10);
    //11 cpi
    hero_set_cpi(D_DPI);
    delay_us(10);
    return 0;
}

void hero_upload_firmware(void) 
{
    for (size_t i = 0; i < FIRMWARE_LENGTH; i += 2) {
		spi_write_adv(0x2E, firmware_data[i]);
		spi_write_adv(0x2F, firmware_data[i + 1]);
	}
    spi_write(0x80);
}


report_hero_t hero_read_burst(void) {
    static report_hero_t data = {0};
    HERO_CS_LOW;
    spi_write_motion();
    delay_us(2);
    HERO_CS_HIGH;
    data.mdy = spi_rx_buf[1];
    data.dy  = spi_rx_buf[2];
    data.mdx = spi_rx_buf[3];
    data.dx  = spi_rx_buf[4];
    data.dx |= (data.mdx << 8);
    // data.dx = data.dx * -1;
    data.dy |= (data.mdy << 8);
    // data.dy = data.dy * -1;
    return data;
}

static void send_mouse_report(report_hero_t data){
        static uint8_t btn = 0;
    if (data.dx!=0||data.dy!=0|| btn !=g_mouse_report.buttons|| g_mouse_report.v!=0) 
    {  
        g_mouse_report.x = constrain_hid(data.dx);
        g_mouse_report.y = constrain_hid(data.dy)* -1;
        host_mouse_send(&g_mouse_report);
        btn = g_mouse_report.buttons;
        g_mouse_report.v = 0;
    }
}
void sers_task_thread (void *p_context)
{
    hero_init();
    while (1)
    {
        send_mouse_report(hero_read_burst());
        wait_ms(SERS_TASK_TIMERS);
    }
}


void  spi_write(uint8_t Dat)
{
    spi_tx_buf[0] = Dat;
    //设置SPI传输描述符
    spim3_xfer.p_tx_buffer = spi_tx_buf;
    spim3_xfer.tx_length = 1;
    spim3_xfer.p_rx_buffer = spi_rx_buf;
    spim3_xfer.rx_length = 1;
    //传输完成标志设置为false
    spi_xfer_done = false;
    	//拉低CS，使能W25Q128FV
    //启动数据传输
    ret_code_t  err_code =nrfx_spim_xfer(&spi, &spim3_xfer, 0);
    APP_ERROR_CHECK(err_code);
    //等待SPI传输完成
    while (!spi_xfer_done)
    {
    }
}
void  spi_read()
{
    //设置SPI传输描述符
    spim3_xfer.p_tx_buffer = spi_tx_buf;
    spim3_xfer.tx_length = 0;
    spim3_xfer.p_rx_buffer = spi_rx_buf;
    spim3_xfer.rx_length = 5;
    //传输完成标志设置为false
    spi_xfer_done = false;
    //启动数据传输
    ret_code_t  err_code =nrfx_spim_xfer(&spi, &spim3_xfer, 0);
    APP_ERROR_CHECK(err_code);
    //等待SPI传输完成
    while (!spi_xfer_done)
    {
    }
}

//SPIM事件处理函数
void spi_event_handler(nrfx_spim_evt_t const *p_event,
                       void *p_context)
{
    //设置SPIM传输完成
    spi_xfer_done = true;
}

//初始化spi
void spi_init(void)
{
    nrf_gpio_cfg_output(HERO_CS_PIN);
    // nrf_gpio_cfg_output(HERO_RESET_PIN);
    HERO_CS_HIGH;
    //使用默认配置参数初始化SPI配置结构体
    nrfx_spim_config_t spi_config = NRFX_SPIM_DEFAULT_CONFIG;
    //重写SPIM信号连接的引脚配置
    spi_config.ss_pin = NRFX_SPIM_PIN_NOT_USED;
    spi_config.miso_pin = HERO_MISO_PIN;
    spi_config.mosi_pin = HERO_MOSI_PIN;
    spi_config.sck_pin = HERO_SCK_PIN;
    spi_config.frequency = NRF_SPIM_FREQ_4M;
    spi_config.mode = NRF_SPIM_MODE_3;

    //初始化SPIM
    APP_ERROR_CHECK(nrfx_spim_init(&spi, &spi_config, spi_event_handler, NULL));
}


void spi_uninit(void){
    // HERO_CS_LOW;
    // spi_write_adv(REG_Shutdown, 0xb6);  // Shutdown first
    // HERO_CS_HIGH;
    wait_ms(10);
    nrfx_spim_uninit(&spi);
}
