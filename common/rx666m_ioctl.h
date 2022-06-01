/*
 * RX666m driver
 * Copyright (C) 2020 Marcin Odrzywolski <emk6@wp.pl>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/ioctl.h>

#define SI5351_ADDR                     (0x60 << 1 )
#define MCP4725_ADDR					(0x61 << 1)
#define R820T2_ADDR      				(0x34)

#define PCA9557_ADDR (0x19) ///< Default i2c address
#define PCA9557_CMD_INPUT       (0x00)
#define PCA9557_CMD_OUTPUT      (0x01)
#define PCA9557_CMD_POLARITY    (0x02)
#define PCA9557_CMD_CONFIG      (0x03)

#define MYDRBASE 'k'
#define RX666M_IS_BOOTLOADER_RUNNING	_IO(  MYDRBASE, 1)
#define RX666M_WRITE_RAM 				_IO(  MYDRBASE, 2)
#define RX666M_I2C_READ					_IOWR(	MYDRBASE, 3, rx666m_ioctl_i2c_transfer_t)
#define RX666M_I2C_WRITE				_IOWR(	MYDRBASE, 4, rx666m_ioctl_i2c_transfer_t)
#define RX666M_GPIO_WRITE 				_IO(  MYDRBASE, 5)
#define RX666M_START 					_IO(  MYDRBASE, 6)

typedef struct
{
	uint8_t			*firmware;
	uint32_t		address;
	int32_t			len;
}rx666m_ioctl_write_ram_t;

typedef struct
{
	uint8_t			address;
	uint8_t			reg;
	uint8_t			len;
	uint8_t			data[256];
}rx666m_ioctl_i2c_transfer_t;
