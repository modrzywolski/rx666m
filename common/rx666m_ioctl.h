#include <linux/ioctl.h>

#define SI5351_ADDR                                (0x60 << 1 )
#define MCP4725_ADDR								(0x61 << 1)
#define R820T2_ADDR      (0x34) //  ((0x1A)<<1)

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
