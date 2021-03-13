#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>      /* open */ 
#include <unistd.h>     /* exit */
#include <sys/ioctl.h>  /* ioctl */
#include <stdint.h>
#include <sys/stat.h>
#include <functional>
//#include <type_traits>
#include <iostream>
#include <functional>

#include "../common/rx666m_ioctl.h"

#include "LowLevel.hpp"

#include <sys/syscall.h> //tmp

#define MAX_FWIMG_SIZE          (512 * 1024)

#define GPIO_LED_RED	(1<<0)
#define GPIO_LED_YELLOW (1<<1)
#define GPIO_LED_BLUE	(1<<2)
#define GPIO_SEL0		(1<<3)
#define GPIO_SEL1		(1<<4)
#define GPIO_SHDWN		(1<<5)
#define GPIO_DITH		(1<<6)
#define GPIO_RANDO		(1<<7)

#define GPIO_DEFAULT ( GPIO_LED_RED | GPIO_LED_YELLOW | GPIO_LED_BLUE | GPIO_SEL1 | 1*GPIO_DITH | 1*GPIO_RANDO)
#define GPIO_DEFAULT_DC ( GPIO_LED_RED | GPIO_LED_YELLOW | GPIO_LED_BLUE | 1*GPIO_DITH | 1*GPIO_RANDO)

#define IS_BIG_ENDIAN (*(uint16_t *)"\0\xff" < 0x100)

static const char devName[] = "/dev/rx666m";
static const char imagePath[]= "/home/emk/workspace/rx666m/RX666.img";

/* Array representing physical size of EEPROM corresponding to each size encoding. */
const int i2c_eeprom_size[] =
{
        1024,           // bImageCtl[2:0] = 'b000
        2048,           // bImageCtl[2:0] = 'b001
        4096,           // bImageCtl[2:0] = 'b010
        8192,           // bImageCtl[2:0] = 'b011
        16384,          // bImageCtl[2:0] = 'b100
        32768,          // bImageCtl[2:0] = 'b101
        65536,          // bImageCtl[2:0] = 'b110
        131072          // bImageCtl[2:0] = 'b111
};


//tuner jest podlaczony do: CLK2 PLLB
typedef struct
{
	unsigned int address; /* 16-bit register address */
	unsigned char value; /* 8-bit register data */

} si5351a_revb_register_t;


//ADC-clk0
si5351a_revb_register_t xregs[] =
{
	{ 0x000F, 0x00 },
	{ 0x0010, 0x0F },
	{ 0x0011, 0x8C },
	{ 0x0012, 0x8C },
	{ 0x0013, 0x8C },
	{ 0x0014, 0x8C },
	{ 0x0015, 0x8C },
	{ 0x0016, 0x8C },
	{ 0x0017, 0x8C },
	{ 0x001A, 0x00 },
	{ 0x001B, 0x1B },
	{ 0x001C, 0x00 },
	{ 0x001D, 0x0E },
	{ 0x001E, 0x97 },
	{ 0x001F, 0x00 },
	{ 0x0020, 0x00 },
	{ 0x0021, 0x13 },
	{ 0x002A, 0x00 },
	{ 0x002B, 0x01 },
	{ 0x002C, 0x00 },
	{ 0x002D, 0x05 },
	{ 0x002E, 0x00 },
	{ 0x002F, 0x00 },
	{ 0x0030, 0x00 },
	{ 0x0031, 0x00 },
	{ 0x005A, 0x00 },
	{ 0x005B, 0x00 },
	{ 0x0095, 0x00 },
	{ 0x0096, 0x00 },
	{ 0x0097, 0x00 },
	{ 0x0098, 0x00 },
	{ 0x0099, 0x00 },
	{ 0x009A, 0x00 },
	{ 0x009B, 0x00 },
	{ 0x00A2, 0x00 },
	{ 0x00A3, 0x00 },
	{ 0x00A4, 0x00 },
	{ 0x00A5, 0x00 },
	{ 0x00B7, 0x52 },
};

//ADC-clk0, R820T-clk2
si5351a_revb_register_t regs[] =
{
	{ 0x0002, 0x53 },
	{ 0x0003, 0x00 },
	{ 0x0004, 0x20 },
	{ 0x0007, 0x00 },
	{ 0x000F, 0x00 },
	{ 0x0010, 0x0F },
	{ 0x0011, 0x8C },
	{ 0x0012, 0x0F },
	{ 0x0013, 0x8C },
	{ 0x0014, 0x8C },
	{ 0x0015, 0x8C },
	{ 0x0016, 0x8C },
	{ 0x0017, 0x8C },
	{ 0x001A, 0x00 },
	{ 0x001B, 0x1B },
	{ 0x001C, 0x00 },
	{ 0x001D, 0x0E },
	{ 0x001E, 0x97 },
	{ 0x001F, 0x00 },
	{ 0x0020, 0x00 },
	{ 0x0021, 0x13 },
	{ 0x002A, 0x00 },
	{ 0x002B, 0x01 },
	{ 0x002C, 0x00 },
	{ 0x002D, 0x05 },
	{ 0x002E, 0x00 },
	{ 0x002F, 0x00 },
	{ 0x0030, 0x00 },
	{ 0x0031, 0x00 },
	{ 0x003A, 0x00 },
	{ 0x003B, 0x01 },
	{ 0x003C, 0x00 },
	{ 0x003D, 0x0C },
	{ 0x003E, 0x00 },
	{ 0x003F, 0x00 },
	{ 0x0040, 0x00 },
	{ 0x0041, 0x00 },
	{ 0x005A, 0x00 },
	{ 0x005B, 0x00 },
	{ 0x0095, 0x00 },
	{ 0x0096, 0x00 },
	{ 0x0097, 0x00 },
	{ 0x0098, 0x00 },
	{ 0x0099, 0x00 },
	{ 0x009A, 0x00 },
	{ 0x009B, 0x00 },
	{ 0x00A2, 0x00 },
	{ 0x00A3, 0x00 },
	{ 0x00A4, 0x00 },
	{ 0x00A5, 0x00 },
	{ 0x00A7, 0x00 },
	{ 0x00B7, 0x52 },

};

LowLevel::LowLevel() : devHandle(-1), VGAPresent(-1), DCPresent(-1),DCEnabled(false)
{
	memset(&tuner_data, 0, sizeof(tuner_data));
}

LowLevel::~LowLevel()
{
	CloseDev();
}

void LowLevel::Init()
{
	std::cerr << "Open dev\n";
	OpenDev();
	std::cerr << "Opened\n";

	if(IsBootloaderRunning())
	{
		std::cerr << "load fw\n";
		LoadFirmware();
		std::cerr << "start fw\n";
		sleep(1);
		std::cerr << "reopen dev\n";
		CloseDev();
		OpenDev();
		FX3Start();
	}

	std::cerr << "init clk\n";
	InitClk();
	//Si5351init();

	GpioWrite( GPIO_DEFAULT );
	SetMode();

	isVGAPresent();
	if( isDCPresent() )
	{
		std::cerr << "Initailizing R820T2\n";
		r82xx_init(&tuner_data);
		r82xx_set_freq(&tuner_data, 102000000);
	}
}

void LowLevel::SetMode()
{
	if(isDCPresent())
	{
		DCEnabled=true;
		GpioWrite( GPIO_DEFAULT_DC );
	}
}

bool LowLevel::isRunning()
{
	return (devHandle>0);
}

void LowLevel::SetHFGain(uint16_t gain)
{
	if(devHandle < 0)
		return;

	rx666m_ioctl_i2c_transfer_t i2c_trans;
	int r;

	memset(&i2c_trans, 0, sizeof(i2c_trans));
	i2c_trans.address = MCP4725_ADDR;
	i2c_trans.reg  = (gain >> 8) & 0x0f;
	i2c_trans.len = 1;
	i2c_trans.data[0]=gain & 0xff;
	r=ioctl(devHandle, RX666M_I2C_WRITE, &i2c_trans);
	printf("r=%d\n", r);
}

void LowLevel::SetAD8331Gain(double gain)
{
	uint32_t igain;
	if(devHandle < 0)
		return;

	if(gain < AD8331_MinGain)
		gain = AD8331_MinGain;
	if(gain > AD8331_MaxGain)
		gain = AD8331_MaxGain;

	std::cerr << "AD8331 " << gain << " dB" << std::endl;

	gain -= AD8331_MinGain;


	igain = static_cast<double>(0xfff) * gain / (AD8331_MaxGain - AD8331_MinGain);
	//std::cerr << "AD8331 0x" << std::hex << igain << ", " << gain << std::endl;

	rx666m_ioctl_i2c_transfer_t i2c_trans;
	int r;

	memset(&i2c_trans, 0, sizeof(i2c_trans));
	i2c_trans.address = MCP4725_ADDR;
	i2c_trans.reg  = (igain >> 8) & 0x0f;
	i2c_trans.len = 1;
	i2c_trans.data[0]=igain & 0xff;
	r=ioctl(devHandle, RX666M_I2C_WRITE, &i2c_trans);
	//printf("r=%d\n", r);
}

void LowLevel::SetHFGain2(double gain)
{
	if( gain <= ATTN2_Gain )
	{
		//std::cerr << "ATTN -20dB" << std::endl;
		SetAttn(ATTN2_Gain);
		gain -= ATTN2_Gain;
	}
	else if( gain < AD8331_MinGain )
	{
		//std::cerr << "ATTN -10dB" << std::endl;
		SetAttn(ATTN1_Gain);
		gain -= ATTN1_Gain;
	}
	else
	{
		//std::cerr << "ATTN 0dB" << std::endl;
		SetAttn(0);
	}

	SetAD8331Gain(gain);
}

void LowLevel::SetDCGain(uint16_t gain)
{
	if( isRunning() && DCEnabled )
	{
		r82xx_set_gain(&tuner_data, true, gain*10);
	}
}

void LowLevel::SetDCFreq(uint32_t freq)
{
	if( isRunning() && DCEnabled )
	{
		if(freq < 320000000)
			freq = 320000000;
		if(freq > 2000000000lu)
			freq = 2000000000lu;
		r82xx_set_freq(&tuner_data, freq);
	}
}

void LowLevel::SetAttn(double gain)
{

	if(gain >= 0.0)
		gain = 0.0;
	else if (gain < ATTN2_Gain)
		gain = ATTN2_Gain;

	int32_t igain = ceil(gain / 10.0);

	//printf("gain=%d\n", (int)igain);

	if(isRunning() && !DCEnabled)
	{
		uint8_t gpio = GPIO_DEFAULT & (0xff ^ (GPIO_SEL0|GPIO_SEL1));

		switch(igain)
		{
			case 0:  //0dB
				gpio |= GPIO_SEL1; break;
				break;
			case -1: //-10dB
				gpio |= GPIO_SEL1 | GPIO_SEL0; break;
			case -2: //-20dB
				gpio |=             GPIO_SEL0; break;
		}

		GpioWrite( gpio );
	}
}

void LowLevel::SetAGC(bool on)
{
	peakDetector.SetAGC(on);
}

uint16_t LowLevel::GetHFGain()
{
	if(devHandle < 0)
		return 0;

	rx666m_ioctl_i2c_transfer_t i2c_trans;
	int r;

	i2c_trans.address = MCP4725_ADDR;
	i2c_trans.reg  = 0;
	i2c_trans.len = 3;

	r=ioctl(devHandle, RX666M_I2C_READ, &i2c_trans);
	//printf("r=%d\n", r);
	//printf("data[0]=%x\n", (uint32_t)i2c_trans.data[0]);
	//printf("data[1]=%x\n", (uint32_t)i2c_trans.data[1]);
	//printf("data[2]=%x\n", (uint32_t)i2c_trans.data[2]);

	uint16_t dac;
	dac = ((uint16_t)i2c_trans.data[1] << 4);
	dac |= i2c_trans.data[2]>>4;

	return dac;
}

int LowLevel::Read(void *buffer, size_t cnt)
{
	if(devHandle < 0)
		return 0;

	int r, i;
	uint16_t *ptr=reinterpret_cast<uint16_t*>(buffer);

	r = read(devHandle, buffer, cnt);
	//r = cnt;
	//fprintf(stderr, "r=%d\n", r);

	using namespace std::placeholders;

	auto gainSet = std::bind(&LowLevel::SetHFGain2, this, _1);

	if(r > 0)
	{
		//descramble data
		if(GPIO_DEFAULT & GPIO_RANDO)
		{
			for(i=0;i<r/2;i++)
			{
				if(ptr[i]&1)
				{
					ptr[i]=ptr[i] ^ 0xfffe;
				}
			}
		}

		//correct endian
		if(!IS_BIG_ENDIAN)
		{
			for(i=0;i<r/2;i++)
			{
				ptr[i]=__builtin_bswap16(ptr[i]);
			}
		}

		static uint16_t aa=0;
		if(0)
		{
			for(i=0;i<r/4;i+=2)
			{
				ptr[i*2]=aa;
				ptr[i*2+1]=rand()%65536lu;
			}
		}
		++aa;

	}

#if 0
	uint64_t avg =0;
	for(i=0;i<r/2;i++)
	{
		avg += ptr[i];
	}
	if(r >= 2)
	{
		avg /= r/2;
		avgValue = avg;
	}
#endif

#if 0
	if(r > 0)
	{
		uint16_t *ptr=reinterpret_cast<uint16_t*>(buffer);

		std::generate(ptr, ptr+r/2, std::ref(nco));

		if(0){

			size_t const ndump=16;

			printf("dumping1: %p(%03ld)", ptr, bufnum);
			for (size_t i = 0; i < r; i++)
			{
				if(i<ndump)
				{
					printf("%04X.", (uint32_t)ptr[i]);
				}
				if(i>=r-ndump)
				{
					printf(",%04X", (uint32_t)ptr[i]);
				}
			}
			printf("\n");
		}
	}
#endif

#if 0
	if(r > 0)
	{
		for(i=0;i<r/2;i++)
		{
			peakDetector.feed(ptr[i], gainSet);
		}
	}
#endif

	return r;
}

void LowLevel::OpenDev()
{

	if(devHandle == -1)
	{
		devHandle = open(devName, 0);
		if (devHandle < 0)
		{
			printf ("Can't open device: %s\n", devName);
		}
  	}
}

void LowLevel::CloseDev()
{
	if(devHandle != -1)
	{
		close(devHandle);
		devHandle = -1;
	}
}

void LowLevel::LoadFirmware()
{
	if(devHandle < 0)
		return;

	if(IsBootloaderRunning())
	{
		std::cerr << "Bootloader running, writting firmware\n";

		if(FX3Download(imagePath))
		{
			std::cerr << "Can't read firmware: " << imagePath << "\n"; 
		}
	}

}

bool LowLevel::IsBootloaderRunning()
{
	if(devHandle < 0)
		return false;

	int i;
	char c=1;

	i = 0;
	c = ioctl(devHandle, RX666M_IS_BOOTLOADER_RUNNING, i++);

	if (c < 0)
	{
		//printf( "IsBootloaderRunning failed: %d\n", (int)c);
		return false;
	}
	else
	{
		//printf("OK: %d\n", (int)c);
		return true;
	}
}

void LowLevel::FX3Start()
{
	if(devHandle < 0)
		return;

	int r;

	r = ioctl(devHandle, RX666M_START, 0);

	if (r < 0)
	{
		printf ("FX3Start failed:%d\n", r);
	}
}


int LowLevel::SendI2cbyte(uint32_t address, uint8_t reg, uint8_t value)
{
	int r;

	if(devHandle < 0)
		return -1;

	rx666m_ioctl_i2c_transfer_t i2c_trans;

	memset(&i2c_trans, 0, sizeof(i2c_trans));

	i2c_trans.address = address;
	i2c_trans.reg  = reg;
	i2c_trans.len = 1;
	i2c_trans.data[0]=value;

	int i;
    for(i=0;i<i2c_trans.len;i++)
	{
        //printf("reg[%03x] <- 0x%02x\n", reg+i, (int)i2c_trans.data[i]);
	}

	r=ioctl(devHandle, RX666M_I2C_WRITE, &i2c_trans);
	//printf("r=%d\n", r);

	return r;
}

int LowLevel::SendI2cbytes(uint32_t address, uint8_t reg, const uint8_t *values, size_t len)
{
	int r;

	if(devHandle < 0)
		return -1;

	rx666m_ioctl_i2c_transfer_t i2c_trans;

	//printf("sizeof(i2c_trans)=%ld\n", sizeof(i2c_trans));

	memset(&i2c_trans, 0, sizeof(i2c_trans));

	i2c_trans.address = address;
	i2c_trans.reg  = reg;
	i2c_trans.len = len;
	memcpy(&i2c_trans.data, values, len);

	int i;
    for(i=0;i<i2c_trans.len;i++)
	{
        //printf("reg[%03x] <- 0x%02x\n", reg+i, (int)i2c_trans.data[i]);
	}

	r=ioctl(devHandle, RX666M_I2C_WRITE, &i2c_trans);
	//printf("r=%d\n", r);

	return r;
}

#if 0
int LowLevel::SendI2cbytes(uint32_t address, uint8_t *values, size_t len)
{
	int r;

	if(devHandle < 0 || len < 1)
		return -1;

	rx666m_ioctl_i2c_transfer_t i2c_trans;

	printf("sizeof(i2c_trans)=%ld\n", sizeof(i2c_trans));

	memset(&i2c_trans, 0, sizeof(i2c_trans));

	i2c_trans.address = address;
	i2c_trans.reg  = *values;
	i2c_trans.len = len-1;
	if(len > 1)
		memcpy(&i2c_trans.data[1], &values[1], len-1);

	int i;
    for(i=0;i<i2c_trans.len;i++)
	{
        printf("reg[%03d] <- 0x%02x\n", i, (int)i2c_trans.data[i]);
	}

	r=ioctl(devHandle, RX666M_I2C_WRITE, &i2c_trans);
	printf("r=%d\n", r);

	return r;
}
#endif

int LowLevel::RecvI2cbytes(uint32_t address, uint8_t reg, uint8_t *values, size_t len)
{
	int r;

	if(devHandle < 0)
		return -1;

	rx666m_ioctl_i2c_transfer_t i2c_trans;

	memset(&i2c_trans, 0, sizeof(i2c_trans));

	i2c_trans.address = address;
	i2c_trans.reg  = reg;
	i2c_trans.len = len;

	r=ioctl(devHandle, RX666M_I2C_READ, &i2c_trans);
	//printf("r=%d\n", r);

	memcpy(values, i2c_trans.data, i2c_trans.len);

    int i;
	for(i=0;i<i2c_trans.len;i++)
		printf("reg[%03d]=0x%02x\n", reg+i, (int)i2c_trans.data[i]);

	return r;
}

void LowLevel::GpioWrite(uint8_t gpio)
{
	if(devHandle < 0)
		return;

//	printf("GpioWrite %02x\n", (int)gpio);
	int r;

	r = ioctl(devHandle, RX666M_GPIO_WRITE, gpio);

	if (r < 0)
	{
		printf ("GpioWrite failed:%d\n", r);
	}
}

int LowLevel::SendFW(unsigned char *firmware, uint32_t address, int32_t len)
{
	if(devHandle < 0)
		return -1;

	rx666m_ioctl_write_ram_t cmd;

	cmd.firmware = firmware;
	cmd.address = address;
	cmd.len = len;

	printf("sending ioctl RX666M_WRITE_RAM address=%x, len=%x\n", address, len);
	int r = ioctl(devHandle, RX666M_WRITE_RAM, &cmd);
	if (r < 0)
	{
		printf ("ioctl_load_fw failed:%d\n", r);
	}

	return r;
}

int LowLevel::FX3ReadImage( const char *filename, unsigned char *buf, int *romsize, int *filesize)
{
        int fd;
		int nbr;
        struct stat filestat;

        if (stat (filename, &filestat) != 0)
		{
                fprintf (stderr, "Error: Failed to stat file %s\n", filename);
                return -1;
        }

        // Verify that the file size does not exceed our limits.
        *filesize = filestat.st_size;
        if (*filesize > MAX_FWIMG_SIZE)
		{
                fprintf (stderr, "Error: File size exceeds maximum firmware image size\n");
                return -2;
        }

        fd = open (filename, O_RDONLY);
        if (fd < 0)
		{
                fprintf (stderr, "Error: File not found\n");
                return -3;
        }
        nbr = read (fd, buf, 2);                /* Read first 2 bytes, must be equal to 'CY'    */
        if (nbr != 2 || strncmp ((char *)buf,"CY",2))
		{
                fprintf (stderr, "Error: Image does not have 'CY' at start.\n");
                return -4;
        }
        nbr = read (fd, buf, 1);                /* Read 1 byte. bImageCTL       */
        if (nbr != 1 || buf[0] & 0x01)
		{
                fprintf (stderr, "Error: Image does not contain executable code\n");
                return -5;
        }
        if (romsize != 0)
                *romsize = i2c_eeprom_size[(buf[0] >> 1) & 0x07];

        nbr = read (fd, buf, 1);                /* Read 1 byte. bImageType      */
        if (nbr != 1 || !(buf[0] == 0xB0))
		{
                fprintf (stderr, "Error: Not a normal FW binary with checksum\n");
                return -6;
        }

        // Read the complete firmware binary into a local buffer.
        lseek (fd, 0, SEEK_SET);
        nbr = read (fd, buf, *filesize);

        close (fd);
        return 0;
}

int LowLevel::FX3Download(const char *imagefile)
{
        unsigned char *fwBuf;
        unsigned int  *data_p;
        unsigned int i, checksum;
        unsigned int address, length;
        int r, index, filesize;

        fwBuf = (unsigned char *)calloc (1, MAX_FWIMG_SIZE);
        if (fwBuf == 0)
		{
                fprintf (stderr, "Error: Failed to allocate buffer to store firmware binary\n");
                return -1;
        }

        // Read the firmware image into the local RAM buffer.
        r = FX3ReadImage(imagefile, fwBuf, NULL, &filesize);
        if (r != 0)
		{
                fprintf (stderr, "Error: Failed to read firmware file %s\n", imagefile);
                free (fwBuf);
                return -2;
        }

		printf("Image loaded, sending to device\n");
        // Run through each section of code, and use vendor commands to download them to RAM.
        index    = 4;
        checksum = 0;
        while (index < filesize)
		{
                data_p  = (unsigned int *)(fwBuf + index);
                length  = data_p[0];
                address = data_p[1];
				printf("Writting segment: %x of len %x\n", address, length);
                if (length != 0)
				{
                        for (i = 0; i < length; i++)
                                checksum += data_p[2 + i];
						r = SendFW(fwBuf + index + 8, address, length * 4);
                        if ( r != 0)
						{
                                fprintf (stderr, "Error: Failed to download data to FX3 RAM\n");
                                free (fwBuf);
                                return -3;
                        }
                } 
				else
				{
                        if (checksum != data_p[2])
						{
                                fprintf (stderr, "Error: Checksum error in firmware binary\n");
                                free (fwBuf);
                                return -4;
                        }

						sleep(1);
						r = SendFW(NULL, address, 0);

                        if (r != 0)
                                printf ("Info: Ignored error in control transfer: %d\n", r);
                        break;
                }

                index += (8 + length * 4);
        }
		printf("image sent\n");

        free (fwBuf);
        return 0;
}

void LowLevel::InitClk()
{
	//Disable Outputs Set CLKx_DIS high; Reg. 3 = 0xFF
	SendI2cbyte(SI5351_ADDR, 3, 0xff);

	//Powerdown all output drivers Reg. 16, 17, 18, 19, 20, 21, 22, 23 = 0x80
	SendI2cbyte(SI5351_ADDR, 16, 0x80);
	SendI2cbyte(SI5351_ADDR, 17, 0x80);
	SendI2cbyte(SI5351_ADDR, 18, 0x80);
	SendI2cbyte(SI5351_ADDR, 19, 0x80);
	SendI2cbyte(SI5351_ADDR, 20, 0x80);
	SendI2cbyte(SI5351_ADDR, 21, 0x80);
	SendI2cbyte(SI5351_ADDR, 22, 0x80);
	SendI2cbyte(SI5351_ADDR, 23, 0x80);

	//Set interrupt masks(see register 2 description)
	SendI2cbyte(SI5351_ADDR, 0x0002, 0x53);

	int i, n=sizeof(regs)/sizeof(regs[0]);

	for( i=0; i<n; i++)
		SendI2cbyte(SI5351_ADDR, regs[i].address, regs[i].value);

	//Apply PLLA and PLLB soft resetReg. 177 = 0xAC
	SendI2cbyte(SI5351_ADDR, 177, 0xAC);

	//Enable desired outputs(see Register 3)
	SendI2cbyte(SI5351_ADDR, 0x0003, 0x00);
}


bool LowLevel::isVGAPresent()
{
	if(!isRunning())
		return false;

	if(VGAPresent >= 0)
		return VGAPresent ? true : false;
	
	uint8_t values[1];

	if( !RecvI2cbytes(MCP4725_ADDR, 0, values, sizeof(values)))
		VGAPresent = 1;
	else
		VGAPresent = 0;

	printf("VGA is %s\n", VGAPresent ? "present" : "absent");
	return VGAPresent ? true : false;
}

bool LowLevel::isDCPresent()
{
	if(!isRunning())
		return false;

	return false;

	if(DCPresent >= 0)
		return DCPresent ? true : false;

	uint8_t values[1];

	if( !RecvI2cbytes(R820T2_ADDR, 0, values, sizeof(values)))
		DCPresent = 1;
	else
		DCPresent = 0;

	printf("DC is %s\n", DCPresent ? "present" : "absent");

	return DCPresent ? true : false;
}

void LowLevel::ReadThread(void)
{
	size_t total = 0;

	std::cerr << "ThreadEntry pid=" << syscall(SYS_gettid) << std::endl;
	pthread_setname_np(pthread_self(), "rx666m reader");

	while(!StopReader)
	{
		char sbuffer[packet_size];
		auto buffer = getRingBuffer().getWrBuf();

		if(buffer)
		{
			int res = Read(buffer->data.data(), buffer->data.size());
			//fprintf(stderr, "res=%d %p %ld\n", res, buffer->data.data(), buffer->bytesAvail);
			if(res > 0)
			{
				if( rand() % 101 > 98 && res > 1)
				{
					//res--;
				}

				buffer->bytesAvail = res;
				buffer->bytesRead = 0;
				getRingBuffer().commit(buffer);
			}
		}
		else
		{
			//read data and drop - for statistics only
			//printf("dropping\n");
			//tutaj dodac statystyki do dropping
			Read(sbuffer, sizeof(sbuffer));
		}

	}

	printf("Done: %ld bytes\n", total);
}

int LowLevel::ActivateReader()
{
    if (! ReadThreadHandle.joinable())
    {
		getRingBuffer().reset();
		missionStart = boost::posix_time::second_clock::local_time();
		StopReader = false;
        ReadThreadHandle = std::thread(&LowLevel::ReadThread, this);
    }

	return 0;
}

void LowLevel::DeactivateReader()
{
    if (ReadThreadHandle.joinable())
    {
		StopReader = true;
		ReadThreadHandle.join();
    }
}

void LowLevel::DumpReaderStats()
{
	auto missionTime = boost::posix_time::second_clock::local_time() - missionStart;

	auto out = boost::format( "[Transferred: %s][Lost: %s][Duration: %s][AGC gain: %.1f][Peak: 0x%04x Avg: 0x%04x %s %s]") 
				% human_readable_bytes(getRingBuffer().statsTotal())
				% human_readable_bytes(getRingBuffer().statsLost())
				% human_readable_duration(missionTime.total_seconds())
				% peakDetector.getGain()
				% peakDetector.getPeakValue()
				% avgValue
				% (peakDetector.isFastClipping() ? " FASTCLIPPING" : "")
				% (peakDetector.isSlowClipping() ? " SLOWCLIPPING" : "")
				;

	std::cerr << "\33[2K\r" << out;; 
	if(0)
	std::cerr 	<< std::endl
				<< human_readable_bytes(getRingBuffer().statsTotal()) << " transferred" << std::endl
				<< human_readable_bytes(getRingBuffer().statsLost()) << " lost" << std::endl
				<< missionTime.seconds() << " (s) mission time" << std::endl
				<< human_readable_duration(missionTime.total_seconds()) << " mission time" << std::endl
				<< std::hex << static_cast<int>(peakDetector.getPeakValue()) << " peak value" << (peakDetector.isFastClipping() ? " FASTCLIPPING" : "") << (peakDetector.isSlowClipping() ? " SLOWCLIPPING" : "") << std::endl
				<< std::dec << "agc gain: " << peakDetector.getGain() << std::endl;
}


#define SI_CLK0_CONTROL		16			// Registers
#define SI_CLK1_CONTROL		17
#define SI_CLK2_CONTROL		18
#define SI_SYNTH_PLL_A		26
#define SI_SYNTH_PLL_B		34
#define SI_SYNTH_MS_0		42
#define SI_SYNTH_MS_1		50
#define SI_SYNTH_MS_2		58
#define SI_PLL_RESET		(0x177)

#define SI_R_DIV_1		(0)     //  0b00000000		/
#define SI_R_DIV_2		(0x10)  //  0b00010000
#define SI_R_DIV_4		(0x20)  //  0b00100000
#define SI_R_DIV_8		(0x30)  //  0b00110000
#define SI_R_DIV_16		(0x40)  //  0b01000000
#define SI_R_DIV_32		(0x50)  //  0b01010000
#define SI_R_DIV_64		(0x60)  //  0b01100000
#define SI_R_DIV_128	(0x70)  //  0b01110000

#define SI_CLK_SRC_PLL_A	0b00000000
#define SI_CLK_SRC_PLL_B	0b00100000

#define SI5351_FREQ	27000000			// Crystal frequency
#define SI5351_PLL_FIXED                80000000000ULL

#define SI5351_ADDR        			   (0x60 << 1 )
#define SI5351_XTAL_FREQ                25000000

#define SI5351_CRYSTAL_LOAD             183
#define SI5351_CRYSTAL_LOAD_MASK        (3<<6)
#define SI5351_CRYSTAL_LOAD_0PF         (0<<6)
#define SI5351_CRYSTAL_LOAD_6PF         (1<<6)
#define SI5351_CRYSTAL_LOAD_8PF         (2<<6)
#define SI5351_CRYSTAL_LOAD_10PF        (3<<6)

#define SI5351_PLL_INPUT_SOURCE         15
#define SI5351_CLKIN_DIV_MASK           (3<<6)
#define SI5351_CLKIN_DIV_1              (0<<6)
#define SI5351_CLKIN_DIV_2              (1<<6)
#define SI5351_CLKIN_DIV_4              (2<<6)
#define SI5351_CLKIN_DIV_8              (3<<6)
#define SI5351_PLLB_SOURCE              (1<<3)
#define SI5351_PLLA_SOURCE              (1<<2)


#define DbgPrintf printf
#define ADC_FREQ  (64000000)
#define R820T_FREQ (32000000)

void LowLevel::Si5351init()
{
	// Set crystal load capacitance
    SendI2cbyte(SI5351_ADDR, SI5351_CRYSTAL_LOAD, SI5351_CRYSTAL_LOAD_6PF | 0b00010010);
	SendI2cbyte(SI5351_ADDR, SI_CLK0_CONTROL, 0x80); // clocks off
	SendI2cbyte(SI5351_ADDR, SI_CLK1_CONTROL, 0x80);
	SendI2cbyte(SI5351_ADDR, SI_CLK2_CONTROL, 0x80);
    //if ((GetmodeRF() == HFMODE) || (GetmodeRF() == VLFMODE))
    //      si5351aSetFrequency(ADC_FREQ, R820T_ZERO);
    //else
          si5351aSetFrequency(ADC_FREQ,R820T_FREQ);
    DbgPrintf("Si5351_init()\n");
}


//#define _PLLDEBUG_
//
// Set up specified PLL with mult, num and denom
// mult is 15..90
// num is 0..1,048,575 (0xFFFFF)
// denom is 0..1,048,575 (0xFFFFF)
//
void LowLevel::setupPLL(UINT8 pll, UINT8 mult, UINT32 num, UINT32 denom)
{
	UINT32 P1;					// PLL config register P1
	UINT32 P2;					// PLL config register P2
	UINT32 P3;					// PLL config register P3
	UINT8 data[8];

	// the actual multiplier is  mult + num / denom
#ifdef _PLLDEBUG_
    double fa;
    double x = 27000000.0;
    fa = x * ((double) mult + (double)num / (double)denom);
	DbgPrintf((char *) "pll = %d , mult = %d , num = %d , denom = %d\n", pll, mult, num, denom );
	DbgPrintf((char *) "pll target %e \n", fa);
#endif

	P1 = (UINT32)(128 * ((float)num / (float)denom));
	P1 = (UINT32)(128 * (UINT32)(mult) + P1 - 512);
	P2 = (UINT32)(128 * ((float)num / (float)denom));
	P2 = (UINT32)(128 * num - denom * P2);
	P3 = denom;

	data[0] = (P3 & 0x0000FF00) >> 8;
	data[1] = (P3 & 0x000000FF);
	data[2] = (P1 & 0x00030000) >> 16;
	data[3] = (P1 & 0x0000FF00) >> 8;
	data[4] = (P1 & 0x000000FF);
	data[5] = ((P3 & 0x000F0000) >> 12) | ((P2 & 0x000F0000) >> 16);
	data[6] = (P2 & 0x0000FF00) >> 8;
	data[7] = (P2 & 0x000000FF);

	SendI2cbytes(SI5351_ADDR, pll, data, sizeof(data));
}
//
// Set up MultiSynth with integer divider and R divider
// R divider is the bit value which is OR'ed onto the appropriate register, it is a #define in si5351a.h
//
void LowLevel::setupMultisynth( UINT8 synth, UINT32 divider, UINT8 rDiv)
{
	UINT32 P1;	// Synth config register P1
	UINT32 P2;	// Synth config register P2
	UINT32 P3;	// Synth config register P3
	UINT8 data[8];

#ifdef _PLLDEBUG_
	DbgPrintf("setupMultisynth synth = %d  divider = %d  rDiv= %d \n", synth, divider, rDiv);
	DbgPrintf("output expected f = %e\n", fa / divider);
#endif

	P1 = 128 * divider - 512;
	P2 = 0;							// P2 = 0, P3 = 1 forces an integer value for the divider
	P3 = 1;

	data[0] = (P3 & 0x0000FF00) >> 8;
	data[1] = (P3 & 0x000000FF);
	data[2] = ((P1 & 0x00030000) >> 16) | rDiv ;
	data[3] = (P1 & 0x0000FF00) >> 8 ;
	data[4] = (P1 & 0x000000FF);
	data[5] = ((P3 & 0x000F0000) >> 12) | ((P2 & 0x000F0000) >> 16);
	data[6] = (P2 & 0x0000FF00) >> 8;
	data[7] = (P2 & 0x000000FF);

	SendI2cbytes(SI5351_ADDR, synth, data, sizeof(data));
}
//
// Switches off Si5351a output
// Example: si5351aOutputOff(SI_CLK0_CONTROL);
// will switch off output CLK0
//
void LowLevel::si5351aOutputOff(UINT8 clk)
{
	SendI2cbyte(SI5351_ADDR, clk, 0x80);		// Refer to SiLabs AN619 to see bit values - 0x80 turns off the output stage
}

void LowLevel::si5351aSetFrequency(UINT32 freq, UINT32 freq2)
{
    UINT32 frequency;
	UINT32 pllFreq;
	UINT32 xtalFreq = SI5351_FREQ;
	UINT32 l;
	double f;
	UINT8 mult;
	UINT32 num;
	UINT32 denom;
	UINT32 divider;
	UINT32 rdiv;

	double corr = 0.9999314;
	//double corr = 1.0;

	DbgPrintf("BBRF103 si5351 SetFreq ADC sampling:%d R820T reference:%d\n", freq, freq2);

	rdiv = (UINT32) SI_R_DIV_1;

	frequency = (UINT32)((double) freq * corr);

	while (frequency < 1000000)
	{
		frequency = frequency * 2;
		rdiv += SI_R_DIV_2;
	}
#ifdef _PLLDEBUG_
	DbgPrintf((char *) "\nCLK0 frequency %d \n", frequency);
#endif
	divider = 900000000UL / frequency;// Calculate the division ratio. 900,000,000 is the maximum internal
									// PLL frequency: 900MHz
	if (divider % 2) divider--;		// Ensure an even integer division ratio

	pllFreq = divider * frequency;	// Calculate the pllFrequency: the divider * desired output frequency
#ifdef _PLLDEBUG_
	DbgPrintf((char *) "pllA Freq  %d \n", pllFreq);
#endif
	mult = pllFreq / xtalFreq;		// Determine the multiplier to get to the required pllFrequency
	l = pllFreq % xtalFreq;			// It has three parts:
	f =(double) l;					// mult is an integer that must be in the range 15..90
	f *= 1048575;					// num and denom are the fractional parts, the numerator and denominator
	f /= xtalFreq;					// each is 20 bits (range 0..1048575)
	num = (UINT32) f;				// the actual multiplier is  mult + num / denom
	denom = 1048575;				// For simplicity we set the denominator to the maximum 1048575
									// Set up PLL A with the calculated multiplication ratio
	setupPLL(SI_SYNTH_PLL_A, mult, num, denom);
									// Set up MultiSynth divider 0, with the calculated divider.
									// The final R division stage can divide by a power of two, from 1..128.
									// represented by constants SI_R_DIV1 to SI_R_DIV128 (see si5351a.h header file)
									// If you want to output frequencies below 1MHz, you have to use the
									// final R division stage
	setupMultisynth(SI_SYNTH_MS_0, divider, rdiv);
									// Reset the PLL. This causes a glitch in the output. For small changes to
									// the parameters, you don't need to reset the PLL, and there is no glitch
	SendI2cbyte((UINT8)SI5351_ADDR, (UINT8)SI_PLL_RESET, (UINT8) 0x20); //pllA
									// Finally switch on the CLK0 output (0x4F)
									// and set the MultiSynth0 input to be PLL A
	SendI2cbyte(SI5351_ADDR, SI_CLK0_CONTROL, 0x4F | SI_CLK_SRC_PLL_A);

	if (freq2 > 0)
	{
		// calculate clk2
		frequency = (UINT32)((double)freq2 * corr);
		rdiv = SI_R_DIV_1;
		xtalFreq = SI5351_FREQ;
		while (frequency <= 1000000)
		{
			frequency = frequency * 2;
			rdiv += SI_R_DIV_2;
		}
#ifdef _PLLDEBUG_
		DbgPrintf((char *) "\nCLK2 frequency %d \n", frequency);
#endif
		divider = 900000000UL / frequency;// Calculate the division ratio. 900,000,000 is the maximum internal
										  // PLL frequency: 900MHz
		if (divider % 2) divider--;		// Ensure an even integer division ratio

		pllFreq = divider * frequency;	// Calculate the pllFrequency: the divider * desired output frequency
#ifdef _PLLDEBUG_
		DbgPrintf((char *) "pllB Freq  %d \n", pllFreq);
#endif
		mult = pllFreq / xtalFreq;		// Determine the multiplier to get to the required pllFrequency
		l = pllFreq % xtalFreq;			// It has three parts:
		f = (double)l;							// mult is an integer that must be in the range 15..90
		f *= 1048575;					// num and denom are the fractional parts, the numerator and denominator
		f /= xtalFreq;					// each is 20 bits (range 0..1048575)
		num = (UINT32)f;				// the actual multiplier is  mult + num / denom
		denom = 1048575;				// For simplicity we set the denominator to the maximum 1048575

										// Set up PLL B with the calculated multiplication ratio
		setupPLL(SI_SYNTH_PLL_B, mult, num, denom);
		// Set up MultiSynth divider 0, with the calculated divider.
		// The final R division stage can divide by a power of two, from 1..128.
		// represented by constants SI_R_DIV1 to SI_R_DIV128 (see si5351a.h header file)
		// If you want to output frequencies below 1MHz, you have to use the
		// final R division stage

		setupMultisynth(SI_SYNTH_MS_2, divider, rdiv);
		// Reset the PLL. This causes a glitch in the output. For small changes to
		// the parameters, you don't need to reset the PLL, and there is no glitch
		SendI2cbyte((UINT8)SI5351_ADDR, (UINT8)SI_PLL_RESET, (UINT8)0x80); //pllB
		// Finally switch on the CLK2 output (0x4C)
		// and set the MultiSynth0 input to be PLL A
		SendI2cbyte(SI5351_ADDR, SI_CLK2_CONTROL, 0x4C | SI_CLK_SRC_PLL_B);  // select PLLB
		// calculate clk2
	}

}

