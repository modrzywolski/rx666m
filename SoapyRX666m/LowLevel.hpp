/*
 * SoapySDR RX666m driver
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

#pragma once

#include <stdint.h>
#include <cmath>
#include <algorithm>
#include <thread>
#include <vector>
#include <tuple>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <iostream>
#include <boost/format.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "tuner_r82xx.h"
#include "RingBuffer.hpp"
#include "NCO.hpp"
#include "Utils.hpp"

#if AGC
	#include "AGC.hpp"
#endif

const size_t	packet_size = 4096;
const double AD8331_MinGain = -3.0;
const double AD8331_MaxGain = 45.0;
const double ATTN1_Gain = -31.0;
const double ATTN2_Gain1 = -10.0;
const double ATTN2_Gain2 = -20.0;
const double totalGainMax = AD8331_MaxGain;
const double totalGainMin = ATTN1_Gain + ATTN2_Gain2 + AD8331_MinGain;



class LowLevel
{
public:
	LowLevel();
	~LowLevel();

	//Init interface
	int Init();

	//Control Interface
	void SetAD8331Gain(double gain);
	void SetHFGainDistribute(double gain);
	void SetAttn1(double gain);
	void SetAttn2(double gain);
	void SetDCGain(uint16_t gain);
	void SetDCFreq(uint32_t freq);
	void SetAGC(bool on);
	int Read(void *buffer, size_t cnt);

	bool isVGAPresent();
	bool isATTNPresent();
	bool isDCPresent();
	bool isRunning();
	void SetMode();

	//Reader stuff
	int ActivateReader();
	void DeactivateReader();
	void DumpReaderStats();
	RingBuf& getRingBuffer() { return ringBuffer; }

	template <class T>
	int WriteToFile( T writeData )
	{
		int i;
		for(i=0;i<100;i++)
		{
			auto buf = getRingBuffer().getRdBuf(1000000); //1s
			if(buf)
			{
				writeData(buf);
				getRingBuffer().free(buf);
			}

			if(getRingBuffer().statsLastErr() < 0)
				return -1;
		}

		return 0;
	}

protected:

	int OpenDev();
	void CloseDev();
	int LoadFirmware();
	bool IsBootloaderRunning();

	int FX3Download(const char *imagefile);
	int FX3ReadImage( const char *filename, unsigned char *buf, int *romsize, int *filesize);
	int FX3Start();

	int SendFW(unsigned char *firmware, uint32_t address, int32_t len);
	int SendI2cbyte(uint32_t address, uint8_t reg, uint8_t value);
	int SendI2cbytes(uint32_t address, uint8_t reg, const uint8_t *values, size_t len);
	int RecvI2cbytes(uint32_t address, uint8_t reg, uint8_t *values, size_t len);
	void GpioWrite(uint8_t gpio);
	int InitClk();
	int InitPca9557();

	//tuner handling
	void shadow_store(struct r82xx_priv *priv, uint8_t reg, const uint8_t *val, int len);
	int r82xx_write(struct r82xx_priv *priv, uint8_t reg, const uint8_t *val, unsigned int len);
	int r82xx_write_reg(struct r82xx_priv *priv, uint8_t reg, uint8_t val);
	int r82xx_read_cache_reg(struct r82xx_priv *priv, int reg);
	int r82xx_write_reg_mask(struct r82xx_priv *priv, uint8_t reg, uint8_t val, uint8_t bit_mask);
	uint8_t r82xx_bitrev(uint8_t byte);
	int r82xx_read(struct r82xx_priv *priv, uint8_t reg, uint8_t *val, int len);
	int r82xx_set_mux(struct r82xx_priv *priv, uint32_t freq);
	int r82xx_set_pll(struct r82xx_priv *priv, uint32_t freq);
	int r82xx_sysfreq_sel(struct r82xx_priv *priv, uint32_t freq, uint32_t delsys);
	int r82xx_set_tv_standard(struct r82xx_priv *priv, unsigned bw, uint32_t delsys);
	int r82xx_read_gain(struct r82xx_priv *priv);
	int r82xx_set_gain(struct r82xx_priv *priv, int set_manual_gain, int gain);
	int r82xx_set_bandwidth(struct r82xx_priv *priv, int bw, uint32_t rate);
	int r82xx_set_freq(struct r82xx_priv *priv, uint32_t freq);
	int r82xx_standby(struct r82xx_priv *priv);
	int r82xx_xtal_check(struct r82xx_priv *priv);
	int r82xx_init(struct r82xx_priv *priv);
	int rtlsdr_i2c_write_fn(void *dev, uint8_t addr, uint8_t *buf, int len);
	int rtlsdr_i2c_read_fn(void *dev, uint8_t addr, uint8_t *buf, int len);

	//Internal Reader stuff
	void ReadThread();

	//atributes
	int	devHandle;
	int VGAPresent;
	int ATTNPresent;
	int DCPresent;
	bool DCEnabled;
	r82xx_priv tuner_data;

	bool StopReader;
	std::thread ReadThreadHandle;
	
	RingBuf			ringBuffer;
	boost::posix_time::ptime	missionStart;
#if AGC
	AGC	peakDetector;
#endif
	NCO nco;
};


