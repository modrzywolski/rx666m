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

const size_t	packet_size = 4096;
const size_t	num_packets = 256;
const double AD8331_MinGain = -3.0;
const double AD8331_MaxGain = 45.0;
const double ATTN1_Gain = -10.0;
const double ATTN2_Gain = -20.0;
const uint32_t clippingLevel = 0xf800;
const size_t   clippingRateThreshold = 0;
const double lowThreshold = 0.8;
const double totalGainMax = AD8331_MaxGain;
const double totalGainMin = ATTN2_Gain + AD8331_MinGain;

typedef uint32_t UINT32; //tmp
typedef uint8_t UINT8; //tmp


inline std::string human_readable_bytes(double a)
{
	std::array<const char*, 6> prefixes = {"B", "kiB", "MiB", "GiB", "TiB", "PiB"};
	size_t i = 0;

	while(i+1 < prefixes.size() && a > 1024)
	{
		a/=1024;
		i++;
	}

	return (boost::format("%.2f %s") % a % prefixes[i]).str();
}

inline std::string human_readable_duration(size_t a)
{
	std::array<const char*, 3> prefixes = {"h", "m", "s"};
	std::array<size_t, 3> frac;

	frac[0]=a / 3600; a -= frac[0] * 3600;
	frac[1]=a / 60; a -= frac[1] * 60;
	frac[2]=a;

	return (
			boost::format("%d%s %d%s %d%s") 
					% frac[0] % prefixes[0]
					% frac[1] % prefixes[1]
					% frac[2] % prefixes[2]
		).str();
}


class NCO
{
public:
	const uint64_t fclk = 64000000ULL;
	const uint64_t freq =  fclk/16;
	NCO(): dphase( freq * (1ULL<<32) / fclk ), phase(0)
	{
	}

	uint16_t	operator()()
	{
		double arg = 2.0*M_PI*(phase >> 16)/(1lu<<16);
		int32_t out = (1lu<<15) + (::sin(arg) * ((1lu<<15)-1));

		//printf("[arg=%lf, out=%08X]\n", arg, out);
		phase += dphase;

		return out;
	}

protected:
	uint32_t	dphase;
	uint32_t	phase;
};

class FNCO
{
public:
	const static uint64_t fclk = 64000000ULL;
	const static uint64_t freq =  fclk/16;
	FNCO(): dphase( freq * (1ULL<<32) / fclk ), phase(0)
	{
	}

	void setfreq(uint64_t f)
	{
		dphase = f * (1ULL<<32) / fclk;
	}

	double	operator()()
	{
		double arg = 2.0*M_PI*(phase >> 16)/(1lu<<16);

		phase += dphase;

		return ::sin(arg);
	}

protected:
	uint32_t	dphase;
	uint32_t	phase;
};


class LPF
{
public:
	LPF() : state(0), alpha(0.9)
	{
	}
	
	double feed(double value)
	{
		value = alpha * value + (1-alpha) * state;
		state = value;

		return value;
	}

protected:
	double 	state;
	double 	alpha;
};

class AGC
{
public:
	const double decay_factor = 0.90;

	typedef void (*GainSetFunc)(double gain);

	AGC() : peakValue(0), stopWorker(false)
	{
		attackRate = 1;
		decayRate = 2;
		gain = 0;
		ignoreTransients = 0;
		agcEnabled = false;
		fastClippingEvents = 0;
		slowClippingEvents = 0;

		worker = std::thread(&AGC::peakDecay, this);
	}

	~AGC()
	{
		if(worker.joinable())
		{
			stopWorker=true;
			worker.join();
		}
	}

	void SetAGC(bool on)
	{
		agcEnabled = on;
	}

	template<class GainSetFunc>
	inline void feed(uint32_t value, GainSetFunc gainSet)
	{
		if(ignoreTransients)
		{
			ignoreTransients--;
			return;
		}

		if(value > clippingLevel)
		{
			++fastClippingEvents;
		}

		value = lpf.feed(value);
		if(value > peakValue)
		{
			peakValue = value;
		}

		if(value > clippingLevel)
		{
			++slowClippingEvents;
			++clippingRate;
		}

		if( performDecay.fetch_and(false) )
		{
			double newGain = calcGain(peakValue);

			if(fabs(newGain - gain) >= 0.1)
			{
				ignoreTransients = 1;
				if(agcEnabled)
				{
					//std::cerr << "set gain" << std::endl;
					gainSet(gain);
				}

			}
			clippingRate = 0;
			gain = newGain;

			peakValue = static_cast<double>(peakValue) * decay_factor;
		}
	}

	double calcGain(double value)
	{
		double rate, newGain;


		/*if(value > clippingLevel)
		{
			++clippingEvents;
			++clippingRate;
		}*/

		if(clippingRate > clippingRateThreshold)
			rate = -decayRate;
		else if(value < lowThreshold * clippingLevel)
			rate = attackRate;
		else
			rate = 0.0;

		newGain = gain + rate;
		if(newGain > totalGainMax)
			newGain = totalGainMax;
		else if(newGain < totalGainMin)
			newGain = totalGainMin;

#if 1
		if(agcEnabled)
		{
			std::cerr << "Level: " << std::hex << (int)getPeakValue() << (isFastClipping() ? " FASTCLIPPING" : "") << (isSlowClipping() ? " SLOWCLIPPING" : "") << std::endl;

			std::cerr << "NewGain: " << newGain << std::endl;
		}
#endif

		return newGain;
	}

	void peakDecay()
	{
		while(!stopWorker)
		{
			performDecay = true;


			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
	}

	uint16_t getPeakValue()
	{
		return peakValue;
	}

	bool isFastClipping()
	{
		return std::atomic_exchange(&fastClippingEvents, size_t(0)) > 0;
	}

	bool isSlowClipping()
	{
		return std::atomic_exchange(&slowClippingEvents, size_t(0)) > 0;
	}

	double getGain()
	{
		return gain;
	}

protected:
	std::atomic<uint32_t> peakValue;
	std::thread		worker;
	bool			stopWorker;
	std::atomic<int>	performDecay;
	std::atomic<size_t>	clippingEvents;
	std::atomic<size_t> fastClippingEvents;	
	std::atomic<size_t>	slowClippingEvents;
	std::atomic<size_t> clippingRate;

	LPF		lpf;
	double	attackRate;
	double	decayRate;
	double  gain;
	size_t	ignoreTransients;
	bool	agcEnabled;
};

#if 0
class ReaderContext
{
public:
	typedef std::tuple<uint8_t*, size_t> RawBuffer;

	size_t roundup(size_t r)
	{
		return r / packet_size * packet_size;
	}
	void reset()
	{
		peak_value = 0;
		lost = 0;
		total = 0;
		rdPtr = 0;
		wrPtr = 0;
		count = 0;
		buffer.clear();
		buffer.resize( roundup(32*1024*1024) );
	}

	RawBuffer getFirstFree()
	{
		std::lock_guard<std::mutex> lg(data_guard);

		if(count + num_packets*packet_size > buffer.size())
		{
			lost += packet_size;
			printf("getFirstFree: overflow\n");
			return RawBuffer(NULL, 0);
		}

		//printf("getFirstFree: %p, %ld\n", &buffer.data()[wrPtr], std::min(buffer.size()-count, num_packets*packet_size));

		return RawBuffer(&buffer.data()[wrPtr], std::min(buffer.size()-count, num_packets*packet_size));
	}

	void commitPacket(const RawBuffer &buf)
	{
		std::lock_guard<std::mutex> lg(data_guard);

		if(std::get<0>(buf) != &buffer.data()[wrPtr])
		{
			printf("commitPacket: err 1 %p, %ld\n", std::get<0>(buf), std::get<1>(buf));

			return;
		}

		if(count + std::get<1>(buf) > buffer.size())
		{
			printf("commitPacket: err 2 %p, %ld\n", std::get<0>(buf), std::get<1>(buf));

			return;
		}


		wrPtr += std::get<1>(buf);
		wrPtr %= buffer.size();
		count += std::get<1>(buf);
		total += std::get<1>(buf);

		//printf("count=%ld size=%ld rdPtr=%ld wrPtr=%ld\n", count, buffer.size(), rdPtr, wrPtr);
		//printf("commitPacket: %p, %ld\n", std::get<0>(buf), std::get<1>(buf));

		dataAvailNotifier.notify_one();
	}

	RawBuffer getAvailData(size_t maxsize)
	{
		std::unique_lock<std::mutex> lock(data_guard);

		if(!count)
		{
			dataAvailNotifier.wait_for(lock, std::chrono::milliseconds(timeoutMs), [this]{return count != 0;});
		}

		//printf("count=%ld size=%ld rdPtr=%ld\n", count, buffer.size(), rdPtr);
		maxsize = std::min(maxsize, count);
		maxsize = std::min(maxsize, buffer.size()-rdPtr);

		//printf("getAvailData: %p, %ld\n", &buffer.data()[rdPtr], maxsize);

		return RawBuffer(&buffer.data()[rdPtr], maxsize);
	}

	void free(const RawBuffer &buf)
	{
		std::lock_guard<std::mutex> lg(data_guard);

		if( std::get<0>(buf) == &buffer.data()[rdPtr] && std::get<1>(buf) <= count )
		{
			//printf("free: %p, %ld\n", std::get<0>(buf), std::get<1>(buf));

			rdPtr += std::get<1>(buf);
			rdPtr %= buffer.size();
			count -= std::get<1>(buf);
		}
		else
		{
			printf("free: err %p, %ld\n", std::get<0>(buf), std::get<1>(buf));
		}
	}

	std::mutex				data_guard;
    std::condition_variable	dataAvailNotifier;
	size_t					rdPtr;
	size_t					wrPtr;
	size_t					count;

	std::vector<uint8_t>	buffer;
	size_t					total;
	size_t					lost;
	uint16_t				peak_value;
};
#endif
class LowLevel
{
public:
	LowLevel();
	~LowLevel();

	//Init interface
	void Init();

	//Control Interface
	void SetHFGain(uint16_t gain);
	void SetHFGain2(double gain);
	void SetAD8331Gain(double gain);
	uint16_t GetHFGain();
	void SetAttn(double gain);
	void SetDCGain(uint16_t gain);
	void SetDCFreq(uint32_t freq);
	void SetAGC(bool on);
	int Read(void *buffer, size_t cnt);

	bool isVGAPresent();
	bool isDCPresent();
	bool isRunning();
	void SetMode();

	//Reader stuff
	int ActivateReader();
	void DeactivateReader();
	void DumpReaderStats();
	RingBuf& getRingBuffer() { return ringBuffer; }

#if 0
	ReaderContext::RawBuffer getAvailData(size_t maxsize)
	{
		return ReaderContextData.getAvailData(maxsize);
	}

	void freeData(ReaderContext::RawBuffer &buf)
	{
		ReaderContextData.free(buf);
	}
#endif
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
		}

		return 0;
	}

protected:

	void OpenDev();
	void CloseDev();
	void LoadFirmware();
	bool IsBootloaderRunning();

	int FX3Download(const char *imagefile);
	int FX3ReadImage( const char *filename, unsigned char *buf, int *romsize, int *filesize);
	void FX3Start();

	int SendFW(unsigned char *firmware, uint32_t address, int32_t len);
	int SendI2cbyte(uint32_t address, uint8_t reg, uint8_t value);
	//int SendI2cbytes(uint32_t address, uint8_t *values, size_t len);
	int SendI2cbytes(uint32_t address, uint8_t reg, const uint8_t *values, size_t len);
	int RecvI2cbytes(uint32_t address, uint8_t reg, uint8_t *values, size_t len);
	void GpioWrite(uint8_t gpio);
	void InitClk();

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

//clk handling
void Si5351init();
void si5351aSetFrequency(UINT32 freq, UINT32 freq2);
void si5351aOutputOff(UINT8 clk);
void setupMultisynth( UINT8 synth, UINT32 divider, UINT8 rDiv);
void setupPLL(UINT8 pll, UINT8 mult, UINT32 num, UINT32 denom);

	//Internal Reader stuff
	void ReadThread();

	//atributes
	int	devHandle;
	int VGAPresent;
	int DCPresent;
	bool DCEnabled;
	r82xx_priv tuner_data;

	uint16_t	adcPeakLevel;
	bool StopReader;
	std::thread ReadThreadHandle;
	//ReaderContext	ReaderContextData;
	RingBuf			ringBuffer;
	boost::posix_time::ptime	missionStart;
	uint16_t	avgValue;
	AGC	peakDetector;
	NCO nco;
};


