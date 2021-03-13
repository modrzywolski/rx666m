#pragma once

#include <stdint.h>
#include <math.h>
#include <iostream>
#include <algorithm>
#include <thread>

const uint32_t clippingLevel = 0xf800;
const size_t   clippingRateThreshold = 0;
const double lowThreshold = 0.8;

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

