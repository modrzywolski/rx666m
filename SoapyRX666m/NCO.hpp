#pragma once

#include <stdint.h>
#include <math.h>

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


