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
#include <math.h>

class NCO
{
public:
	const static uint64_t fclk = 64000000ULL;
	const static uint64_t freq =  fclk/16;
	NCO(): dphase( freq * (1ULL<<32) / fclk ), phase(0)
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

