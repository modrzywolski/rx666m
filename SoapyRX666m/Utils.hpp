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

#include <boost/format.hpp>


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

