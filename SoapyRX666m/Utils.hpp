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

