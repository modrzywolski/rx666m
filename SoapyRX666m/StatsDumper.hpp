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

class StatsDumper
{
public:
	StatsDumper(LowLevel &_driver) : driver(_driver), stop(false), showStats(false)
	{
		threadHandle = std::thread(&StatsDumper::run, this);
	}
	~StatsDumper()
	{
		if(threadHandle.joinable())
		{
			stop = true;
			threadHandle.join();
		}
	}

	void run()
	{
		while(!stop)
		{
			if(showStats)
				driver.DumpReaderStats();
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}
	}

	void show( bool _showStats=true )
	{
		showStats = _showStats;
	}

protected:

	LowLevel&	driver;
	bool		stop;
	bool		showStats;
    std::thread	threadHandle;
};


