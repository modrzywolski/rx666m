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

#include <zlib.h>
#include "RingBuffer.hpp"

class GzFileWriter
{
public:
	GzFileWriter() : outf(NULL)
	{
	}

	void openFile(const char *filename)
	{
		outf = gzopen(filename, "wb1");
		
		if(!outf)
		{
			throw std::runtime_error( "Cannot open file" );
		}

		if(Z_OK != gzsetparams(outf, Z_BEST_SPEED, Z_DEFAULT_STRATEGY))
		{
			throw std::runtime_error( "Cannot open file2" );
		}
	}

	void openStdout()
	{
		outf = gzdopen(STDOUT_FILENO, "wb");
		
		if(!outf)
		{
			throw std::runtime_error( "Cannot open stdout stream" );
		}

	}

	~GzFileWriter()
	{
		if(outf)
		{
			gzclose(outf);
		}
	}

	void writeData(RingBufEntry *buf)
	{
		if(outf)
		{
			char *d=buf->data.data();
			size_t n=buf->bytesAvail;

//			while(n>0)
//			{
//			if(!gzwrite(outf, d, std::min<size_t>(4096, n)))
			if(!gzwrite(outf, buf->data.data(), buf->bytesAvail))
			{
				throw std::runtime_error( "Cannot write to file" );
			}
			/*if(n>=4096)
			{
				n-=4096;
				d+=4096;

			}
			else
			{
				d+=n;
				n=0;
			}
				
			}*/
		}
	}

protected:
	gzFile			outf;
};

