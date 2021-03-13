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

