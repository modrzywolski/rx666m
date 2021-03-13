/*
 * The MIT License (MIT)
 * 
 * Copyright (c) 2015 Charles J. Cliffe
 * Copyright (c) 2015-2017 Josh Blum

 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "SoapyRX666m.hpp"
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Time.hpp>
#include <algorithm> //min
#include <climits> //SHRT_MAX
#include <cstring> // memcpy
#include <iostream>


#include <unistd.h> //tmp
		static double last;


std::vector<std::string> SoapyRX666m::getStreamFormats(const int direction, const size_t channel) const {
    std::vector<std::string> formats;

    //formats.push_back(SOAPY_SDR_CS8);
    //formats.push_back(SOAPY_SDR_CS16);
    formats.push_back(SOAPY_SDR_CF32);
	//formats.push_back(SOAPY_SDR_U16);

    return formats;
}

std::string SoapyRX666m::getNativeStreamFormat(const int direction, const size_t channel, double &fullScale) const {
    //check that direction is SOAPY_SDR_RX
     if (direction != SOAPY_SDR_RX) {
         throw std::runtime_error("RTL-SDR is RX only, use SOAPY_SDR_RX");
     }

     fullScale = 32768;
     return SOAPY_SDR_CS16;
}

SoapySDR::ArgInfoList SoapyRX666m::getStreamArgsInfo(const int direction, const size_t channel) const {
    //check that direction is SOAPY_SDR_RX
     if (direction != SOAPY_SDR_RX) {
         throw std::runtime_error("RTL-SDR is RX only, use SOAPY_SDR_RX");
     }

    SoapySDR::ArgInfoList streamArgs;

    SoapySDR::ArgInfo bufflenArg;
    bufflenArg.key = "bufflen";
    bufflenArg.value = std::to_string(DEFAULT_BUFFER_LENGTH);
    bufflenArg.name = "Buffer Size";
    bufflenArg.description = "Number of bytes per buffer, multiples of 512 only.";
    bufflenArg.units = "bytes";
    bufflenArg.type = SoapySDR::ArgInfo::INT;

    streamArgs.push_back(bufflenArg);

    SoapySDR::ArgInfo buffersArg;
    buffersArg.key = "buffers";
    buffersArg.value = std::to_string(DEFAULT_NUM_BUFFERS);
    buffersArg.name = "Ring buffers";
    buffersArg.description = "Number of buffers in the ring.";
    buffersArg.units = "buffers";
    buffersArg.type = SoapySDR::ArgInfo::INT;

    streamArgs.push_back(buffersArg);

    SoapySDR::ArgInfo asyncbuffsArg;
    asyncbuffsArg.key = "asyncBuffs";
    asyncbuffsArg.value = "0";
    asyncbuffsArg.name = "Async buffers";
    asyncbuffsArg.description = "Number of async usb buffers (advanced).";
    asyncbuffsArg.units = "buffers";
    asyncbuffsArg.type = SoapySDR::ArgInfo::INT;

    streamArgs.push_back(asyncbuffsArg);

    return streamArgs;
}

/*******************************************************************
 * Stream API
 ******************************************************************/

SoapySDR::Stream *SoapyRX666m::setupStream(
        const int direction,
        const std::string &format,
        const std::vector<size_t> &channels,
        const SoapySDR::Kwargs &args)
{
    if (direction != SOAPY_SDR_RX)
    {
        throw std::runtime_error("RX-666m is RX only, use SOAPY_SDR_RX");
    }

    //check the channel configuration
    if (channels.size() > 1 or (channels.size() > 0 and channels.at(0) != 0))
    {
        throw std::runtime_error("setupStream invalid channel selection");
    }

    //check the format
    if (format == SOAPY_SDR_CF32)
    {
        SoapySDR_log(SOAPY_SDR_INFO, "Using format CF32.");
    }
    else
    {
        throw std::runtime_error("setupStream invalid format");
    }

    bufferLength = DEFAULT_BUFFER_LENGTH;
    if (args.count("bufflen") != 0)
    {
        try
        {
            int bufferLength_in = std::stoi(args.at("bufflen"));
            if (bufferLength_in > 0)
            {
                bufferLength = bufferLength_in;
            }
        }
        catch (const std::invalid_argument &){}
    }
    SoapySDR_logf(SOAPY_SDR_DEBUG, "RTL-SDR Using buffer length %d", bufferLength);

    numBuffers = DEFAULT_NUM_BUFFERS;
    if (args.count("buffers") != 0)
    {
        try
        {
            int numBuffers_in = std::stoi(args.at("buffers"));
            if (numBuffers_in > 0)
            {
                numBuffers = numBuffers_in;
            }
        }
        catch (const std::invalid_argument &){}
    }
    SoapySDR_logf(SOAPY_SDR_DEBUG, "RTL-SDR Using %d buffers", numBuffers);

    asyncBuffs = 0;
    if (args.count("asyncBuffs") != 0)
    {
        try
        {
            int asyncBuffs_in = std::stoi(args.at("asyncBuffs"));
            if (asyncBuffs_in > 0)
            {
                asyncBuffs = asyncBuffs_in;
            }
        }
        catch (const std::invalid_argument &){}
    }

    //clear async fifo counts
    _buf_tail = 0;
    _buf_count = 0;
    _buf_head = 0;

    //allocate buffers
    _buffs.resize(numBuffers);
    for (auto &buff : _buffs) buff.data.reserve(bufferLength);
    for (auto &buff : _buffs) buff.data.resize(bufferLength);
	for (auto &buff : _buffs) std::fill(buff.data.begin(), buff.data.end(), 0);

    return (SoapySDR::Stream *) this;
}

void SoapyRX666m::closeStream(SoapySDR::Stream *stream)
{
    //this->deactivateStream(stream, 0, 0);
    _buffs.clear();

	driver.DeactivateReader();
}

size_t SoapyRX666m::getStreamMTU(SoapySDR::Stream *stream) const
{
    return bufferLength / BYTES_PER_SAMPLE;
}

int SoapyRX666m::activateStream(
        SoapySDR::Stream *stream,
        const int flags,
        const long long timeNs,
        const size_t numElems)
{
	std::cerr << "activateStream" << std::endl;

    if (flags != 0) return SOAPY_SDR_NOT_SUPPORTED;

	driver.Init();
	driver.ActivateReader();
	driver.SetAD8331Gain(HFGain);
	driver.SetDCGain(UHFGain);
	driver.SetAttn(HFAttn);
	driver.SetDCFreq(centerFrequency);

    return 0;
}

int SoapyRX666m::deactivateStream(SoapySDR::Stream *stream, const int flags, const long long timeNs)
{
	std::cerr << "deactivateStream" << std::endl;

#if 0
    if (flags != 0) return SOAPY_SDR_NOT_SUPPORTED;


    if (_rx_async_thread.joinable())
    {
        streamDeactivating = true;
        _rx_async_thread.join();
    }
#endif
    return 0;
}

void SoapyRX666m::convertSamples( void *out_buff, void *in_buff, size_t st, size_t n)
{
	float *ftarget = reinterpret_cast<float*>(out_buff);
	uint16_t *ptr=reinterpret_cast<uint16_t*>(in_buff);

	for (size_t i = 0; i < n; i++)
	{
		if(0)
		{
			//ftarget[st + i * 2] = nco2()/2 + nco3()/2;
			ftarget[(st + i) * 2] = 0.444; //nco4()/2;
			ftarget[(st + i) * 2 + 1] = 0.0;
		}
		else if(0)
		{
			ftarget[(st + i) * 2] = float(rand()%65536lu)/65536lu;
			ftarget[(st + i) * 2 + 1] = 0.0;
		}
		else
		{
			//ftarget[st + i * 2] = swapByteOrder(ptr[i]);
			ftarget[(st + i) * 2] = float(ptr[i])/65536lu;
			ftarget[(st + i) * 2 + 1] = 0.0;
		}
	}
}

void checkSamples( void *out_buff, size_t st, size_t n)
{
	float *ftarget = reinterpret_cast<float*>(out_buff);

	for (size_t i = 0; i < n; i++)
	{
		if(1)
		{
			//ftarget[st + i * 2] = nco2()/2 + nco3()/2;
			if(fabs(ftarget[st + i * 2] - 0.444) > 0.001)
				fprintf(stderr, "I does not match %g\n", ftarget[st + i * 2]);
			if(fabs(ftarget[st + i * 2 + 1]) > 0.001)
				fprintf(stderr, "Q does not match: %g\n", ftarget[st + i * 2 + 1]);
		}
	}
}


int SoapyRX666m::readStream(
        SoapySDR::Stream *stream,
        void * const *buffs,
        size_t numElems,
        int &flags,
        long long &timeNs,
        const long timeoutUs)
{
	size_t returnedElems = 0, n;

	//flags = 0;

    //this is the user's buffer for channel 0
    void *buff0 = buffs[0];

	//memset(buff0, 0, 2*4*numElems);
	//numElems =  2048 ; 
//	std::min<size_t>(numElems, 2048);
	//printf("2) _buf_tail=%04ld, _buf_head=%04ld, _buf_count=%04ld, bufferedElems=%04ld, numElems=%04ld\n", _buf_tail, _buf_head, _buf_count.load(), bufferedElems, numElems);

	if(0) if(rand() % 20 == 0)
	{
		std::this_thread::sleep_for(std::chrono::microseconds(1000000 + rand()%100));
		fprintf(stderr, "sleep\n");
	}

	//convertSamples( buff0, NULL, 0, numElems);
	//return numElems;

	auto buf = driver.getRingBuffer().getRdBuf(timeoutUs);

	if(!buf)
		return SOAPY_SDR_TIMEOUT;

	if( rand() % 101 > 90)
	{
	//	buf->bytesRead+=2;
	}
	int x=0;
//	int aa[1024]={0};
	while(returnedElems < numElems)
	{
		if(buf->bytesRead >= buf->bytesAvail)
		{
			driver.getRingBuffer().free(buf);
			buf = driver.getRingBuffer().getRdBuf(timeoutUs);
			if(!buf)
				return SOAPY_SDR_TIMEOUT;
		}

		n = std::min<size_t>((buf->bytesAvail-buf->bytesRead)/2, (numElems-returnedElems));
		if(0)
		{
		fprintf(stderr, "[%06x,%06x,%06x,%06x,%06x]", (unsigned int)buf->bytesRead, (unsigned int)buf->bytesAvail, (unsigned int)numElems, (unsigned int)returnedElems, (unsigned int)n);
		for(int x=0; x<std::min<size_t>(64, n/2); x+=2)
			fprintf(stderr, "%02x%02x ", (unsigned int)(unsigned char)buf->data.data()[buf->bytesRead+x], (unsigned int)(unsigned char)buf->data.data()[buf->bytesRead+x+1]);
		fprintf(stderr, "\n");
		}
		convertSamples( buff0, &buf->data.data()[buf->bytesRead], returnedElems, n);
		buf->bytesRead += n*2;
		returnedElems += n;
		//aa[x]=n;
		x++;
	}

	if(buf->bytesRead >= buf->bytesAvail)
	{
		driver.getRingBuffer().free(buf);
	}

#if 0
	if(x>1) fprintf(stderr, "%ld completed in %d (%d,%d,%d)\n", numElems, x, aa[0], aa[1], aa[2]);
#endif

	//printf("3) _buf_tail=%04ld, _buf_head=%04ld, _buf_count=%04ld, bufferedElems=%04ld, returnedElems=%04ld, numElems=%04ld\n", _buf_tail, _buf_head, _buf_count.load(), bufferedElems, returnedElems, numElems);
	//driver.getRingBuffer().stats();
	//fprintf(stderr, "returnedElems=%04ld, numElems=%04ld\n", returnedElems, numElems);

	if(0)if(returnedElems>0)
	{
		double current = ((float*)buff0)[0];

		if((current - last)/last > 0.02)
		{
			fprintf(stderr, "diff %g vs %g\n", current, last);


			fprintf(stderr, "diff %g vs %g vs %g\n", ((float*)buff0)[0], ((float*)buff0)[2*(returnedElems-1)], last);

			fprintf(stderr, "returnedElems=%04ld, numElems=%04ld n=%04ld x=%04d\n", returnedElems, numElems, n, x);
		}

		last = ((float*)buff0)[2*(returnedElems-1)];
	}

	if(returnedElems != numElems)
		fprintf(stderr, "size mistmatch\n");

	if(driver.getRingBuffer().statsCount() > 1)
		flags |= SOAPY_SDR_MORE_FRAGMENTS;

	//checkSamples( buff0, 0, returnedElems);

    return returnedElems;
}

/*******************************************************************
 * Direct buffer access API
 ******************************************************************/

size_t SoapyRX666m::getNumDirectAccessBuffers(SoapySDR::Stream *stream)
{
    return 0; //_buffs.size();
}

int SoapyRX666m::getDirectAccessBufferAddrs(SoapySDR::Stream *stream, const size_t handle, void **buffs)
{
 //   buffs[0] = (void *)_buffs[handle].data.data();
    return SOAPY_SDR_NOT_SUPPORTED;
}

int SoapyRX666m::acquireReadBuffer(
    SoapySDR::Stream *stream,
    size_t &handle,
    const void **buffs,
    int &flags,
    long long &timeNs,
    const long timeoutUs)
{
	return SOAPY_SDR_NOT_SUPPORTED;
}

void SoapyRX666m::releaseReadBuffer(
    SoapySDR::Stream *stream,
    const size_t handle)
{
}

