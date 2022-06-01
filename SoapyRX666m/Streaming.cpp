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

#include "SoapyRX666m.hpp"
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Time.hpp>
#include <algorithm>
#include <iostream>


std::vector<std::string> SoapyRX666m::getStreamFormats(const int direction, const size_t channel) const
{
    std::vector<std::string> formats;

    formats.push_back(SOAPY_SDR_CF32);

    return formats;
}

std::string SoapyRX666m::getNativeStreamFormat(const int direction, const size_t channel, double &fullScale) const
{
     if (direction != SOAPY_SDR_RX)
	 {
         throw std::runtime_error("RX666m is RX only");
     }

     fullScale = 32768;
     return SOAPY_SDR_CS16;
}

SoapySDR::ArgInfoList SoapyRX666m::getStreamArgsInfo(const int direction, const size_t channel) const 
{
     if (direction != SOAPY_SDR_RX)
	 {
         throw std::runtime_error("RX666m is RX only");
     }

    SoapySDR::ArgInfoList streamArgs;

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
        throw std::runtime_error("RX-666m is RX only");
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

    return (SoapySDR::Stream *) this;
}

void SoapyRX666m::closeStream(SoapySDR::Stream *stream)
{
	SoapySDR_logf(SOAPY_SDR_INFO, "Close stream");
	driver.DeactivateReader();
}

size_t SoapyRX666m::getStreamMTU(SoapySDR::Stream *stream) const
{
    return ring_chunk_size / 2; // two bytes per sample
}

int SoapyRX666m::activateStream( SoapySDR::Stream *stream, const int flags, const long long timeNs, const size_t numElems)
{
	SoapySDR_logf(SOAPY_SDR_INFO, "Activate stream");

    if (flags != 0)
		return SOAPY_SDR_NOT_SUPPORTED;

	driver.Init();
	driver.ActivateReader();
	driver.SetAD8331Gain(HFGain);
	driver.SetDCGain(UHFGain);
	driver.SetAttn2(HFAttn1);
	driver.SetAttn1(HFAttn2);
	driver.SetDCFreq(centerFrequency);

    return 0;
}

int SoapyRX666m::deactivateStream(SoapySDR::Stream *stream, const int flags, const long long timeNs)
{
	SoapySDR_logf(SOAPY_SDR_INFO, "Deactivate stream");

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
			ftarget[st + i * 2] = nco2()/2 + nco3()/2;
			ftarget[(st + i) * 2 + 1] = 0.0;
		}
		else
		{
			ftarget[(st + i) * 2] = float(ptr[i])/65536lu;
			ftarget[(st + i) * 2 + 1] = 0.0;
		}
	}
}

int SoapyRX666m::readStream( SoapySDR::Stream *stream, void * const *buffs, size_t numElems, int &flags, long long &timeNs, const long timeoutUs)
{
	size_t returnedElems = 0, n;
    void *buff0 = buffs[0];
	auto buf = driver.getRingBuffer().getRdBuf(timeoutUs);

	if(!buf)
		return SOAPY_SDR_TIMEOUT;

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
		convertSamples( buff0, &buf->data.data()[buf->bytesRead], returnedElems, n);
		buf->bytesRead += n*2;
		returnedElems += n;
	}

	if(buf->bytesRead >= buf->bytesAvail)
	{
		driver.getRingBuffer().free(buf);
	}

	if(driver.getRingBuffer().statsCount() > 1)
		flags |= SOAPY_SDR_MORE_FRAGMENTS;

    return returnedElems;
}

/*******************************************************************
 * Direct buffer access API
 ******************************************************************/

size_t SoapyRX666m::getNumDirectAccessBuffers(SoapySDR::Stream *stream)
{
    return 0;
}

int SoapyRX666m::getDirectAccessBufferAddrs(SoapySDR::Stream *stream, const size_t handle, void **buffs)
{
    return SOAPY_SDR_NOT_SUPPORTED;
}

int SoapyRX666m::acquireReadBuffer( SoapySDR::Stream *stream, size_t &handle, const void **buffs, int &flags, long long &timeNs, const long timeoutUs)
{
	return SOAPY_SDR_NOT_SUPPORTED;
}

void SoapyRX666m::releaseReadBuffer( SoapySDR::Stream *stream, const size_t handle)
{
}

