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

#include <memory.h>
#include <sys/ioctl.h>

#include "SoapyRX666m.hpp"
#include <SoapySDR/Time.hpp>
#include <algorithm>
#include "../common/rx666m_ioctl.h"


SoapyRX666m::SoapyRX666m(const SoapySDR::Kwargs &args):
    sampleRate(64000000),
    centerFrequency(0),
    ppm(0),
    ticks(false),
	HFAttn(0.0),
	HFGain(0.0),
	UHFGain(0.0),
	AGCEnabled(false)
{
    if (args.count("label") != 0) SoapySDR_logf(SOAPY_SDR_INFO, "Opening %s...", args.at("label").c_str());

	nco3.setfreq(FNCO::fclk/4);
	nco4.setfreq(FNCO::fclk/16);
}

SoapyRX666m::~SoapyRX666m(void)
{
}

/*******************************************************************
 * Identification API
 ******************************************************************/

std::string SoapyRX666m::getDriverKey(void) const
{
    return "RX666m";
}

std::string SoapyRX666m::getHardwareKey(void) const
{
    return "OTHER";
}

SoapySDR::Kwargs SoapyRX666m::getHardwareInfo(void) const
{
    SoapySDR::Kwargs args;

    args["origin"] = "emk6@wp.pl";

    return args;
}

/*******************************************************************
 * Channels API
 ******************************************************************/

size_t SoapyRX666m::getNumChannels(const int dir) const
{
	return (dir == SOAPY_SDR_RX) ? 1 : 0;
}

bool SoapyRX666m::getFullDuplex(const int direction, const size_t channel) const
{
    return false;
}

/*******************************************************************
 * Antenna API
 ******************************************************************/

std::vector<std::string> SoapyRX666m::listAntennas(const int direction, const size_t channel) const
{
    std::vector<std::string> antennas;
    antennas.push_back("HF");
	antennas.push_back("UHF");
    return antennas;
}

void SoapyRX666m::setAntenna(const int direction, const size_t channel, const std::string &name)
{
    if (direction != SOAPY_SDR_RX)
    {
        throw std::runtime_error("setAntena failed: RX only supported");
    }
}

std::string SoapyRX666m::getAntenna(const int direction, const size_t channel) const
{
    return "HF";
}

/*******************************************************************
 * Frontend corrections API
 ******************************************************************/

bool SoapyRX666m::hasDCOffsetMode(const int direction, const size_t channel) const
{
    return false;
}

bool SoapyRX666m::hasFrequencyCorrection(const int direction, const size_t channel) const
{
    return false;
}

void SoapyRX666m::setFrequencyCorrection(const int direction, const size_t channel, const double value)
{
    ppm = int(value);
}

double SoapyRX666m::getFrequencyCorrection(const int direction, const size_t channel) const
{
    return double(ppm);
}

/*******************************************************************
 * Gain API
 ******************************************************************/

std::vector<std::string> SoapyRX666m::listGains(const int direction, const size_t channel) const
{
    std::vector<std::string> results;

	results.push_back("HFVGA");
	results.push_back("HFATTN");
	results.push_back("UHFVGA");

    return results;
}

bool SoapyRX666m::hasGainMode(const int direction, const size_t channel) const
{
    return true;
}

void SoapyRX666m::setGainMode(const int direction, const size_t channel, const bool automatic)
{
	AGCEnabled = automatic;
	driver.SetAGC(automatic);
}

bool SoapyRX666m::getGainMode(const int direction, const size_t channel) const
{
    return false;
}

void SoapyRX666m::setGain(const int direction, const size_t channel, const std::string &name, const double value)
{
	if(!AGCEnabled)
	{
		if(name == "HFVGA")
		{
			HFGain = value;
			driver.SetAD8331Gain(HFGain);
		}
		else if(name == "UHFVGA")
		{
			UHFGain = value;
			driver.SetDCGain(UHFGain);
		}
		else if(name == "HFATTN")
		{
			HFAttn = value;
			driver.SetAttn(HFAttn);
		}
	}
}

double SoapyRX666m::getGain(const int direction, const size_t channel, const std::string &name) const
{
	if(name == "HFVGA")
	{
		return HFGain;
	}
	else if(name == "UHFVGA")
	{
		return UHFGain;
	}
	else if(name == "HFATTN")
	{
		return HFAttn;
	}

    return 0;
}

SoapySDR::Range SoapyRX666m::getGainRange(const int direction, const size_t channel, const std::string &name) const
{
	if(name == "HFVGA")
	{
		return SoapySDR::Range(-3, 45);
	}
	else if(name == "HFATTN")
	{
		return SoapySDR::Range(-20, 0);
    }
	else if(name == "UHFVGA")
	{
		return SoapySDR::Range(0, 60);
	}

    return SoapySDR::Range(0, 0);
}

/*******************************************************************
 * Frequency API
 ******************************************************************/

void SoapyRX666m::setFrequency(
        const int direction,
        const size_t channel,
        const std::string &name,
        const double frequency,
        const SoapySDR::Kwargs &args)
{
    if (name == "RF")
    {
        centerFrequency = (uint32_t) frequency;
		driver.SetDCFreq(frequency);
    }

    if (name == "CORR")
    {
        ppm = (int) frequency;
    }
}

double SoapyRX666m::getFrequency(const int direction, const size_t channel, const std::string &name) const
{
    if (name == "RF")
    {
        return (double) centerFrequency;
    }

    if (name == "CORR")
    {
        return (double) ppm;
    }

    return 0;
}

std::vector<std::string> SoapyRX666m::listFrequencies(const int direction, const size_t channel) const
{
    std::vector<std::string> names;
    names.push_back("RF");
    names.push_back("CORR");
    return names;
}

SoapySDR::RangeList SoapyRX666m::getFrequencyRange( const int direction, const size_t channel, const std::string &name) const
{
    SoapySDR::RangeList results;

    if (name == "RF")
    {
        results.push_back(SoapySDR::Range(0, 32000000));
        results.push_back(SoapySDR::Range(   32100000lu, 1800000000lu));
    }
    if (name == "CORR")
    {
        results.push_back(SoapySDR::Range(-1000, 1000));
    }
    return results;
}

SoapySDR::ArgInfoList SoapyRX666m::getFrequencyArgsInfo(const int direction, const size_t channel) const
{
    SoapySDR::ArgInfoList freqArgs;
	
    return freqArgs;
}

/*******************************************************************
 * Sample Rate API
 ******************************************************************/

void SoapyRX666m::setSampleRate(const int direction, const size_t channel, const double rate)
{
    long long ns = SoapySDR::ticksToTimeNs(ticks, sampleRate);
    sampleRate = rate;

    SoapySDR_logf(SOAPY_SDR_DEBUG, "Setting sample rate: %d", sampleRate);

    ticks = SoapySDR::timeNsToTicks(ns, sampleRate);
}

double SoapyRX666m::getSampleRate(const int direction, const size_t channel) const
{
    return sampleRate;
}

SoapySDR::RangeList SoapyRX666m::getSampleRateRange(const int direction, const size_t channel) const
{
	SoapySDR::RangeList sr;

	sr.push_back( SoapySDR::Range(4000000, 64000000, 1000000) );

	return sr;
}

std::vector<double> SoapyRX666m::listSampleRates(const int direction, const size_t channel) const
{
    std::vector<double> results;

    results.push_back(64000000);

    return results;
}

void SoapyRX666m::setBandwidth(const int direction, const size_t channel, const double bw)
{
    //SoapySDR::Device::setBandwidth(direction, channel, bw);
}

double SoapyRX666m::getBandwidth(const int direction, const size_t channel) const
{
   return 32000000;
}

std::vector<double> SoapyRX666m::listBandwidths(const int direction, const size_t channel) const
{
    std::vector<double> results;

	results.push_back(32000000);



    return results;
}

/*******************************************************************
 * Time API
 ******************************************************************/

std::vector<std::string> SoapyRX666m::listTimeSources(void) const
{
    std::vector<std::string> results;

    results.push_back("sw_ticks");

    return results;
}

std::string SoapyRX666m::getTimeSource(void) const
{
    return "sw_ticks";
}

bool SoapyRX666m::hasHardwareTime(const std::string &what) const
{
    return what == "" || what == "sw_ticks";
}

long long SoapyRX666m::getHardwareTime(const std::string &what) const
{
    return SoapySDR::ticksToTimeNs(ticks, sampleRate);
}

void SoapyRX666m::setHardwareTime(const long long timeNs, const std::string &what)
{
    ticks = SoapySDR::timeNsToTicks(timeNs, sampleRate);
}

/*******************************************************************
 * Settings API
 ******************************************************************/

SoapySDR::ArgInfoList SoapyRX666m::getSettingInfo(void) const
{
    SoapySDR::ArgInfoList setArgs;

    return setArgs;
}

void SoapyRX666m::writeSetting(const std::string &key, const std::string &value)
{
}

std::string SoapyRX666m::readSetting(const std::string &key) const
{
    SoapySDR_logf(SOAPY_SDR_WARNING, "Unknown setting '%s'", key.c_str());
    return "";
}

