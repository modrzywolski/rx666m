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
#include <SoapySDR/Registry.hpp>
#include <mutex>
#include <map>


static std::vector<SoapySDR::Kwargs> findRX666m(const SoapySDR::Kwargs &args)
{
    std::vector<SoapySDR::Kwargs> results;

	SoapySDR::Kwargs devInfo;
	devInfo["label"] = "rx666m";
	devInfo["serial"] = "1";
    devInfo["device_id"] = "1";

	results.push_back(devInfo);

    return results;
}

static SoapySDR::Device *createRX666m(const SoapySDR::Kwargs &args)
{
    return new SoapyRX666m(args);
}

static SoapySDR::Registry registerRX666m("rx666m", &findRX666m, &createRX666m, SOAPY_SDR_ABI_VERSION);
