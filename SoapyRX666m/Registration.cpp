/*
 * The MIT License (MIT)
 * 
 * Copyright (c) 2015 Charles J. Cliffe

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
