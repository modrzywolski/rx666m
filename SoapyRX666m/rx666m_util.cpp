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

#include <iostream>
#include "LowLevel.hpp"
#include "Spectrum.hpp"
#include "GzFileWriter.hpp"
#include <chrono>
#include <csignal>
#include <cmath>
#include <fstream>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/format.hpp>

bool 		stopApp = false;
double		vgaGain;
double 		attn;
std::string outputFile;

namespace po = boost::program_options;
LowLevel driver;

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

class FileWriter
{
public:
	FileWriter() : outf(NULL), file(NULL) {}

	void openFile(const char *filename)
	{
		std::ofstream *file = new std::ofstream;

		file->open(filename, std::ios::out | std::ios::binary);

		outf = file;
	}

	void openStdout()
	{
		outf = &std::cout;
	}

	~FileWriter()
	{
		if(file)
		{
			if(file->is_open())
				file->close();
			delete file;
		}
	}

	void writeData(RingBufEntry *buf)
	{
		if(outf)
		{
			outf->write(buf->data.data(), buf->bytesAvail);
		}
	}

protected:
	std::ostream 	*outf;
	std::ofstream	*file;
};

void exitHandler( int signum )
{
	stopApp = true;
}

bool is_close(double a, double b, double epsilon = 1e-5)
{ 
	return std::fabs(a - b) < epsilon; 
}

int main(int ac, char *av[])
{
	po::options_description desc("RX666m command line utility application");
	desc.add_options()
		("help", "produce help message")
		("vga_gain", po::value<double>()->default_value(0)->notifier(
            [](double g)
            {
                if (g < AD8331_MinGain || g > AD8331_MaxGain )
					throw po::invalid_option_value( std::to_string(g) );
            }),
			(boost::format("VGA gain, %1%..%2% dB") % AD8331_MinGain % AD8331_MaxGain ).str().c_str() )
		("attn", po::value<double>()->default_value(0)->notifier(
            [](double g)
            {
                if (!is_close(g, ATTN2_Gain) && !is_close(g, ATTN1_Gain) && !is_close(g, 0.0) )
					throw po::invalid_option_value( std::to_string(g) );
            }),
			(boost::format("Attenuator level, valid levels: %1%, %2%, %3%") % ATTN2_Gain % ATTN1_Gain % 0.0 ).str().c_str() )
		("agc", po::bool_switch(), "Enable software AGC")
		("out",	po::value<std::string>(), "Output file for samples recording")
		("gz",	po::bool_switch(), "Compress compres data as .gz file")
		("stdout",	po::bool_switch(), "Output samples to standard output")
		("spectrum",  po::bool_switch(), "Shows spectrum")
	;

	po::variables_map vm;
	po::store(po::parse_command_line(ac, av, desc), vm);
	po::notify(vm);

	if (vm.count("help"))
	{
		std::cerr << desc << "\n";
		return 1;
	}


	Spectrum spec;

	signal(SIGINT, exitHandler);
	signal(SIGTERM, exitHandler); 

	driver.Init();


	driver.SetAD8331Gain( vm["vga_gain"].as< double >() );
	driver.SetAttn( vm["attn"].as< double >() );
	driver.SetAGC( vm["agc"].as< bool >() );

	FileWriter fw;
	GzFileWriter gzfw;

	if(vm["gz"].as< bool >())
	{
		if(vm.count("out"))
			gzfw.openFile(vm["out"].as< std::string >().c_str());
		else if ( vm["stdout"].as< bool >() )
			gzfw.openStdout();
	}
	else
	{
		if(vm.count("out"))
			fw.openFile(vm["out"].as< std::string >().c_str());
		else if (vm["stdout"].as< bool >())
			fw.openStdout();
	}

	driver.ActivateReader();
	StatsDumper dumper(driver);

	if ( vm["spectrum"].as< bool >() )
		printf("\033[2J\n");

	if ( !vm["spectrum"].as< bool >()) // && !vm["stdout"].as< bool >() )
		dumper.show();

	while(!stopApp)
	{
		using namespace std::placeholders;

		if ( vm["spectrum"].as< bool >() )
			driver.WriteToFile( std::bind(&Spectrum::feed, &spec, _1) );
		else if( vm["gz"].as< bool >() )
			driver.WriteToFile( std::bind(&GzFileWriter::writeData, &gzfw, _1) );
		else
			driver.WriteToFile( std::bind(&FileWriter::writeData, &fw, _1) );

		//std::this_thread::sleep_for(std::chrono::milliseconds(100 + rand()%100));
	}

	std::this_thread::sleep_for(std::chrono::milliseconds(100 + rand()%100));

	driver.DumpReaderStats();
	driver.DeactivateReader();

	return 0;
}
