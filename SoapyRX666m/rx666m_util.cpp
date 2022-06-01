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
#include <chrono>
#include <csignal>
#include <cmath>
#include <fstream>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/format.hpp>

#include "FileWriter.hpp"
#include "StatsDumper.hpp"

bool 		stopApp = false;
double		vgaGain;
double 		attn;
std::string outputFile;

namespace po = boost::program_options;
LowLevel driver;

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
#if 1
		("attn1", po::value<double>()->default_value(0)->notifier(
            [](double g)
            {
				if (g < ATTN1_Gain || g > 0.0 )
					throw po::invalid_option_value( std::to_string(g) );
            }),
			(boost::format("Attenuator level, %1%..%2% dB") % 0.0 % ATTN1_Gain ).str().c_str() )
#endif
		("attn2", po::value<double>()->default_value(0)->notifier(
            [](double g)
            {
                if (!is_close(g, ATTN2_Gain2) && !is_close(g, ATTN2_Gain1) && !is_close(g, 0.0) )
					throw po::invalid_option_value( std::to_string(g) );
            }),
			(boost::format("Attenuator level, valid levels: %1%, %2%, %3%") % ATTN2_Gain2 % ATTN2_Gain1 % 0.0 ).str().c_str() )
		("agc", po::bool_switch(), "Enable software AGC")
		("out",	po::value<std::string>(), "Output file for samples recording")
		("stdout",	po::bool_switch(), "Output samples to standard output")
		("nostats",	po::bool_switch(), "Hides stats")
	;

	po::variables_map vm;
	po::store(po::parse_command_line(ac, av, desc), vm);
	po::notify(vm);

	if (vm.count("help"))
	{
		std::cerr << desc << "\n";
		return 1;
	}


	signal(SIGINT, exitHandler);
	signal(SIGTERM, exitHandler); 

	driver.Init();
	driver.getRingBuffer().resize( 2048 ); //set bigger buffer


	driver.SetAD8331Gain( vm["vga_gain"].as< double >() );
	driver.SetAttn1( vm["attn1"].as< double >() );
	driver.SetAttn2( vm["attn2"].as< double >() );
	driver.SetAGC( vm["agc"].as< bool >() );

	FileWriter fw;

	if(vm.count("out"))
		fw.openFile(vm["out"].as< std::string >().c_str());
	else if (vm["stdout"].as< bool >())
		fw.openStdout();

	driver.ActivateReader();
	StatsDumper dumper(driver);

	if (!vm["nostats"].as< bool >())
		dumper.show();

	while(!stopApp)
	{
		using namespace std::placeholders;

		driver.WriteToFile( std::bind(&FileWriter::writeData, &fw, _1) );
	}

	driver.DumpReaderStats();
	driver.DeactivateReader();

	return 0;
}
