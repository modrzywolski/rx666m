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


