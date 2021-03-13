#pragma once

#include <fftw3.h>
#include "RingBuffer.hpp"

class Spectrum
{
public:

	Spectrum();
	~Spectrum();

	void feed(RingBufEntry *buf);
	void calculate();

protected:
	fftw_plan fftplan;

	double			*input;
	fftw_complex	*result;
	size_t			termWidth;
	size_t			termHieght;

};
