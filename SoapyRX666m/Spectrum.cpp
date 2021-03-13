#include "Spectrum.hpp"

#include <fftw3.h>

#define NUM_POINTS 1024* 1024 /4/16
#define SIG_SIZE	64
#define SAMPLE_RATE 64000000

/* Never mind this bit */

#include <stdio.h>
#include <math.h>
#include <memory.h>

#define REAL 0
#define IMAG 1

void do_something_with(fftw_complex* result, int termWidth) {
    int64_t i;
	//printf("\033[2J\033[0;0H====================================================\n");
	printf("\033[0;0H====================================================\n");

    for (i = 0; i <= NUM_POINTS/2; ++i) {
        double mag = sqrt(result[i][REAL] * result[i][REAL] +
                          result[i][IMAG] * result[i][IMAG]);
		double dbmag;

		if(mag < 0.001)
			mag = 0;
		else
			dbmag = 0.0 + 20*::log10(mag);

		if(dbmag < 0)
			dbmag = 0;

		size_t khz = (SAMPLE_RATE/2 * i / (NUM_POINTS/2))/1000;
		//int lev = mag*termWidth/10000;
		int lev = dbmag;
		if(lev>termWidth) lev=termWidth;
		if(khz >= 10694 && khz < 10699)
		{
	        printf("[%06ld] %s%s%04d\n", khz, std::string(lev,'*').c_str(), std::string(termWidth-lev,' ').c_str(), (int)mag);
		}
    }
}

double
parzen (int i, int nn)
{
  return (1.0 - fabs (((double)i-0.5*(double)(nn-1))
		      /(0.5*(double)(nn+1))));
}

double
welch (int i, int nn)
{
  return (1.0-(((double)i-0.5*(double)(nn-1))
	       /(0.5*(double)(nn+1)))
	  *(((double)i-0.5*(double)(nn-1))
	    /(0.5*(double)(nn+1))));
}

double
hanning (int i, int nn)
{
  return ( 0.5 * (1.0 - cos (2.0*M_PI*(double)i/(double)(nn-1))) );
}

/* Reference: "Digital Filters and Signal Processing" 2nd Ed.
 * by L. B. Jackson. (1989) Kluwer Academic Publishers.
 * ISBN 0-89838-276-9
 * Sec.7.3 - Windows in Spectrum Analysis
 */
double
hamming (int i, int nn)
{
  return ( 0.54 - 0.46 * cos (2.0*M_PI*(double)i/(double)(nn-1)) );
}

double
blackman (int i, int nn)
{
  return ( 0.42 - 0.5 * cos (2.0*M_PI*(double)i/(double)(nn-1))
	  + 0.08 * cos (4.0*M_PI*(double)i/(double)(nn-1)) );
}

double
steeper (int i, int nn)
{
  return ( 0.375
	  - 0.5 * cos (2.0*M_PI*(double)i/(double)(nn-1))
	  + 0.125 * cos (4.0*M_PI*(double)i/(double)(nn-1)) );
}

void
windowing (int n, const double *data, int flag_window, double scale,
	   double *out)
{
  int i;
  for (i = 0; i < n; i ++)
    {
      switch (flag_window)
	{
	case 1: // parzen window
	  out [i] = data [i] * parzen (i, n) / scale;
	  break;

	case 2: // welch window
	  out [i] = data [i] * welch (i, n) / scale;
	  break;

	case 3: // hanning window
	  out [i] = data [i] * hanning (i, n) / scale;
	  break;

	case 4: // hamming window
	  out [i] = data [i] * hamming (i, n) / scale;
	  break;

	case 5: // blackman window
	  out [i] = data [i] * blackman (i, n) / scale;
	  break;

	case 6: // steeper 30-dB/octave rolloff window
	  out [i] = data [i] * steeper (i, n) / scale;
	  break;

	default:
	  fprintf (stderr, "invalid flag_window\n");
	case 0: // square (no window)
	  out [i] = data [i] / scale;
	  break;
	}
    }
}

/* Resume reading here */

#include <sys/ioctl.h>

Spectrum::Spectrum()
{
	input = fftw_alloc_real(NUM_POINTS);
	result = fftw_alloc_complex(NUM_POINTS);

    fftplan = fftw_plan_dft_r2c_1d(NUM_POINTS,
                                      input,
                                      result,
                                      FFTW_MEASURE);

   struct winsize w;
   ioctl(0, TIOCGWINSZ, &w);

   termWidth = w.ws_col;
   termHieght = w.ws_row;
}

Spectrum::~Spectrum()
{
    fftw_destroy_plan(fftplan);
	fftw_free(input);
	fftw_free(result);
}

void Spectrum::feed(RingBufEntry *buf)
{
	static int cnt=0;

	if(++cnt % 10 == 0)
	if(buf)
	{
		size_t i, n;

		n = std::min<size_t>(buf->bytesAvail/2, NUM_POINTS/1);

		memset(input, 0, sizeof(double)*NUM_POINTS);
		memset(result, 0, sizeof(fftw_complex)*NUM_POINTS);

		for(i=0; i<n; ++i)
		{
			input[i] = reinterpret_cast<uint16_t*>(buf->data.data())[i] / 65536.0;
		}

		//windowing (NUM_POINTS, input, 3, 1.0, input);

		fftw_execute(fftplan);
		do_something_with(result, termWidth-15);
	}
}


