#pragma once

#include <vector>
#include <array>
#include <algorithm>
#include <iostream>
#include <mutex>
#include <condition_variable>
#include <boost/format.hpp>

constexpr size_t	ring_chunk_size	= 4096*128;
constexpr size_t	ring_total_entries = 128;

class RingBufEntry
{
public:

	RingBufEntry()
	{
		std::generate(data.begin(), data.end(), []() { return 0*rand()%0x100; } );  //tmp
	}

	size_t						bytesRead;
	size_t						bytesAvail;
	std::array<char, ring_chunk_size> data;
};

class RingBuf
{
public:
	RingBuf()
	{
		reset();
		alloc();
	}

	void reset()
	{
		wrPtr = 0;
		rdPtr = 0;
		count = 0;
		total = 0;
		lost = 0;
	}

	~RingBuf()
	{
		dealloc();
	}

	RingBufEntry* getRdBuf(size_t timeoutUs)
	{
		std::unique_lock<std::mutex> lock(data_guard);

		if(!count)
		{
			dataAvailNotifier.wait_for(lock, std::chrono::microseconds(timeoutUs), [this]{return count != 0;});
		}

		if(!count)
		{
			return NULL;
		}

		return ring[rdPtr];
	}

	RingBufEntry* getWrBuf()
	{
		std::lock_guard<std::mutex> lg(data_guard);

		if(count >= ring.size())
		{
			++lost;
			return NULL;
		}

		return ring[wrPtr];
	}

	void free(RingBufEntry *e)
	{
		std::lock_guard<std::mutex> lg(data_guard);

		if(e != ring[rdPtr])
			throw std::logic_error( "Bad ring buffer entry" );

		rdPtr = (rdPtr + 1) % ring.size();
		--count;
	}

	void commit(RingBufEntry *e)
	{
		std::lock_guard<std::mutex> lg(data_guard);

		if(e != ring[wrPtr])
			throw std::logic_error( "Bad ring buffer entry" );

		total += e->bytesAvail;
		wrPtr = (wrPtr + 1) % ring.size();
		count++;

		dataAvailNotifier.notify_one();
	}

	void stats()
	{
		std::cerr << boost::format("wrPtr=%ld rdPtr=%ld count=%ld\n") % wrPtr % rdPtr % count;
		if(count)
		{
			std::cerr << boost::format("[%ld].read=%ld,total=%ld\n") % rdPtr % ring[rdPtr]->bytesRead % ring[rdPtr]->bytesAvail;
		}
	}

	size_t statsTotal() { return total; }
	size_t statsLost() { return lost; }
	size_t statsCount() { return count; }
protected:

	void alloc()
	{
		ring.reserve(ring_total_entries);
		ring.resize(ring_total_entries);

		std::generate(ring.begin(), ring.end(), []() { return new RingBufEntry; } );
	}

	void dealloc()
	{
		std::for_each(ring.begin(), ring.end(), [](RingBufEntry *e) { delete e; } );

	}

	size_t						wrPtr;
	size_t						rdPtr;
	size_t						count;
	size_t						total;
	size_t						lost;
	std::vector<RingBufEntry*>	ring;

	std::mutex					data_guard;
	std::condition_variable		dataAvailNotifier;

};
