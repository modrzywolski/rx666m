#include "RingBuffer.hpp"

int main()
{
	RingBuf buf;

	buf.stats();
	buf.getRdBuf(0);
	buf.commit(buf.getWrBuf());
	buf.commit(buf.getWrBuf());
	buf.getWrBuf();
	buf.stats();
	buf.getRdBuf(0);
	buf.stats();
	buf.getRdBuf(0);
	buf.stats();
	buf.getRdBuf(0);
	buf.stats();

	return 0;
}
