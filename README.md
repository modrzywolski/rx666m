
RX666m support under GQRX 

Driver Installation
cd driver && sudo ./dkms-install.sh /path/to/RX666.img

SoapySDR Interface Library Installation
cd SoapyRX666m && mkdir build && cd build && cmake -DCMAKE_INSTALL_PREFIX=/opt/gnuradio-3.8 -DCMAKE_BUILD_TYPE=RelWithDebInfo .. && make && sudo make install && sudo ldconfig



Additional tools for recoding spectrum:

Recoding raw samples
rx666m_util --vga_gain 7 --attn -10 --out rx.raw
rx666m_util --vga_gain 7 --attn -10 --stdout | zstd --fast > rx.raw.zstd #with realtime compression

Playing raw samples in gqrx
mkfifo /tmp/rx.iq

while true; do; rx666m_convert --in rx > /tmp/rx.iq; done
or with compresion:
zstdcat rx.raw.zstd | rx666m_convert --stdin > /tmp/rx.iq


