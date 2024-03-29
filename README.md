## RX666m support under GQRX/Linux ##

This repository contains the source code enabling RX-666 and RX-666m support under GQRX/Linux through SoapySDR driver framework.
RX-666m is a modified version of RX-666, I added variable gain LNA (AD8331) and attenateur(PE4302) in front of HF path.
The driver treats AD8331/PE4302 as optional and can cope without one.

The driver can be easily modified to support any other FX3 based SDR, like RX-103 RX-888 RX-888 mkII.

Currently only direct sampling mode is supported.

Driver has two parts: kernel mode driver and user mode SoapySDR driver.

## Driver Installation ##
```clone respository git clone https://github.com/modrzywolski/rx666m.git```

```cd driver && sudo ./dkms-install.sh /path/to/RX666.img```

SoapySDR Interface Library Installation

```cd SoapyRX666m && mkdir build && cd build && cmake -DCMAKE_INSTALL_PREFIX=/opt/gnuradio-3.10 -DCMAKE_BUILD_TYPE=RelWithDebInfo .. && make && sudo make install && sudo ldconfig```


## SoapySDR driver validation ##
```
# SoapySDRUtil --find="driver=rx666m"

######################################################
##     Soapy SDR -- the SDR abstraction library     ##
######################################################

Found device 0
  device_id = 1
  driver = rx666m
  label = rx666m
  serial = 1
```


## Additional tools for recoding spectrum/kernel driver testing: ##

Recoding raw samples

```rx666m_util --vga_gain 7 --attn1 -8 --attn2 -10 --out rx.raw```

```rx666m_util --vga_gain 7 --attn1 -30 --attn2 -10 --stdout | zstd --fast > rx.raw.zstd #with realtime compression```

Playing raw samples in gqrx

```mkfifo /tmp/rx.iq```

```while true; do; rx666m_convert --in rx > /tmp/rx.iq; done```

or with compresion:

```zstdcat rx.raw.zstd | rx666m_convert --stdin > /tmp/rx.iq```


