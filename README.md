

Driver Installation
cd driver && sudo ./dkms-install.sh

SoapySDR Interface Library Installation
cd SoapyRX666m && mkdir build && cmake -DCMAKE_INSTALL_PREFIX=/opt/gnuradio-3.8 -DCMAKE_BUILD_TYPE=RelWithDebInfo .. && make && sudo make install && sudo ldconfig




