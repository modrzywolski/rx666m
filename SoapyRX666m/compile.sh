#!/bin/bash

(
rm -rf build &&
mkdir -p build &&
cd build &&
cmake -DCMAKE_INSTALL_PREFIX=/opt/gnuradio-3.8 -DCMAKE_BUILD_TYPE=RelWithDebInfo  .. &&
make &&
sudo make install &&
sudo ldconfig
)
