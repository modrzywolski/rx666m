#!/bin/bash

IMAGE_NAME=RX666.img
DRV_NAME=rx666m
DRV_VERSION=1.0
FILES="*.c Makefile dkms.conf ../common/rx666m_ioctl.h"

if [ ! "x$1" = "x" ]
then
	IMAGE_NAME="$1"
fi
if [ ! -f "${IMAGE_NAME}" ]
then
	echo "Firmware image '$IMAGE_NAME' not found. Please copy image file to current directory or provide path to image as an argument." 2>&1 
	exit 1
fi

if [[ $EUID -ne 0 ]];
then
	echo "Insufficient priviliges. Try: 'sudo ./dkms-install.sh'" 2>&1
	exit 1
else
	echo "Installing DKMS driver..."
fi


mkdir -p /usr/src/${DRV_NAME}-${DRV_VERSION} /usr/share/rx666m/
cp -f 52-rx666m.rules /etc/udev/rules.d/
cp $FILES /usr/src/${DRV_NAME}-${DRV_VERSION}
cp "${IMAGE_NAME}" /usr/share/rx666m/

dkms add -m ${DRV_NAME} -v ${DRV_VERSION}
dkms build -m ${DRV_NAME} -v ${DRV_VERSION}
dkms install -m ${DRV_NAME} -v ${DRV_VERSION}
RESULT=$?

udevadm control --reload-rules

echo "Done."

exit $RESULT
