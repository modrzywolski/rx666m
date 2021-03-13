#!/bin/bash


DRV_NAME=rx666m
DRV_VERSION=1.0


if [[ $EUID -ne 0 ]];
then
	echo "Insufficient priviliges. Try: 'sudo ./dkms-install.sh'" 2>&1
	exit 1
else
	echo "Removing dkms driver..."
fi

dkms remove ${DRV_NAME}/${DRV_VERSION} --all
rm -rf /usr/src/${DRV_NAME}-${DRV_VERSION} /usr/share/rx666m/
rm /etc/udev/rules.d/52-rx666m.rules

udevadm control --reload-rules

RESULT=$?
if [[ "$RESULT" != "0" ]];
then
  echo "Error." 2>&1
else
  echo "Done."
fi

exit $RESULT
