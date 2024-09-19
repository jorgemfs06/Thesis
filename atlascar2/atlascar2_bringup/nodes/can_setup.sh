#!/bin/bash

ifconfig can0
#sudo ip link set can0 up type can bitrate 500000
SUDO_PASSWORD="Bitcho_2006"
echo $SUDO_PASSWORD | sudo -S /sbin/ip link set can0 down
echo $SUDO_PASSWORD | sudo -S /sbin/ip link set can0 up type can bitrate 500000