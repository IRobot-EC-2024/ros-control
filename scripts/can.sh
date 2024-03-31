#!/usr/bin/bash

PASSWORD=nuc12

echo $PASSWORD | sudo -S ip link set can0 type can bitrate 1000000
echo $PASSWORD | sudo -S ifconfig can0 up
echo $PASSWORD | sudo -S ip link set can1 type can bitrate 1000000
echo $PASSWORD | sudo -S ifconfig can1 up
