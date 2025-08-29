#!/bin/bash
{
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", MODE:="0777", SYMLINK+="pibot"'
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0777", SYMLINK+="pibot"'
echo 'KERNEL=="ttyACM*", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0042", MODE:="0777", SYMLINK+="pibot"' 
echo 'KERNEL=="ttyACM*", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE:="0777", SYMLINK+="pibot"'
} >> /etc/udev/rules.d/pibot.rules

service udev reload
sleep 2
service udev restart

