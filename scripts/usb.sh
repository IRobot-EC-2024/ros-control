#!/usr/bin/bash

PASSWORD=nuc12

# 经过五次开机测试 STMicroelectronics Virtual COM Port有四次在/dev/bus/usb/003/* 有一次在/dev/bus/usb/001/*
echo $PASSWORD | sudo -S chmod 777 /dev/bus/usb/003/*
echo $PASSWORD | sudo -S chmod 777 /dev/bus/usb/001/*

