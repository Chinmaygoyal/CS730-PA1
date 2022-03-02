#!/bin/bash

make
sudo rmmod cryptocard_mod
sudo insmod cryptocard_mod.ko
dmesg
