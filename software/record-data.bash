#!/usr/bin/env bash

# record one second of data from serial port output and stops it.
# the file is saved into logs/data.bin.
# this is used to analyze output data.
timeout 5s bash -c '(stty raw; cat > logs/data.bin) < /dev/cu.usbmodemTEST1'