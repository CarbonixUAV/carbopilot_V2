#!/bin/bash

[ -f eeprom.bin ] || {
    echo "Creating eeprom.bin"
    cp eeprom-base.bin eeprom.bin
}

nice ../../Tools/autotest/sim_vehicle.py -D -G -f quadplane-ice --console --map -L SpringValley3 --aircraft test "$*" -A --uartF=sim:megasquirt
