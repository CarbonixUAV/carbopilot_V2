#!/bin/bash

# This file is mainly used by workflows/cx_build_compare.yml to build the firmware for a specific board without bootloader and clean.

# Exit immediately if a command exits with a non-zero status
set -e

# The board to build for is passed as an argument to the script
BOARD=$1

if [ "$BOARD" == "CubeOrange" ] || [ "$BOARD" == "CubeOrange-Volanti" ] || [ "$BOARD" == "CubeOrange-Ottano" ] || [ "$BOARD" == "sitl" ]
then
    echo "Configuring Plane for $BOARD..."
    ./waf configure --board $BOARD
    echo "Compiling Plane for $BOARD..."
    ./waf plane
elif [ "$BOARD" == "CarbonixF405" ] || [ "$BOARD" == "CarbonixF405-no-crystal" ]
then
    echo "Configuring AP_Periph for $BOARD..."
    ./waf configure --board $BOARD
    echo "Compiling AP_Periph for $BOARD..."
    ./waf AP_Periph
fi

echo "Build for $BOARD completed."
