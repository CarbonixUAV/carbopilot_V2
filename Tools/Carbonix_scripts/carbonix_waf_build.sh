#!/bin/bash


# Check unstaged changes
if [[ $(git status --porcelain) ]]; then
    echo "There are unstaged changes in the repository."
    exit 1
fi

# Check outdated submodules
# git submodule update --remote --quiet
# outdated_submodules=$(git submodule status | grep -e '-[[:alnum:]]' | awk '{print $2}')
# if [[ -n $outdated_submodules ]]; then
#     echo "The following submodules are not updated:"
#     echo "$outdated_submodules"
#     exit 1
# fi

echo "Running distclean..."
./waf distclean

# ArduPlane with CubeOrange
echo "Compiling ArduPlane for CarbonixCubeOrange..."
./waf configure --board CarbonixCubeOrange
./waf plane
./waf configure --board CarbonixCubeOrange --bootloader
./waf bootloader

echo "Compiling ArduPlane for CubeOrange..."
./waf configure --board CubeOrange
./waf plane
./waf configure --board CubeOrange --bootloader
./waf bootloader

echo "Compiling AP_Periph for CarbonixF405..."
./waf configure --board CarbonixF405
./waf AP_Periph
./waf configure --board CarbonixF405 --bootloader
./waf bootloader

echo "Compiling AP_Periph for CarbonixF405_M1..."
./waf configure --board CarbonixF405-M1
./waf AP_Periph
./waf configure --board CarbonixF405-M1 --bootloader
./waf bootloader

echo "Compiling AP_Periph for CarbonixF405_M2..."
./waf configure --board CarbonixF405-M2
./waf AP_Periph
./waf configure --board CarbonixF405-M2 --bootloader
./waf bootloader

echo "Compiling AP_Periph for CarbonixF405_M3..."
./waf configure --board CarbonixF405-M3
./waf AP_Periph
./waf configure --board CarbonixF405-M3 --bootloader
./waf bootloader

echo "Compiling AP_Periph for CarbonixF405_M4..."
./waf configure --board CarbonixF405-M4
./waf AP_Periph
./waf configure --board CarbonixF405-M4 --bootloader
./waf bootloader

echo "Compiling AP_Periph for CarbonixF405_M5..."
./waf configure --board CarbonixF405-M5
./waf AP_Periph
./waf configure --board CarbonixF405-M5 --bootloader
./waf bootloader

echo "Compiling AP_Periph for CarbonixF405_LWing..."
./waf configure --board CarbonixF405-LWing
./waf AP_Periph
./waf configure --board CarbonixF405-LWing --bootloader
./waf bootloader

echo "Compiling AP_Periph for CarbonixF405_RWing..."
./waf configure --board CarbonixF405-RWing
./waf AP_Periph
./waf configure --board CarbonixF405-RWing --bootloader
./waf bootloader

echo "Compiling AP_Periph for CarbonixF405_RTail..."
./waf configure --board CarbonixF405-RTail
./waf AP_Periph
./waf configure --board CarbonixF405-RTail --bootloader
./waf bootloader

echo "Compiling AP_Periph for CarbonixF405_RTail..."
./waf configure --board CarbonixF405-LTail
./waf AP_Periph
./waf configure --board CarbonixF405-LTail --bootloader
./waf bootloader

# echo "Compiling SITL for CarbonixF405..."
# ./waf configure --board sitl
# ./Tools/autotest/sim_vehicle.py --console -f quadplane --vehicle=ArduPlane