#!/bin/bash

# script to build cygwin binaries for using in MissionPlanner
# the contents of artifacts directory is uploaded to:
# https://firmware.ardupilot.org/Tools/MissionPlanner/sitl/

# the script assumes you start in the root of the ardupilot git tree

set -x

# Get Carbonix version number
FIRMWARE_VERSION=$(grep -oP 'define AP_CUSTOM_FIRMWARE_STRING "\K(.*)(?=")' libraries/AP_HAL_ChibiOS/hwdef/CarbonixCommon/version.inc)
COMMIT_ID=$(git rev-parse --short HEAD)

# TOOLCHAIN=i686-pc-cygwin
TOOLCHAIN=x86_64-pc-cygwin
GPP_COMPILER="${TOOLCHAIN}-g++"

$GPP_COMPILER -print-sysroot

SYS_ROOT=$($GPP_COMPILER -print-sysroot)
echo "SYS_ROOT=$SYS_ROOT"

rm -rf artifacts
mkdir artifacts

(
    python ./waf --color yes --toolchain $TOOLCHAIN --board sitl configure --define AP_CUSTOM_FIRMWARE_STRING=\"$FIRMWARE_VERSION\" 2>&1
    python ./waf plane 2>&1
) | tee artifacts/build.txt
    # python ./waf copter 2>&1
    # python ./waf heli 2>&1
    # python ./waf rover 2>&1
    # python ./waf sub 2>&1

# copy both with exe and without to cope with differences
# between windows versions in CI
cp -v build/sitl/bin/arduplane artifacts/${FIRMWARE_VERSION}-${COMMIT_ID}.exe

# cp -v build/sitl/bin/arduplane artifacts/ArduPlane.elf.exe
# cp -v build/sitl/bin/arducopter artifacts/ArduCopter.elf.exe
# cp -v build/sitl/bin/arducopter-heli artifacts/ArduHeli.elf.exe
# cp -v build/sitl/bin/ardurover artifacts/ArduRover.elf.exe
# cp -v build/sitl/bin/ardusub artifacts/ArduSub.elf.exe

# cp -v build/sitl/bin/arduplane artifacts/ArduPlane.elf
# cp -v build/sitl/bin/arducopter artifacts/ArduCopter.elf
# cp -v build/sitl/bin/arducopter-heli artifacts/ArduHeli.elf
# cp -v build/sitl/bin/ardurover artifacts/ArduRover.elf
# cp -v build/sitl/bin/ardusub artifacts/ArduSub.elf

# Find all cyg*.dll files returned by cygcheck for each exe in artifacts
# and copy them over
for exe in artifacts/*.exe; do 
    echo $exe
    cygcheck $exe | grep -oP 'cyg[^\s\\/]+\.dll' | while read -r line; do
      cp -v /usr/bin/$line artifacts/
    done
done

# Process Carbonix SITL parameters
for file in libraries/AP_HAL_ChibiOS/hwdef/CarbonixCommon/sitl_params/*.parm
do
    destfolder=artifacts/$(basename $file .parm)-${FIRMWARE_VERSION}-${COMMIT_ID}
    mkdir -p $destfolder
    outfile=$destfolder/defaults.parm
    echo "Processing $(basename $file)"
    
    # Run parse_sitl_params.py script passing full path to .parm file and output folder
    python Tools/Carbonix_scripts/process_sitl_defaults.py $file $outfile

    # Create batch script to launch SITL with the correct parameters
    if [[ $file == *"realflight"* ]]; then
        model="flightaxis"
    else
        model="quadplane"
    fi
    printf "rem Launch at Eli Field\r\n..\\${FIRMWARE_VERSION}-${COMMIT_ID}.exe -O 40.0594626,-88.5513292,206.0,0 --serial0 tcp:0 -M ${model} --defaults defaults.parm\r\n" > $destfolder/launch.bat
done

git log -1 > artifacts/git.txt
ls -l artifacts/
