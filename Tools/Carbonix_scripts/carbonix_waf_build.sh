#!/bin/bash

# Exit if any command fails
set -e

echo "Running distclean..."
./waf distclean

main_boards=("CubeOrange" "CubeOrangePlus" "CubeOrange-Volanti" "CubeOrangePlus-Volanti" "CubeOrange-Ottano" "CubeOrangePlus-Ottano")
for board in "${main_boards[@]}"; do
  echo "Compiling ArduPlane for $board..."
  ./waf configure --board "$board" -g
  ./waf plane
done

periph_boards=("CarbonixF405" "CarbonixF405-no-crystal")

# Build all periph board with custom parameters
for board in "${periph_boards[@]}"; do
  for file in $(find libraries/AP_HAL_ChibiOS/hwdef/CarbonixCommon/cpn_params/ -name "*.parm"); do
    # Extract the filename without the extension 
    filename=$(basename -- "$file")
    filename="${filename%.*}"
    # Extract the Parent folder name 
    foldername=$(basename -- "$(dirname -- "$file")")
    echo "Compiling AP_Periph for $board with $filename with foldername $foldername..."
    # Create extra hwdef file
    printf "undef CAN_APP_NODE_NAME\ndefine CAN_APP_NODE_NAME \"$board-$filename\"" > temp.hwdef
    
    # Compile AP_Periph for each board
    echo "Compiling AP_Periph for $board with $filename..."
    ./waf configure --board "$board" --extra-hwdef=temp.hwdef --default-parameters="$file" -g
    ./waf AP_Periph
    
    # Rename build outputs
    mkdir -p output/$foldername/${filename}_$board    
    # Move all the files (not folders) in build/$board/bin to build/$board/bin/$filename
    find build/$board/bin -maxdepth 1 -type f -exec mv {} output/$foldername/${filename}_$board \;
    # Move default $file in the output folder
    cp $file output/$foldername/${filename}_$board

    # Cleanup
    rm temp.hwdef
  done
done

# Build all Default periph board
for board in "${periph_boards[@]}"; do
  echo "Compiling AP_Periph for $board..."
  ./waf configure --board "$board" -g
  ./waf AP_Periph
done

echo "Script completed successfully."
