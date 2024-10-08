#!/bin/bash

# Exit if any command fails
set -e

# echo "Running distclean..."
# ./waf distclean
# rm -Rf output

# main_boards=("CubeOrange" "CubeOrangePlus" "CubeOrange-Volanti" "CubeOrangePlus-Volanti" "CubeOrange-Ottano" "CubeOrangePlus-Ottano")
# for board in "${main_boards[@]}"; do
#   echo "Compiling ArduPlane for $board..."
#   ./waf configure --board "$board" -g
#   ./waf plane
# done

periph_boards=("CarbonixF405" "CarbonixF405-no-crystal")

# Create a magic string as a placeholder for the board name, this will be
# find/replaced within the AP_Periph binary. This makes the build much faster,
# as changing CAN_APP_NODE_NAME causes pretty-much a full rebuild.
board_magic_string="lm3eBX7cJeaUer67lXdkNr83q2WzPRbE2MxAgnGq9esBjPVecYynx9Pry5sRJMgCX8384NTYZRDpuR8K"

# We use a subset of that magic string, because we don't want to waste the full
# 80 bytes (max length allowed in the NodeStatus message) on the board name.
# We use 40 bytes, which is slightly more than the longest board name:
# "CarbonixF405-no-crystal-Volanti-LWing" (at time of writing).
# If we ever need more than 40 bytes, we can increase this limit.
max_board_name_length=40
board_magic_string=${board_magic_string:0:max_board_name_length}

# Create extra hwdef file
printf "undef CAN_APP_NODE_NAME\ndefine CAN_APP_NODE_NAME \"$board_magic_string\"" > temp.hwdef

# Build all periph board with custom parameters
for board in "${periph_boards[@]}"; do
  echo "Compiling AP_Periph for $board..."
  ./waf configure --board "$board" --extra-hwdef=temp.hwdef
  ./waf AP_Periph
  bin_folder=build/$board/bin
  for file in $(find libraries/AP_HAL_ChibiOS/hwdef/CarbonixCommon/cpn_params/ -name "*.parm"); do
    echo "Processing parameter file $file for $board..."
    # Extract the filename without the extension 
    filename=$(basename -- "$file")
    filename="${filename%.*}"
    # Extract the Parent folder name 
    foldername=$(basename -- "$(dirname -- "$file")")

    # Create output folder
    output_folder=output/$foldername/${filename}_$board
    mkdir -p $output_folder
    # Copy param file in the output folder
    cp $file $output_folder

    # The find/replace operation works by converting the binary to a single long
    # line of hex, then uses sed to replace the magic string with the board name
    # that has been padded with zeros to the same length as the magic string,
    # then converts the hex back to binary.
    new_board_name="$board-$filename"
    if [ ${#new_board_name} -gt $max_board_name_length ]; then
      echo "Board name '$new_board_name' is too long (max $max_board_name_length bytes)"
      exit 1
    fi
    board_magic_string_hex=$(echo -n "$board_magic_string" | xxd -p | tr -d '\n')
    board_name_hex=$(echo -n "$new_board_name" | xxd -p | tr -d '\n')
    # Pad with zeros to the same length as the magic string
    board_name_hex=$(printf "%-${#board_magic_string_hex}s" "$board_name_hex" | tr ' ' '0')
    
    for binary in $bin_folder/AP_Periph $bin_folder/AP_Periph.bin; do
      # Embed the parameters
      echo "Embedding parameter file $filename into $binary..."
      Tools/scripts/apj_tool.py $binary --set-file $file &> /dev/null

      # Set the board name
      echo "Setting board name to $new_board_name in $binary..."
      cp $binary $binary.bak
      xxd -p $binary.bak | tr -d '\n' | sed "s/$board_magic_string_hex/$board_name_hex/g" | xxd -r -p > $binary
      cp $binary $output_folder
    done
    echo ""
  done
done

# Cleanup
rm temp.hwdef

# Build all Default periph board
for board in "${periph_boards[@]}"; do
  echo "Compiling AP_Periph for $board..."
  ./waf configure --board "$board" -g
  ./waf AP_Periph
done

echo "Script completed successfully."
