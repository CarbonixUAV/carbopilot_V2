#!/bin/bash

# Exit if any command fails
set -e

# echo "Running distclean..."
# ./waf distclean
# rm -Rf output

if [ "$#" -ne 1 ]; then
  echo "Usage: $0 <board_name>"
  exit 1
fi

# Set the board name
board=$1

is_periph_board= ./waf configure --board "$board" -g | tee >(grep -c "env set AP_PERIPH=1")

if [ $is_periph_board -eq 0 ]; then
  echo "Compiling ArduPlane for $board..."
  ./waf plane
  echo "Script finished successfully."
  exit 0
fi

# For periph boards, we take the extra step of generating a modified binary
# for each CPN param file in the AP_HAL_ChibiOS/hwdef/CarbonixCommon/cpn_params
# folder. This modified binary will have the CPN parameters embedded in it, and
# the board name set to the board name followed by the CPN parameter file name.

# We build a version of AP_Periph with a long random string as the board name.
# This random string is replaced with the actual board name in the final binary.
# The random string is chosen to be long enough to accommodate the longest board
# name that we expect to use. Currently, we use 40, which is a little larger
# than the longest board name we have used so far:
# "CarbonixF405-no-crystal-Volanti-LWing". (and error is thrown if the board
# name is too long, so we can adjust this if we ever need a longer name)
max_board_name_length=40
board_magic_string="lm3eBX7cJeaUer67lXdkNr83q2WzPRbE2MxAgnGq9esBjPVecYynx9Pry5sRJMgCX8384NTYZRDpuR8K"
if [ ${#board_magic_string} -lt $max_board_name_length ]; then
  echo "max_board_name_length is too long (max ${#board_magic_string} bytes)"
  exit 1
fi
board_magic_string=${board_magic_string:0:max_board_name_length}

# Compile AP_Periph
echo "Compiling AP_Periph for $board..."
printf "undef CAN_APP_NODE_NAME\ndefine CAN_APP_NODE_NAME \"$board_magic_string\"" > temp.hwdef
./waf configure --board "$board" --extra-hwdef=temp.hwdef -g
./waf AP_Periph

# Loop through all CPN parameter files and generate modified binaries
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
    cp $binary $binary.tmp
    xxd -p $binary | tr -d '\n' | sed "s/$board_magic_string_hex/$board_name_hex/g" | xxd -r -p > $binary.tmp
    mv $binary.tmp $output_folder/$(basename $binary)
  done
  echo ""
done

# Cleanup
rm temp.hwdef

echo "Script finished successfully."
