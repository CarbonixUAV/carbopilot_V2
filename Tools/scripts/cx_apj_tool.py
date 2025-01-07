#!/usr/bin/env python3
"""
Tool to manipulate Carbonix AP_Periph firmware files.

This tool can be used to set the default parameters in a firmware file and set
the board name. AP_Periph has an additional APP_DESCRIPTOR within it for the
bootloader to check the integrity of the firmware. This tool also makes sure
to fix the CRCs in the APP_DESCRIPTOR when the firmware is modified.

To modify the board name, the input binary needs to have been compiled with a
known string that is guaranteed to show up only once in the binary. The new
name is padded with null bytes to match the length of the old name and then
replaces the old name in the binary.

AP_FLAKE8_CLEAN
"""
import struct
import argparse
from uploader import crc32
from apj_tool import embedded_defaults


def replace_board_name(image, old_name, new_name):
    """Find the old_board_name in the image and replace it with the board_name

    Args:
        image (bytes): The image to replace the board name in
        old_name (str): The board name currently in the image
        new_name (str): The new board name to replace the old name with

    Raises:
        LookupError: When the old_name is not found in the image
        AssertionError: When the old_name is found multiple times in the image
        ValueError: When the new_name is longer than the old_name

    Returns:
        bytes: The image with the board name replaced
    """
    old_bytes = old_name.encode()
    new_bytes = new_name.encode()
    offset = image.find(old_bytes)
    if offset == -1:
        raise LookupError(f"'{old_name}' not found in image")
    if image.find(old_bytes, offset+1) != -1:
        raise AssertionError(f"'{old_name}' found multiple times in image")
    if len(new_bytes) > len(old_bytes):
        raise ValueError(f"New board name '{new_name}' is too long")
    # Pad the new name with null bytes to match the length of the old name
    new_bytes += b'\0' * (len(old_bytes) - len(new_bytes))
    image = image[:offset] + new_bytes + image[offset+len(old_bytes):]
    return image


def fix_app_descriptor(img):
    """Find the APP_DESCRIPTOR in the binary file and update the CRCs.

    This function is based on set_app_descriptor in chibios.py. It currently
    only supports unsigned app descriptors.

    Args:
        img (bytes): The image to fix

    Raises:
        LookupError: When the APP_DESCRIPTOR is not found
        AssertionError: When the image length does not match the APP_DESCRIPTOR

    Returns:
        bytes: The image with the APP_DESCRIPTOR updated
    """
    descriptor = b'\x40\xa2\xe4\xf1\x64\x68\x91\x06'
    offset = img.find(descriptor)
    if offset == -1:
        raise LookupError('No APP_DESCRIPTOR found in image')
    offset += 8
    desc_len = 16
    # Get the existing app descriptor
    crc1, crc2, img_len, githash = struct.unpack('<IIII', img[offset:offset+desc_len])
    if img_len != len(img):
        raise AssertionError('Bad APP_DESCRIPTOR: image length mismatch')
    img1 = bytearray(img[:offset])
    img2 = bytearray(img[offset+desc_len:])
    crc1 = to_unsigned(crc32(img1))
    crc2 = to_unsigned(crc32(img2))
    desc = struct.pack('<IIII', crc1, crc2, len(img), githash)
    img = img[:offset] + desc + img[offset+desc_len:]
    return img


def to_unsigned(i):
    """Convert a possibly signed integer to unsigned"""
    if i < 0:
        i += 2**32
    return i


def __main__():
    """Main function for the command line tool"""
    parser = argparse.ArgumentParser(description='Fix app descriptor in binary file')
    parser.add_argument('bin_file', help='binary file to fix')
    parser.add_argument('--old-board-name', help='board name to replace in the binary file')
    parser.add_argument('--new-board-name', help='board name to replace the magic name with')
    parser.add_argument('--defaults', help='file with default parameters')
    parser.add_argument('--output', help='output file')
    args = parser.parse_args()

    if args.old_board_name and not args.new_board_name:
        parser.error('--old-board-name requires --new-board-name')
    if args.new_board_name and not args.old_board_name:
        parser.error('--new-board-name requires --old-board-name')
    if not args.old_board_name and not args.defaults:
        parser.error('Nothing to be done. Please provide --old-board-name or --defaults')
    if args.new_board_name and len(args.old_board_name.encode()) < len(args.new_board_name.encode()):
        parser.error('--new-board-name too long to fit in the space occupied by --old-board-name')

    defaults = embedded_defaults(args.bin_file)

    if args.output:
        defaults.filename = args.output

    if args.defaults:
        if not defaults.find():
            raise LookupError('No defaults found in firmware')
        defaults.set_file(args.defaults)

    if args.old_board_name:
        defaults.firmware = replace_board_name(defaults.firmware, args.old_board_name, args.new_board_name)

    defaults.firmware = fix_app_descriptor(defaults.firmware)
    defaults.save()


if __name__ == '__main__':
    __main__()
