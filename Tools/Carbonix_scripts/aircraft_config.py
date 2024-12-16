"""
Build ArduPlane firmware and collect periph firmware to bundle for AFQT.

This script reads an XML file that contains the aircraft configuration and
peripheral firmware information. It replaces the 'cx_pilot_commit_id'
placeholder with the actual commit ID, copies the XML file to the ROMFS_custom
directory to embed it in @ROMFS, copies the lua scripts from CarbonixCommon into
ROMFS_custom/scripts, builds the ArduPlane firmware for the flight controller,
and organizes the output files in a directory structure that can be bundled for AFQT.

Usage:
    python aircraft_config.py <xml_file> <commit_id> [--skip_periph]

AP_FLAKE8_CLEAN

"""
import os
import glob
import shutil
import argparse
import xml.etree.ElementTree as ET


def get_flight_controller_board_name(xml_file : str) -> str:
    """Get the flight controller's board name from the XML file.

    Returns, for example, 'CubeOrange-Ottano'.

    Args:
        xml_file (str): Path to the XML file.
    Returns:
        str: Board name of the flight controller.
    """
    tree = ET.parse(xml_file)
    root = tree.getroot()
    flight_controller = root.find('flight_controller')
    if flight_controller is None:
        raise AssertionError(f"'flight_controller' element not found in {xml_file}")
    board_name = flight_controller.find('board_name')
    if board_name is None:
        raise AssertionError(f"'board_name' element not found in the 'flight_controller' element in {xml_file}")
    return board_name.text


def copy_configuration_file(xml_file : str, commit_id : str) -> str:
    """Copy the XML file to the ROMFS_custom directory.

    Copies the aircraft definition XML file to a special directory to embed in
    @ROMFS. In the process, it also replaces the 'cx_pilot_commit_id'
    placeholder with the actual commit ID.

    Args:
        xml_file (str): Path to the XML file.
        commit_id (str): The commit ID to replace the placeholder with.
    Returns:
        str: Path to the destination file.
    """
    with open(xml_file, 'r') as file:
        content = file.read()
    content = content.replace('cx_pilot_commit_id', commit_id)
    destination_path = 'ROMFS_custom/AircraftConfiguration.xml'
    os.makedirs(os.path.dirname(destination_path), exist_ok=True)
    with open(destination_path, 'w') as file:
        file.write(content)
    print(f"Copied {xml_file} to {destination_path}")
    return destination_path


def copy_lua_scripts(xml_file : str) -> None:
    """Copy the Lua scripts to the scripts directory.

    Extracts the Lua scripts from the XML file and copies them to the
    'ROMFS_custom/scripts' directory.

    Args:
        xml_file (str): Path to the XML file.
    """
    tree = ET.parse(xml_file)
    root = tree.getroot()
    lua_script_list = root.find('lua_script_list')
    if lua_script_list is None:
        raise AssertionError(f"'lua_script_list' element not found in {xml_file}")

    for lua_script in lua_script_list.findall('lua_script'):
        source_path = lua_script.find('source_path')
        if source_path is None:
            raise AssertionError(f"'source_path' element not found in {xml_file}")
        destination_path = lua_script.find('destination_path')
        if destination_path is None:
            raise AssertionError(f"'destination_path' element not found in {xml_file}")
        # copy the file in 'ROMFS_custom/scripts/' directory
        destination_path = f'ROMFS_custom/scripts/{destination_path.text}'
        os.makedirs(os.path.dirname(destination_path), exist_ok=True)
        shutil.copy(source_path.text, destination_path)
        print(f"Copied {source_path.text} to {destination_path}")


def extract_aircraft_info(xml_file : str) -> tuple:
    """Extract the aircraft model and model version from the XML file.

    Args:
        xml_file (str): Path to the XML file.
    Returns:
        tuple: A tuple containing the aircraft model and model
    """
    tree = ET.parse(xml_file)
    root = tree.getroot()

    aircraft = root.find('aircraft')
    if aircraft is not None:
        model = aircraft.find('model')
        if model is None:
            raise AssertionError(f"'model' element not found in the 'aircraft' element of {xml_file}")
        model_version = aircraft.find('model_version')
        if model_version is None:
            raise AssertionError(f"'model_version' element not found in the 'aircraft' element of {xml_file}")
        return model.text, model_version.text


def get_periph_board_names(xml_file : str) -> set:
    """Get a set of peripheral firmware board names from the XML file.

    Returns a set of board names for all managed peripheral firmwares. For
    example,
    {
        'Ottano-M1_CarbonixF405',
        'Ottano-M2_CarbonixF405',
        ...
    }
    Only peripherals with a 'firmware_path' element are included in this set.
    Other peripherals are not managed by us and do not need to be bundled by
    this script.

    Args:
        xml_file (str): Path to the XML file.
    Returns:
        set: Set of peripheral board names.
    """
    tree = ET.parse(xml_file)
    root = tree.getroot()

    cpn_list = root.find('cpn_list')
    if cpn_list is None:
        raise AssertionError(f"'cpn_list' element not found in {xml_file}")

    periph_firmware_paths = set()
    for cpn in cpn_list.findall('cpn'):
        board_name = cpn.find('board_name')
        if board_name is None:
            raise AssertionError(f"'board_name' missing for CPN {cpn.get('id')} in {xml_file}")
        firmware_path = cpn.find('firmware_path')
        # If there is no firmware path, we don't need to bundle it
        if firmware_path is None:
            continue
        periph_firmware_paths.add(board_name.text)

    return periph_firmware_paths


# Organize the build output files
def organize_output(xml_file : str, fc_firmware_name : str, peripherals : set) -> None:
    """Organize the build output files in a directory structure for AFQT.

    The output directory structure is as follows:
    final-output/
    ├── <model>_<model_version>/
    │   ├── <fc_firmware_name>/
    │   ├── <peripheral_1>/
    │   ├── <peripheral_2>/
    │   ├── ...
    │   ├── ReleaseNotes.txt
    │   └── AFQT/
    │       └── target.xml

    Args:
        xml_file (str): Path to the XML file.
        fc_firmware_name (str): Name of the flight controller firmware binary.
        peripherals (set): Set of peripheral board names.
    """
    shutil.rmtree('final-output', ignore_errors=True)
    model, model_version = extract_aircraft_info(xml_file)

    output_dir = 'final-output'
    final_output_dir = os.path.join(output_dir, f"{model}_{model_version}")
    os.makedirs(final_output_dir, exist_ok=True)
    print(f"Output directory created at {final_output_dir}")

    # Move the firmware binary to the output directory
    firmware_bin = f'build/{fc_firmware_name}/bin'
    if not os.path.exists(firmware_bin):
        raise FileNotFoundError(f"Flight controler firmware binary not found for {fc_firmware_name}")
    shutil.copytree(firmware_bin, os.path.join(final_output_dir, fc_firmware_name))
    print(f"Moved {firmware_bin} binaries to {final_output_dir}/{fc_firmware_name}")

    # Move the periph firmware binary to the output directory
    if len(peripherals) > 0:
        for board_name in peripherals:
            source_dir = glob.glob(f'periph-build/*/{board_name}')
            if len(source_dir) == 0:
                raise FileNotFoundError(f'Could not find periph-build/*/{board_name}')
            if len(source_dir) > 1:
                raise FileNotFoundError(f'Multiple directories found for periph-build/*/{board_name}')
            source_dir = source_dir[0]
            shutil.copytree(source_dir, os.path.join(final_output_dir, board_name))
            print(f"Moved {board_name} binaries to {final_output_dir}/{board_name}")

    # move Release Notes  ArduPlane/ReleaseNotes.txt to the output directory
    release_notes = 'ArduPlane/ReleaseNotes.txt'
    if os.path.exists(release_notes):
        shutil.copy(release_notes, final_output_dir)
        print(f"Moved ReleaseNotes.txt to {final_output_dir}")

    # create a directory AFQT and Rename the xml file to target.xml and move it to the output directory
    os.makedirs(os.path.join(final_output_dir, 'AFQT'), exist_ok=True)
    target_xml = os.path.join(final_output_dir, 'AFQT', 'target.xml')
    shutil.copy(xml_file, target_xml)
    print(f"Moved {xml_file} to {target_xml}")


def build_flight_controller_firmware(board_name : str) -> None:
    """Build ArduPlane firmware for the flight controller.

    Args:
        board_name (str): Name of the board in hwdef, e.g., CubeOrange-Ottano
    """
    result = os.system(f"./waf configure --board={board_name}")
    if result != 0:
        raise RuntimeError(f"Error configuring firmware for {board_name}")
    result = os.system("./waf plane")
    if result != 0:
        raise RuntimeError(f"Error building firmware for {board_name}")


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('xml_file', help='Path to the XML file')
    parser.add_argument('commit_id', help='Commit ID to replace')
    parser.add_argument('--skip_periph', action='store_true', help='Skip peripheral firmware bundling')
    args = parser.parse_args()

    print('XML file:', args.xml_file)
    print('Commit ID:', args.commit_id)

    xml_file = copy_configuration_file(args.xml_file, args.commit_id)
    copy_lua_scripts(xml_file)
    fc_board_name = get_flight_controller_board_name(xml_file)

    build_flight_controller_firmware(fc_board_name)
    peripherals = set()
    if args.skip_periph:
        print('Skipping peripheral firmware bundling')
    else:
        print('Bundling peripheral firmware')
        peripherals = get_periph_board_names(xml_file)
    organize_output(xml_file, fc_board_name, peripherals)
    print('Done')
