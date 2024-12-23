'''
Script to generate folder structure for Cygwin SITL bundles

This can also be imported as a module to use the copy_frame_scripts function,
which copies frame scripts defined in vehicleinfo.py to the appropriate
destination folder.
'''
import os
import glob
import shutil
import argparse
from pysim import vehicleinfo


def copy_frame_scripts(frame, scripts_root, script_patterns):
    '''
    Copy scripts for a frame to the appropriate destination folder

    frame: str
        The name of the frame
    scripts_root: str
        The root folder to copy the scripts to
    script_patterns: list
        A list of glob patterns to match the scripts to copy. Each pattern
        can be a string or a tuple of two strings. If a tuple, the first
        string is the pattern to match, and the second string is the
        destination folder or destination file (to rename the script). Renaming
        is only allowed for a single file (no wildcards in the pattern). If
        a single string, the script will be copied to the root of the
        destination folder.
    '''
    for script_pattern in script_patterns:
        # Determine the destination path
        if isinstance(script_pattern, tuple) and len(script_pattern) == 2:
            script_pattern, script_dest = script_pattern
            script_dest = os.path.join(scripts_root, script_dest)
        elif isinstance(script_pattern, str):
            script_dest = scripts_root
        else:
            raise ValueError(f'Script entry in {frame} is invalid. Must be a string or a tuple of two strings')

        # If script_dest ends in .lua, then we are renaming a single file
        if script_dest.endswith('.lua'):
            file = os.path.join(os.path.dirname(__file__), script_pattern)
            if not os.path.exists(file):
                raise FileNotFoundError(f'File not found: {script_pattern} in {frame}')
            script_list = [file]
        else:
            # Expand the pattern
            script_list = glob.glob(
                os.path.join(os.path.dirname(__file__), script_pattern)
            )
            if not script_list:
                raise FileNotFoundError(f'No scripts found for pattern {script_pattern} in {frame}')

        # Install each script
        for script in script_list:
            if script_dest.endswith('.lua'):
                os.makedirs(os.path.dirname(script_dest), exist_ok=True)
            else:
                os.makedirs(script_dest, exist_ok=True)
            shutil.copy(script, script_dest)


def main():
    parser = argparse.ArgumentParser(description='Generate a bundle of scripts for a vehicle')
    parser.add_argument('firmware_id', help='The firmware id of the vehicle')
    args = parser.parse_args()

    artifacts_root = 'artifacts'

    frames = vehicleinfo.VehicleInfo().options['Carbonix']['frames']
    for frame in frames:
        frame_root = os.path.join(artifacts_root, frame + '-' + args.firmware_id)
        if os.path.exists(frame_root):
            shutil.rmtree(frame_root)
        os.makedirs(frame_root)
        # copy the default parameter file
        defaults = frames[frame].get('default_params_filename', '')
        if defaults:
            defaults = os.path.join(os.path.dirname(__file__), defaults)
            defaults = os.path.abspath(defaults)
            shutil.copy(defaults, frame_root)
        # copy the scripts
        script_patterns = frames[frame].get('scripts', [])
        scripts_root = os.path.join(frame_root, 'scripts')
        copy_frame_scripts(frame, scripts_root, script_patterns)
        # create the batch file launcher
        with open(os.path.join(frame_root, 'launch.bat'), 'w') as f:
            f.write('rem Launch at Eli Field\r\n')
            f.write('cd %~dp0\r\n')
            launch_line = f'..\\{args.firmware_id}.exe'
            launch_line += ' -O 40.0594626,-88.5513292,206.0,0'
            launch_line += ' --serial0 tcp:0'
            launch_line += f' -M {frames[frame].get("model", frame)}'
            launch_line += '\r\n'
            f.write(launch_line)


if __name__ == "__main__":
    main()
