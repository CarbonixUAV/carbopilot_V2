"""

Prepare folders, scripts, and extra files for SITL testing in VSCode.

For use with a launch.json file like this:

```json
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "SITL Debug",
            "type": "cppdbg",
            "request": "launch",
            "preLaunchTask": "config/build sitl",
            "cwd": "${workspaceFolder}/.vscode/${input:aircraft}",
            "program": "${workspaceFolder}/build/sitl/bin/arduplane",
            "args": [
                "-M", "${input:aircraft_model}",
                "-O", "40.0594626,-88.5513292,206.0,0", // Eli Field
                "--defaults", "${workspaceFolder}/build/${input:aircraft}.parm",
            ],
            "miDebuggerPath": "/usr/bin/gdb",
            "postRemoteConnectCommands": [
                {
                    "description": "Set breakpoint at AP_HAL::panic",
                    "text": "-break-insert AP_HAL::panic",
                    "ignoreFailures": false
                }
            ],
            "MIMode": "gdb",
            "launchCompleteCommand": "exec-run",
        },
    ],
    "inputs": [
        {
            "id": "aircraft",
            "type": "command",
            "command": "shellCommand.execute",
            "args": {
                "command": "python ${workspaceFolder}/Tools/autotest/cx_prep_sitl.py",
            }
        },
        {
            "id": "aircraft_model",
            "type": "command",
            "command": "shellCommand.execute",
            "args": {
                "command": "cat ${workspaceFolder}/.vscode/${input:aircraft}.model",
                "useSingleResult": true,
            }
        },
    ]
}
```
"""

import os
import glob
from pysim import vehicleinfo

script_dir = os.path.dirname(__file__)
output_dir = os.path.join(script_dir, '../../.vscode')

vinfo = vehicleinfo.VehicleInfo()

# Get the list of vehicles
frame_opts = vinfo.options['Carbonix']['frames']
vehicles = list(frame_opts.keys())

for vehicle in vehicles:
    opts = frame_opts[vehicle]

    # Create the directory for the vehicle
    vehicle_dir = os.path.join(output_dir, vehicle)
    os.makedirs(vehicle_dir, exist_ok=True)

    # Write the model name to a file
    model = opts['model']
    if model == 'flightaxis':
        model += f':{os.environ["wslhost"]}'
    with open(os.path.join(output_dir, f'{vehicle}.model'), 'w') as f:
        f.write(model)

    # Set up scripts
    # Delete all symlinks in the scripts folder
    scripts_dir = os.path.join(vehicle_dir, 'scripts')
    if os.path.exists(scripts_dir):
        for script in os.listdir(scripts_dir):
            script_path = os.path.join(scripts_dir, script)
            if os.path.islink(script_path):
                os.remove(script_path)
    for pattern in opts['scripts']:
        # pattern is either a string or a tuple of two strings. If it is a
        # string, it's a single filepath or a glob pattern. If it is a tuple,
        # the first element is the filepath/glob and the second element is the
        # destination folder or the desination filepath.
        dst_folder = os.path.join(vehicle_dir, 'scripts')
        dst_folder = os.path.abspath(dst_folder)
        if isinstance(pattern, tuple):
            dst_folder = os.path.join(dst_folder, pattern[1])
            pattern = pattern[0]
        pattern = os.path.abspath(os.path.join(script_dir, pattern))

        # If dst_folder has an extension at the end, it is a file
        file_rename = None
        if os.path.splitext(dst_folder)[1]:
            # If the destination folder has an extension, it is a file
            # and we need to rename the script to that file
            file_rename = os.path.basename(dst_folder)
            dst_folder = os.path.dirname(dst_folder)
        os.makedirs(dst_folder, exist_ok=True)

        # Create a symlink for each script
        for script in glob.glob(pattern):
            # Get the filename from the path
            filename = os.path.basename(script)
            if file_rename:
                filename = file_rename
            dst = os.path.join(dst_folder, filename)
            # Create the symlink
            if os.path.exists(dst):
                os.remove(dst)
            os.symlink(os.path.abspath(script), dst)

    # Copy the defaults file (string or list of strings)
    defaults = opts['default_params_filename']
    if not isinstance(defaults, str):
        raise ValueError("cx_prep_sitl.py does not support lists of default param files")
    defaults = os.path.abspath(os.path.join(script_dir, defaults))
    dst = os.path.join(vehicle_dir, "defaults.parm")
    if os.path.islink(dst) or os.path.exists(dst):
        os.remove(dst)
    os.symlink(defaults, dst)

# Dump the list of vehicles to stdout to populate the dropdown
print('\n'.join(vehicles))
