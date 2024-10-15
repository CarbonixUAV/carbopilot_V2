#!/usr/bin/env python

"""
Parameter File Validation Script

This script validates a set of parameter files against the generated metadata
for a specific firmware type (e.g. Plane, AP_Periph). The following checks are
performed on each parameter in the files:

- Ensure that the parameter exists in the metadata.
- Validate that the parameter's value falls within a specified range, if applicable.
- Confirm that the parameter's value matches one of the predefined valid values.
- Check bitmask parameters to ensure all set bits are valid according to the metadata.

Usage:
    python validate_params.py --vehicle <vehicle_type> <parameter_files>

Where:
    - `<vehicle_type>` is the type of vehicle for which to generate metadata.
      Valid values are: Sub, Plane, Blimp, Copter, Tracker, Rover, AP_Periph.
    - `<parameter_files>` is a list of parameter file paths to validate,
       supporting wildcard patterns.

Example:
    python Tools/Carbonix_scripts/param_check.py --vehicle Plane Tools/autotest/default_params/*plane.parm

AP_FLAKE8_CLEAN

"""

import os
import json
import re
import glob
import subprocess
from fnmatch import fnmatch
from argparse import ArgumentParser


def parse_arguments():
    """Parses command-line arguments, and expands any wildcard patterns."""
    parser = ArgumentParser(description='Validate parameter files')
    parser.add_argument('files', nargs='+', help='Parameter files to validate')
    parser.add_argument(
        '--vehicle', required=True, help='Vehicle type to generate for'
    )

    # Add flags for enabling/disabling checks
    parser.add_argument('--no-missing', action='store_true', help='Disable missing check')
    parser.add_argument('--no-redefinition', action='store_true', help='Disable redefinition check')
    parser.add_argument('--no-readonly', action='store_true', help='Disable read-only check')
    parser.add_argument('--no-bitmask', action='store_true', help='Disable bitmask check')
    parser.add_argument('--no-range', action='store_true', help='Disable range check')
    parser.add_argument('--no-values', action='store_true', help='Disable values check')

    args = parser.parse_args()

    # Expand any wildcards in the list of files
    files = []
    for pattern in args.files:
        files += glob.glob(pattern, recursive=True)
    args.files = files

    return args


def main():
    """Main function for the script.

    Generates the metadata for the specified vehicle, and checks each file
    against the metadata. It prints a success or failure message for each file
    and exits with an error code if any file fails validation.
    """
    args = parse_arguments()

    if not args.files:
        print('No files found')
        exit(1)

    metadata = generate_metadata(args.vehicle)

    # Dictionary to store error messages for each file
    messages = {} # {filename: [error messages]}

    # Check each file, and store any error messages
    for file in args.files:
        msgs = check_file(file, metadata, args)
        messages[os.path.relpath(file)] = msgs

    # Print the success/failure for each file
    for file in messages:
        if not messages[file]:
            print(f'{file}: Passed')
        else:
            print(f'{file}: Failed')
            for msg in messages[file]:
                print(f'  {msg}')

    # Check if any files failed (i.e. have error messages)
    if any(messages[file] for file in messages):
        exit(1)


def check_file(file, metadata, args=None):
    """Checks a single parameter file against the metadata.

    Loads the parameters from the specified file and validates each parameter
    against the provided metadata dictionary. It returns a list of error
    messages for any invalid parameters found, or an empty list if all
    parameters are valid.

    Args:
        file (str): The path to the parameter file to be checked.
        metadata (dict): A dictionary containing the parameter metadata.
        args (Namespace): An optional namespace containing flags to skip checks.

    Returns:
        list: A list of error messages if any parameters are invalid.
    """
    params, msgs = load_params(file, args)

    skip_missing = args.no_missing if args else False

    for param in params:
        if param not in metadata:
            if not skip_missing:
                msgs.append(f'{param} not found in metadata')
        else:
            value, checks_disabled = params[param]
            # If checks are disabled in a comment, then we enable all checks
            # regardless of args (by passing None). We want to enforce that
            # the DISABLE_CHECKS in the comment is necessary.
            msg = check_param(
                param,
                value,
                metadata[param],
                args if not checks_disabled else None,
            )
            # Assert that the DISABLE_CHECKS flag is necessary
            if checks_disabled:
                msg = (
                    f'{param} does not need DISABLE_CHECKS'
                    if msg is None
                    else None
                )
            msgs.append(msg) if msg is not None else None

    return msgs


def check_param(name, value, metadata, args=None):
    """Checks a single parameter against its metadata definition.

    Validates the specified parameter. If the metadata contains multiple types
    of validity fields (e.g. Range and Values), only the first one encountered
    will be checked in this priority order: ReadOnly, Bitmask, Range, Values.

    Args:
        name (str): The name of the parameter to be checked.
        value (float): The value of the parameter as a float.
        metadata (dict): A dictionary containing metadata for the parameter.
        args (Namespace): An optional namespace containing flags to skip checks.

    Returns:
        str: An error message if the parameter is invalid, or None otherwise.
    """
    # List of checks with their corresponding skip flags and check functions
    checks = [
        (
            'ReadOnly',
            args.no_readonly if args else False,
            lambda: f'{name} is read only' if metadata['ReadOnly'] else None,
        ),
        (
            'Bitmask',
            args.no_bitmask if args else False,
            lambda: check_bitmask(name, value, metadata['Bitmask']),
        ),
        (
            'Range',
            args.no_range if args else False,
            lambda: check_range(name, value, metadata['Range']),
        ),
        (
            'Values',
            args.no_values if args else False,
            lambda: check_values(name, value, metadata['Values']),
        ),
    ]

    for key, skip_flag, check_func in checks:
        if key in metadata:
            return check_func() if not skip_flag else None

    # If none of the above, it's automatically valid
    return None


def check_bitmask(name, value, metadata):
    """Validates a parameter against its bitmask metadata.

    Checks if the parameter value is a positive integer and if each bit set in
    the value corresponds to a valid bit in the metadata. It returns an error
    message if any invalid bits are set.

    Args:
        name (str): The name of the parameter being checked.
        value (float): The value of the parameter as a float.
        metadata (dict): A dictionary containing bitmask definitions.

    Returns:
        str: An error message if the parameter is invalid, or None otherwise.
    """
    if not float(value).is_integer():
        return f'{name}: {value:g} is not an integer'

    value = int(float(value))

    # Maximum number of expected bits
    N = 64

    if value < 0:
        return f'{name}: {value} is negative'

    if value >= (1 << N):
        return f'{name}: {value} is larger than {N} bits'

    # Loop through the set bits and confirm all correspond to something
    for i in range(64):
        # Break if we've checked the highest set bit
        if value < (1 << i):
            break
        # Check if the bit is set, and if so, if it's described in the metadata
        if value & (1 << i) and str(i) not in metadata:
            return f'{name}: bit {i} is not valid'

    return None


def check_range(name, value, metadata):
    """Validates a parameter against its range metadata.

    Checks if the parameter value falls within the defined minimum and maximum
    range in the metadata. It returns an error message if the value is out of
    range.

    Args:
        name (str): The name of the parameter being checked.
        value (float): The value of the parameter as a float.
        metadata (dict): A dictionary containing the 'low' and 'high' range.

    Returns:
        str: An error message if the parameter is invalid, or None otherwise.
    """

    if value < float(metadata['low']):
        return f'{name}: {value:g} is below minimum value {metadata["low"]}'
    if value > float(metadata['high']):
        return f'{name}: {value:g} is above maximum value {metadata["high"]}'

    return None


def check_values(name, value, metadata):
    """Validates a parameter against its list of valid values.

    Checks if the parameter value is one of the valid values listed in the
    metadata. It returns an error message if the value is not in the list.

    Args:
        name (str): The name of the parameter being checked.
        value (float): The value of the parameter as a float.
        metadata (dict): A dictionary containing valid values for the parameter.

    Returns:
        str: An error message if the parameter is invalid, or None otherwise.
    """
    if f'{float(value):g}' not in metadata:
        return f'{name}: {value:g} is not a valid value'

    return None


def load_params(file, args=None, depth=0):
    """Loads a parameter file and returns parameters and errors.

    Reads the specified parameter file, stripping out comments. It checks the
    comments for a DISABLE_CHECKS flag. It builds a dictionary of parameters
    and their values/disabled checks, and returns it along with any errors
    encountered.

    It will also check for redefinition of parameters within a file, including
    handling @include and @delete directives, and log an error unless
    args.no_redefinition is set.

    Args:
        file (str): The path to the parameter file to be loaded.
        args (Namespace): An optional namespace containing flags to skip checks.
        depth (int): The depth of the recursive call to prevent infinite loops.

    Returns:
        tuple: (params, errors)
            - params (dict): A dictionary where the keys are parameter names
              and the values are tuples containing the parameter value as a
              float and a boolean indicating if checks are disabled.
            - errors (list): A list of error messages encountered during
              parsing.
    """
    if depth > 10:
        raise ValueError("Too many levels of @include")

    params = {}
    errors = []

    skip_redefinition = args.no_redefinition if args else False

    with open(file, 'r') as file_object:
        lines = file_object.readlines()

    # Build up seen parameters to check for redefinition
    seen_params = set()
    for line in lines:
        if line.startswith('@include'):
            rel_path = line.split(maxsplit=1)[1].strip()
            path = os.path.join(os.path.dirname(file), rel_path)
            params2, _ = load_params(path, args, depth + 1)
            seen_params.update(params2.keys())
        elif line.startswith('@delete'):
            pattern = line.split(maxsplit=1)[1].strip()
            seen_params = {
                p for p in seen_params if not fnmatch(p, pattern)
            }

    for i, line in enumerate(lines):
        # Strip whitespace
        processed_line = line.strip()

        # Skip any lines that start with @ to ignore includes and deletes
        if processed_line.startswith('@'):
            continue

        # Strip out stuff like the @READONLY tag
        processed_line = re.sub(r'@\S*', '', processed_line)

        # Handle comments
        comment = ''
        if '#' in processed_line:
            i = processed_line.index('#')
            comment = processed_line[i + 1:].strip()
            processed_line = processed_line[:i].strip()

        # Check for DISABLE_CHECKS flag
        checks_disabled = 'DISABLE_CHECKS' in comment

        # Check that a valid reason is provided if checks are disabled
        explanation = ''.join(comment.split('DISABLE_CHECKS')).strip()
        if checks_disabled and len(explanation) < 5:
            errors.append(
                f'Explanation required for disabled checks: `{line.strip()}`'
            )
            continue  # Skip to next line after logging the error

        # Strip whitespace again
        processed_line = processed_line.strip()

        # Skip any empty lines
        if not processed_line:
            continue

        # Split on , or any whitespace
        parts = re.split(r'[,\s]+', processed_line, 1)

        # Check for redefinition
        if parts[0] in seen_params and not skip_redefinition:
            errors.append(f'{parts[0]} redefined')
        seen_params.add(parts[0])

        # Try to convert the value string to a float
        try:
            value = parts[1]
            if '.' in value:
                value = float(value)  # Convert hex to float
            else:
                # int(x, 0) will handle 0x and 0b prefixes
                value = float(int(value, 0))  # Convert hex to float
            # Store the parameter in the dictionary
            params[parts[0]] = (value, checks_disabled)
        except (IndexError, ValueError):
            errors.append(f'Error parsing line: `{line.strip()}`')

    return params, errors


def generate_metadata(vehicle):
    """Generates and returns metadata for a specific vehicle.

    Runs an external script to generate the metadata for the specified vehicle
    in JSON format. It checks if the metadata file was successfully created and
    updated, loads the metadata, flattens it to remove parameter groups, and
    returns it as a dictionary. The temporary metadata file is then deleted
    after processing.

    Args:
        vehicle (str): The type of vehicle for which to generate metadata.

    Returns:
        dict: A dictionary with parameter names as keys and metadata for that
        parameter as values.

    Raises:
        SystemExit: If the metadata generation fails, the metadata file is not
        created, or the file was not updated.
    """

    print(f'Generating metadata for vehicle {vehicle}...', end=' ')

    metadata_script = os.path.join(
        os.path.dirname(__file__), '../autotest/param_metadata/param_parse.py'
    )
    metadata_script = os.path.abspath(metadata_script)
    metadata_file = 'apm.pdef.json'
    # If the metadata file already exists, store the modification time so we
    # can check if it gets updated
    previous_mtime = (
        os.path.getmtime(metadata_file) if os.path.exists(metadata_file) else 0
    )

    try:
        subprocess.run(
            ['python3', metadata_script, f'--vehicle={vehicle}', '--format=json'],
            check=True,
            capture_output=True,
            text=True
        )
    except subprocess.CalledProcessError as e:
        print(f'Error generating metadata: {e.stderr}')
        exit(1)

    if not os.path.exists(metadata_file):
        print(f"Error: Metadata file '{metadata_file}' was not created.")
        exit(1)

    current_mtime = os.path.getmtime(metadata_file)
    if current_mtime <= previous_mtime:
        print(f"Error: Metadata file '{metadata_file}' was not updated.")
        exit(1)

    print('Done')

    with open(metadata_file, 'r') as file:
        json_file = json.load(file)

    # Delete the metadata file, as we don't need it anymore
    os.remove(metadata_file)

    # Flatten the json file to remove the parameter groups
    metadata = {}
    for group in json_file:
        metadata.update(json_file[group])

    return metadata


if __name__ == '__main__':
    main()
