#!/usr/bin/env python

"""
Parameter File Checker Unit Tests

AP_FLAKE8_CLEAN
"""

import os
import time
import unittest
from unittest.mock import MagicMock, patch, mock_open
from param_check import (
    load_params,
    check_param,
    check_range,
    check_values,
    check_bitmask,
    generate_metadata,
    check_file,
    main
)


class TestParamCheck(unittest.TestCase):

    def test_load_params(self):
        # Mock parameter file content
        mock_file_content = '''
        PARAM1, 1
        PARAM2\t0x10 @READONLY # Comment
        # Comment
        @tag
        @include filename.parm
        PARAM3   42.5#comment
        PARAM4, 0b1111 #DISABLE_CHECKS for some reason
        '''

        with patch('builtins.open', mock_open(read_data=mock_file_content)):
            params, msgs = load_params('fake_file.parm')

        # Test the correct parsing of parameters and DISABLE_CHECKS flag
        self.assertEqual(params['PARAM1'], (1.0, False))
        self.assertEqual(params['PARAM2'], (16.0, False))
        self.assertEqual(params['PARAM3'], (42.5, False))
        self.assertEqual(params['PARAM4'], (15.0, True))
        self.assertEqual(msgs, [])

        # Bad parameter lines that should fail to parse and exit the program
        bad_lines = [
            'PARAM4 1 2\n',
            'PARAM4\n',
            'P#ARAM4 1\n',
            'PARAM4, 0x10.0\n',
        ]
        for line in bad_lines:
            content = mock_file_content + line
            with patch('builtins.open', mock_open(read_data=content)):
                _, msgs = load_params('fake_file.parm')
            self.assertEqual(msgs, [f'Error parsing line: `{line.strip()}`'])

        # Test that the program exits when DISABLE_CHECKS does not have a reason
        content = mock_file_content + 'PARAM4, 0x10 #DISABLE_CHECKS: \n'
        with patch('builtins.open', mock_open(read_data=content)):
            _, msgs = load_params('fake_file.parm')
        self.assertEqual(msgs, ['Explanation required for disabled checks: `PARAM4, 0x10 #DISABLE_CHECKS:`'])

    def test_check_range(self):
        # Mock metadata and parameters
        metadata = {'low': 0.0, 'high': 100.0}
        self.assertIsNone(check_range('PARAM', 50.0, metadata))
        self.assertEqual(check_range('PARAM', -1.0, metadata), 'PARAM: -1 is below minimum value 0.0')
        self.assertEqual(check_range('PARAM', 101.0, metadata), 'PARAM: 101 is above maximum value 100.0')

    def test_check_values(self):
        # Mock metadata and parameters
        metadata = {'0': 'Off', '1': 'On'}
        self.assertIsNone(check_values('PARAM', 0, metadata))
        self.assertEqual(check_values('PARAM', 2, metadata), 'PARAM: 2 is not a valid value')

    def test_check_bitmask(self):
        # Mock metadata and parameters
        metadata = {'0': 'Bit0', '1': 'Bit1'}
        self.assertIsNone(check_bitmask('PARAM', 3.0, metadata))
        self.assertEqual(check_bitmask('PARAM', 4.0, metadata), 'PARAM: bit 2 is not valid')
        self.assertEqual(check_bitmask('PARAM', 1.5, metadata), 'PARAM: 1.5 is not an integer')

    @patch('subprocess.run')
    @patch('os.remove')
    @patch('os.path.getmtime')
    @patch('os.path.exists', return_value=True)
    @patch('builtins.open', new_callable=mock_open, read_data='{}')
    @patch('json.load', return_value={'GROUP1': {'PARAM1': {}}, 'GROUP2': {'PARAM2': {}}})
    def test_generate_metadata(self, mock_json, mock_open, mock_exists, mock_getmtime, mock_remove, mock_subprocess):
        # When the function calls getmtime, it will return the current time
        mock_getmtime.side_effect = lambda path: time.time()

        # Call the function
        metadata = generate_metadata('Plane')

        # Test subprocess was called correctly
        metadata_script = os.path.join(
            os.path.dirname(__file__), '../autotest/param_metadata/param_parse.py'
        )
        metadata_script = os.path.abspath(metadata_script)
        mock_subprocess.assert_called_once_with(
            ['python3', metadata_script, '--vehicle=Plane', '--format=json'],
            check=True,
            capture_output=True,
            text=True
        )

        # Test that the metadata was loaded and that the json was flattened
        self.assertEqual(metadata, {'PARAM1': {}, 'PARAM2': {}})

    def test_check_param(self):
        metadata = {
            'Description': 'This is a test parameter',
            'ReadOnly': True,
            'Bitmask': {'0': 'Bit0', '1': 'Bit1', '8': 'Bit8'},
            'Range': {'low': '0.0', 'high': '100.0'},
            'Values': {'0': 'Off', '1': 'On'}
        }
        args = type('', (), {})()  # Creating a simple object to simulate args
        args.no_readonly = False
        args.no_bitmask = False
        args.no_range = False
        args.no_values = False

        # Test ReadOnly
        self.assertEqual(check_param('PARAM', 0, metadata, args), 'PARAM is read only')
        args.no_readonly = True
        self.assertIsNone(check_param('PARAM', 0, metadata, args))

        # Test Bitmask
        del metadata['ReadOnly']  # Remove ReadOnly to test the next priority
        self.assertIsNone(check_param('PARAM', 256.0, metadata, args)) # 256 would fail the other checks, but pass bitmask
        self.assertEqual(check_param('PARAM', 1.5, metadata, args), 'PARAM: 1.5 is not an integer')
        self.assertEqual(check_param('PARAM', -1, metadata, args), 'PARAM: -1 is negative')
        self.assertEqual(
            check_param('PARAM', 18446744073709551616, metadata, args),
            'PARAM: 18446744073709551616 is larger than 64 bits'
        )
        self.assertEqual(check_param('PARAM', 4, metadata, args), 'PARAM: bit 2 is not valid')
        args.no_bitmask = True
        self.assertIsNone(check_param('PARAM', 4, metadata, args))

        # Test Range
        del metadata['Bitmask']  # Remove Bitmask to test the next priority
        self.assertIsNone(check_param('PARAM', 50, metadata, args)) # 50 will fail the values check, but pass the range check
        self.assertEqual(check_param('PARAM', 101, metadata, args), 'PARAM: 101 is above maximum value 100.0')
        self.assertEqual(check_param('PARAM', -1, metadata, args), 'PARAM: -1 is below minimum value 0.0')
        args.no_range = True
        self.assertIsNone(check_param('PARAM', -1, metadata, args))

        # Test Values
        del metadata['Range']  # Remove Range to test the next priority
        self.assertIsNone(check_param('PARAM', 0, metadata, args))
        self.assertEqual(check_param('PARAM', 2, metadata, args), 'PARAM: 2 is not a valid value')
        args.no_values = True
        self.assertIsNone(check_param('PARAM', 2, metadata, args))

        # Test parameter with no range, bitmask, or value restrictions
        del metadata['Values']
        self.assertIsNone(check_param('PARAM', 0, metadata, args)) # Should pass no matter what

    @patch('param_check.check_param')
    def test_check_file(self, mock_check_param):
        mock_args = type('', (), {})()  # Creating a simple object to simulate args
        mock_args.no_missing = False

        # Case 1: All parameters pass their checks
        mock_file_content = "PARAM1, 10\nPARAM2, 20\n"
        mock_metadata = {'PARAM1': {}, 'PARAM2': {}}
        # Mock check_param to return None for all params, indicating they pass validation
        mock_check_param.side_effect = lambda name, value, metadata, args: None
        with patch('builtins.open', mock_open(read_data=mock_file_content)):
            msgs = check_file('fake_file.parm', mock_metadata, mock_args)
        # Check that no error messages were returned
        self.assertEqual(msgs, [])
        # Check that check_param was called for each parameter
        self.assertEqual(mock_check_param.call_count, 2)

        # Case 2: Missing parameter (PARAM3 not in metadata)
        mock_file_content += "PARAM3, 30\n"
        with patch('builtins.open', mock_open(read_data=mock_file_content)):
            msgs = check_file('fake_file.parm', mock_metadata, mock_args)
        # Check that a missing parameter error is reported
        self.assertEqual(msgs, ['PARAM3 not found in metadata'])

        # Case 3: Missing parameter but with no-missing flag
        mock_args.no_missing = True
        with patch('builtins.open', mock_open(read_data=mock_file_content)):
            msgs = check_file('fake_file.parm', mock_metadata, mock_args)
        # Check that no error messages are returned when no-missing is enabled
        self.assertEqual(msgs, [])

        # Case 4: Valid parameter with DISABLE_CHECKS flag, and invalid parameter
        # (should report both errors)
        mock_file_content = "PARAM1, 50.0 # DISABLE_CHECKS: reason\nPARAM2, 0\n"
        # Mock check_param so PARAM1 is valid, but not PARAM2
        mock_check_param.side_effect = lambda name, value, metadata, args: (
            None if name in ['PARAM1'] else f'{name}: Error'
        )
        with patch('builtins.open', mock_open(read_data=mock_file_content)):
            msgs = check_file('fake_file.parm', mock_metadata, mock_args)
        self.assertEqual(
            msgs,
            [
                'PARAM1 does not need DISABLE_CHECKS',
                'PARAM2: Error',
            ],
        )

        # Case 5: Invalid parameter but with DISABLE_CHECKS (should pass)
        mock_file_content = "PARAM1, 150.0\nPARAM2, 200.0 # DISABLE_CHECKS: reason\n"
        with patch('builtins.open', mock_open(read_data=mock_file_content)):
            msgs = check_file('fake_file.parm', mock_metadata, mock_args)
        # Check that no error messages are returned because DISABLE_CHECKS is valid
        self.assertEqual(msgs, [])

    @patch('param_check.parse_arguments')
    @patch('param_check.generate_metadata')
    @patch('param_check.check_file')
    @patch('builtins.print')
    def test_main(self, mock_print, mock_check_file, mock_generate_metadata, mock_parse_arguments):

        # Setup mock for parse_arguments
        mock_args = MagicMock()
        mock_args.vehicle = 'Plane'
        mock_args.files = ['file1.parm', 'file2.parm']
        mock_parse_arguments.return_value = mock_args

        # Setup mock for generate_metadata
        mock_generate_metadata.return_value = {'PARAM1': {}, 'PARAM2': {}}

        # Setup mock for check_file
        mock_check_file.side_effect = lambda file, metadata, args: [] if file == 'file1.parm' else ['Error']

        # Call main function
        with patch('sys.argv', ['param_check.py']):
            with self.assertRaises(SystemExit):
                main()

        # Check that generate_metadata was called correctly
        mock_generate_metadata.assert_called_once_with('Plane')

        # Check that check_file was called for each expanded file
        self.assertEqual(mock_check_file.call_count, 2)
        mock_check_file.assert_any_call('file1.parm', {'PARAM1': {}, 'PARAM2': {}}, mock_args)
        mock_check_file.assert_any_call('file2.parm', {'PARAM1': {}, 'PARAM2': {}}, mock_args)

        # Check that print was called correctly
        mock_print.assert_any_call('file1.parm: Passed')
        mock_print.assert_any_call('file2.parm: Failed')
        mock_print.assert_any_call('  Error')

        # Test that the program exits when no files are provided
        mock_print.reset_mock()
        mock_args.files = []
        mock_parse_arguments.return_value = mock_args
        with patch('sys.argv', ['param_check.py']):
            with self.assertRaises(SystemExit):
                main()

        mock_print.assert_called_once_with('No files found')


if __name__ == '__main__':
    unittest.main()
