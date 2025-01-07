import unittest
from unittest.mock import patch, MagicMock
import struct

# Import the functions and the script itself
from cx_apj_tool import replace_board_name, fix_app_descriptor, to_unsigned, __main__ as cx_apj_tool_main


class TestReplaceBoardName(unittest.TestCase):
    def test_replace_valid(self):
        image = b"This is the old_board_name and nothing else"
        old_name = "old_board_name"
        new_name = "new_name"
        updated_image = replace_board_name(image, old_name, new_name)
        self.assertEqual(updated_image, b"This is the new_name\x00\x00\x00\x00\x00\x00 and nothing else")

    def test_old_name_not_found(self):
        image = b"This is the old_board_name and nothing else"
        old_name = "missing_name"
        new_name = "new_name"
        with self.assertRaises(LookupError):
            replace_board_name(image, old_name, new_name)

    def test_old_name_appears_multiple_times(self):
        image = b"This is the old_board_name and old_board_name again"
        old_name = "old_board_name"
        new_name = "new_name"
        with self.assertRaises(AssertionError):
            replace_board_name(image, old_name, new_name)

    def test_new_name_too_long(self):
        image = b"This is the old_board_name and nothing else"
        old_name = "old_board_name"
        new_name = "new_board_name_too_long"
        with self.assertRaises(ValueError):
            replace_board_name(image, old_name, new_name)


class TestFixAppDescriptor(unittest.TestCase):
    def test_fix_valid_descriptor(self):
        # Mock an image with a valid descriptor
        descriptor = b'\x40\xa2\xe4\xf1\x64\x68\x91\x06'
        githash = 0x12345678
        img_len = 64
        crc1 = 0x11111111
        crc2 = 0x22222222
        app_descriptor = struct.pack('<IIII', crc1, crc2, img_len, githash)
        image = b'\x00' * 8 + descriptor + app_descriptor + b'\x00' * (img_len - 32)

        def mock_crc32(data):
            return 0x33333333  # Return a mock CRC value

        with patch('cx_apj_tool.crc32', side_effect=mock_crc32):
            updated_image = fix_app_descriptor(image)

        # Extract and validate the updated descriptor
        new_crc1, new_crc2, new_img_len, new_githash = struct.unpack('<IIII', updated_image[16:32])
        self.assertEqual(new_crc1, 0x33333333)
        self.assertEqual(new_crc2, 0x33333333)
        self.assertEqual(new_img_len, img_len)
        self.assertEqual(new_githash, githash)

    def test_descriptor_not_found(self):
        image = b"This binary does not contain a descriptor"
        with self.assertRaises(LookupError):
            fix_app_descriptor(image)

    def test_image_length_mismatch(self):
        descriptor = b'\x40\xa2\xe4\xf1\x64\x68\x91\x06'
        githash = 0x12345678
        img_len = 64
        crc1 = 0x11111111
        crc2 = 0x22222222
        app_descriptor = struct.pack('<IIII', crc1, crc2, img_len, githash)
        image = b'\x00' * 8 + descriptor + app_descriptor + b'\x00' * (img_len - 32)

        # Modify image length to simulate a mismatch
        image = image[:-1]

        with self.assertRaises(AssertionError):
            fix_app_descriptor(image)


class TestToUnsigned(unittest.TestCase):
    def test_positive_value(self):
        self.assertEqual(to_unsigned(10), 10)

    def test_negative_value(self):
        self.assertEqual(to_unsigned(-10), 4294967286)  # 2^32 - 10


class TestMain(unittest.TestCase):
    @patch("cx_apj_tool.argparse.ArgumentParser.parse_args")
    @patch("cx_apj_tool.embedded_defaults")
    def test_main_with_replace_board_name(self, mock_defaults, mock_parse_args):
        args = type('', (), {})()
        args.bin_file = "input.bin"
        args.old_board_name = "old_name"
        args.new_board_name = "new_name"
        args.defaults = None
        args.output = "output.bin"
        mock_parse_args.return_value = args

        # Mock of the embedded_defaults object. Contains a firmware bytes
        # object and a set_file() method, which modifies the firmware bytes
        mock_defaults.return_value = EmbeddedDefaultsMock()

        cx_apj_tool_main()

        # Assert that the board name was replaced in the firmware field
        expected_firmware = b"stuff\x40\xa2\xe4\xf1\x64\x68\x91\x06\x18\x1c\x16\x2e\x7a\x33\x3d\x6e\x2e\x00\x00\x00\x00\x00\x00\x00new_namemorestuff" # noqa
        self.assertEqual(mock_defaults.return_value.firmware, expected_firmware)

        mock_defaults.assert_called_once_with("input.bin")
        mock_defaults.return_value.save.assert_called_once()

    @patch("cx_apj_tool.argparse.ArgumentParser.parse_args")
    @patch("cx_apj_tool.embedded_defaults")
    def test_main_with_defaults(self, mock_defaults, mock_parse_args):
        args = type('', (), {})()
        args.bin_file = "input.bin"
        args.old_board_name = None
        args.new_board_name = None
        args.defaults = "defaults_file"
        args.output = "output.bin"
        mock_parse_args.return_value = args

        mock_defaults.return_value = EmbeddedDefaultsMock()

        cx_apj_tool_main()

        # Assert that the parameter set_file method worked as expected
        expected_firmware = b"STUFF\x40\xa2\xe4\xf1\x64\x68\x91\x06\xb1\x82\x69\xce\x61\xc1\xab\x96\x2e\x00\x00\x00\x00\x00\x00\x00old_namemoreSTUFF" # noqa
        self.assertEqual(mock_defaults.return_value.firmware, expected_firmware)

        mock_defaults.return_value.find.assert_called_once()
        mock_defaults.return_value.set_file.assert_called_once_with("defaults_file")
        mock_defaults.return_value.save.assert_called_once()

    @patch("cx_apj_tool.argparse.ArgumentParser.parse_args")
    @patch("cx_apj_tool.argparse.ArgumentParser.error")
    def test_main_error_on_missing_args(self, mock_error, mock_parse_args):
        # Define test cases for invalid inputs
        test_cases = [
            {
                "test_name": "missing_both_old_and_new_board_name",
                "old_board_name": None,
                "new_board_name": None,
                "defaults": None,
                "expected_error": "Nothing to be done. Please provide --old-board-name or --defaults",
            },
            {
                "test_name": "new_board_name_without_old",
                "old_board_name": None,
                "new_board_name": "new_name",
                "defaults": None,
                "expected_error": "--new-board-name requires --old-board-name",
            },
            {
                "test_name": "old_board_name_without_new",
                "old_board_name": "old_name",
                "new_board_name": None,
                "defaults": None,
                "expected_error": "--old-board-name requires --new-board-name",
            },
            {
                "test_name": "new_name_too_long",
                "old_board_name": "short_name",
                "new_board_name": "this_name_is_way_too_long",
                "defaults": None,
                "expected_error": "--new-board-name too long to fit in the space occupied by --old-board-name",
            },
        ]

        for case in test_cases:
            with self.subTest(test_name=case["test_name"]):
                # Mock the args object for the current test case
                args = type('', (), {})()
                args.bin_file = "input.bin"
                args.old_board_name = case["old_board_name"]
                args.new_board_name = case["new_board_name"]
                args.defaults = case["defaults"]
                args.output = "output.bin"
                mock_parse_args.return_value = args

                # Configure the mock error method to raise SystemExit
                def mock_error_side_effect(message):
                    self.assertEqual(message, case["expected_error"])
                    raise SystemExit(2)

                mock_error.side_effect = mock_error_side_effect

                with self.assertRaises(SystemExit) as cm:
                    cx_apj_tool_main()

                # Assert the exit code is correct
                self.assertEqual(cm.exception.code, 2)

                # Assert that parser.error was called exactly once
                mock_error.assert_called_once()

                # Reset the mock for the next test case
                mock_error.reset_mock()

    @patch("cx_apj_tool.argparse.ArgumentParser.parse_args")
    @patch("cx_apj_tool.embedded_defaults")
    def test_noapjtool_params(self, mock_defaults, mock_parse_args):
        args = type('', (), {})()
        args.bin_file = "input.bin"
        args.old_board_name = None
        args.new_board_name = None
        args.defaults = "defaults_file"
        args.output = "output.bin"
        mock_parse_args.return_value = args

        mock_defaults.return_value = EmbeddedDefaultsMock(has_apjtool_params=False)

        with self.assertRaises(LookupError):
            cx_apj_tool_main()


class EmbeddedDefaultsMock(MagicMock):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.firmware = b"stuff\x40\xa2\xe4\xf1\x64\x68\x91\x06\x00\x00\x00\x00\x00\x00\x00\x00\x2e\x00\x00\x00\x00\x00\x00\x00old_namemorestuff" # noqa
        self.set_file = MagicMock(side_effect=self.mock_set_file)
        self.find = MagicMock(side_effect=self.mock_find)
        self.has_apjtool_params = True
        if "has_apjtool_params" in kwargs:
            self.has_apjtool_params = kwargs["has_apjtool_params"]

    def mock_set_file(self, file_path):
        """Just needs to modify firmware in some way"""
        self.firmware = self.firmware.replace(b"stuff", b"STUFF")

    def mock_find(self):
        return bool(self.has_apjtool_params)


if __name__ == "__main__":
    unittest.main()
