'''
Fly Carbonix aircraft in SITL

AP_FLAKE8_CLEAN

'''

from quadplane import AutoTestQuadPlane
from pysim import vehicleinfo
from vehicle_test_suite import AutoTestTimeoutException, NotAchievedException
from cx_vehicle_bundle import copy_frame_scripts


class AutoTestCarbonix(AutoTestQuadPlane):
    def init(self):
        super().init()
        self.install_frame_scripts()
        self.reboot_sitl()

    def default_parameter_list(self):
        return super().default_parameter_list() | {
            "ARMING_MIS_ITEMS": 0,      # disable mission check
            "BRD_SAFETY_DEFLT": 0,      # disable safety switch
            "FENCE_AUTOENABLE": 0,      # disable fences
            "FENCE_ENABLE": 0,
            "FS_GCS_ENABL": 0,          # disable GCS failsafe
            "TERRAIN_FOLLOW": 0,        # disable terrain follow (causes prearm fail; terrain requests require extra steps)
        }

    def default_frame(self):
        return 'ottano-headless'

    def log_name(self):
        return 'Carbonix'

    def vehicleinfo_key(self):
        return 'Carbonix'

    def install_frame_scripts(self):
        '''installs all scripts specified for the frame in vehicleinfo'''
        options = vehicleinfo.VehicleInfo().options[self.vehicleinfo_key()]
        frame_bits = options['frames'][self.frame]
        script_patterns = frame_bits.get('scripts', [])
        scripts_root = self.installed_script_path('')
        copy_frame_scripts(self.frame, scripts_root, script_patterns)

    def assert_no_text(self, *args, **kwargs):
        '''Assert that a text message does not come in within a timeout'''
        try:
            text = self.wait_text(*args, **kwargs)
        except AutoTestTimeoutException:
            return
        raise AssertionError(f"Text '{text}' appeared")

    def CX_BIT(self):
        '''Test Carbonix's Built-in-Test (BIT) script'''

        def TestESCTelemetry(index):
            '''Test a single ESC'''
            index = int(index)
            self.context_push()
            self.context_collect('STATUSTEXT')
            self.wait_ready_to_arm()

            self.assert_no_text('^CX_BIT:.*', regex=True, check_context=True)

            # Fail the ESC telemetry for the specified index
            self.progress(f'Failing ESC telemetry for ESC {index}')
            self.set_parameter('SIM_ESC_TLM_FAIL', 1 << index)

            # Wait for the prearm failure and the error message
            self.wait_not_ready_to_arm()
            lost_text = f'CX_BIT: ESC {index + 1} Telemetry Lost'
            self.wait_text(lost_text, check_context=True)
            self.progress("'" + lost_text + "':" + ' Success!')

            # Clear the failure
            self.progress(f'Clearing ESC telemetry failure for ESC {index}')
            self.context_clear_collection('STATUSTEXT')
            self.set_parameter('SIM_ESC_TLM_FAIL', 0)
            recovered_text = f'CX_BIT: ESC {index + 1} Telemetry Recovered'
            self.wait_text(recovered_text, check_context=True)
            # Confirm we didn't get lost/recovered/lost/recovered during that time
            self.assert_no_text(lost_text, timeout=1, regex=True, check_context=True)

            # Engage the safety switch and trigger a telemetry failure, and
            # confirm that we don't get the telemetry lost message
            self.progress(f'Engaging safety switch and failing ESC telemetry for ESC {index}')
            self.context_clear_collection('STATUSTEXT')
            self.set_safetyswitch_on()
            self.set_parameter('SIM_ESC_TLM_FAIL', 1 << index)
            self.assert_no_text('^CX_BIT:.*', regex=True, check_context=True)
            self.set_parameter('SIM_ESC_TLM_FAIL', 0)
            self.set_safetyswitch_off()

            # And one more time, confirm no error messages are present
            self.context_clear_collection('STATUSTEXT')
            self.assert_no_text('^CX_BIT:.*', regex=True, check_context=True)
            self.context_pop()

        def TestMotorFail(esc_index, servo_index, is_pusher=False):
            '''Test a single VTOL motor failure'''
            esc_index = int(esc_index)
            servo_index = int(servo_index)
            self.context_push()
            self.context_collect('STATUSTEXT')
            self.wait_ready_to_arm()

            self.arm_vehicle()
            if is_pusher:
                self.change_mode('MANUAL')
                self.set_rc(3, 1500)
            else:
                self.change_mode('QSTABILIZE')

            # Confirm no error messages are present
            self.assert_no_text('CX_BIT.*', regex=True, check_context=True)

            # Fail the ESC telemetry for the specified index
            self.progress(f'Failing Motor {esc_index}')
            self.set_parameter('SIM_ENGINE_FAIL', 1 << servo_index)

            # Wait for the error message
            lost_text = f'CX_BIT: ESC {esc_index + 1} RPM Drop'
            self.wait_text(lost_text, check_context=True)
            self.progress("'" + lost_text + "':" + ' Success!')

            # Clear the failure
            self.progress(f'Fixing Motor {esc_index}')
            self.context_clear_collection('STATUSTEXT')
            self.set_parameter('SIM_ENGINE_FAIL', 0)
            recovered_text = f'CX_BIT: ESC {esc_index + 1} RPM Recovered'
            self.wait_text(recovered_text, check_context=True)
            # Confirm we didn't get lost/recovered/lost/recovered during that time
            self.assert_no_text(lost_text, timeout=1, regex=True, check_context=True)

            # And one more time, confirm no error messages are present
            self.context_clear_collection('STATUSTEXT')
            self.assert_no_text('CX_BIT.*', regex=True, check_context=True)
            self.disarm_vehicle()
            self.context_pop()

        def TestGPSPrearm(index):
            '''Test GPS prearm checks'''
            index = int(index)
            self.context_push()

            # Get the parameter names for the GPSs
            if index == 0:
                this_numsats = 'SIM_GPS_NUMSATS'
                other_numsats = 'SIM_GPS2_NUMSATS'
                this_fixtype = 'SIM_GPS_FIXTYPE'
            elif index == 1:
                this_numsats = 'SIM_GPS2_NUMSATS'
                other_numsats = 'SIM_GPS_NUMSATS'
                this_fixtype = 'SIM_GPS2_FIXTYPE'
            else:
                raise ValueError('Only 2 GPSs are supported')
            self.set_parameters({this_numsats: 30, other_numsats: 30})
            self.wait_ready_to_arm()

            # Reduce the number of satellites to trigger a prearm failure
            self.progress(f'Reducing number of satellites for GPS {index}')
            self.set_parameter(this_numsats, 6)
            self.wait_not_ready_to_arm()

            # Restore the number of satellites
            self.progress(f'Restoring number of satellites for GPS {index}')
            self.set_parameter(this_numsats, 30)
            self.wait_ready_to_arm()

            # Cause a large satellite count difference between the two GPSs
            self.progress('Causing large satellite count difference')
            self.set_parameters({this_numsats: 30, other_numsats: 50})
            self.wait_not_ready_to_arm()

            # Check that our script does not complain about low satellite count
            # when there is insufficient fix (the existing GPS prearm check
            # should be the one to complain about lack of fix)
            self.progress('Checking low satellite count with no fix')
            self.set_parameter(this_fixtype, 1) # No fix
            # Disable ArduPilot's GPS prearm check to use wait_ready_to_arm
            # (23 bits, except for bits 0 and 3)
            self.set_parameter('ARMING_CHECK', 0x7FFFF6)
            self.wait_ready_to_arm()

            # Restore everything
            self.context_pop()

        # Count the number of ESCs
        frame_class = self.get_parameter('Q_FRAME_CLASS')
        if frame_class == 1:  # Quad
            num_vtols = 4
        elif frame_class == 4:  # OctaQuad
            num_vtols = 8
        else:
            raise ValueError(f'Unsupported frame class {frame_class}')
        has_engine = self.get_parameter('ICE_ENABLE')

        # Find the servo assignments for the ESCs
        vtol_servos = {}
        pusher_servo = None
        for i in range(1, 32):
            try:
                assignment = self.get_parameter(f'SERVO{i}_FUNCTION')
                if 33 <= assignment <= 40:
                    vtol_servos[assignment - 33] = i - 1
                elif assignment == 70:
                    pusher_servo = i - 1
            except NotAchievedException:
                break
        assert len(vtol_servos) == num_vtols
        assert pusher_servo is not None

        self.start_subtest('Test ESC telemetry warnings')
        for i in range(num_vtols):
            TestESCTelemetry(i)
        if not has_engine:
            TestESCTelemetry(num_vtols) # Test the pusher

        self.start_subtest('Test VTOL motor failures')
        for i in range(num_vtols):
            TestMotorFail(i, vtol_servos[i])
        if not has_engine:
            TestMotorFail(num_vtols, pusher_servo, is_pusher=True)

        self.start_subtest('Test GPS')
        for i in range(2):
            TestGPSPrearm(i)

    def disabled_tests(self):
        return dict()

    def tests(self):
        return [
            self.CX_BIT,
        ]
