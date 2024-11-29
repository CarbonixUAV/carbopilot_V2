import os
import argparse
import numpy as np
from pymavlink import mavutil


def main():
    # argparse
    parser = argparse.ArgumentParser(description="Parse a log file and generate a header file for FFT replay")
    parser.add_argument("log_file", help="Path to the log file")

    args = parser.parse_args()

    output_header = os.path.join(
        'libraries/AP_GyroFFT',
        os.path.splitext(os.path.basename(args.log_file))[0] + ".h"
    )
    parse_log_and_generate_header(args.log_file, output_header)


def parse_log_and_generate_header(log_file, output_header, duration=40, target_rate=50):
    # Open the log
    mlog = mavutil.mavlink_connection(log_file)

    # Fields to extract
    timestamps = []
    ndn = []
    nf1 = []
    nf2 = []
    nf3 = []

    # Extract FTN[1] messages
    while True:
        msg = mlog.recv_match(type="FTN")
        if msg is None:
            break
        if msg.I != 1:
            continue
        if msg.TimeUS > duration * 1e6:
            break
        timestamps.append(msg.TimeUS / 1e6)  # Convert to seconds
        ndn.append(msg.NDn)
        nf1.append(msg.NF1)
        nf2.append(msg.NF2)
        nf3.append(msg.NF3)

    # Scan for the values 1s after takeoff
    ndn_last = 0
    nf1_last = 8
    nf2_last = 8
    nf3_last = 8
    takeoff_time = None
    mlog.rewind()
    while True:
        msg = mlog.recv_match(type=["QTUN", "FTN"])
        if msg is None:
            break
        if msg.get_type() == "QTUN":
            if takeoff_time is None and msg.ThO > 0.25:
                takeoff_time = msg.TimeUS / 1e6
            continue
        if takeoff_time is None:
            continue
        if msg.I != 1:
            continue

        if takeoff_time is None:
            takeoff_time = msg.TimeUS / 1e6

        ndn_last = msg.NDn
        nf1_last = msg.NF1
        nf2_last = msg.NF2
        nf3_last = msg.NF3

        if msg.TimeUS / 1e6 - takeoff_time > 1:
            break


    # Check that we found what we needed
    if not timestamps:
        print(f"Could not find FTN[1] messages in first {duration} seconds of log file.")
        return

    # Limit to the first `duration` seconds
    timestamps = np.array(timestamps)
    ndn = np.array(ndn)
    nf1 = np.array(nf1)
    nf2 = np.array(nf2)
    nf3 = np.array(nf3)

    # Resample to target rate
    target_timestamps = np.arange(0, duration, 1 / target_rate)
    # ndn_resampled = np.interp(target_timestamps, timestamps, ndn, left=ndn[0])
    # nf1_resampled = np.interp(target_timestamps, timestamps, nf1, left=nf1[0])
    # nf2_resampled = np.interp(target_timestamps, timestamps, nf2, left=nf2[0])
    # nf3_resampled = np.interp(target_timestamps, timestamps, nf3, left=nf3[0])

    ndn_resampled = np.zeros(len(target_timestamps))
    nf1_resampled = np.zeros(len(target_timestamps))
    nf2_resampled = np.zeros(len(target_timestamps))
    nf3_resampled = np.zeros(len(target_timestamps))
    for i, target_time in enumerate(target_timestamps):
        idx = np.argmin(np.abs(timestamps - target_time))
        ndn_resampled[i] = ndn[idx]
        nf1_resampled[i] = nf1[idx]
        nf2_resampled[i] = nf2[idx]
        nf3_resampled[i] = nf3[idx]

    if ndn_last > 0:
        ndn_resampled[-1] = ndn_last
        nf1_resampled[-1] = nf1_last
        nf2_resampled[-1] = nf2_last
        nf3_resampled[-1] = nf3_last
    else:
        print("Warning: could not find takeoff")

    # Generate the .h file
    with open(output_header, 'w', encoding='utf-8') as f:
        f.write("// Auto-generated header for FFT replay\n")

        f.write("#ifndef FFT_NOTCH_REPLAY_H\n")
        f.write("#define FFT_NOTCH_REPLAY_H\n\n")

        f.write(f"constexpr uint32_t SAMPLE_PERIOD_MS = {int(1000 / target_rate)};\n")
        f.write(f"constexpr size_t FFT_SAMPLES = {len(target_timestamps)};\n")

        # Write NDn array
        f.write("constexpr int FTN_NDn[FFT_SAMPLES] = {")
        f.write(", ".join(f"{int(val)}" for val in ndn_resampled))
        f.write("};\n\n")

        # Write NF arrays
        f.write("constexpr float FTN_NF[FFT_SAMPLES][3] = {\n")
        for nf1_val, nf2_val, nf3_val in zip(nf1_resampled, nf2_resampled, nf3_resampled):
            f.write(f"    {{ {nf1_val:.3f}, {nf2_val:.3f}, {nf3_val:.3f} }},\n")
        f.write("};\n")

        f.write("#endif\n")

    print(f"Header file '{output_header}' generated successfully.")

if __name__ == "__main__":
    main()
