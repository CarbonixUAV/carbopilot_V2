"""
Engine Parameter Calibration Script for ArduPilot SITL

This script is used to calibrate engine behavior parameters for ArduPilot SITL
simulations. It handles calibration for cylinder head temperature (CHT and
CHT2) and fuel consumption using flight log data. The calibration process
optimizes key fit parameters to minimize the difference between actual and
simulated values from the log data.

Features:
  - Independent calibration for CHT, CHT2, and fuel consumption.
  - Visualization of actual vs simulated results for all three metrics.

Input CSV File Format Requirements: The input file should be a CSV file with
the following columns:

  - Time: Time in milliseconds since the start of the log.
  - RPM: Engine revolutions per minute (RPM).
  - Airspeed: Airspeed in meters per second (m/s).
  - IMT: Intake manifold temperature in degrees Celsius (°C).
  - CHT: Cylinder head temperature 1 in degrees Celsius (°C).
  - CHT2: Cylinder head temperature 2 in degrees Celsius (°C).
  - FuelConsumption: Fuel consumption rate in mg/min.

The best way to generate this is using UAVLogViewer, plot these variables
(making sure to low-pass the IMT a bit), zoom into the region you want to use
for the fit, and hit "export data". Then, edit the CSV to ensure that the first
row of the CSV file contains the column names as specified above. The order of
the columns is not important, but the naming is.

Usage:
  - To run calibration:
    python sitl_engine_curve_fit.py <log_file>
  - To skip calibration and plot results with current parameters:
    python sitl_engine_curve_fit.py <log_file> --skip_calibration

THIS SCRIPT TAKES SEVERAL MINUTES TO RUN ON MY MACHINE. PLEASE BE PATIENT.

AP_FLAKE8_CLEAN

"""

import argparse
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize

# Aircraft Nominal Constants These values are mostly arbitrary reference values
# that define how the the following fit parameters are scaled.
# - IDLE_RPM: Engine idle RPM
# - FULL_RPM: Engine full throttle RPM
# - CRUISE_SPEED: Cruise airspeed in m/s
# - CRUISE_RPM: Average engine RPM at cruise speed (used to the nominal burn)
NOMINAL_VALUES = {
    "IDLE_RPM": 1500,
    "FULL_RPM": 7100,
    "CRUISE_SPEED": 25,
    "CRUISE_RPM": 5000,
}

# CHT and CHT2 Fit Parameters
# These parameters define the thermal behavior of the engine:
# - FULL_DT_CRUISE: Full throttle delta temperature at cruise speed (°C).
# - IDLE_DT_CRUISE: Idle throttle delta temperature at cruise speed (°C).
# - IDLE_DT_HOVER: Idle throttle delta temperature while hovering (°C).
# - TCONST_CRUISE: Time constant for thermal inertia at cruise speed (s).
#   Higher values mean more thermal inertia.
# - KAPPA: Scaling factor for how thermal inertia changes with airspeed.
#   Increasing this will make on-ground warmup take longer.
CHT_FIT_PARAMS = {
    "FULL_DT_CRUISE": 232.4,
    "IDLE_DT_CRUISE": 92.2,
    "TCONST_CRUISE": 18.9,
    "IDLE_DT_HOVER": 116.8,
    "KAPPA": 7.84,
}

CHT2_FIT_PARAMS = {
    "FULL_DT_CRUISE": 203.2,
    "IDLE_DT_CRUISE": 83.9,
    "TCONST_CRUISE": 28.6,
    "IDLE_DT_HOVER": 115.5,
    "KAPPA": 2.00,
}

CHT_FIT_BOUNDS = {
    "FULL_DT_CRUISE": (200, 360),
    "IDLE_DT_CRUISE": (70, 110),
    "TCONST_CRUISE": (5, 50),
    "IDLE_DT_HOVER": (90, 120),
    "KAPPA": (0.2, 20),
}

# Fuel Consumption Fit Parameters
# These parameters define the fuel consumption behavior of the engine:
# - BURN_RATE: Fuel burn rate at cruise RPM (kg/hr).
FUEL_FIT_PARAMS = {
    "BURN_RATE": 0.816,
}

FUEL_FIT_BOUNDS = {
    "BURN_RATE": (0.5, 5),
}

# ----- Functions -----


def load_log_data(file_path: str) -> pd.DataFrame:
    """
    Load and preprocess log data from CSV file.
    """
    data = pd.read_csv(file_path)
    data["TimeStep"] = (
        data["Time"].diff().fillna(0) / 1000
    )  # Time step in seconds
    data["FuelConsumption"] *= 60 / 1000  # Convert fuel consumption to kg/hr
    return data


def calculate_cht_steady(
    rpm: float, airspeed: float, intake_temp: float, params: dict[str, float]
) -> float:
    """
    Calculate steady-state CHT based on parameters. See my writeup in the lua
    script for my derivation of these formulas
    """
    # Constants
    idle_dt_hover = params["IDLE_DT_HOVER"]
    idle_dt_cruise = params["IDLE_DT_CRUISE"]
    full_dt_cruise = params["FULL_DT_CRUISE"]
    v_cruise = NOMINAL_VALUES["CRUISE_SPEED"]
    idle_rpm = NOMINAL_VALUES["IDLE_RPM"]
    full_rpm = NOMINAL_VALUES["FULL_RPM"]

    # Solve for C2
    c2 = (idle_dt_hover / idle_dt_cruise - 1) / np.sqrt(v_cruise)
    # Derive the full-throttle delta temperature for stationary condition
    full_dt_hover = full_dt_cruise * (1 + c2 * np.sqrt(v_cruise))
    # Use that to solve for C1 and C3
    c1 = (full_dt_hover - idle_dt_hover) / (full_rpm ** 3 - idle_rpm ** 3)
    c3 = full_dt_hover - c1 * full_rpm ** 3

    # Calculate the steady state CHT at our actual airspeed
    dt_steady = (c1 * rpm**3 + c3) / (1 + c2 * np.sqrt(airspeed))
    if rpm == 0:
        dt_steady = 0
    return dt_steady + intake_temp


def update_cht(
    current_cht: float,
    cht_steady: float,
    airspeed: float,
    time_step: float,
    params: dict[str, float],
) -> float:
    """
    Update decayed CHT based on thermal inertia.
    """
    tau = (
        params["TCONST_CRUISE"]
        * (1 + params["KAPPA"] * np.sqrt(NOMINAL_VALUES["CRUISE_SPEED"]))
        / (1 + params["KAPPA"] * np.sqrt(airspeed))
    )
    decay_factor = 1 - np.exp(-time_step / tau)
    return current_cht * (1 - decay_factor) + cht_steady * decay_factor


def calculate_fuel_consumption(rpm: float, params: dict[str, float]) -> float:
    """
    Calculate fuel consumption based on RPM.
    """
    return params["BURN_RATE"] * (rpm / NOMINAL_VALUES["CRUISE_RPM"]) ** 3


def calibrate_cht(
    data: pd.DataFrame,
    cht_column: str,
    params: dict[str, float],
    bounds: dict[str, float],
) -> dict[str, float]:
    """
    Calibrate parameters for CHT or CHT2.
    """

    def objective(values: np.ndarray) -> float:
        current_params = dict(zip(params.keys(), values))
        errors = []
        current_cht = data[cht_column].iloc[0]
        for _, row in data.iterrows():
            time_step = row["TimeStep"]
            rpm = row["RPM"]
            airspeed = row["Airspeed"]
            imt = row["IMT"]
            cht_actual = row[cht_column]
            cht_steady = calculate_cht_steady(
                rpm, airspeed, imt, current_params
            )
            current_cht = update_cht(
                current_cht, cht_steady, airspeed, time_step, current_params
            )
            errors.append((cht_actual - current_cht) ** 2)
        return np.mean(errors)

    initial_guess = list(params.values())
    result = minimize(
        objective,
        initial_guess,
        bounds=[bounds[key] for key in params.keys()],
        method="L-BFGS-B",
    )
    return dict(zip(params.keys(), result.x))


def calibrate_fuel(
    data: pd.DataFrame, params: dict[str, float], bounds: dict[str, float]
) -> dict[str, float]:
    """
    Calibrate parameters for fuel consumption.
    """

    def objective(values) -> float:
        current_params = dict(zip(params.keys(), values))
        errors = []
        for _, row in data.iterrows():
            rpm = row["RPM"]
            fuel_actual = row["FuelConsumption"]
            fuel_simulated = calculate_fuel_consumption(rpm, current_params)
            errors.append((fuel_actual - fuel_simulated) ** 2)
        return np.mean(errors)

    initial_guess = list(params.values())
    result = minimize(
        objective,
        initial_guess,
        bounds=list(bounds.values()),
        method="L-BFGS-B",
    )
    return dict(zip(params.keys(), result.x))


def plot_results(
    data: pd.DataFrame,
    params_cht: dict[str, float],
    params_cht2: dict[str, float],
    params_fuel: dict[str, float],
):
    """
    Plot actual vs simulated results for CHT, CHT2, and FuelConsumption.
    """
    current_cht = data["CHT"].iloc[0]
    current_cht2 = data["CHT2"].iloc[0]
    simulated_cht = []
    simulated_cht2 = []
    simulated_fuel = []

    for _, row in data.iterrows():
        time_step = row["TimeStep"]
        rpm = row["RPM"]
        airspeed = row["Airspeed"]
        imt = row["IMT"]

        # Simulate CHT
        cht_steady = calculate_cht_steady(rpm, airspeed, imt, params_cht)
        current_cht = update_cht(
            current_cht, cht_steady, airspeed, time_step, params_cht
        )
        simulated_cht.append(current_cht)

        # Simulate CHT2
        cht2_steady = calculate_cht_steady(rpm, airspeed, imt, params_cht2)
        current_cht2 = update_cht(
            current_cht2, cht2_steady, airspeed, time_step, params_cht2
        )
        simulated_cht2.append(current_cht2)

        # Simulate Fuel Consumption
        simulated_fuel.append(calculate_fuel_consumption(rpm, params_fuel))

    plt.figure(figsize=(12, 8))

    # Plot CHT
    plt.subplot(3, 1, 1)
    plt.plot(data["Time"], data["CHT"], label="Actual", alpha=0.7)
    plt.plot(data["Time"], simulated_cht, label="Simulated", linestyle="--")
    plt.ylabel("CHT1 (°C)")
    plt.legend(loc="upper right")

    # Plot CHT2
    plt.subplot(3, 1, 2)
    plt.plot(data["Time"], data["CHT2"], label="Actual", alpha=0.7)
    plt.plot(data["Time"], simulated_cht2, label="Simulated", linestyle="--")
    plt.ylabel("CHT2 (°C)")
    plt.legend(loc="upper right")

    # Plot Fuel Consumption
    plt.subplot(3, 1, 3)
    plt.plot(
        data["Time"],
        data["FuelConsumption"],
        label="Actual",
        alpha=0.7,
    )
    plt.plot(
        data["Time"],
        simulated_fuel,
        label="Simulated",
        linestyle="--",
    )
    plt.ylabel("Fuel Consumption (kg/hr)")
    plt.legend(loc="upper right")

    plt.xlabel("Time (s)")
    plt.tight_layout()
    plt.show()


def main():
    """Main entry point of the script."""
    parser = argparse.ArgumentParser(
        description="Calibrate engine parameters and plot results."
    )
    parser.add_argument("log_file", help="Path to the log file.")
    parser.add_argument(
        "--skip-calibration",
        action="store_true",
        help="Skip calibration and just generate plots.",
    )
    args = parser.parse_args()

    log_data = load_log_data(args.log_file)

    if args.skip_calibration:
        print("Skipping calibration...")
        cht_fit_params = CHT_FIT_PARAMS
        cht2_fit_params = CHT2_FIT_PARAMS
        fuel_fit_params = FUEL_FIT_PARAMS
    else:
        print("Calibrating CHT...")
        cht_fit_params = calibrate_cht(
            log_data, "CHT", CHT_FIT_PARAMS, CHT_FIT_BOUNDS
        )
        print("CHT:", cht_fit_params)

        print("Calibrating CHT2...")
        cht2_fit_params = calibrate_cht(
            log_data, "CHT2", CHT2_FIT_PARAMS, CHT_FIT_BOUNDS
        )
        print("CHT2:", cht2_fit_params)

        print("Calibrating Fuel Consumption...")
        fuel_fit_params = calibrate_fuel(
            log_data, FUEL_FIT_PARAMS, FUEL_FIT_BOUNDS
        )
        print("Fuel:", fuel_fit_params)

    # Generate combined plots
    plot_results(log_data, cht_fit_params, cht2_fit_params, fuel_fit_params)


if __name__ == "__main__":
    main()
