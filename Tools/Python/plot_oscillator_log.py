from pathlib import Path
import sys

import pandas as pd
import matplotlib.pyplot as plt

SHOW_PLOTS = True

SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parents[1]

DEFAULT_CSV_PATH = PROJECT_ROOT / "Saved" / "SimulationLogs" / "OscillatorLog.csv"
DEFAULT_OUTPUT_DIR = PROJECT_ROOT / "Saved" / "SimulationPlots"

# IMPORTANT:
# The names used in "columns" must match exactly the CSV headers exported
# from Unreal/C++ in FOscillatorCsvLogger::WriteToFile().
# If a CSV column is renamed in C++, update this plot configuration too.
# Plot configuration structure:
#
# PLOTS = [
#     {
#         "filename": Output PNG filename,
#         "title": Figure title,
#         "subplots": [
#             {
#                 "ylabel": Y-axis label,
#                 "columns": CSV column names to plot together,
#             },
#             ...
#         ],
#     },
#     ...
# ]
#
# Each top-level entry generates one figure.
# Each subplot entry generates one subplot inside that figure.
# All column names must match CSV headers exported from Unreal/C++.
PLOTS = [
    {
        "filename": "state.png",
        "title": "Oscillator state",
        "subplots": [
            {
                "ylabel": "Position / Displacement [m]",
                "columns": [
                    "position",
                    "displacement",
                ],
            },
            {
                "ylabel": "Velocity / Acceleration",
                "columns": [
                    "velocity",
                    "acceleration",
                ],
            },
        ],
    },
    {
        "filename": "forces.png",
        "title": "Oscillator forces",
        "subplots": [
            {
                "ylabel": "Force [N]",
                "columns": [
                    "spring_force",
                    "damping_force",
                    "net_force",
                ],
            },
        ],
    },
    {
        "filename": "energy_balance.png",
        "title": "Energy balance",
        "subplots": [
            {
                "ylabel": "Energy [J]",
                "columns": [
                    "mechanical_energy",
                    "dissipated_energy",
                    "total_energy_with_losses",
                ],
            },
            {
                "ylabel": "Energy error [J]",
                "columns": [
                    "sim_energy_error",
                ],
            },
        ],
    },
    {
        "filename": "energy_error.png",
        "title": "Energy conservation error",
        "subplots": [
            {
                "ylabel": "Absolute error [J]",
                "columns": [
                    "sim_energy_error",
                ],
            },
            {
                "ylabel": "Relative error",
                "columns": [
                    "relative_sim_energy_error",
                ],
            },
        ],
    }
]


def plot_figure(df: pd.DataFrame, plot_config: dict, output_dir: Path) -> None:
    if "simulation_time" not in df.columns:
        raise KeyError("CSV column not found: simulation_time")

    subplot_count = len(plot_config["subplots"])

    fig, axes = plt.subplots(
        subplot_count,
        1,
        sharex=True,
        figsize=(10, 3.5 * subplot_count),
    )

    if subplot_count == 1:
        axes = [axes]

    for axis, subplot_config in zip(axes, plot_config["subplots"]):
        for column in subplot_config["columns"]:
            if column not in df.columns:
                print(f"Warning: CSV column not found: {column}")
                continue

            axis.plot(
                df["simulation_time"],
                df[column],
                label=column,
            )

        axis.set_ylabel(subplot_config.get("ylabel", ""))
        axis.grid(True)
        axis.legend()

    axes[-1].set_xlabel("Simulation time [s]")
    fig.suptitle(plot_config["title"])
    fig.tight_layout()

    output_path = output_dir / plot_config["filename"]
    fig.savefig(output_path, dpi=150, bbox_inches="tight")


def main() -> int:

    if len(sys.argv) >= 2:
        csv_path = Path(sys.argv[1]).resolve()
    else:
        csv_path = DEFAULT_CSV_PATH

    if not csv_path.exists():
        print(f"CSV file not found: {csv_path}")
        return 1

    output_dir = DEFAULT_OUTPUT_DIR
    output_dir.mkdir(parents=True, exist_ok=True)

    df = pd.read_csv(csv_path)

    for plot_config in PLOTS:
        plot_figure(df, plot_config, output_dir)

    if SHOW_PLOTS:
        plt.show()
        print(f"Plots displayed and written to: {output_dir}")
    else:
        plt.close("all")
        print(f"Plots written to: {output_dir}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())