from pathlib import Path
import sys

import pandas as pd
import matplotlib.pyplot as plt

SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parents[1]

DEFAULT_CSV_PATH = PROJECT_ROOT / "Saved" / "SimulationLogs" / "OscillatorLog.csv"
DEFAULT_OUTPUT_DIR = PROJECT_ROOT / "Saved" / "SimulationPlots"

from plot_config import PLOTS, SHOW_PLOTS

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