from pathlib import Path
import sys

import pandas as pd
import matplotlib.pyplot as plt


SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parents[1]

DEFAULT_CSV_PATH = PROJECT_ROOT / "Saved" / "SimulationLogs" / "OscillatorLog.csv"
DEFAULT_OUTPUT_DIR = PROJECT_ROOT / "Saved" / "SimulationPlots"

SHOW_PLOTS = False

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

    # Position plot
    plt.figure()
    plt.plot(df["simulation_time"], df["position"])
    plt.xlabel("Simulation time [s]")
    plt.ylabel("Position [m]")
    plt.title("Oscillator position")
    plt.grid(True)
    plt.savefig(output_dir / "position_vs_time.png", dpi=150, bbox_inches="tight")
    if SHOW_PLOTS:
        plt.show()
    else:
        plt.close("all")

    # Energy plot
    plt.figure()
    plt.plot(df["simulation_time"], df["mechanical_energy"], label="Mechanical")
    plt.plot(df["simulation_time"], df["dissipated_energy"], label="Dissipated")
    plt.plot(df["simulation_time"], df["total_energy_with_losses"], label="Total with losses")
    plt.xlabel("Simulation time [s]")
    plt.ylabel("Energy [J]")
    plt.title("Oscillator energy balance")
    plt.grid(True)
    plt.legend()
    plt.savefig(output_dir / "energy_balance.png", dpi=150, bbox_inches="tight")
    if SHOW_PLOTS:
        plt.show()
    else:
        plt.close("all")

    # Simulation error plot
    plt.figure()
    plt.plot(df["simulation_time"], df["sim_energy_error"])
    plt.xlabel("Simulation time [s]")
    plt.ylabel("Energy error [J]")
    plt.title("Simulation energy balance error")
    plt.grid(True)
    plt.savefig(output_dir / "simulation_energy_error.png", dpi=150, bbox_inches="tight")
    if SHOW_PLOTS:
        plt.show()
    else:
        plt.close("all")

    print(f"Plots written to: {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())