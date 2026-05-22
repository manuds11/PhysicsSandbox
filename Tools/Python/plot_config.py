# Defines wether or not plots are shown in Python API such as spyder. Plots are saved as .png anyways.
SHOW_PLOTS = True

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

