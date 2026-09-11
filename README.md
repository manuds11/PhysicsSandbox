# PhysicsSandbox

A C++ / Unreal Engine project for developing and testing real-time physics simulation systems.

The project follows a progressive approach: starting from basic Unreal Engine mechanics and moving towards increasingly general dynamic-system simulation.

## Repository Structure

- `Source/PhysicsSandbox/` — main C++ simulation code
- `Content/` — Unreal Engine assets and project content
- `Tools/` — Python and auxiliary analysis tools

## Projects

### DebugProbe

Early experiments in spatial motion, orientation, runtime controls and camera behaviour used to establish the Unreal Engine simulation workflow.

### Harmonic Oscillator

First complete physical model, introducing numerical integration, fixed-timestep simulation, energy analysis and data visualisation with Python.

### MultiSystems

A modular framework for systems composed of multiple bodies and physical elements.

It started with spring-damper interactions and force accumulation, and was later extended with geometric constraints, Lagrange multipliers and a matrix constraint solver for coupled systems such as multiple pendulums.

## Direction

The long-term objective is to progressively extend this architecture towards engineering-oriented dynamic simulations, including marine systems and Dynamic Positioning applications.

## Technology

- Unreal Engine 5
- C++
- Python
- Git / GitHub
