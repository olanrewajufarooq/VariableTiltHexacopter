# Variable Tilt Hexacopter

## Overview

This repository contains a **Simscape/Simulink model** of a **Variable Tilt Hexacopter**, designed to simulate and analyze the dynamics and control of a hexacopter with adjustable propeller tilting angles. The model aims to facilitate research in **geometric control and flight dynamics study**.

## Repository Structure

```plaintext
📂 VariableTiltHexacopter
  ├── 📁 modelling_scripts           # MATLAB scripts for modelling and model analysis
    ├── 📄 compute_params.slx        # Simulink file for evaluating inertia tensors, etc.
    ├── 📄 fixed_wrench_analysis.mlx # Livescript for Evaluating the body wrench for fixed tilt conf.
  ├── 📄 README.md                   # This documentation file
  ├── 📄 param.m                     # MATLAB Script for Hexacopter Parameters
  ├── 📄 simulation.slx              # Simulation Model
```

## Getting Started

### Prerequisites

- **MATLAB & Simulink** (R2021b or later recommended)
- **Simscape Multibody** (for physics-based modeling)
- **Control System Toolbox** (optional, for implementing controllers)

### Installation

1. Clone the repository:
   ```sh
   git clone https://github.com/olanrewajufarooq/VariableTiltHexacopter.git
2. Open MATLAB and navigate to the repository folder.
3. Run the `param.m` file before running any Simulink model.
4. Run `simulation.slx` and view in the Mechanics Explorer on MATLAB.

### Usage

Ensure you run the `param.m` file before running either of the Simulink files.

