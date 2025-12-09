# Synthetic-Benchmark-Dataset-of-an-Avionic-Main-Fuel-Pump-System-for-Data-Driven-Fault-Diagnosis
This repository contains the MATLAB/Simulink model and scripts used to simulate an avionic main fuel pump system and to generate multivariate time-series data for fault diagnosis and anomaly detection experiments. The code accompanies the paper:

Janzen F. L., Moddemann L., Diedrich A., Niggemann O.:  
Avionic Main Fuel Pump Simulation and Fault-Diagnosis Benchmark, IFAC World Congress, 2026.

---
**Repository structure**
- `mainFuelPumpV4.slx`  
  Simulink / Simscape Fluids model of the avionic main fuel pump system, including tank, main pump, bypass, Fuel Metering Unit (FMU), pressure relief valve and injectors.
  
- `generate_throttle_signal.m`  
  MATLAB script that creates a throttle command profile (0–1) representing a typical flight power profile and saves it as `throttle_signal.mat`.
  
- `throttle_signal.mat`
  Example throttle profile used as input for the simulation. Can be regenerated with `generate_throttle_signal.m`.
  
- `generateRuns_with_identifiers.m`  
  MATLAB script that:
  - runs the Simulink model (choose value to change between healthy and faulty annotation runs),
  - injects predefined fault scenarios,
  - writes time-series data and identifiers to CSV files.

- `time_series_csv/`  
  Directory for CSV time series.  

- `README.md`  
  This file.

- `LICENSE`  
  License information for the repository (see file for details).

- `.gitattributes`  
  Git LFS configuration (e.g., for large CSV files in `time_series_csv/`).
---
**Requirements**

To run the simulations you will need:

- MATLAB with Simulink
- Simscape and Simscape Fluids toolboxes (and any additional toolboxes required by the model)
- Sufficient RAM and disk space to store the generated CSV files
 
The model was developed within MATLAB R2025a version.
---
**Clone the repository**

   ```bash
   git clone https://github.com/Elix96J/Avionic_Main_Fuel_Pump_Simulation_and_Fault-Diagnosis_Benchmark.git
