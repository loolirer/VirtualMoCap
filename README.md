# Virtual MoCap: Optical Tracking Systems Design

## 🔍 Overview

This repository contains the software developed for a **Digital Twin for Optical Tracking Systems Design**. The tool was designed to simulate, evaluate, and optimize motion capture systems within a virtual environment, built using *Python* and robotics simulation. The DT is currently integrated to *CoppeliaSim* but multi-platform integrations can be explored.  

## 🔖 Features

- **Configurable Camera Setup**: Easily adjust camera poses, resolutions, focal lengths, lens distortion parameters, and other intrinsic values;
- **Lighting & Noise Simulation**: Simulate real-world lighting and sensor noise to test how they affect tracking accuracy;
- **Synthetic Data Generation**: Create and export synthetic datasets for performance benchmarking;
- **Marker-based Tracking**: Evaluate tracking performance with adjustable marker configurations, including position, trajectory, and size;
- **Performance Analysis**: Tools for visualizing and calculating reconstruction errors to compare against ground truth values.

## 🏗️ Installation 

1. Clone the repository:
    ```bash
    git clone https://github.com/loolirer/VirtualMoCap
    ```
2. Create a Python Virtual Enviroment (highly recommended);
3. Install required dependencies:
    ```bash
    pip install -r requirements.txt
    ```
4. Ensure **CoppeliaSim 4.6.0 (rev. 16)** is installed and linked to the Python Virtual Enviroment.

## 📂 Organization

├── `debug/`   # Isolated environments to test models and features separately 
|
└── `modules/` # Reusable modules developed by using the debug environments as base 

Check above directories for more detailed information.

## 📖 Academia

This software replicates the experiments outlined in **["Optical tracking system based on COTS components"](https://ieeexplore.ieee.org/document/10053039)**, adapting the system to a virtual environment. Please refer to the paper for a deeper understanding of the experimental setup and results.

The results of this experiment were submitted in **["A digital twin for optical tracking systems design"]()** for [ICARA 2025](https://www.icara.us).

---