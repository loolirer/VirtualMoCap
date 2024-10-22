# 🧪 Application Environments

## 🔍 Overview

This directory contains environments for applications of the Digital Twin. They encompass all of the features tested and developed in the debug directory.

## 📂 Organization

    ├── coppeliasim/ # A complete virtual arena workflow implemented in CoppeliaSim
    |    
    ├── mocaprasp/   # An integration with MoCap Rasp Optical Tracking System
    |
    └── ICARA2025/   # Experiment made to validate the digital twin, submitted for ICARA 2025

## ⚔️ Usage

1. Choose an environment directory;
2. Open and set the Jupyter Notebook contained in the directory;
3. Start CoppeliaSim and load the `.ttt` file with the same name as the jupyter notebook;
4. If the directory has a `data_visualizer.ttt`, load it and start the simulation as well; 
5. Customize camera configurations and parameters in the jupyter notebooks or the CoppeliaSim scenes to match your desired setup;
6. Run the Jupyter Notebook provided to control the simulation and capture data;
7. Analyse the data to your liking.

Create your own environment or modify one of the already implemented for testing!

---