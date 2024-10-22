# 🪲 Debug

## 🔍 Overview

This directory contains debug environments for singular aspects of a motion capture system. They are the sandbox testing grounds for developing the modules that are later reused in full software use.

## 📂 Organization

The following structure outlines in sequence a intuitive way for studying the system.

├── `blob_detection/`        # Marker segmentation and pixel coordinate extration from images
|    
├── `linear_projection/`     # Pinhole camera model study for vision sensors
|
├── `epipolar_geometry/`     # Epipolar geometry fundamentals and point triangulation using stereo vision
|
├── `lens_distortion/`       # Lens distortion modeling (radial, tangential and fisheye distortion models)
|
├── `image_noise/`           # Imaging noise modeling for stochastic simulation
|
├── `multiple_view/`         # Multiple view geometry with lens distortion and noise models
|
├── `socket_network/`        # Asynchronous UDP messaging 
|
├── `synchronization/`       # Blob position interpolation for async capture systems
|
├── `extrinsic_calibration/` # Extrinsic calibration and bundle adjustment techniques
|
├── `reference_update/`      # Update coordinate system reference frame
|
└── `virtual_arena/`         # Virtual motion capture system with every aspect above integrated

Each Jupyter Notebook contained in the directories provide a more in-depth explanation of the math behind the modules.

## ⚔️ Usage

1. Choose a debug scope directory;
2. Open and set the Jupyter Notebook contained in the directory;
3. Start CoppeliaSim and load the `.ttt` file with the same name as the jupyter notebook;
4. Customize camera configurations and parameters in the jupyter notebooks or the CoppeliaSim scenes to match your desired setup;
5. Run the Jupyter Notebook provided to control the simulation and capture data;
6. Analyse the data to your liking.

---