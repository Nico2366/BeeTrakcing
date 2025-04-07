🐝 Studying Honeybees’ Landing on Flowers: An Experimental Setup
This repository contains the MATLAB-based technical solution for a bio-inspired vision tracking system. Our goal was to reconstruct the 3D trajectory of a landing honeybee — modeled here by a fake 3D-printed bee — using high-speed cameras and deep learning-based detection and tracking algorithms.

📌 Project Overview
Landing behavior in honeybees can provide valuable insights for designing robust autonomous landing strategies for aerial robots. This experimental setup replicates the landing of a honeybee on a flower within a controlled flight arena, using vision-based tools for accurate 3D reconstruction.

📁 Repository Structure
.
├── 3D_Arena/               # Video and images for Arena reconstruction
├── calibration/            # Video and images for Calibration
├── trajectoire_csv/        # .csv files of trajectory         
├── YOLO/                   # YOLO files
└── README.md

🧰 Requirements
MATLAB R2023a or later

Computer Vision Toolbox™

YOLO (PyTorch export for inference compatibility)

BYTETrack dependencies (Python-based, bridged to MATLAB)

👤 Credits
Project Lead: Nicolas Salvage
