# 🐝 Studying Honeybees’ Landing on Flowers: An Experimental Setup
### _Nicolas Salvage, Antoine HP Morice, Julien R Serres_

This repository contains the **MATLAB-based technical solution** for a bio-inspired vision tracking system.  
🎯 **Objective**: Reconstruct the 3D trajectory of a landing honeybee using stereo vision and deep learning-based detection & tracking algorithms.

---

## 🚀 Project Overview

Landing behavior in honeybees can inspire robust autonomous landing strategies for aerial robots.  
This experimental setup replicates the landing of a honeybee on a moving flower within a controlled flight arena, using stereo vision and deep learning techniques for accurate 3D reconstruction.

---

## 📂 Repository Structure

```bash
📁 3D_arene/           # Arena video
📁 calibration/        # Calibration video
📁 trajectoire_csv/    # Trajectory files
📄 TrackingYOLO.m      # YOLO object detection & tracking
📄 TrackingInsect.m    # Insect-specific tracking logic
📄 reconstruction3D.m  # 3D point reconstruction from 2D detections
📄 ImageDecoupe.m      # Frame pre-processing & cropping
📄 README.md

```
---
🧰 Requirements
MATLAB R2023a or later

Computer Vision Toolbox™

YOLO (PyTorch export for inference compatibility)

BYTETrack dependencies (Python-based, bridged to MATLAB)

---

👤 Credits
Project Lead: Nicolas Salvage
