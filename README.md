<h1 align="center">🐝 Studying Honeybees’ Landing on Flowers</h1>
<h3 align="center">An Experimental Setup for Vision-Based 3D Trajectory Tracking in MATLAB</h3>

<p align="center">
  <b>Nicolas Salvage</b>, <b>[Ton Nom]</b>, <b>[Autres Co-auteurs]</b>  
  <br>
  [Votre institution ici]  
  <br>
  <i>2025 – Project Highlight</i>
</p>

---

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

- MATLAB R2023a or later
- Computer Vision Toolbox™
- YOLO (You Only Look Once, Redmon et al. 2016) 
- BYTETrack.yaml 

---

👤 Credits
Project Lead: Nicolas Salvage
