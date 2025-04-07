<h1 align="center">🐝 Studying Honeybees’ Landing on Flowers</h1>
<h3 align="center">An Experimental Setup for Vision-Based 3D Trajectory Tracking in MATLAB</h3>

<p align="center">
  <b>Nicolas Salvage</b>, <b>Antoine HP Morice</b>, <b>Julien R Serres</b>  
  <br>
  Aix Marseille Univ, CNRS, ISM, Marseille,France  
  <br>
  <i>2025 – Project</i>
</p>

---

🎯 **Objective**: Reconstruct the 3D trajectory of a landing honeybee using stereo vision and deep learning-based detection & tracking algorithms.

---

## 📂 Repository Structure

```bash
📁 3D_arene/                 # Arena videos&images
  📁 camera-1/               # camera-1 images
  📁 camera-2/               # camera-2 images
📁 calibration/              # Calibration videos&images
  📁 camera-1/               # camera-1 images
  📁 camera-2/               # camera-2 images
📁 trajectoire_csv/          # Trajectory files
📄 Arena_Reconstruction.m    # Flying arena reconstruction function
📄 Calibration_Stereo.m      # Stereo vision Calibration function
📄 ImageDecoupe.m            # Recovering key images for calibration function
📄 README.md
📄 reconstruction3D.m        # figure function of 3D reconstruction of the arena
📄 TrackingInsect.m          # Starts calibration and 3D reconstruction of the arena and trajectories   
📄 TrackingYOLO.m            # Yolo trajectories triangulation function       
📄Traitement_Image.m         # pre-processing of calibration videos function

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
