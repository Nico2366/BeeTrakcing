%...................................................................
% Author : Nicolas Salvage
% Last update : 07/04/2025
% function : Lance la calibration et la reconstruction 3D de l'arène
% et des trajectoires. 
%...................................................................

% pyrunfile("YOLO_BeeTracking.py")

close all
clear 
clc

% Pré-traitement vidéo 
Traitement_image();

% Recupération des images clé pour calibration
ImageDecoupe(1);

% Etallonage du systeme stereo 
[stereoParams,theta_x,theta_y,theta_z] = Calibration_Stereo();

ImageDecoupe(2);

% Reconstruction de l'arene en 3D
Arena_Reconstruction(stereoParams,theta_x,theta_y,theta_z)

% Reconstruction des Trajectoires
%Trajectorie_Reconstruction()