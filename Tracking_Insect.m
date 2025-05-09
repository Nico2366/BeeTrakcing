%...................................................................
% Author : Nicolas Salvage
% Last update : 07/04/2025
% function : Lance la calibration et la reconstruction 3D de l'arène
% et des trajectoires. 
%...................................................................
close all
clear 
clc

% Pré-traitement vidéo 
Traitement_Image();

% Recupération des images clé pour calibration
ImageDecoupe(1);

% Etallonage du systeme stereo 
[stereoParams] = Calibration_Stereo();

% Recupération des images clé pour le cube
ImageDecoupe(2);
 
% Reconstruction de l'arene en 3D
[Average_Reference_point,R] = Arena_Reconstruction(stereoParams);

% Reconstruction des Trajectoires
points3D_traj = Trajectorie_Reconstruction(R,Average_Reference_point,stereoParams);