%...................................................................
% Author : Nicolas Salvage
% Last update : 07/04/2025
% function : Calibration stereo vision function 
%...................................................................

function [stereoParams,theta_x,theta_y,theta_z] = Calibration_Stereo()


    % 1. Sélection des dossiers d'images
    folder1 = 'calibration/camera-1'; 
    folder2 = 'calibration/camera-2'; 
    
    % Charger les images des deux caméras
    leftImages = imageDatastore(folder1);
    rightImages = imageDatastore(folder2);
    
    % 2. Détection des coins de la mire
    % Détection de l’échiquier
    [imagePoints, boardSize, validIdx] = detectCheckerboardPoints(leftImages.Files, rightImages.Files,'PartialDetections',false);
    
    nb_image_delet = size(leftImages.Files,1) - sum(validIdx);
    disp(['Nombre d''images ou échequier pas détecter : ', num2str(nb_image_delet)])
    
    % Filtrer les images où la mire est mal détectée
    leftImages.Files = leftImages.Files(validIdx);
    rightImages.Files = rightImages.Files(validIdx);
    
    % 3. Spécifier les coordonnées réelles de l'échiquier
    squareSize = 29; % Taille des carrés en mm
    worldPoints = patternWorldPoints("checkerboard", boardSize, squareSize);
    
    % 4. Calibration du système stéréo
    % Lire une image pour récupérer la taille
    I = readimage(leftImages, 1);
    imageSize = [size(I, 1), size(I, 2)];
    
    % Première calibration
    stereoParamsInitial = estimateCameraParameters(imagePoints, worldPoints, 'ImageSize', imageSize);
    
    % Récupération des erreurs de reprojection
    reprojectionErrors1 = stereoParamsInitial.CameraParameters1.ReprojectionErrors; % Mx2xN
    reprojectionErrors2 = stereoParamsInitial.CameraParameters2.ReprojectionErrors; % Mx2xN
    
    meanErrors1 = squeeze(mean(vecnorm(reprojectionErrors1, 2, 2), 1)); % 1xN
    meanErrors2 = squeeze(mean(vecnorm(reprojectionErrors2, 2, 2), 1)); % 1xN
    indices = find(meanErrors1 >= 1| meanErrors2 >= 1);
    
    % Affichage du nombre d'images supprimées
    numImagesRemoved = numel(indices);
    disp(['Nombre d''images supprimées : ', num2str(numImagesRemoved)]);
    
    % Suppression des images et des points d'image correspondants
    imagePointsFiltered = imagePoints;
    worldPointsFiltered = worldPoints;
    
    % Supprimer les images aux indices correspondants (3ème dimension dans le tableau 4D)
    imagePointsFiltered(:,:,  indices, :) = []; 
    
    % Recalibration avec les images filtrées
    stereoParams = estimateCameraParameters(imagePointsFiltered, worldPoints, 'ImageSize', imageSize,'EstimateTangentialDistortion',true);
    
    
    % 5. Visualisation de la précision de la calibration
    
    figure;
    subplot(2,2,3)
    showReprojectionErrors(stereoParams);
    title("Erreurs de reprojection");
    subplot(2,2,4)
    showExtrinsics(stereoParams)
    
    % 6. Affichage des résultats
    
    R  = stereoParams.RotationOfCamera2;
    
    % Calcul des angles d'Euler (en radians)
    theta_x = atan2(R(3,2), R(3,3));  % Rotation autour de l'axe X
    theta_y = atan2(-R(3,1), sqrt(R(3,2)^2 + R(3,3)^2));  % Rotation autour de l'axe Y
    theta_z = atan2(R(2,1), R(1,1));  % Rotation autour de l'axe Z
    
    disp('Rotation entre les caméras :');
    disp(stereoParams.RotationOfCamera2 );
    
    % Affichage des résultats en degrés
    disp(['Rotation autour de X: ', num2str(rad2deg(theta_x)), '°']);
    disp(['Rotation autour de Y: ', num2str(rad2deg(theta_y)), '°']);
    disp(['Rotation autour de Z: ', num2str(rad2deg(theta_z)), '°']);
    
    
    disp('Translation entre les caméras :');
    disp(stereoParams.TranslationOfCamera2);
    
    
    
    % 7. Rectification des images
    
    I1 = readimage(leftImages, 1);
    I2 = readimage(rightImages, 1);
    
    [J1, J2, reprojection_mat] = rectifyStereoImages(I1, I2, stereoParams, 'OutputView', 'full');
    stereoAnaglyphe = stereoAnaglyph(J1, J2);
    
    % Afficher l'image anaglyphe pour vérifier l'alignement
    subplot(2,2,1)
    imshow(stereoAnaglyphe);
    title('Images rectifiées');
    
    % Disparité 
    
    disparityRange = [64, 192]; % Ajustez selon vos images
    disparityMap = disparitySGM(rgb2gray(J1),rgb2gray(J2), 'DisparityRange', disparityRange,'UniquenessThreshold',5);
     % disparityMap = medfilt2(disparityMap, [2 2]); % Filtrage médian
     % disparityMap(disparityMap < 0) = 0; % Supprimer les valeurs négatives
    
    subplot(2,2,2)
    imshow(disparityMap, disparityRange);
    colormap jet;
    colorbar;
    title('Carte de disparité');


end



