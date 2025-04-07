%...................................................................
% Author : Nicolas Salvage
% Last update : 07/04/2025
% function : Recupération frame clé 
%...................................................................

function ImageDecoupe(i)
    
    if i == 1
        % Spécifier le dossier contenant les vidéos
        videoFolder =  'calibration'; 
        
        % Spécifier le dossier de sortie pour enregistrer les images
        outputFolder1 = 'calibration/camera-1';
        if ~exist(outputFolder1, 'dir')
            mkdir(outputFolder1);
        end
    
        outputFolder2 = 'calibration/camera-2';
        if ~exist(outputFolder2, 'dir')
            mkdir(outputFolder2);
        end

    else
        
                % Spécifier le dossier contenant les vidéos
        videoFolder =  '3D_arene'; 
        
        % Spécifier le dossier de sortie pour enregistrer les images
        outputFolder1 = '3D_arene/camera-1';
        if ~exist(outputFolder1, 'dir')
            mkdir(outputFolder1);
        end
    
        outputFolder2 = '3D_arene/camera-2';
        if ~exist(outputFolder2, 'dir')
            mkdir(outputFolder2);
        end


    end
    
    % Lister tous les fichiers vidéo dans le dossier
    videoFiles = dir(fullfile(videoFolder, '*.MP4'));
    
    % Initialiser un compteur global d'images
    globalImageCount = 0;
    
    % Initialiser une variable pour garder trace de la caméra actuelle
    currentCamera = '';
    
    % Parcourir chaque fichier vidéo
    for i = 1:length(videoFiles)
        videoFile = fullfile(videoFolder, videoFiles(i).name);
        
        % Lire la vidéo
        video = VideoReader(videoFile);
    
        % Obtenir des informations sur la vidéo
        frameRate = video.FrameRate; % Taux d'images par seconde (fps)
        framesPerSecond = round(frameRate)/6; % Nombre d'images par seconde
        frameInterval = framesPerSecond; % Extraire une image toutes les secondes
    
        % Déterminer le préfixe en fonction de la vidéo (camera-1 ou camera-2)
        if contains(videoFiles(i).name, 'camera-1')
            cameraPrefix = 'camera-1';
        elseif contains(videoFiles(i).name, 'camera-2')
            cameraPrefix = 'camera-2';
        else
            continue; % Si le nom de fichier ne contient pas "camera-1" ou "camera-2", passer à la vidéo suivante
        end
    
        % Vérifier si la caméra a changé
        if ~strcmp(currentCamera, cameraPrefix)
            % Réinitialiser le compteur d'images pour chaque nouvelle caméra
            globalImageCount = 0;
            currentCamera = cameraPrefix;
        end
    
        % Initialiser un compteur de frames pour chaque vidéo
        frameNumber = 0;
    
        % Parcourir chaque image de la vidéo
        while hasFrame(video)
            frame = readFrame(video); % Lire une image de la vidéo
            frameNumber = frameNumber + 1; % Incrémenter le compteur de frames
    
            % Vérifier si cette image correspond à une seconde
            if mod(frameNumber, frameInterval) == 1
                globalImageCount = globalImageCount + 1; % Incrémenter le compteur global d'images
    

                if contains(videoFiles(i).name, 'camera-1')
                    
                    % Construire le nom du fichier pour l'image
                    outputFileName1 = fullfile(outputFolder1, sprintf('%s-%04d.jpg', cameraPrefix, globalImageCount));
                
                    % Sauvegarder l'image
                    imwrite(frame, outputFileName1);

                elseif contains(videoFiles(i).name, 'camera-2')

                    % Construire le nom du fichier pour l'image
                    outputFileName2= fullfile(outputFolder2, sprintf('%s-%04d.jpg', cameraPrefix, globalImageCount));
                
                    % Sauvegarder l'image
                    imwrite(frame, outputFileName2);

                else
                    continue; % Si le nom de fichier ne contient pas "camera-1" ou "camera-2", passer à la vidéo suivante
                end
            end
        end
    end
    
    fprintf('Découpage terminé ! %d images enregistrées ', globalImageCount);

end