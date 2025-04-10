%...................................................................
% Author : Nicolas Salvage
% Last update : 07/04/2025
% function : pre-traitement des vidéo de calibration 
%...................................................................


function Traitement_Image()
    
    % Spécifier le dossier contenant les vidéos
    videoFolder =  'calibration'; 

    % Lister tous les fichiers vidéo dans le dossier
    videoFile = dir(fullfile(videoFolder, '*.MP4'));
    num = ['1' '2'];

    for i = 1:length(videoFile)  

        vidReader = VideoReader([videoFile(i).folder '\' videoFile(i).name]);
        
        if contains(videoFile(i).name, 'camera-1')

            outputVideoFile = 'calibration\camera-1_corrigee.mp4';
           
        elseif contains(videoFile(i).name, 'camera-2')

            outputVideoFile = 'calibration\camera-2_corrigee.mp4';

        end

        vidWriter = VideoWriter(outputVideoFile, 'MPEG-4');
        vidWriter.FrameRate = vidReader.FrameRate; % Conserver le même taux de frames
        open(vidWriter);

        % Initialisation des variables pour stocker les résultats
        mse_values = [];
        psnr_values = [];
        ssim_values = [];
        
        % Lire la première frame
        if hasFrame(vidReader)
            prevFrame = readFrame(vidReader);
            grayPrevFrame = rgb2gray(prevFrame); % Convertir en niveaux de gris
        
            % Prétraitement : réduction du bruit et amélioration
            frameSmooth = imgaussfilt(grayPrevFrame, 2); % Flou gaussien
            frameSharp = imsharpen(frameSmooth, 'Radius', 3, 'Amount', 1.5); % Amélioration de la netteté
        
            % Remplacer la composante en niveaux de gris dans la frame couleur
            correctedFrame = repmat(frameSharp, [1 1 3]);
        
            % Écrire la frame corrigée dans la nouvelle vidéo
            writeVideo(vidWriter, correctedFrame);
        end
        
        
        % Parcourir toutes les frames de la vidéo
        frameIndex = 1;

        totalFrames = vidReader.NumFrames;
        expectedImages = totalFrames;
        f = waitbar(0, ['pré-traitement des vidéo - Caméra ' num(i)]);

        while hasFrame(vidReader)
            currFrame = readFrame(vidReader);
            grayCurrFrame = rgb2gray(currFrame); % Frame actuelle en niveaux de gris
        
            % Prétraitement : réduction du bruit et amélioration
            frameSmooth = imgaussfilt(grayCurrFrame, 2); % Flou gaussien
            frameSharp = imsharpen(frameSmooth, 'Radius', 3, 'Amount', 1.5); % Amélioration de la netteté
        
            % Calcul de MSE
            mse = immse(frameSharp, grayPrevFrame);
            mse_values = [mse_values, mse];
        
            % Calcul de PSNR
            psnr_value = psnr(frameSharp, grayPrevFrame);
            psnr_values = [psnr_values, psnr_value];
        
            % Calcul de SSIM
            ssim_value = ssim(frameSharp, grayPrevFrame);
            ssim_values = [ssim_values, ssim_value];
        
            % Mise à jour de la frame précédente
            grayPrevFrame = frameSharp;
        
            % Remplacer la composante en niveaux de gris dans la frame couleur
            correctedFrame = repmat(frameSharp, [1 1 3]);
        
            % Écrire la frame corrigée dans la nouvelle vidéo
            writeVideo(vidWriter, correctedFrame);
        
            frameIndex = frameIndex + 1;
            % Mise à jour de la barre de progression
            waitbar(frameIndex / expectedImages, f, ...
                    sprintf('Caméra %s - %d / %d images', num(i), frameIndex, expectedImages));
        end
        
        % Fermer l'objet VideoWriter
        close(vidWriter);
        
        % Affichage des résultats
        figure;
        subplot(3,1,1);
        plot(mse_values, 'r');
        title('Évolution de MSE entre frames');
        xlabel('Numéro de Frame');
        ylabel('MSE');
        
        subplot(3,1,2);
        plot(psnr_values, 'b');
        title('Évolution de PSNR entre frames');
        xlabel('Numéro de Frame');
        ylabel('PSNR (dB)');
        
        subplot(3,1,3);
        plot(ssim_values, 'g');
        title('Évolution de SSIM entre frames');
        xlabel('Numéro de Frame');
        ylabel('SSIM');
        
        sgtitle('Analyse des variations entre frames prétraitées');

    end
    
    close(f); % Fermer la barre de progression à la fin de la vidéo
    fprintf('Correction terminée');

end