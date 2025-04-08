%...................................................................
% Author : Nicolas Salvage
% Last update : 07/04/2025
% function : Recupération frame clé 
%...................................................................

function ImageDecoupe(i)

    if i == 1
        videoFolder =  'calibration'; 
        outputFolder1 = 'calibration/camera-1';
        if ~exist(outputFolder1, 'dir')
            mkdir(outputFolder1);
        end
        outputFolder2 = 'calibration/camera-2';
        if ~exist(outputFolder2, 'dir')
            mkdir(outputFolder2);
        end
    else
        videoFolder =  '3D_arene'; 
        outputFolder1 = '3D_arene/camera-1';
        if ~exist(outputFolder1, 'dir')
            mkdir(outputFolder1);
        end
        outputFolder2 = '3D_arene/camera-2';
        if ~exist(outputFolder2, 'dir')
            mkdir(outputFolder2);
        end
    end

    videoFiles = dir(fullfile(videoFolder, '*.MP4'));
    globalImageCount = 0;
    currentCamera = '';
    num = ['1' '2'];

    for idx = 1:length(videoFiles)

        videoFile = fullfile(videoFolder, videoFiles(idx).name);
        video = VideoReader(videoFile);
        frameRate = video.FrameRate;
        frameInterval = round(frameRate / 6);

        if contains(videoFiles(idx).name, 'camera-1_corrigee')
            cameraPrefix = 'camera-1';
            outputFolder = outputFolder1;
            camNum = 1;
        elseif contains(videoFiles(idx).name, 'camera-2_corrigee')
            cameraPrefix = 'camera-2';
            outputFolder = outputFolder2;
            camNum = 2;
        else
            continue;
        end

        if ~strcmp(currentCamera, cameraPrefix)
            globalImageCount = 0;
            currentCamera = cameraPrefix;
        end

        frameNumber = 0;
        totalFrames = video.NumFrames;
        expectedImages = floor(totalFrames / frameInterval);
        f = waitbar(0, ['Découpe des images - Caméra ' num(camNum)]);

        while hasFrame(video)
            frame = readFrame(video);
            frameNumber = frameNumber + 1;

            if mod(frameNumber, frameInterval) == 1
                globalImageCount = globalImageCount + 1;

                outputFileName = fullfile(outputFolder, sprintf('%s-%04d.jpg', cameraPrefix, globalImageCount));
                imwrite(frame, outputFileName);

                % Mise à jour de la barre de progression
                waitbar(globalImageCount / expectedImages, f, ...
                    sprintf('Caméra %s - %d / %d images', num(camNum), globalImageCount, expectedImages));
            end
        end

        close(f); % Fermer la barre de progression à la fin de la vidéo
    end

    fprintf('Découpage terminé ! %d images enregistrées\n', globalImageCount);
end