%% Vidéo Corriger 

% Charger la vidéo
videoFile = 'C:\Users\Nicolas\Nextcloud\YOLO11\Tracking_Video\camera-2_C0008_sync.mp4';
vidReader = VideoReader(videoFile);

% Créer un objet VideoWriter pour la vidéo corrigée
outputVideoFile = 'C:\Users\Nicolas\Nextcloud\YOLO11\Tracking_Video\camera-2_C0008_corrigee.mp4';
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

%% Charger une image
image = imread('C:\Users\Nicolas\Nextcloud\Matlab\captured_images\camera-1\camera-1-0001.jpg'); % Remplace par le chemin de ton image

% Appliquer un filtre Gaussien pour la réduction du bruit
sigma = 2; % Valeur du sigma pour la variance du filtre
image_gaussian = imgaussfilt(image, sigma);

% Création d'un filtre de sharpening
kernel_sharpen = [0 -1  0; 
                 -1  5 -1; 
                  0 -1  0];

% Appliquer le filtre de sharpening avec convolution
image_sharpened = imfilter(image_gaussian, kernel_sharpen, 'same');

% Affichage des résultats
figure;
subplot(1,3,1), imshow(image), title('Image Originale');
subplot(1,3,2), imshow(image_gaussian), title('Flou Gaussien (Réduction du bruit)');
subplot(1,3,3), imshow(image_sharpened), title('Sharpening (Amélioration de la netteté)');

% Sauvegarde des images traitées
imwrite(image_gaussian, 'image_gaussian.jpg');
imwrite(image_sharpened, 'image_sharpened.jpg');


% Découpage en 3 parties égales
img_orig = image;
img_gaussian = image_gaussian;
img_sharpened = image_sharpened;

% Convertir en niveaux de gris
gray_orig = rgb2gray(img_orig);
gray_gaussian = rgb2gray(img_gaussian);
gray_sharpened = rgb2gray(img_sharpened);

% Calcul des histogrammes
hist_orig = imhist(gray_orig);
hist_gaussian = imhist(gray_gaussian);
hist_sharpened = imhist(gray_sharpened);

% Affichage des images et histogrammes
figure;

% Affichage des images
subplot(2,3,1), imshow(gray_orig), title('Image Originale');
subplot(2,3,2), imshow(gray_gaussian), title('Flou Gaussien');
subplot(2,3,3), imshow(gray_sharpened), title('Sharpening');

% Affichage des histogrammes
subplot(2,3,4), plot(hist_orig, 'k'), title('Histogramme - Original'), xlabel('Intensité des pixels'), ylabel('Nombre de pixels');
subplot(2,3,5), plot(hist_gaussian, 'b'), title('Histogramme - Flou Gaussien'), xlabel('Intensité des pixels'), ylabel('Nombre de pixels');
subplot(2,3,6), plot(hist_sharpened, 'r'), title('Histogramme - Sharpening'), xlabel('Intensité des pixels'), ylabel('Nombre de pixels');

% Ajustement de l'affichage
sgtitle('Comparaison des Transformations d''Image');

%%

% Lecture de la vidéo
videoFile = 'C:\Users\Nicolas\Nextcloud\YOLO11\Tracking_Video\camera-1_C0003.MP4';
vidReader = VideoReader(videoFile);
framePrev = rgb2gray(readFrame(vidReader));

% Initialisation de l'optical flow (Lucas-Kanade)
opticFlow = opticalFlowLK('NoiseThreshold',0.009);

% Boucle sur les frames
while hasFrame(vidReader)
    % Lire la frame suivante et la convertir en niveaux de gris
    frameCurr = rgb2gray(readFrame(vidReader));
    
    % Prétraitement : réduction du bruit et amélioration
    frameSmooth = imgaussfilt(frameCurr, 2); % Flou gaussien
    frameSharp = imsharpen(frameSmooth, 'Radius', 3, 'Amount', 1.5); % Amélioration de la netteté
    
    % Calcul de l'optical flow entre la frame précédente et actuelle
    flow = estimateFlow(opticFlow, frameSharp);
    
    % Visualiser le flow sur la frame
    imshow(frameSharp);
    hold on;
    plot(flow, 'DecimationFactor',[5 5],'ScaleFactor',10);
    hold off;
    
    % Mise à jour pour la prochaine itération
    framePrev = frameCurr;
    
    pause(0.05); % pour contrôler la vitesse de visualisation
end

%%

% Charger la vidéo
videoFile = 'C:\Users\Nicolas\Nextcloud\YOLO11\Tracking_Video\camera-1_C0003.MP4';
vidReader = VideoReader(videoFile);

% Initialisation des variables pour stocker les résultats
mse_values = [];
psnr_values = [];
ssim_values = [];

% Lire la première frame
if hasFrame(vidReader)
    prevFrame = rgb2gray(readFrame(vidReader)); % Convertir en niveaux de gris
end

% Parcourir toutes les frames de la vidéo
frameIndex = 1;
while hasFrame(vidReader)
    currFrame = rgb2gray(readFrame(vidReader)); % Frame actuelle en niveaux de gris
    
    % Calcul de MSE
    mse = immse(currFrame, prevFrame);
    mse_values_no = [mse_values_no, mse];

    % Calcul de PSNR
    psnr_value = psnr(currFrame, prevFrame);
    psnr_values_no = [psnr_values_no, psnr_value];

    % Calcul de SSIM
    ssim_value = ssim(currFrame, prevFrame);
    ssim_values_no = [ssim_values_no, ssim_value];

    % Mise à jour de la frame précédente
    prevFrame = currFrame;
    
    
    frameIndex = frameIndex + 1;
end

% Affichage des résultats
figure;
subplot(3,1,1);
plot(mse_values_no, 'r');
title('Évolution de MSE entre frames');
xlabel('Numéro de Frame');
ylabel('MSE');

subplot(3,1,2);
plot(psnr_values_no, 'b');
title('Évolution de PSNR entre frames');
xlabel('Numéro de Frame');
ylabel('PSNR (dB)');

subplot(3,1,3);
plot(ssim_values_no, 'g');
title('Évolution de SSIM entre frames');
xlabel('Numéro de Frame');
ylabel('SSIM');

sgtitle('Analyse des variations entre frames non traitée');

%% Analyse avec pre-traitement 

% Charger la vidéo
videoFile = 'C:\Users\Nicolas\Nextcloud\YOLO11\Calibration vidéo\Normal\camera-1_C0003.MP4';
vidReader = VideoReader(videoFile);

% Initialisation des variables pour stocker les résultats
mse_values = [];
psnr_values = [];
ssim_values = [];

% Lire la première frame
if hasFrame(vidReader)
    prevFrame = rgb2gray(readFrame(vidReader)); % Convertir en niveaux de gris

    % Prétraitement : réduction du bruit et amélioration
    frameSmooth = imgaussfilt(prevFrame, 2); % Flou gaussien
    frameSharp = imsharpen(frameSmooth, 'Radius', 3, 'Amount', 1.5); % Amélioration de la netteté

    prevFrame = frameSharp;
    
end

% Parcourir toutes les frames de la vidéo
frameIndex = 1;
while hasFrame(vidReader)
    currFrame = rgb2gray(readFrame(vidReader)); % Frame actuelle en niveaux de gris
    
    % Prétraitement : réduction du bruit et amélioration
    frameSmooth = imgaussfilt(currFrame, 2); % Flou gaussien
    frameSharp = imsharpen(frameSmooth, 'Radius', 3, 'Amount', 1.5); % Amélioration de la netteté
    
    % Calcul de MSE
    mse = immse(frameSharp, prevFrame);
    mse_values = [mse_values, mse];

    % Calcul de PSNR
    psnr_value = psnr(frameSharp, prevFrame);
    psnr_values = [psnr_values, psnr_value];

    % Calcul de SSIM
    ssim_value = ssim(frameSharp, prevFrame);
    ssim_values = [ssim_values, ssim_value];

    % Mise à jour de la frame précédente
    prevFrame = frameSharp;
    
    
    frameIndex = frameIndex + 1;
end

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

sgtitle('Analyse des variations entre frames Pré-traiter');



%% Analyse signal

videoReader = VideoReader('C:\Users\Nicolas\Nextcloud\YOLO11\Tracking_Video\camera-1_C0002_corrigee.mp4');

i =1;
figure;

while hasFrame(videoReader)
    frame = readFrame(videoReader);
    if size(frame,3) == 3
        frameGray = rgb2gray(frame);
    else
        frameGray = frame;
    end

    brisqueScore(i) = brisque(frameGray);
    niqeScore(i) = niqe(frameGray);

    fprintf('NIQE score: %.2f\n', niqeScore(i));
    fprintf('BRISQUE score: %.2f\n', brisqueScore(i));
    
    subplot(1,2,1)
    hold on
    plot(i,brisqueScore(i),'o')
    drawnow;

    subplot(1,2,2)
    hold on
    plot(i,niqeScore(i),'o')
    drawnow;
    
    i = i+1;
end

