close all
clc
clear

% Sélectionner les deux vidéos avec uigetfile
[videoFile1, videoPath1] = uigetfile('*.MP4', 'Sélectionner la première vidéo');
[videoFile2, videoPath2] = uigetfile('*.MP4', 'Sélectionner la deuxième vidéo');

% Construire les chemins complets des vidéos
videoPath1 = fullfile(videoPath1, videoFile1);
videoPath2 = fullfile(videoPath2, videoFile2);

[~, videoName1, ~] = fileparts(videoFile1);
[~, videoName2, ~] = fileparts(videoFile2);

% Exécuter FFmpeg avec Conda via 'conda run'
disp('Activation de Conda et exécution de FFmpeg');
condaPath = 'C:\Users\Nicolas\anaconda3\Scripts\conda.exe';  % Remplace par le chemin correct vers Conda sur ton PC
command1 = [condaPath, ' run -n ultralytics-env ffprobe -v error -select_streams d:0 -show_entries stream_tags=timecode -of default=noprint_wrappers=1 "' videoPath1 '" > "C:\Users\Nicolas\Nextcloud\Matlab\Reconstruction3D\timecode.txt"'];
command2 = [condaPath, ' run -n ultralytics-env ffprobe -v error -select_streams d:0 -show_entries stream_tags=timecode -of default=noprint_wrappers=1 "' videoPath2 '" > "C:\Users\Nicolas\Nextcloud\Matlab\Reconstruction3D\timecode2.txt"'];

% Exécuter les commandes
system(command1);
system(command2);

% Définir les chemins des fichiers timecode
cheminFichier1 = 'C:\Users\Nicolas\Nextcloud\Matlab\Reconstruction3D\timecode.txt';
cheminFichier2 = 'C:\Users\Nicolas\Nextcloud\Matlab\Reconstruction3D\timecode2.txt';

% Charger les vidéos
video1 = VideoReader(videoPath1);
video2 = VideoReader(videoPath2);

% Fonction pour extraire le timecode depuis un fichier
function timecode = extraireTimecode(chemin)
    if exist(chemin, 'file')
        contenu = fileread(chemin);
        pattern = 'timecode=(\d+:\d+:\d+:\d+)';
        match = regexp(contenu, pattern, 'tokens');
        if ~isempty(match)
            timecode = match{1}{1};
        else
            error(['⚠️ Timecode non trouvé dans : ', chemin]);
        end
    else
        error(['❌ Fichier introuvable : ', chemin]);
    end
end

try
    % Extraction des timecodes des fichiers
    tc1 = extraireTimecode(cheminFichier1);
    tc2 = extraireTimecode(cheminFichier2);

    disp(['📸 Timecode caméra 1 : ', tc1]);
    disp(['📸 Timecode caméra 2 : ', tc2]);

    % Définir la fréquence d'images (FPS)
    fps = video1.framerate; % Ajuste en fonction de ta vidéo

    % Fonction pour convertir timecode en nombre total de frames
    % tc1 = str2double(split(tc1, ':'))';
    % tc2 = str2double(split(tc2, ':'))';
    timecodeToFrames = @(tc) sum(str2double(split(tc, ':')) .* [fps*60*60, fps*60, fps, 1]);

    % Conversion en frames
    frames1 = timecodeToFrames(tc1);
    frames2 = timecodeToFrames(tc2);

    % Calcul du décalage
    offset = frames2 - frames1;
    disp(['📏 Décalage entre les caméras : ', num2str(offset), ' frames']);

catch ME
    disp(['❗ Erreur : ', ME.message]);
end

% Nombre total de frames
frames1 = video1.NumFrames;
frames2 = video2.NumFrames;

% Sélection des indices des images clés
frame_indices = [1, round(frames1/2), (frames1-abs(offset(4)))];

% Création d'une figure pour afficher les images
figure;

for i = 1:length(frame_indices)

    if offset(4) < 0
        idx1 = frame_indices(i)  ;
        idx2 = frame_indices(i) + abs(offset(4)) ;  % Applique l'offset de synchronisation
    else
        idx1 = frame_indices(i) + abs(offset(4))   ;
        idx2 = frame_indices(i);  % Applique l'offset de synchronisation
    end

    % Vérifier que les indices sont dans la plage des frames disponibles
    if idx2 > frames2
        idx2 = frames2;
    elseif idx2 < 1
        idx2 = 1;
    end
    
    % Lire les frames exactes
    frame1 = read(video1, idx1);
    frame2 = read(video2, idx2);
    
    % Affichage des images extraites
    subplot(2, length(frame_indices), i);
    imshow(frame1);
    title(sprintf('Vidéo 1 - Frame %d', idx1));
    
    subplot(2, length(frame_indices), i + length(frame_indices));
    imshow(frame2);
    title(sprintf('Vidéo 2 - Frame %d', idx2));
end

fprintf('✅ Affichage des images clés terminé.\n');

% Créer un objet VideoWriter pour réécrire la première vidéo après décalage
outputVideoPath1 = [ videoName1 '_sync.MP4'];
outputVideoPath2 = [ videoName2 '_sync.MP4'];

outputVideo1 = VideoWriter(outputVideoPath1, 'MPEG-4');
outputVideo2 = VideoWriter(outputVideoPath2, 'MPEG-4');

outputVideo1.FrameRate = video1.framerate;
outputVideo2.FrameRate = video1.framerate;

open(outputVideo1);
open(outputVideo2);

idx1 = 1;
idx2 = 1;

if offset(4) < 0

    idx2 = idx2 + abs(offset(4));
else

    idx1 = idx1 + abs(offset(4));
end

% Réécrire la vidéo synchronisée avec l'offset
for i = 1:(frames1-abs(offset(4)))
    
    % Lire les frames exactes
    frame1 = read(video1, idx1);
    frame2 = read(video2, idx2);

    idx1 = idx1+1;
    idx2 = idx2+1;

    % Écrire les frames dans les fichiers de sortie
    writeVideo(outputVideo1, frame1);
    writeVideo(outputVideo2, frame2);
end

% Fermer les fichiers de sortie
close(outputVideo1);
close(outputVideo2);

fprintf('✅ Les vidéos synchronisées ont été enregistrées avec succès.\n');
