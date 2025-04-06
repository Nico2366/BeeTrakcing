% Récuperer trajectoire camera 1

trajectory = readtable('trajectoire_camera-1_C0002_corrigee.csv');

trajectory1 = table2array(trajectory);

flower_camera_1 = [];
bee1_camera_1 = [];
bee2_camera_1 = [];

for i = 1:length(trajectory1)
    
    frame = trajectory1(i,1)+1;
    ID  = trajectory1(i,2);

    if ID == 2

        flower_camera_1(frame,:) = [trajectory1(i,3) trajectory1(i,4)];

    elseif ID == 14
        
        bee1_camera_1(frame,:) = [trajectory1(i,3) trajectory1(i,4)];

    elseif ID == 15

        bee2_camera_1(frame,:) = [trajectory1(i,3) trajectory1(i,4)];

    end

end

bee1_camera_1(isnan(bee1_camera_1)) = 0;
bee2_camera_1(isnan(bee2_camera_1)) = 0;

figure;
hold on 
plot(flower_camera_1(:,1),flower_camera_1(:,2),'ro')
plot(bee1_camera_1(:,1),bee1_camera_1(:,2),'ro')
plot(bee2_camera_1(:,1),bee2_camera_1(:,2),'ro')

% Camera 2 

trajectory = readtable('trajectoire_camera-2_C0002_corrigee.csv');

trajectory2 = table2array(trajectory);

flower_camera_2 = [];
bee1_camera_2 = [];
bee2_camera_2 = [];

for i = 1:length(trajectory2)
    
    frame = trajectory2(i,1)+1;
    ID  = trajectory2(i,2);

    if ID == 81

        flower_camera_2(frame,:) = [trajectory2(i,3) trajectory2(i,4)];

    elseif ID == 91
        
        bee1_camera_2(frame,:) = [trajectory2(i,3) trajectory2(i,4)];

    elseif ID == 92 

        bee2_camera_2(frame,:) = [trajectory2(i,3) trajectory2(i,4)];

    end

end

bee1_camera_2(isnan(bee1_camera_2)) = 0;
bee2_camera_2(isnan(bee2_camera_2)) = 0;

plot(flower_camera_2(:,1),flower_camera_2(:,2),'bo')
plot(bee1_camera_2(:,1),bee1_camera_2(:,2),'bo')
plot(bee2_camera_2(:,1),bee2_camera_2(:,2),'bo')

%% triangulate bee

undistortedPoints_camera_1  = undistortPoints(bee1_camera_1, stereoParams.CameraParameters1);
undistortedPoints_camera_2  = undistortPoints(bee1_camera_2, stereoParams.CameraParameters2);

[points3D_traj,error] = triangulate(undistortedPoints_camera_1, undistortedPoints_camera_2, stereoParams);

theta = deg2rad(180);
Rx = [1, 0, 0; 0, cos(theta), -sin(theta); 0, sin(theta), cos(theta)];
theta = atand(points3D(7,3) / points3D(7,1));
theta_rad = deg2rad(theta);
Ry = [cos(theta_rad), 0, sin(theta_rad); 0, 1, 0; -sin(theta_rad), 0, cos(theta_rad)];

points3D_traj = points3D_traj - Average_Reference_point;
points3D_traj = (Ry * points3D_traj')';
points3D_traj = (Rx * points3D_traj')';

ind = points3D_traj(:,3) > 500;
points3D_traj(ind,:) = [];


% figure;
for i = 1:length(points3D_traj)

    
    hold on
    %plot3(points3D_traj(1,1),points3D_traj(1,3),points3D_traj(1,2),'k*')
    plot3(points3D_traj(i,1),points3D_traj(i,3),points3D_traj(i,2),'r*')
    grid on
    xlabel('x')
    ylabel('y')
    zlabel('z')
    
    drawnow

end


%% triangulate flower


undistortedPoints_camera_1  = undistortPoints(flower_camera_1, stereoParams.CameraParameters1);
undistortedPoints_camera_2  = undistortPoints(flower_camera_2, stereoParams.CameraParameters2);


[points3D_traj,~] = triangulate(undistortedPoints_camera_1, undistortedPoints_camera_2, stereoParams);

points3D_traj = points3D_traj -point3D_triangulate(8,:);
points3D_traj = (Ry * points3D_traj')';
points3D_traj = (Rx * points3D_traj')';

ind = points3D_traj(:,3) > 500;
points3D_traj(ind,:) = [];


% figure;
for i = 1:length(points3D_traj)

    
    hold on
    %plot3(points3D_traj(1,1),points3D_traj(1,3),points3D_traj(1,2),'k*')
    plot3(points3D_traj(i,1),points3D_traj(i,3),points3D_traj(i,2),'b*')
    grid on
    xlabel('x')
    ylabel('y')
    zlabel('z')
    
    drawnow

end

%%

dist = sqrt(((points3D_traj(end,1)-points3D_traj(1,1))^2)+((points3D_traj(end,2)-points3D_traj(1,2))^2)+((points3D_traj(end,3)-points3D_traj(1,3))^2));
dist_reel = sqrt(((points3D(8,1)-points3D(2,1))^2)+((points3D(8,2)-points3D(2,2))^2)+((points3D(8,3)-points3D(2,3))^2));
dist_theo = sqrt((530^2) + (470^2) + (500^2));

%% V2

% Lire le fichier CSV et convertir en tableau numérique
trajectory = readtable('trajectoire_camera-1_C0002_corrigee.csv');
data = table2array(trajectory);

% Extraire tous les ID et compter leur fréquence d'apparition
uniqueIDs = unique(data(:,2));
counts = arrayfun(@(id) sum(data(:,2)==id), uniqueIDs);

% Trier les ID par fréquence décroissante et récupérer les 4 premiers
[~, idx] = sort(counts, 'descend');
topIDs = uniqueIDs(idx(1:4));

% Initialiser une cellule pour stocker les positions (x et y) pour chacun des 4 ID
positions = cell(length(topIDs),1);
for k = 1:length(topIDs)
    positions{k} = [];  % Chaque cellule contiendra [frame, x, y]
end

% Parcourir toutes les lignes du fichier et enregistrer les positions pour les top IDs
for i = 1:size(data,1)
    frame = data(i,1) + 1;  % Ajustement du numéro de frame
    ID = data(i,2);
    pos = [data(i,3) data(i,4)];
    % Vérifier si cet ID fait partie des topIDs
    ind = find(topIDs == ID, 1);
    if ~isempty(ind)
        positions{ind} = [positions{ind}; frame, pos];
    end
end

% Affichage des trajectoires pour les 4 ID de la camera 1
figure;
hold on;
colors = ['r', 'g', 'b', 'm']; % Couleurs d'exemple pour différencier les ID
for j = 1:length(topIDs)
    if ~isempty(positions{j})
        % On trace uniquement les coordonnées x et y (colonne 2 et 3)
        plot(positions{j}(:,2), positions{j}(:,3), [colors(j) 'o'], 'DisplayName', ['ID ' num2str(topIDs(j))]);
    end
end
title('Camera 1 : 4 IDs avec le plus grand nombre de positions');
xlabel('Position X');
ylabel('Position Y');
legend;
hold off;

% camera2

% Lire le fichier CSV et convertir en tableau numérique pour camera 2
trajectory2 = readtable('trajectoire_camera-2_C0002_corrigee.csv');
data2 = table2array(trajectory2);

% Extraire tous les ID et compter leur fréquence d'apparition
uniqueIDs2 = unique(data2(:,2));
counts2 = arrayfun(@(id) sum(data2(:,2)==id), uniqueIDs2);

% Trier les ID par fréquence décroissante et récupérer les 4 premiers
[~, idx2] = sort(counts2, 'descend');
topIDs2 = uniqueIDs2(idx2(1:4));

% Initialiser une cellule pour stocker les positions pour chacun des 4 ID
positions2 = cell(length(topIDs2),1);
for k = 1:length(topIDs2)
    positions2{k} = [];  % Chaque cellule contiendra [frame, x, y]
end

% Parcourir toutes les lignes du fichier et enregistrer les positions pour les top IDs
for i = 1:size(data2,1)
    frame = data2(i,1) + 1;  % Ajustement du numéro de frame
    ID = data2(i,2);
    pos = [data2(i,3) data2(i,4)];
    % Vérifier si cet ID fait partie des topIDs
    ind = find(topIDs2 == ID, 1);
    if ~isempty(ind)
        positions2{ind} = [positions2{ind}; frame, pos];
    end
end

% Affichage des trajectoires pour les 4 ID de la camera 2
figure;
hold on;
colors = ['c', 'k', 'y', 'm']; % Choix de couleurs différent pour la camera 2
for j = 1:length(topIDs2)
    if ~isempty(positions2{j})
        plot(positions2{j}(:,2), positions2{j}(:,3), [colors(j) 'o'], 'DisplayName', ['ID ' num2str(topIDs2(j))]);
    end
end
title('Camera 2 : 4 IDs avec le plus grand nombre de positions');
xlabel('Position X');
ylabel('Position Y');
legend;
hold off;


%% V3 analyse par dossier

% Définir le dossier contenant les CSV
folder = 'C:\Users\Nicolas\Nextcloud\Matlab\Reconstruction3D\trajectoire_csv';

% Lister tous les fichiers CSV dans le dossier
files = dir(fullfile(folder, '*.csv'));

% Initialiser une structure pour stocker les résultats
results = struct('filename', {}, 'camera', {}, 'topIDs', {}, 'trajectories', {});

% Parcourir chaque fichier CSV
for i = 1:length(files)
    filename = fullfile(folder, files(i).name);
    disp(['Traitement de : ' filename]);
    
    % Lire le fichier CSV et le convertir en tableau numérique
    T = readtable(filename);
    data = table2array(T);
    
    % Déterminer la caméra à partir du nom du fichier
    if startsWith(files(i).name, 'trajectoire_camera-1')
        cam = 1;
    elseif startsWith(files(i).name, 'trajectoire_camera-2')
        cam = 2;
    else
        cam = NaN;  % fichier non reconnu
    end
    
    % Identifier les IDs présents et compter leur nombre d'apparitions
    uniqueIDs = unique(data(:,2));
    counts = arrayfun(@(id) sum(data(:,2) == id), uniqueIDs);
    
    % Sélectionner les 4 IDs avec le plus de points (longue trajectoire)
    [~, idx] = sort(counts, 'descend');
    nTop = min(4, numel(uniqueIDs));
    topIDs = uniqueIDs(idx(1:nTop));
    
    % Initialiser une cellule pour stocker les trajectoires pour chaque top ID
    trajectories = cell(nTop, 1);
    for k = 1:nTop
        trajectories{k} = [];  % Chaque cellule contiendra [frame, x, y]
    end
    
    % Parcourir les lignes du fichier et stocker les points des top IDs
    for j = 1:size(data,1)
        frame = data(j,1) + 1;  % Ajustement du numéro de frame si nécessaire
        id = data(j,2);
        pos = [data(j,3) data(j,4)];
        
        ind = find(topIDs == id, 1);
        if ~isempty(ind)
            trajectories{ind} = [trajectories{ind}; frame, pos];
        end
    end
    
    % Stocker les résultats pour ce fichier dans la structure
    results(end+1).filename = files(i).name;
    results(end).camera = cam;
    results(end).topIDs = topIDs;
    results(end).trajectories = trajectories;
end

% Affichage 
for i = 1:length(results)/2
    figure;
    hold on;
    colors = ['r', 'g', 'b', 'm']; % Couleurs pour différencier les trajectoires
    for k = 1:length(results(i).topIDs)
        traj1 = results(i).trajectories{k};
        traj2 = results(i+(length(results)/2)).trajectories{k};
        if ~isempty(traj1)
            plot(traj1(:,2), traj1(:,3), [colors(k) '*-'], 'DisplayName', ['ID ' num2str(results(i).topIDs(k))]);
            plot(traj2(:,2), traj2(:,3), [colors(k) 'o-'], 'DisplayName', ['ID ' num2str(results(i).topIDs(k))]);
        end
    end
    title(['Trajectoires pour ', results(i).filename]);
    xlabel('Position X');
    ylabel('Position Y');
    legend;
    hold off;
end


%% triangulation 
for i = 1:length(results)/2
    figure;
    hold on;
    colors = ['r', 'g', 'b', 'm']; % Couleurs pour différencier les trajectoires
    traj1 = [];
    traj2 = [];
    for k = 1:length(results(i).topIDs)
    
            traj1 = results(i).trajectories{k};
            traj2 = results(i+(length(results)/2)).trajectories{k};

            traj1(isnan(traj1)) = 0;
            traj2(isnan(traj2)) = 0;
    
            undistortedPoints_camera_1  = undistortPoints(traj1(:,[2 3]), stereoParams.CameraParameters1);
            undistortedPoints_camera_2  = undistortPoints(traj2(:,[2 3]), stereoParams.CameraParameters2);
    
            [points3D_traj,error] = triangulate(undistortedPoints_camera_1, undistortedPoints_camera_2, stereoParams);
    
            theta = deg2rad(180);
            Rx = [1, 0, 0; 0, cos(theta), -sin(theta); 0, sin(theta), cos(theta)];
            theta = atand(points3D(7,3) / points3D(7,1));
            theta_rad = deg2rad(theta);
            Ry = [cos(theta_rad), 0, sin(theta_rad); 0, 1, 0; -sin(theta_rad), 0, cos(theta_rad)];
            
            points3D_traj = points3D_traj - Average_Reference_point;
            points3D_traj = (Ry * points3D_traj')';
            points3D_traj = (Rx * points3D_traj')';
            
            ind = points3D_traj(:,3) > 500;
            points3D_traj(ind,:) = [];
            
            plot3(points3D_traj(:,1),points3D_traj(:,3),points3D_traj(:,2),[colors(k) '*'])
            grid on
            xlabel('x')
            ylabel('y')
            zlabel('z')
            
            drawnow

    end
end

%% V5 

% Paramètres et dossier
folder = 'C:\Users\Nicolas\Nextcloud\Matlab\Reconstruction3D\trajectoire_csv';
files = dir(fullfile(folder, '*.csv'));

% On stocke les résultats séparément pour cam1 et cam2
results_cam1 = struct('filename', {}, 'commonPart', {}, 'topIDs', {}, 'trajectories', {});
results_cam2 = struct('filename', {}, 'commonPart', {}, 'topIDs', {}, 'trajectories', {});

% Extraction des trajectoires dans chaque fichier CSV
for i = 1:length(files)
    filename = fullfile(folder, files(i).name);
    disp(['Traitement de : ' filename]);
    
    % Lecture du CSV et conversion en tableau numérique
    T = readtable(filename);
    data = table2array(T);
    % Remplacer les NaN par 0 pour éviter les erreurs
    data(isnan(data)) = 0;
    
    % Détermination de la caméra et extraction de la partie commune du nom
    if startsWith(files(i).name, 'trajectoire_camera-1')
        cam = 1;
        commonPart = extractAfter(files(i).name, 'trajectoire_camera-1_');
    elseif startsWith(files(i).name, 'trajectoire_camera-2')
        cam = 2;
        commonPart = extractAfter(files(i).name, 'trajectoire_camera-2_');
    else
        cam = NaN;
        commonPart = '';
    end
    
    % Comptage des occurrences pour chaque ID (colonne 2)
    uniqueIDs = unique(data(:,2));
    counts = arrayfun(@(id) sum(data(:,2)==id), uniqueIDs);
    
    % Sélectionner les 4 IDs avec le plus de points
    [~, idx] = sort(counts, 'descend');
    nTop = min(4, numel(uniqueIDs));
    topIDs = uniqueIDs(idx(1:nTop));
    
    % Extraction des trajectoires pour ces top IDs
    trajectories = cell(nTop,1);
    for k = 1:nTop
        trajectories{k} = [];  % Chaque cellule contiendra [frame, x, y]
    end
    
    for j = 1:size(data,1)
        frame = data(j,1) + 1;  % Ajustement du numéro de frame si nécessaire
        id = data(j,2);
        pos = [data(j,3) data(j,4)];
        % Remplacer d'éventuels NaN dans les positions
        pos(isnan(pos)) = 0;
        
        ind = find(topIDs == id, 1);
        if ~isempty(ind)
            trajectories{ind} = [trajectories{ind}; frame, pos];
        end
    end
    
    % Stockage dans la structure adaptée selon la caméra
    newEntry.filename = files(i).name;
    newEntry.commonPart = commonPart;
    newEntry.topIDs = topIDs;
    newEntry.trajectories = trajectories;
    if cam == 1
        results_cam1(end+1) = newEntry;
    elseif cam == 2
        results_cam2(end+1) = newEntry;
    end
end

% Apparier les fichiers caméra-1 et caméra-2 et traiter les trajectoires
% Pour chaque fichier de cam1, on recherche le fichier cam2 correspondant (même commonPart)
for i = 1:length(results_cam1)
    commonPart = results_cam1(i).commonPart;
    idx2 = find(strcmp({results_cam2.commonPart}, commonPart));
    if isempty(idx2)
        disp(['Aucun fichier caméra-2 trouvé pour ' results_cam1(i).filename]);
        continue;
    end
    % On prend la première correspondance trouvée
    result1 = results_cam1(i);
    result2 = results_cam2(idx2(1));
    
    % Calcul de la matrice de coût entre trajectoires de cam1 et cam2 (basé sur les frames communes)
    nTraj1 = numel(result1.trajectories);
    nTraj2 = numel(result2.trajectories);
    costMatrix = Inf(nTraj1, nTraj2);
    for a = 1:nTraj1
        traj1 = result1.trajectories{a};
        for b = 1:nTraj2
            traj2 = result2.trajectories{b};
            commonFrames = intersect(traj1(:,1), traj2(:,1));
            if ~isempty(commonFrames)
                dists = zeros(length(commonFrames),1);
                for k = 1:length(commonFrames)
                    idx1 = find(traj1(:,1)==commonFrames(k), 1);
                    idx2 = find(traj2(:,1)==commonFrames(k), 1);
                    pos1 = traj1(idx1, 2:3);
                    pos2 = traj2(idx2, 2:3);
                    dists(k) = norm(pos1-pos2);
                end
                costMatrix(a,b) = mean(dists);
            end
        end
    end
    
    disp(['Matrice de diff pour le couple : ' result1.filename ' et ' result2.filename]);
    disp(costMatrix);
    
    % Appariement naïf : pour chaque trajectoire de cam1, choisir celle de cam2 avec le coût minimum
    matches = zeros(nTraj1,1);
    for a = 1:nTraj1
        [minCost, b] = min(costMatrix(a,:));
        matches(a) = b;
        fprintf('Trajectoire %d de Cam1 (ID %d) correspond à Trajectoire %d de Cam2 (diff moyen = %.2f)\n', ...
            a, result1.topIDs(a), b, minCost);
    end
    
    % Triangulation et affichage 3D
    % On suppose ici que stereoParams et Average_Reference_point sont définis
    figure;
    hold on;
    colors = ['r', 'g', 'b', 'm'];  % Couleurs pour différencier
    for a = 1:nTraj1
        % Récupérer la trajectoire de cam1 et la trajectoire appariée de cam2
        traj1 = result1.trajectories{a};
        traj2 = result2.trajectories{matches(a)};
        
        % Remplacer les NaN par 0 (déjà fait en amont, mais pour être sûr)
        traj1(isnan(traj1)) = 0;
        traj2(isnan(traj2)) = 0;
        
        % Correction de la distorsion
        undistortedPoints_camera_1 = undistortPoints(traj1(:,2:3), stereoParams.CameraParameters1);
        undistortedPoints_camera_2 = undistortPoints(traj2(:,2:3), stereoParams.CameraParameters2);
        
        % Triangulation pour obtenir les points 3D
        [points3D_traj, error] = triangulate(undistortedPoints_camera_1, undistortedPoints_camera_2, stereoParams);
        
        theta = deg2rad(180);
        Rx = [1, 0, 0; 0, cos(theta), -sin(theta); 0, sin(theta), cos(theta)];
        theta_val = Reference_angle;
        theta_rad = deg2rad(theta_val);
        Ry = [cos(theta_rad), 0, sin(theta_rad); 0, 1, 0; -sin(theta_rad), 0, cos(theta_rad)];
        
        % Correction par translation puis rotation
        points3D_traj = points3D_traj - Average_Reference_point;
        points3D_traj = (Ry * points3D_traj')';
        points3D_traj = (Rx * points3D_traj')';
        
        % Suppression des points aberrants 
        ind = (points3D_traj(:,1) < 0 | points3D_traj(:,1) > 600) | ...
        (points3D_traj(:,2) > 0 | points3D_traj(:,2) < -600) | ...
        (points3D_traj(:,3) < 0 | points3D_traj(:,3) > 600);
        points3D_traj(ind,:) = [];

        % Affichage en 3D
        plot3(points3D_traj(:,1), points3D_traj(:,3), points3D_traj(:,2), [colors(a) '*']);
        grid on;
        xlabel('x'); ylabel('y'); zlabel('z');

    end
    hold off;
end

%% Avec statistique

% Paramètres et dossier
folder = 'C:\Users\Nicolas\Nextcloud\Matlab\Reconstruction3D\trajectoire_csv';
files = dir(fullfile(folder, '*.csv'));

% On stocke les résultats séparément pour cam1 et cam2
results_cam1 = struct('filename', {}, 'commonPart', {}, 'topIDs', {}, 'trajectories', {});
results_cam2 = struct('filename', {}, 'commonPart', {}, 'topIDs', {}, 'trajectories', {});

% Initialisation tableau pour enregistrer les stats
all_stats = {};
header = {...
    'Filename', 'Axis', 'Mean Error', 'Std Error', 'Max Error', 'RMSE', ...
    'Correlation', 'P-value', 'N Points', 'First Diff', 'Last Diff', ...
    'eucl_mean', 'eucl_rmse', 'eucl_max'};
all_stats(end+1, :) = header;

% Extraction des trajectoires dans chaque fichier CSV
for i = 1:length(files)
    filename = fullfile(folder, files(i).name);
    disp(['Traitement de : ' filename]);
    
    % Lecture du CSV et conversion en tableau numérique
    T = readtable(filename);
    data = table2array(T);
    % Remplacer les NaN par 0 pour éviter les erreurs
    data(isnan(data)) = 0;
    
    % Détermination de la caméra et extraction de la partie commune du nom
    if startsWith(files(i).name, 'trajectoire_camera-1')
        cam = 1;
        commonPart = extractAfter(files(i).name, 'trajectoire_camera-1_');
    elseif startsWith(files(i).name, 'trajectoire_camera-2')
        cam = 2;
        commonPart = extractAfter(files(i).name, 'trajectoire_camera-2_');
    else
        cam = NaN;
        commonPart = '';
    end
    
    % Comptage des occurrences pour chaque ID (colonne 2)
    uniqueIDs = unique(data(:,2));
    counts = arrayfun(@(id) sum(data(:,2)==id), uniqueIDs);
    
    % Sélectionner les 4 IDs avec le plus de points
    [~, idx] = sort(counts, 'descend');
    nTop = min(4, numel(uniqueIDs));
    topIDs = uniqueIDs(idx(1:nTop));
    
    % Extraction des trajectoires pour ces top IDs
    trajectories = cell(nTop,1);
    for k = 1:nTop
        trajectories{k} = [];  % Chaque cellule contiendra [frame, x, y]
    end
    
    for j = 1:size(data,1)
        frame = data(j,1) + 1;  % Ajustement du numéro de frame si nécessaire
        id = data(j,2);
        pos = [data(j,3) data(j,4)];
        % Remplacer d'éventuels NaN dans les positions
        pos(isnan(pos)) = 0;
        
        ind = find(topIDs == id, 1);
        if ~isempty(ind)
            trajectories{ind} = [trajectories{ind}; frame, pos];
        end
    end
    
    % Stockage dans la structure adaptée selon la caméra
    newEntry.filename = files(i).name;
    newEntry.commonPart = commonPart;
    newEntry.topIDs = topIDs;
    newEntry.trajectories = trajectories;
    if cam == 1
        results_cam1(end+1) = newEntry;
    elseif cam == 2
        results_cam2(end+1) = newEntry;
    end
end

% trajectoire de référence 3D
traj_ref = [linspace(10,545,100)', linspace(0,460,100)', linspace(-10,-510,100)'];

% Apparier les fichiers caméra-1 et caméra-2 et traiter les trajectoires
% Pour chaque fichier de cam1, on recherche le fichier cam2 correspondant (même commonPart)
for i = 1:length(results_cam1)
    commonPart = results_cam1(i).commonPart;
    idx2 = find(strcmp({results_cam2.commonPart}, commonPart));
    if isempty(idx2)
        disp(['Aucun fichier caméra-2 trouvé pour ' results_cam1(i).filename]);
        continue;
    end
    % On prend la première correspondance trouvée
    result1 = results_cam1(i);
    result2 = results_cam2(idx2(1));
    
    % Calcul de la matrice de coût entre trajectoires de cam1 et cam2 (basé sur les frames communes)
    nTraj1 = numel(result1.trajectories);
    nTraj2 = numel(result2.trajectories);
    costMatrix = Inf(nTraj1, nTraj2);
    for a = 1:nTraj1
        traj1 = result1.trajectories{a};
        for b = 1:nTraj2
            traj2 = result2.trajectories{b};
            commonFrames = intersect(traj1(:,1), traj2(:,1));
            if ~isempty(commonFrames)
                dists = zeros(length(commonFrames),1);
                for k = 1:length(commonFrames)
                    idx1 = find(traj1(:,1)==commonFrames(k), 1);
                    idx2 = find(traj2(:,1)==commonFrames(k), 1);
                    pos1 = traj1(idx1, 2:3);
                    pos2 = traj2(idx2, 2:3);
                    dists(k) = norm(pos1-pos2);
                end
                costMatrix(a,b) = mean(dists);
            end
        end
    end
    
    disp(['Matrice de diff pour le couple : ' result1.filename ' et ' result2.filename]);
    disp(costMatrix);
    
    % Appariement naïf : pour chaque trajectoire de cam1, choisir celle de cam2 avec le coût minimum
    matches = zeros(nTraj1,1);
    for a = 1:nTraj1
        [minCost, b] = min(costMatrix(a,:));
        matches(a) = b;
        fprintf('Trajectoire %d de Cam1 (ID %d) correspond à Trajectoire %d de Cam2 (diff moyen = %.2f)\n', ...
            a, result1.topIDs(a), b, minCost);
    end
    
    % Triangulation, stockage des points 3D et affichage 3D
    % figure;
    % hold on;
    colors = ['r', 'g', 'b', 'm'];  % Couleurs pour différencier
    % Cellule pour stocker la trajectoire 3D de chaque appariement
    traj3D = cell(nTraj1,1);
    for a = 1:nTraj1
        % Récupérer la trajectoire de cam1 et la trajectoire appariée de cam2
        traj1 = result1.trajectories{a};
        traj2 = result2.trajectories{matches(a)};
        
        % Remplacer les NaN par 0 (déjà fait en amont, mais pour être sûr)
        traj1(isnan(traj1)) = 0;
        traj2(isnan(traj2)) = 0;
        
        % Correction de la distorsion
        undistortedPoints_camera_1 = undistortPoints(traj1(:,2:3), stereoParams.CameraParameters1);
        undistortedPoints_camera_2 = undistortPoints(traj2(:,2:3), stereoParams.CameraParameters2);
        
        % Triangulation pour obtenir les points 3D
        [points3D_traj, error] = triangulate(undistortedPoints_camera_1, undistortedPoints_camera_2, stereoParams);
        
        theta = deg2rad(180+rad2deg(-0.0027));
        Rx = [1, 0, 0; 0, cos(theta), -sin(theta); 0, sin(theta), cos(theta)];
        theta_val = Reference_angle;
        theta_rad = deg2rad(theta_val);
        Ry = [cos(theta_rad), 0, sin(theta_rad); 0, 1, 0; -sin(theta_rad), 0, cos(theta_rad)];
        theta = deg2rad(rad2deg(-0.0098));
        Rz = [cos(theta), -sin(theta), 0; sin(theta),  cos(theta), 0; 0,           0,          1];
    
        
        % Correction par translation puis rotation
        points3D_traj = points3D_traj - Average_Reference_point;
        points3D_traj = (Ry * points3D_traj')';
        points3D_traj = (Rx * points3D_traj')';
        points3D = (Rz * points3D')';

        % Suppression des points aberrants 
        ind = (points3D_traj(:,1) < 0 | points3D_traj(:,1) > 600) | ...
              (points3D_traj(:,2) > 0 | points3D_traj(:,2) < -600) | ...
              (points3D_traj(:,3) < 0 | points3D_traj(:,3) > 600);
        points3D_traj(ind,:) = [];
        
        % Stockage de la trajectoire 3D pour le traitement ultérieur
        traj3D{a} = points3D_traj;
        
        % Affichage en 3D des trajectoire
        % plot3(points3D_traj(:,1), points3D_traj(:,3), points3D_traj(:,2), [colors(a) '*']);
        % grid on;
        % xlabel('x'); ylabel('y'); zlabel('z');
    end
    hold off;
    
    % Recherche de la trajectoire la plus proche de la droite
    % Définition de la droite passant par A=(0,0,0) et B=(565,470,510)
    A = [10, 10, 10];
    B = [545, -500, 460];
    d = B - A;
    d = d / norm(d);  % vecteur directeur unitaire

    % Pour chaque trajectoire 3D, calcul de la distance moyenne à la droite
    avgDistances = Inf(nTraj1,1);
    for a = 1:nTraj1
        pts = traj3D{a};
        if ~isempty(pts)
            % La distance d'un point P à la droite : ||(P-A) x d|| / ||d||
            distances = vecnorm(cross(pts - A, repmat(d, size(pts,1), 1)), 2, 2);
            avgDistances(a) = mean(distances);
        end
    end
    [minDist, minIndex] = min(avgDistances);
    fprintf('Pour le fichier %s, la trajectoire la plus proche de la droite est la trajectoire %d (ID %d) avec une distance moyenne de %.2f\n', ...
        result1.filename, minIndex, result1.topIDs(minIndex), minDist);
    
    % Affichage comparatif : trajectoire la plus proche et trajectoire de référence
    % On récupère la trajectoire "meilleure" et on réorganise ses colonnes pour obtenir [x, y, z]
    best_traj = traj3D{minIndex};
    if ~isempty(best_traj)
        % Réordonner : colonne 1 reste x, colonne 3 devient y et colonne 2 devient z
        best_traj = best_traj(:, [1, 3, 2]);
        
        % Affichage dans un graphique 3D
        figure;
        hold on;
        % Affichage de la trajectoire sélectionnée
        plot3(best_traj(:,1), best_traj(:,2), best_traj(:,3), 'r*-', 'LineWidth', 2, 'DisplayName', sprintf('Traj %d (ID %d)', minIndex, result1.topIDs(minIndex)));
        % Affichage de la trajectoire de référence
        plot3(traj_ref(:,1), traj_ref(:,2), traj_ref(:,3), 'bo-', 'LineWidth', 2, 'DisplayName', 'Traj de référence');
        grid on;
        xlabel('x');
        ylabel('y');
        zlabel('z');
        title(sprintf('Comparaison pour %s', result1.filename));
        legend('show', 'Location', 'Best');
        hold off;

        % 1. Calcul de la distance cumulée de traj_best
        dist_best = sqrt(sum(diff(best_traj).^2, 2));  % Distance entre chaque point successif
        dist_best_cum = [0; cumsum(dist_best)];  % Distance cumulée
        
        % 2. Calcul de la distance cumulée de traj_ref (théorique)
        dist_ref = sqrt(sum(diff(traj_ref).^2, 2));  % Distance entre chaque point successif
        dist_ref_cum = [0; cumsum(dist_ref)];  % Distance cumulée
        
        % 3. Synchronisation des deux trajectoires
        % On va maintenant faire avancer traj_ref en fonction de la distance parcourue par traj_best
        
        % Initialisation des trajectoires synchronisées
        traj_ref_sync = zeros(size(best_traj));
        
        for i = 1:size(best_traj, 1)
            % Trouver la distance cumulative la plus proche dans traj_ref
            % Ici on suppose que traj_ref doit être mappé à la même vitesse que traj_best
            [~, idx_ref] = min(abs(dist_ref_cum - dist_best_cum(i)));
            
            % Assigner la position correspondante de traj_ref à traj_ref_sync
            traj_ref_sync(i, :) = traj_ref(idx_ref, :);
        end
        
        % 4. Comparaison des trajectoires synchronisées
        % Calcul des erreurs euclidiennes entre traj_ref_sync et traj_best
        errors_euclid = sqrt(sum((traj_ref_sync - best_traj).^2, 2));
        
        % Statistiques
        eucl_mean = mean(errors_euclid);
        eucl_rmse = sqrt(mean(errors_euclid.^2));
        eucl_max = max(errors_euclid);
        
        % Affichage des statistiques
        fprintf('Erreur moyenne : %.2f mm\n', eucl_mean);
        fprintf('RMSE : %.2f mm\n', eucl_rmse);
        fprintf('Erreur max : %.2f mm\n', eucl_max);
        
        % Corrélation entre les trajectoires
        for axis = 1:3
            % Calcul des différences entre la trajectoire de référence et la trajectoire estimée
            diffs = traj_ref_sync(:, axis) - best_traj(:, axis);
            mean_err = mean(diffs);
            std_err = std(diffs);
            max_err = max(abs(diffs));
            rmse = sqrt(mean(diffs.^2));
            
            % Calcul du coefficient de corrélation de Pearson et de la p-value associée
            [corr_val, p_val] = corr(traj_ref_sync(:, axis), best_traj(:, axis));
            n_points = size(traj_ref_sync, 1);  % Taille de l'échantillon
            
            % Calcul de la différence au premier et au dernier point
            first_diff = traj_ref_sync(1, axis) - best_traj(1, axis);
            last_diff = traj_ref_sync(end, axis) - best_traj(end, axis);
            
            % Affichage des statistiques pour chaque axe
            fprintf('--- Axe %d ---\n', axis);
            fprintf('  - Erreur moyenne       : %.2f mm\n', mean_err);
            fprintf('  - Écart-type erreur    : %.2f mm\n', std_err);
            fprintf('  - Erreur max           : %.2f mm\n', max_err);
            fprintf('  - RMSE                 : %.2f mm\n', rmse);
            fprintf('  - Corrélation (Pearson): %.3f (p=%.3f, n=%d)\n', corr_val, p_val, n_points);
            fprintf('  - Diff. premier point  : %.2f mm\n', first_diff);
            fprintf('  - Diff. dernier point  : %.2f mm\n', last_diff);
            
            % Stockage des statistiques dans le tableau
            all_stats(end+1, :) = {...
                result1.filename, axes_names{axis}, ...
                mean_err, std_err, max_err, rmse, ...
                corr_val, p_val, n_points, first_diff, last_diff, ...
                eucl_mean, eucl_rmse, eucl_max ...
            };
        end
        
    else
        fprintf('Aucune trajectoire 3D valide pour la comparaison avec la référence dans le fichier %s\n', result1.filename);
    end
end

% Sauvegarde du tableau de statistiques dans un fichier CSV
result_table = cell2table(all_stats(2:end,:), 'VariableNames', all_stats(1,:));
writetable(result_table, 'resultats_stats_trajectoires.csv');
disp('Fichier CSV des statistiques exporté : resultats_stats_trajectoires.csv');


