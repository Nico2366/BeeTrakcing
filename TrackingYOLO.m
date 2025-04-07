%...................................................................
% Author : Nicolas Salvage
% function : Triangulation function 
%...................................................................


% Paramètres et dossier
folder = 'trajectoire_csv';
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
    reconstruction3D(points3D);
    hold on;
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
        points3D = (Rz * points3D_traj')';

        % Suppression des points aberrants 
        ind = (points3D_traj(:,1) < 0 | points3D_traj(:,1) > 600) | ...
              (points3D_traj(:,2) > 0 | points3D_traj(:,2) < -600) | ...
              (points3D_traj(:,3) < 0 | points3D_traj(:,3) > 600);
        points3D_traj(ind,:) = [];
        
        % Stockage de la trajectoire 3D pour le traitement ultérieur
        traj3D{a} = points3D_traj;
        
        %Affichage en 3D des trajectoire
        plot3(points3D_traj(:,1), points3D_traj(:,3), points3D_traj(:,2), [colors(a) '*']);
        grid on;
        xlabel('x'); ylabel('y'); zlabel('z');
    end
    hold off;
    
    
end


