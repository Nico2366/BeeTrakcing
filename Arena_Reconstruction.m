%...................................................................
% Author : Nicolas Salvage
% Last update : 07/04/2025
% function : Detection Point & Reconstruction 3D 
%...................................................................

function [Average_Reference_point,Reference_angle] = Arena_Reconstruction(stereoParams,theta_x,theta_y,theta_z)

    % Sélection des dossiers d'images
    folder_left = '3D_arene\camera-1'; %uigetdir(pwd, 'Sélectionnez le dossier images pour la caméra gauche');
    folder_right = '3D_arene\camera-2'; %uigetdir(pwd, 'Sélectionnez le dossier images pour la caméra droite');
    
    if isequal(folder_left, 0) || isequal(folder_right, 0)
        disp('Sélection annulée.');
        return;
    end
    
    % Chargement des images
    imgExt = {'*.jpg', '*.png', '*.bmp'};
    files_left = []; files_right = [];
    for k = 1:length(imgExt)
        files_left = [files_left; dir(fullfile(folder_left, imgExt{k}))];
        files_right = [files_right; dir(fullfile(folder_right, imgExt{k}))];
    end
    
    nPairs = min(length(files_left), length(files_right));
    if nPairs == 0
        error('Aucune image trouvée.');
    end
    fprintf('Nombre de paires à traiter : %d\n', nPairs);
    
    validPairFound = false;
    
    % Liste pour stocker les boîtes 3D reconstruites
    boxes3D = cell(nPairs, 1);
    all_Reference_point = zeros(nPairs,3);
    
    for i = 1:nPairs
        fprintf('Traitement de la paire %d/%d...\n', i, nPairs);
        image_L = imread(fullfile(folder_left, files_left(i).name));
        image_R = imread(fullfile(folder_right, files_right(i).name));
        
        [points_L, points_R] = detectPoints(image_L, image_R);
        if size(points_L, 1) ~= 8 || size(points_R, 1) ~= 8
            warning('Paire %d ignorée : %d points à gauche, %d points à droite.', i, size(points_L, 1), size(points_R, 1));
            continue;
        end
        
        if ~validPairFound
            disp('Première paire valide trouvée : sélection interactive.');
            ref_left = selectPointsOrder(image_L, points_L);
            ref_right = selectPointsOrder(image_R, points_R);
            ordered_points_L = ref_left;
            ordered_points_R = ref_right;
            validPairFound = true;
        else
            ordered_points_L = autoMatchPoints(ref_left, points_L);
            ordered_points_R = autoMatchPoints(ref_right, points_R);
        end
        
        
        if isempty(ordered_points_L) || isempty(ordered_points_R)
            warning('Paire %d ignorée : correspondance automatique impossible.', i);
            continue;
        end
        
        pts_cam1 = ordered_points_L(:, [2, 1]);
        pts_cam2 = ordered_points_R(:, [2, 1]);
        
        try
            pts_cam1 = undistortPoints(pts_cam1, stereoParams.CameraParameters1);
            pts_cam2 = undistortPoints(pts_cam2, stereoParams.CameraParameters2);
            [points3D, ~] = triangulate(pts_cam1, pts_cam2, stereoParams);
            [points3D,Reference_point,Reference_angle] = process3DPoints(points3D,theta_x,theta_y,theta_z);
    
            % Enregistrer la boîte 3D pour la paire actuelle
            boxes3D{i} = points3D;
            all_Reference_point(i,:) = Reference_point;
    
            ind = all_Reference_point(:,1) == 0;
            all_Reference_point(ind,:) = [];
            Average_Reference_point = mean(all_Reference_point);
            
            fprintf('Paire %d traitée avec succès.\n', i);
        catch ME
            warning('Erreur lors du traitement de la paire %d : %s', i, ME.message);
        end
    end
    
    % Comparer les boîtes avec les boîtes précédentes
    points = compareBoxes3D(boxes3D, i);
    
    % --- Fonctions Utilitaires ---
    
    % Fonction pour comparer les boîtes 3D
    function point = compareBoxes3D(boxes3D, currentIndex)
        
        %supprimer les cellule vide 
        idx = cellfun('isempty',boxes3D);
        boxes3D(idx) = [];
    
        n = 1; % Initialiser n
    
        for j = 1:size(boxes3D, 1) % Boucle à travers les différentes boîtes dans boxes3D (en supposant que chaque ligne représente une boîte)
            for k = 1:8 % Boucle à travers les 8 points de chaque boîte
                point{k}(n, :) = boxes3D{j}(k, :); % Stocker le k-ième point de la j-ième boîte
            end
            n = n + 1; % Incrémenter n pour la boîte suivante
        end
        
        ecart_mean_max = 5; % Seuil d'écart-type pour considérer un outlier
    
        % Initialisation des matrices pour stocker les moyennes et écarts-types
        mean_points = zeros(8, 3); % Moyennes pour chaque point (8 points et 3 dimensions)
        std_points = zeros(8, 3);  % Ecarts-types pour chaque point (8 points et 3 dimensions)
    
       points_theo = [0 -510 470; 565 -510 470; 565 -510 0; 0 -510 0; 0 0 470; 565 0 470; 565 0 0; 0 0 0];
    
        for k = 1:8
        
            mean_av(k, :) = mean(point{k}); % Moyenne pour chaque colonne (x, y, z)
            std_av(k, :) = std(point{k}); % Ecart-type pour chaque colonne (x, y, z)
        
            % Calcul de la différence
            Diff{k} = point{k} - points_theo(k, :);
            
            % Création d'un masque logique pour chaque colonne
            Seuil{k} = all(abs(Diff{k}) < 15, 2);  % Vérifie que toutes les colonnes respectent le seuil
            
            % Application du masque sur les lignes complètes
            point{k} = point{k}(Seuil{k}, :);
        
            mean_ap(k, :) = mean(point{k}); % Moyenne pour chaque colonne (x, y, z)
            std_ap(k, :) = std(point{k}); % Ecart-type pour chaque colonne (x, y, z)
        end
        
        points3D = mean_ap;
        % Calcul des différences moyennes
        d_x = [(points3D(2,1)-points3D(1,1)) (points3D(3,1)-points3D(4,1)) (points3D(6,1)-points3D(5,1)) (points3D(7,1)-points3D(8,1))];
        d_y = [(points3D(5,2)-points3D(1,2)) (points3D(6,2)-points3D(2,2)) (points3D(7,2)-points3D(3,2)) (points3D(8,2)-points3D(4,2))];
        d_z = [(points3D(4,3)-points3D(1,3)) (points3D(3,3)-points3D(2,3)) (points3D(8,3)-points3D(5,3)) (points3D(7,3)-points3D(6,3))];
        
        largeur_mean_x = abs(mean(d_x));
        longueur_mean_y = abs(mean(d_z));
        Hauteur_mean_z = abs(mean(d_y));
        
        Diff_x = largeur_mean_x - 565;
        Diff_y = longueur_mean_y - 470;
        Diff_z = Hauteur_mean_z - 510;
        
        fprintf('Différence moyenne x (en mm) %4f .\n', Diff_x);
        fprintf('Différence moyenne y (en mm) %4f .\n', Diff_y);
        fprintf('Différence moyenne z (en mm) %4f .\n', Diff_z);
        
        
        % Pour visualiser les points sans outliers
        figure;
        hold on;
        for k = 1:8
             plot3(point{k}(:, 1), point{k}(:, 2), point{k}(:, 3), 'o');
             %plot3(mean_points(k,1),mean_points(k,3),mean_points(k,2),'o')
        end
        hold off;
        %Carré
        reconstruction3D(points3D);
       
    end
    
   
    
    function [points_L, points_R] = detectPoints(image_L, image_R)
        points_L = extractPoints(image_L);
        points_R = extractPoints(image_R);
    end
    
    function points = extractPoints(image)
        
        hsv = rgb2hsv(image);
    
        % Création du masque avec les mêmes seuils HSV  (1=Hue, 2=Saturation, 3=Value)
        % Detection du vert
        %mask = (hsv(:,:,1) >= 0.17 & hsv(:,:,1) <= 0.35) & (hsv(:,:,2) >= 0.3 & hsv(:,:,2) <= 1) & (hsv(:,:,3) >= 0.1 & hsv(:,:,3) <= 1);
        
        %Detection du rouge 
        mask = ((hsv(:,:,1) >= 0 & hsv(:,:,1) <= 0.05) | (hsv(:,:,1) >= 0.95 & hsv(:,:,1) <= 1))  & (hsv(:,:,2) >= 0.3 & hsv(:,:,2) <= 1) & (hsv(:,:,3) >= 0.1 & hsv(:,:,3) <= 1);

        mask = bwareaopen(mask, 50); % réglage du seuil de détection
        B = bwboundaries(mask, 'noholes'); % Extraire les frontières des régions
        
        if isempty(B)
            points = [];
            return;
        end
    
        % Calculer le centre moyen de chaque région détectée
        temp = cellfun(@(x) mean(x,1), B, 'UniformOutput', false);
        points = vertcat(temp{:});
    end
    
    
    function ordered_points = selectPointsOrder(image, points)
        figure;
        imshow(image);
        hold on;
        plot(points(:,2), points(:,1), 'ro', 'MarkerSize', 8, 'LineWidth', 2);
        title('Cliquez sur les points dans l''ordre voulu');
        availableIndices = 1:size(points, 1);
        newOrder = zeros(size(points, 1), 1);
    
        for k = 1:size(points, 1)
            [xClick, yClick] = ginput(1);
            distances = sqrt((points(availableIndices,2) - xClick).^2 + (points(availableIndices,1) - yClick).^2);
            [~, localIdx] = min(distances);
            selectedIndex = availableIndices(localIdx);
            newOrder(k) = selectedIndex;
    
            plot(points(selectedIndex,2), points(selectedIndex,1), 'gx', 'MarkerSize', 12, 'LineWidth', 2);
            text(points(selectedIndex,2), points(selectedIndex,1), sprintf(' P%d', k), 'Color', 'green', 'FontSize', 12, 'FontWeight', 'bold');
    
            availableIndices(localIdx) = [];
        end
        hold off;
        ordered_points = points(newOrder, :);
    end
    
    function ordered_points = autoMatchPoints(ref_points, candidate_points)
        ordered_points = zeros(size(ref_points));
        for k = 1:size(ref_points, 1)
            distances = sqrt(sum((candidate_points - ref_points(k,:)).^2, 2));
            [~, idx] = min(distances);
            ordered_points(k,:) = candidate_points(idx,:);
        end
    end
    
    function [points3D,Reference_point,Reference_angle]  = process3DPoints(points3D,theta_x,theta_y,theta_z)
    
        Reference_point = points3D(8,:);
        points3D = points3D - points3D(8,:);
        points3D(5,2) = points3D(5,2) - 30;
        points3D(6,2) = points3D(6,2) - 30;
    
        Reference_angle = atand(points3D(7,3) / points3D(7,1));
        theta_rad = deg2rad(Reference_angle);
        Ry = [cos(theta_rad), 0, sin(theta_rad); 0, 1, 0; -sin(theta_rad), 0, cos(theta_rad)];
        points3D = (Ry * points3D')';
    
        theta = deg2rad(180+rad2deg(theta_x));
        Rx = [1, 0, 0; 0, cos(theta), -sin(theta); 0, sin(theta), cos(theta)];
        points3D = (Rx * points3D')';
    
        theta = deg2rad(rad2deg(theta_z));
        Rz = [cos(theta), -sin(theta), 0; sin(theta),  cos(theta), 0; 0,           0,          1];
        points3D = (Rz * points3D')';
    
    end
end


