%...................................................................
% Author : Nicolas Salvage
% function : Triangulation function 
%...................................................................

function points3D_traj = Trajectorie_Reconstruction(R,point3D_triangulate,stereoParams)

    % Récuperer trajectoire camera 1
    trajectory = readtable('trajectoire_camera-1_C0005.csv');    
    trajectory1 = table2array(trajectory);
    
    flower_camera_1 = [];
    bee1_camera_1 = [];
    
    for i = 1:length(trajectory1)
        
        frame = trajectory1(i,1)+1;
        ID  = trajectory1(i,2);
    
        if ID == 0
    
            flower_camera_1(frame,:) = [trajectory1(i,3) trajectory1(i,4)];
    
        elseif ID == 1
            
            bee1_camera_1(frame,:) = [trajectory1(i,3) trajectory1(i,4)];
            traj1(frame,:) = [frame trajectory1(i,3) trajectory1(i,4)];
    
        end
    
    end
    
    bee1_camera_1(isnan(bee1_camera_1)) = 0;
    
    figure;
    hold on 
    plot(flower_camera_1(:,1),flower_camera_1(:,2),'ro')
    plot(bee1_camera_1(:,1),bee1_camera_1(:,2),'ro')
    
    % Camera 2 
    trajectory = readtable('trajectoire_camera-2_C0005.csv');  
    trajectory2 = table2array(trajectory);
    
    flower_camera_2 = [];
    bee1_camera_2 = [];
    
    for i = 1:length(trajectory2)
        
        frame = trajectory2(i,1)+1;
        ID  = trajectory2(i,2);
    
        if ID == 0
            flower_camera_2(frame,:) = [trajectory2(i,3) trajectory2(i,4)];
    
        elseif ID == 1
            
            bee1_camera_2(frame,:) = [trajectory2(i,3) trajectory2(i,4)];
            traj2(frame,:) = [frame trajectory2(i,3) trajectory2(i,4)];
    
        end
    
    end
    
    bee1_camera_2(isnan(bee1_camera_2)) = 0;
    
    plot(flower_camera_2(:,1),flower_camera_2(:,2),'bo')
    plot(bee1_camera_2(:,1),bee1_camera_2(:,2),'bo')

    % triangulate bee
    
    undistortedPoints_camera_1  = undistortPoints(bee1_camera_1, stereoParams.CameraParameters1);
    undistortedPoints_camera_2  = undistortPoints(bee1_camera_2, stereoParams.CameraParameters2);
    
    [points3D_traj,error] = triangulate(undistortedPoints_camera_1, undistortedPoints_camera_2, stereoParams);
    
    points3D_traj = points3D_traj - point3D_triangulate;
    points3D_traj = (R' * points3D_traj')';
    
    % Suppression des points aberrants 
    ind = (points3D_traj(:,1) > 0 | points3D_traj(:,1) < -600) | ...
          (points3D_traj(:,2) > 0 | points3D_traj(:,2) < -600) | ...
          (points3D_traj(:,3) > 0 | points3D_traj(:,3) < -600);
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

end

