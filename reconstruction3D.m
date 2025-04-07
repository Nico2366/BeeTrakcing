%...................................................................
% Author : Nicolas Salvage
% function : Reconstruction 3D flying Arena function 
%...................................................................


function reconstruction3D(points3D)
    
    % Carré reconstruction
    figure;
    hold on;
    plot3([points3D(1,1); points3D(2,1)], [points3D(1,3); points3D(2,3)], [points3D(1,2); points3D(2,2)], 'b-');
    plot3([points3D(1,1); points3D(4,1)], [points3D(1,3); points3D(4,3)], [points3D(1,2); points3D(4,2)], 'g-');
    plot3([points3D(1,1); points3D(5,1)], [points3D(1,3); points3D(5,3)], [points3D(1,2); points3D(5,2)], 'k-');
    plot3([points3D(2,1); points3D(3,1)], [points3D(2,3); points3D(3,3)], [points3D(2,2); points3D(3,2)], 'r--');
    plot3([points3D(2,1); points3D(6,1)], [points3D(2,3); points3D(6,3)], [points3D(2,2); points3D(6,2)], 'r--');
    plot3([points3D(3,1); points3D(4,1)], [points3D(3,3); points3D(4,3)], [points3D(3,2); points3D(4,2)], 'r--');
    plot3([points3D(3,1); points3D(7,1)], [points3D(3,3); points3D(7,3)], [points3D(3,2); points3D(7,2)], 'r--');
    plot3([points3D(4,1); points3D(8,1)], [points3D(4,3); points3D(8,3)], [points3D(4,2); points3D(8,2)], 'r--');
    plot3([points3D(8,1); points3D(5,1)], [points3D(8,3); points3D(5,3)], [points3D(8,2); points3D(5,2)], 'r--');
    plot3([points3D(5,1); points3D(6,1)], [points3D(5,3); points3D(6,3)], [points3D(5,2); points3D(6,2)], 'r--');
    plot3([points3D(6,1); points3D(7,1)], [points3D(6,3); points3D(7,3)], [points3D(6,2); points3D(7,2)], 'r--');
    plot3([points3D(7,1); points3D(8,1)], [points3D(7,3); points3D(8,3)], [points3D(7,2); points3D(8,2)], 'r--');
    
    x = [points3D(1,1), points3D(2,1), points3D(3,1), points3D(4,1)];
    y = [points3D(1,3), points3D(2,3), points3D(3,3), points3D(4,3)];
    z = [points3D(1,2), points3D(2,2), points3D(3,2), points3D(4,2)];
    c = [0.7 0.7 0.7];
    fill3(x, y, z, c);
    
    % plot3([points3D(2,1); points3D(8,1)], [points3D(2,3); points3D(8,3)], [points3D(2,2); points3D(8,2)], 'y--');
    xlabel('x');
    ylabel('y');
    zlabel('z');
    grid on;
    
    for i = 1:size(points3D,1)
        text(points3D(i,1), points3D(i,3), points3D(i,2), sprintf('P%d', i), 'FontSize', 12, 'Color', 'red');
    end
    hold off;
    
end