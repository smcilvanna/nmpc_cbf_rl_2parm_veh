function out = generateCurriculumEnvironment(curriculum_level, targetPosNormal)
% GENERATERANDOMENVIRONMENT with curriculum support
    % ===== Curriculum Presets =====
    switch curriculum_level
        case 1 % Beginner (no obstacles)
            num_circles = 0;
            min_spacing = 10;
            grid_size = 30;
            targetPathObs = 0;
            % radii_set = [];
            
        case 2 % Novice
            num_circles = 2;
            min_spacing = 8;
            grid_size = 40;
            targetPathObs = 1;
            
        case 3 % Intermediate
            num_circles = 4;
            min_spacing = 4;
            grid_size = 50;
            targetPathObs = 2;
            
        case 4 % Expert
            num_circles = 6;
            min_spacing = 2;
            grid_size = 30;
            targetPathObs = 3;
    end


    targetPos = [grid_size, grid_size];
    targetPos(randi(2)) = targetPosNormal(randi(2)) * grid_size;
    veh_rad = 1.0;
    radii_set = 0.5:0.5:10;
    out = struct;
    out.obsInPath = 0;
    
    while out.obsInPath <= targetPathObs


        circles = zeros(num_circles+2, 3);
        circles(1,:) = [0 0 veh_rad];                   % add a temp circle for the vehicle at start pos
        circles(2,:) = [ targetPos(:).' , veh_rad ];    % add a temp circle for the vehicle at target pos
        current_count = 0;
    
        % % Handle radius selection mode
        % if nargin < 4 || isempty(radii_set)
        %     use_continuous = true;
        % else
        %     use_continuous = false;
        %     validateattributes(radii_set, {'numeric'}, {'vector', '>=', 0.1, '<=', 10});
        % end
        
        % Place circles with collision checking
        while current_count < num_circles
            % Generate candidate circle
            x = rand * grid_size;
            y = rand * grid_size;
            
            % Select radius
            radius = randsample(radii_set, 1);
            
            % Validate spacing constraints
            valid = true;
            for i = 1:current_count+2
                existing = circles(i,:);
                distance = norm([x,y] - existing(1:2));
                min_required = radius + existing(3) + min_spacing;
                
                if distance < min_required
                    valid = false;
                    break;
                end
            end
            
            % Add valid circle
            if valid
                current_count = current_count + 1;
                circles(current_count+2,:) = [x, y, radius];
            end
        end
    
        % Calculate coverage percentage
        total_area = grid_size^2;
        covered_area = sum(pi * circles(:,3).^2);
        coverage = (covered_area / total_area) * 100;
    
    
        fig = figure(Visible="off");
        ax = axes(fig);
        hold on;
        for i = 3:height(circles)
            plotCircle([circles(i,1), circles(i,2)],circles(i,3),'-','r');
        end
        scatter(targetPos(1),targetPos(2),100,"green",'x'); % mark target position
    
        axLim = max(grid_size, max(targetPos));   % want to make the plot full extents of either the grid or the target position
    
        ax.XLim = [0 axLim];
        ax.YLim = [0 axLim];
    
        % put output elements into struct
        out.obstacles = circles(3:end,:);
        
        % check how many obstacles in path
        if height(out.obstacles) > 0
            circlePath = zeros(height(out.obstacles),1);
            for i =1:height(out.obstacles)
                circlePath(i) = lineIntersectsCircle([0,0],targetPos,out.obstacles(i,:));
            end
            out.circlePath = circlePath;
            out.obsInPath = sum(circlePath);
        else
            out.circlePath = 0;
            out.obsInPath = 0;
        end
        
        if height(out.obstacles) < 6
            % if we dont have enough obstacles add dummy values for cbf
            out.obstacles = [out.obstacles ; repmat([1000,1000,1],(6-height(out.obstacles)),1) ];
        end
    
        out.coverage = coverage;
        out.fig = fig;
        out.targetPos = targetPos;
        grid_size = grid_size-2;
    
    end
end

%% LOCAL FUNCTIONS

function isIntersecting = lineIntersectsCircle(A, B, circle)
% INPUTS:
%   A: [x1, y1] (start point)
%   B: [x2, y2] (end point)
%   circle: [x_center, y_center, radius]
% OUTPUT:
%   isIntersecting: true/false

    % Extract circle parameters
    C = circle(1:2);
    radius = circle(3);
    
    % Vector AB (line segment direction)
    AB = B - A;
    
    % Vector AC (from A to circle center)
    AC = C - A;
    
    % Project AC onto AB to find closest point
    t = dot(AC, AB) / dot(AB, AB);
    
    % Clamp t to [0, 1] to stay within the segment
    t_clamped = max(0, min(1, t));
    
    % Closest point on the line segment to the circle center
    D = A + t_clamped * AB;
    
    % Distance from circle center to closest point
    distance = norm(D - C);
    
    % Check if distance ≤ radius OR endpoints are inside the circle
    isIntersecting = (distance <= radius) || ...
                    (norm(A - C) <= radius) || ...
                    (norm(B - C) <= radius);
end