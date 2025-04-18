function out = generateCurriculumEnvironment(curriculum_level,genFig)
% GENERATERANDOMENVIRONMENT with curriculum support
%

    % check for figure generation flag, default to no figure generation
    if ~exist("genFig","var") || ~ islogical(genFig)
        genFig = false;
    end

    % ===== Curriculum Presets =====
    switch curriculum_level
        case 1 % Very-low (no obstacles)
            num_circles = 0;
            min_spacing = 10;
            grid_size = 30;
            targetPathObs = 0;
            % radii_set = [];
            
        case 2 % Low
            num_circles = 2;
            min_spacing = 8;
            grid_size = 40;
            targetPathObs = 1;
            
        case 3 % Intermediate
            num_circles = 4;
            min_spacing = 4;
            grid_size = 60;
            targetPathObs = 2;
            
        case 4 % Hard
            num_circles = 6;
            min_spacing = 2;
            grid_size = 50;
            targetPathObs = 3;
        
        case 5 % Very-hard
            num_circles = 6;
            min_spacing = 1.5;
            % max_spacing = 0.8;
            grid_size = 40;
            targetPathObs = 3;
    end

    targetPosNormal = rand(2,1);    % randomise target positions
    mpcReqObs = 6;                  % number of obstacles to return, number in area determined by cLevel
    veh_rad = 1.0;                  % enlarged vehicle radius to keep obstacles from start and finish points
    radii_set = 0.1:0.1:10;         % set of allowable obstacle radii
    out = struct;                   
    out.obsInPath = -1;             % count number of obstacles directly between start and target
    c5cnt = 0;                  % counter to indicate cirruculum 5 generation is taking too long, use far corner target if so
    
    while ~(out.obsInPath >= targetPathObs)

        targetPos = [grid_size, grid_size];                             % set target positions to far corner of grid 
        if c5cnt < 2000
            targetPos(randi(2)) = targetPosNormal(randi(2)) * grid_size;    % randomise one of the positions within the grid extents
        end
        circles = zeros(num_circles+2, 3);              % empty array for random circular obstacles
        circles(1,:) = [0 0 veh_rad];                   % add a temp circle for the vehicle at start pos
        circles(2,:) = [ targetPos(:).' , veh_rad ];    % add a temp circle for the vehicle at target pos
        current_count = 0;
        
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
        
        while height(out.obstacles) < mpcReqObs
            % fill "no-obstacle" slots with far away small obstacles, RL should learn to ignore
            out.obstacles(end+1,:) = [500 , 500 , 0.1 ];
        end

        if curriculum_level == 5 && coverage < 60
            out.obsInPath = -1;
            c5cnt = c5cnt + 1;
            % could have set of pre-created cLevel 5 maps to choose from if count exceeded
        end
    
    end

    % put output elements into struct
    out.coverage = coverage;
    out.targetPos = targetPos;
    out.mapLimits = getMapLimits(out.obstacles, curriculum_level,targetPos);
    out.mpcReqObs = mpcReqObs;
    out.cLevel = curriculum_level;

    % put optional figure object into output struct if requested
    if genFig
        fig = figure(Visible="off");
        ax = axes(fig);
        hold on;
        for i = 3:height(circles)
            plotCircle([circles(i,1), circles(i,2)],circles(i,3),'-','r');
        end
        scatter(targetPos(1),targetPos(2),100,"green",'x'); % mark target position
        ax.XLim = out.mapLimits.x;
        ax.YLim = out.mapLimits.y;
        out.fig = fig;
    end

end

%% LOCAL FUNCTIONS

function mapLimits = getMapLimits(obstacles,curriculum_level,target)
    if curriculum_level ~= 1
    
        xlims = zeros(height(obstacles),2);
        ylims = zeros(height(obstacles),2);
    
        for i = 1:height(xlims)
            if obstacles(i,1) > 120
                break
            end
            xlims(i,:) = [ (obstacles(i,1) - obstacles(i,3)) , (obstacles(i,1) + obstacles(i,3)) ];
            ylims(i,:) = [ (obstacles(i,2) - obstacles(i,3)) , (obstacles(i,2) + obstacles(i,3)) ];
        end
        mapLimits.x = [min(xlims(:,1)) max(xlims(:,2))];
        mapLimits.y = [min(ylims(:,1)) max(ylims(:,2))];
    else
        mapLimits.x = [ 0 50 ];
        mapLimits.y = [ 0 50 ];
    end
    
    % extend limits if they dont reach the target point on positive x/y axes
    if mapLimits.x(2) < target(1)
        mapLimits.x(2) = target(1);
    end
    if mapLimits.y(2) < target(2)
        mapLimits.y(2) = target(2);
    end

end


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