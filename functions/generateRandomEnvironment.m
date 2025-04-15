function out = generateRandomEnvironment(num_circles, min_spacing, grid_size, targetPos, radii_set)
% GENERATERANDOMENVIRONMENT Create 5 circles with position and radius constraints
% Inputs:
%   min_spacing - Minimum required distance between circle edges
%   radii_set   - (Optional) Discrete set of radii values (0.1-10 if not provided)
% Outputs:
%   circles     - 5x3 matrix [x, y, radius] for each circle
%   coverage    - Percentage of grid area covered by circles
    out = struct;
    veh_rad = 1.0;
    % grid_size = 50;
    % num_circles = 5;
    circles = zeros(num_circles+2, 3);
    circles(1,:) = [0 0 veh_rad];                   % add a temp circle for the vehicle at start pos
    circles(2,:) = [ targetPos(:).' , veh_rad ];    % add a temp circle for the vehicle at target pos
    current_count = 0;

    % Handle radius selection mode
    if nargin < 4 || isempty(radii_set)
        use_continuous = true;
    else
        use_continuous = false;
        validateattributes(radii_set, {'numeric'}, {'vector', '>=', 0.1, '<=', 10});
    end
    
    % Place circles with collision checking
    while current_count < num_circles
        % Generate candidate circle
        x = rand * grid_size;
        y = rand * grid_size;
        
        % Select radius
        if use_continuous
            radius = 0.1 + (10 - 0.1) * rand;
        else
            radius = randsample(radii_set, 1);
        end
        
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
    out.coverage = coverage;
    out.fig = fig;
    out.targetPos = targetPos;
end