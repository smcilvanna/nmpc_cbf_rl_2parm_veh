% Define the grid
x = linspace(-10, 10, 100);
y = linspace(-10, 10, 100);
[X, Y] = meshgrid(x, y);

% Define obstacle
obs_x = 0;
obs_y = 0;
obs_radius = 2;

% CBF parameters
cbf_k1_range = [0.01 0.1 0.5:0.5:10];
cbf_k2_range = [0.1 0.5:0.5:5];
cbf_d = 0.01;

% Vehicle radius
veh_radius = 0.55;

% Create figure
fig = figure;
set(fig, 'Position', [100, 100, 1000, 400]);

% Create subplots
subplot1 = subplot(1,2,1);
subplot2 = subplot(1,2,2);

% Initialize plots
contourf(subplot1, X, Y, zeros(size(X)), 20);
colorbar(subplot1);
hold(subplot1, 'on');
viscircles(subplot1, [obs_x, obs_y], obs_radius, 'Color', 'r');
title(subplot1, 'CBF Constraint');
xlabel(subplot1, 'X');
ylabel(subplot1, 'Y');

contourf(subplot2, X, Y, zeros(size(X)), [0, 1]);
colorbar(subplot2);
hold(subplot2, 'on');

title(subplot2, 'Feasible Space');
xlabel(subplot2, 'X');
ylabel(subplot2, 'Y');

% Animation loop
for k1 = cbf_k1_range
    for k2 = cbf_k2_range
        % Initialize matrices to store results
        B = zeros(size(X));
        V = zeros(size(X));
        
        % Evaluate CBF and feasible volume at each point
        for i = 1:numel(X)
            % Calculate distance to obstacle
            dist = sqrt((X(i) - obs_x)^2 + (Y(i) - obs_y)^2) - obs_radius - veh_radius;
            
            % Evaluate CBF constraint
            % B(i) = k1 * (dist - cbf_d)^k2;
            B(i) = k1 * (1 - exp(-k2 * dist));
            % Calculate volume of feasible control space (simplified)
            if B(i) > 0
                % V(i) = 1; % Simplified: 1 if feasible, 0 if not
                V(i) = max(0, B(i));
            end
        end
        
        % Update plots
        contourf(subplot1, X, Y, B, 20);
        title(subplot1, sprintf('CBF Constraint (k1=%.2f, k2=%.2f)', k1, k2));
        
        contourf(subplot2, X, Y, V, [0, 1]);
        title(subplot2, sprintf('Feasible Space (k1=%.2f, k2=%.2f)', k1, k2));
        viscircles(subplot2, [obs_x, obs_y], obs_radius, 'Color', 'r');
        % Force MATLAB to draw the updated plot
        drawnow
        
        % Pause to control animation speed
        pause(0.1)
    end
end




%%
% Define the grid
x = linspace(-10, 10, 100);
y = linspace(-10, 10, 100);
[X, Y] = meshgrid(x, y);

% Define obstacle
obs_x = 0;
obs_y = 0;
obs_radius = 2;

% CBF parameters
cbf_k1 = 1;
cbf_k2 = 1;
cbf_d = 0.5;

% Vehicle radius
veh_radius = 0.5;

% Initialize matrices to store results
B = zeros(size(X));
V = zeros(size(X));

% Evaluate CBF and feasible volume at each point
for i = 1:numel(X)
    % Calculate distance to obstacle
    dist = sqrt((X(i) - obs_x)^2 + (Y(i) - obs_y)^2) - obs_radius - veh_radius;
    
    % Evaluate CBF constraint
    % B(i) = cbf_k1 * (dist - cbf_d)^cbf_k2;
    B(i) = cbf_k1 * (1 - exp(-cbf_k2 * dist));
    
    % Calculate volume of feasible control space (simplified)
    if B(i) > 0
        V(i) = 1; % Simplified: 1 if feasible, 0 if not
    end
end

% Visualize results
figure;

% Plot CBF constraint
subplot(1,2,1);
contourf(X, Y, B, 20);
colorbar;
hold on;
viscircles([obs_x, obs_y], obs_radius, 'Color', 'r');
title('CBF Constraint');
xlabel('X');
ylabel('Y');

% Plot feasible volume
subplot(1,2,2);
contourf(X, Y, V, [0, 1]);
colorbar;
hold on;
viscircles([obs_x, obs_y], obs_radius, 'Color', 'r');
title('Feasible Space');
xlabel('X');
ylabel('Y');

% Adjust figure properties
set(gcf, 'Position', [100, 100, 1000, 400]);