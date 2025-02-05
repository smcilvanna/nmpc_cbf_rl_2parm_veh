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