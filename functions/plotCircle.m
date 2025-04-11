function plotCircle(center, radius, lineStyle, color)
    % plotCircle Plots a circle on the current figure with specified line style and color
    %   plotCircle(center, radius, lineStyle, color) plots a circle with the specified center, radius,
    %   line style, and color.
    %   center - A 1x2 vector specifying the [x, y] coordinates of the circle's center.
    %   radius - A scalar specifying the radius of the circle.
    %   lineStyle - A character vector or string scalar specifying the line style (e.g., '-', '--').
    %   color - A character vector or string scalar specifying the color ('r', 'g', 'b', 'k', 'gray').

    % Validate inputs
    if length(center) ~= 2
        error('Center must be a 1x2 vector specifying [x, y] coordinates.');
    end
    if ~isscalar(radius) || radius <= 0
        error('Radius must be a positive scalar.');
    end
    validLineStyles = {'-', '--', ':', '-.'};
    if ~ismember(lineStyle, validLineStyles)
        error('Invalid line style. Choose from: ''-'', ''--'', '':'' or ''-.''.');
    end
    validColors = {'r', 'g', 'b', 'k', 'gray'};
    if ~ismember(color, validColors)
        error('Invalid color. Choose from: ''r'', ''g'', ''b'', ''k'', ''gray''.');
    end
    
    % Create the circle using the rectangle function
    rectangle('Position', [center - radius, 2*radius, 2*radius], 'Curvature', [1, 1], ...
              'EdgeColor', color, 'LineStyle', lineStyle);
    
    % Ensure the aspect ratio is equal to make the circle look like a circle
    % axis equal;
    % grid on; % Optional: Add some grid lines for better visualization
end