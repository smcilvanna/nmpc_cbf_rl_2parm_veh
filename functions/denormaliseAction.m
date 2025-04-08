function action = denormaliseAction(naction)
    % DENORMALISEACTION Converts a normalized action vector into a denormalized action vector.
    %
    % Syntax:
    %   action = denormaliseAction(naction)
    %
    % Description:
    %   This function takes a normalized action vector `naction` as input and 
    %   converts it into a denormalized action vector `action`.
    %
    % Inputs:
    %   naction - A 2-element vector containing normalized values in the range [-1, 1].
    %
    % Outputs:
    %   action - A 2-element vector containing denormalized values. The first 
    %            element corresponds to `k1` (range [1.0, 60]) and the second 
    %            element corresponds to `kr` (range [0.1, 0.8]).
    %
    % Example:
    %   naction = [-0.5, 0.5];
    %   action = denormaliseAction(naction);
    %   disp(action); % Output: [30.25; 0.45]
    
    % Define the range for normalized values
    origMin = -1; 
    origMax = 1;
    
    % Define the range for denormalized values
    k1Min = 1.0; 
    k1Max = 60;
    
    rMin = 0.1;  
    rMax = 0.8;

    % Perform linear interpolation to compute denormalized values
    k1 = ((naction(1) - origMin) / (origMax - origMin)) * (k1Max - k1Min) + k1Min;
    kr = ((naction(2) - origMin) / (origMax - origMin)) * ( rMax -  rMin) +  rMin;
    
    % Combine results into a single output vector
    action = [k1 ; kr];
end