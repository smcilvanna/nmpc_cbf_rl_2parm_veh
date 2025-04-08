function nobs = normaliseObservation(obs)
    % NORMALISEOBERVATION Converts a observation into a ormalized observation.
    %
    % Description:
    %   This function takes an observation (obstacle radius in m) and converts it
    %   to a normalised range [-1 1].
    %
    % Inputs:
    %   obs - An observation (obstacle radius in m), between the range [obsMin, obsMax].
    %
    % Outputs:
    %   nobs - Normalised observation in the range [-1 1].
    
    % Set obstacle size range
    obsMin = 0.1; 
    obsMax = 10.0;
    
    % Normalise
    nobs = 2 * (obs - obsMin) ./ (obsMax - obsMin) - 1;
end