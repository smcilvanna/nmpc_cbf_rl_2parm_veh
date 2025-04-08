function obs = denormaliseObservation(nobs)
    % DENORMALISEOBSERVATION Converts a normalized observation into a denormalized observation.
    %
    % Syntax:
    %   obs = denormaliseObservation(nobs)
    %
    % Description:
    %   This function takes a normalized observation 'nobs' as input and 
    %   converts it into a denormalized observation `obs`.
    %
    % Inputs:
    %   obs - A scalar containing normalized value in the range [-1, 1].
    %
    % Outputs:
    %   nobs - Denormalized value of obstacle (radius in m)

    origMin = -1; origMax = 1;
    obsMin = 0.1; obsMax = 10.0;
    obs = ((nobs - origMin) / (origMax - origMin)) * (obsMax - obsMin) + obsMin;
end