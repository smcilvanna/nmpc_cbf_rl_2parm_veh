function out = getObservations(simdata)
%GETNORMALISEDOBSERVATIONS Assemble the observations vector from simdata, return normalised observations
    
    % Observations = [ Vehicle State(7) ; Obstacle-Information(25)       ; MPC-Information(1) ; Target-Information(1) ]
    %                   Lin/Ang Vel (2)          dist/ang obs (18)[3*6]       prevCompTime(1)       %progress2goal(1)
    %                     prev vels (2)             obs radii (6)
    %               target dist/ang (3)        density curLvl (1)
    %
    % Observations                          Real Limits     Normalised Limits
    %  (1) Current Linear Velocity           [-2 2]          [-1 1]
    %  (2) Current Angular Velicity          [-1 1]          [-1 1]
    %  (3) Last Linear Velocity              [-1 1]          [-1 1]
    %  (4) Last Angular Velocity             [-2 2]          [-1 1]
    %  (5) Target Distance                   [ 0 100]        [ 0 1]
    %  (6) sin(target_angle)                 [-1 1]          [-1 1]
    %  (7) cos(target_angle)                 [-1 1]          [-1 1]
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %  (8) Obs1 distance                     [0 100]         [ 0 1]
    %  (9) Obs1 sin(angle)                   [-1 1]          [-1 1]
    % (10) Obs1 cos(angle)                   [-1 1]          [-1 1]
    % (11) Obs1 radius                       [0.01 10]        [ 0 1]
    % (12) Obs2 distance                     [0 100]         [ 0 1]
    % (13) Obs2 sin(angle)                   [-1 1]          [-1 1]
    % (14) Obs2 cos(angle)                   [-1 1]          [-1 1]
    % (15) Obs2 radius                       [0.01 10]        [ 0 1]
    % (16) Obs3 distance                     [0 100]         [ 0 1]
    % (17) Obs3 sin(angle)                   [-1 1]          [-1 1]
    % (18) Obs3 cos(angle)                   [-1 1]          [-1 1]
    % (19) Obs3 radius                       [0.01 10]        [ 0 1]
    % (20) Obs4 distance                     [0 100]         [ 0 1]
    % (21) Obs4 sin(angle)                   [-1 1]          [-1 1]
    % (22) Obs4 cos(angle)                   [-1 1]          [-1 1]
    % (23) Obs4 radius                       [0.01 10]        [ 0 1]
    % (24) Obs5 distance                     [0 100]         [ 0 1]
    % (25) Obs5 sin(angle)                   [-1 1]          [-1 1]
    % (26) Obs5 cos(angle)                   [-1 1]          [-1 1]
    % (27) Obs5 radius                       [0.01 10]        [ 0 1]
    % (28) Obs6 distance                     [0 100]         [ 0 1]
    % (29) Obs6 sin(angle)                   [-1 1]          [-1 1]
    % (30) Obs6 cos(angle)                   [-1 1]          [-1 1]
    % (31) Obs6 radius                       [0.01 10]        [ 0 1]
    % (32) Curriculum Level                  [1 5]           [0 1]
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % (33) Previous MPC Time (ms)            [0 200]         [0 1]
    % (34) % Progress to goal                [0 1 ]          [0 1]
    vrad = simdata.vrad;
    vehxy = simdata.end_current_state(1:2);
    obvs = zeros(34,1);
    obvs(1) = simdata.end_current_state(4);                             %  (1) Current Linear Velocity   
    obvs(2) = simdata.end_current_state(5);                             %  (2) Current Angular Velicity  
    obvs(3) = simdata.states(4,(simdata.mpcIter-simdata.numSteps +1));  %  (3) Last Linear Velocity      
    obvs(4) = simdata.states(5,(simdata.mpcIter-simdata.numSteps +1));  %  (4) Last Angular Velocity               
    obvs(5) = norm( vehxy' - simdata.target(1:2));                       %  (5) Target Distance                     
    obvs(6) = getTargetDirAngles(vehxy,simdata.target(1:2),"sin");      %  (6) sin(target_angle)                   
    obvs(7) = getTargetDirAngles(vehxy,simdata.target(1:2),"cos");      %  (7) cos(target_angle)                   
    for i = 1:height(simdata.obstacles)
        o = (i-1)*4;                            % Loop for obstacle observations
        obxy = simdata.obstacles(i,1:2);
        obvs(11+o)= simdata.obstacles(i,3);                             % (11) Obs radius
        obvs(8+o) = norm( vehxy - obxy ) - vrad - obvs(11+o);           %  (8) Obs distance                       
        obvs(9+o) = getTargetDirAngles(vehxy,obxy,"sin");               %  (9) Obs sin(angle)                     
        obvs(10+o)= getTargetDirAngles(vehxy,obxy,"cos");               % (10) Obs cos(angle)                                           
    end % (obs31) - for 6 obstacles                        
    obvs(32)= simdata.cLevel;                                           % (32) Curriculum Level                    
    obvs(33)= simdata.average_mpc_time*1000;                            % (33) Previous MPC Time (ms)   
    startTgtDist = norm(simdata.target(1:2));
    obvs(34)= (startTgtDist - obvs(5))/startTgtDist;                    % (34) Progress to goal                          
    obvs(34) = max(0,min(obvs(34),1)); % clamp value

    % normalise observations
    nobvs = zeros(size(obvs));            % init empty array               Real Limits     Normalised Limits
    nobvs(1) = Normalizer.normalize11(obvs(1),-2, 2 );                     % [-2 2]          [-1 1]
    nobvs(2) = Normalizer.normalize11(obvs(2),-1, 1 );                     % [-1 1]          [-1 1]
    nobvs(3) = Normalizer.normalize11(obvs(3),-1, 1 );                     % [-1 1]          [-1 1]     
    nobvs(4) = Normalizer.normalize11(obvs(4),-2, 2 );                     % [-2 2]          [-1 1]      
    nobvs(5) = Normalizer.normalize01(obvs(5), 0, 100 );                   % [ 0 100]        [ 0 1]      
    nobvs(6) = obvs(6);                                                    % [-1 1]          [-1 1]      
    nobvs(7) = obvs(7);                                                    % [-1 1]          [-1 1]
    for i = 1:height(simdata.obstacles)
        o = (i-1)*4;                            % Loop for obstacle observations
        nobvs(8+o) = Normalizer.normalize01(obvs(8+o),0,100);              % [0 100]         [ 0 1]
        nobvs(9+o) = obvs(9+o);                                            % [-1 1]          [-1 1]                           
        nobvs(10+o)= obvs(10+o);                                           % [-1 1]          [-1 1]                           
        nobvs(11+o)= Normalizer.normalize01(obvs(11+o),0.01,10);            % [0.01 10]        [ 0 1]                           
    end % (obs31) - for 6 obstacles                        
    nobvs(32)= Normalizer.normalize01(obvs(32),1,5);                       % [1 5]           [0 1]
    nobvs(33)= Normalizer.normalize01(obvs(33),0,200);                     % [0 200]         [0 1]
    nobvs(34)= obvs(34);                                                   % [0 1 ]          [0 1]

    out = nobvs;                     
end

%% LOCAL FUNCTIONS

function out = getTargetDirAngles(veh, tgt, ang)

    
    % Define the position vectors
    x1 = veh(1); y1 = veh(2);  % Position of the first vector
    x2 = tgt(1); y2 = tgt(2);  % Position of the second vector
    
    % Calculate the direction vector (delta_x, delta_y)
    delta_x = x2 - x1;
    delta_y = y2 - y1;
    
    % Calculate the magnitude of the direction vector
    r = sqrt(delta_x^2 + delta_y^2) + eps;
    
    % Calculate sin(theta) and cos(theta)
    sin_theta = delta_y / r;
    cos_theta = delta_x / r;

    if ang == "sin"
        out = sin_theta;
    elseif ang == "cos"
        out = cos_theta;
    else
        error("Angle arg is either sin or cos");
    end

end