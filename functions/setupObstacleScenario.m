function [obs,tgt] = setupObstacleScenario(obs_rad,veh_rad,veh_start,enGap)
% setupObstacleScenario Set position of obstacle and goal point to maintain equal seperation across different sized obstacles
    
    if ~exist("enGap","var")
        enGap = true;
    end

    vx = veh_start(1);
    vy = veh_start(2);
    veh_yaw = veh_start(3);
    approach_sep = 10;                                       % min distance between vehicle clearance radius and obstacle at start
    after_sep = 10;                                          % min distance between perimiter of obstacle and goal point
    v2oCen = veh_rad + approach_sep + obs_rad;              % centre to centre distance between vehicle start and obstacle
    obs = [ (vx + v2oCen*cos(veh_yaw)) ;  (vy + v2oCen*sin(veh_yaw))  ; obs_rad];
    obs(2) = obs(2) - 0.1;
    tgt = [ (obs(1) + obs_rad + after_sep*cos(veh_yaw)) , (obs(2) + obs_rad + after_sep*sin(veh_yaw)) , veh_yaw]';

    if enGap
        o2r = 10;
        o2h = 3*veh_rad + o2r + obs(3);
        o2x = obs(1) - cos(pi/4)*o2h;
        o2y = obs(2) + sin(pi/4)*o2h;
        obs = [obs , [o2x ; o2y ; o2r]];
    end
end