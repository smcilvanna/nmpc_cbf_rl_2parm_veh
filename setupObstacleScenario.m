function [obs,tgt] = setupObstacleScenario(obs_rad,veh_rad,veh_start)
% setupObstacleScenario Set position of obstacle and goal point to maintain equal seperation across different sized obstacles
    vx = veh_start(1);
    vy = veh_start(2);
    veh_yaw = veh_start(3);
    approach_sep = 10;                                       % min distance between vehicle clearance radius and obstacle at start
    after_sep = 10;                                          % min distance between perimiter of obstacle and goal point
    v2oCen = veh_rad + approach_sep + obs_rad;              % centre to centre distance between vehicle start and obstacle
    obs = [ (vx + v2oCen*cos(veh_yaw)) ;  (vy + v2oCen*sin(veh_yaw))  ; obs_rad];
    obs(2) = obs(2) - 0.1;
    tgt = [ (obs(1) + obs_rad + after_sep*cos(veh_yaw)) , (obs(2) + obs_rad + after_sep*sin(veh_yaw)) , veh_yaw]';
end