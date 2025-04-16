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
% (33) Previous MPC Time (ms)            [1 200]         [0 1]
% (34) % Progress to goal                [0 100]         [0 1]



end


%% LOCAL FUNCTIONS

function normalObs = normaliseObservations(observations)



end