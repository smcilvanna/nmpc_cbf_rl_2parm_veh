
addpath("/home/sm/matlab/cbfRL/nmpc_cbf_rl_2parm_veh/functions/")

%% Run for single parameter
clear;
firstrun = ~exist("solver","var") || ~exist("args","var") || ~exist("f","var");
if firstrun
    clc, close all, clear all
    addpath('/home/sm/matlab/com/casadi-3.6.7/');   % ### ADJUST PATH TO CASADI PACKAGE LOACTION ####          
    import casadi.*
    DT = 0.1; N = 20;
    velMax = 2;
    cbfParms = [1,1,0];
    obs_rad = 2;
    veh_rad = 0.55;
    [obstacle, target] = setupObstacleScenario(obs_rad,veh_rad,[0,0,deg2rad(45)]);
    % obstacle = [1000 1000 1];
    [solver, args, f] = createMPCKinematicSolver(DT,N,velMax);
end

% cbfParms = [1,2,1];
simdata = simulationLoop(solver,args,f, cbfParms, obs_rad, N, DT);

% input("Press ENTER to continue to Plots..")

% Plots
visualiseSimulation(simdata);
%%
fig2 = figure();
t = tiledlayout(3, 2);
nexttile
plot(simdata.usafe(:,1));
subtitle("Applied Control Longitudinal")
nexttile
plot(simdata.usafe(:,2))
subtitle("Applied Control Yaw")

nexttile
plot(simdata.ucbf(:,1));
subtitle("CBF-QP Action Longitudinal")
nexttile
plot(simdata.ucbf(:,2))
subtitle("CBF-QP Action Yaw")
hold off

nexttile
plot(simdata.umpc(:,1));
subtitle("MPC Action Longitudinal")
nexttile
plot(simdata.umpc(:,2))
subtitle("MPC Action Yaw")
hold off