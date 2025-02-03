%% Run for single parameter

firstrun = ~exist("solver","var") || ~exist("args","var") || ~exist("f","var");
if firstrun
    clc, close all, clear all
    addpath('/home/sm/matlab/com/casadi-3.6.7/');   % ### ADJUST PATH TO CASADI PACKAGE LOACTION ####          
    import casadi.*
    DT = 0.1; N = 20;
    velMax = 10;
    cbfParms = [0.1,1,1];
    obstacle = [5,5.01,0.5];
    [solver, args, f] = createMPCKinematicSolver(DT,N,velMax,cbfParms,obstacle);
end

cbfParms = [1 ; 15 ; 0.25];
obs_rad = 0.5;
simdata = simulationLoop(solver,args,f, cbfParms, obs_rad, N, DT);

% input("Press ENTER to continue to Plots..")

%% Plots
visualiseSimulation(simdata)
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