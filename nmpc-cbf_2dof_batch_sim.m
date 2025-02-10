cd("C:\Users\14244039\OneDrive - Queen's University Belfast\Documents\MATLAB\cbfrl\cbfrl_2param\cbf_2parm_veh")
return
%%
cd("/home/sm/matlab/cbfRL/nmpc_cbf_rl_2parm_veh");
return
%% >>>>>>>>>>>>>>>>>>>>>>> BATCH RUN V2 <<<<<<<<<<<<<<<<<<<<<<<<<<
addpath("./functions/");
firstrun = ~exist("solver","var") || ~exist("args","var") || ~exist("f","var");
if firstrun
    clc, close all; clearvars -except testListOld;
    addpath('/home/sm/matlab/com/casadi-3.6.7/');   % ### ADJUST PATH TO CASADI PACKAGE LOACTION ####          
    import casadi.*
    DT = 0.1; N = 15;
    velMax = 1;
    cbfParms = [0.01, 0.01, 0.01];
    mpcParms = [10              % Qx[x+y]
                1               % Qx[yaw]
                0.1             % R[v]
                0.1             % R[w]
                100             % Q[x+y]
                1              % Q[yaw]
                zeros(12,1)];   % spare parms
    obs_rad = 5;
    veh_rad = 0.55;
    [obstacle, target] = setupObstacleScenario(obs_rad,veh_rad,[0,0,deg2rad(45)]);
    % obstacle = [1000 1000 1];
    [solver, args, f] = createMPCKinematicSolver(DT,N,velMax,2);
end
todaydate = datestr(datetime('today'), 'yymmdd');
outname = sprintf("./%s_sweep_parm9_A1.mat",todaydate);

fprintf("\n\nDid you change the output mat file name? \nSet as: %s\n\nENTER to begin simulations...\n\n",outname);
input("");
existList = false;
if exist("testListOld","var")
    fprintf("\n\nThere is an existing test list, will ignore existing tests...\n\nENTER to begin simulations...\n\n");
    input("");
    existList = true;
end

% Create test list for simulations
%F1 (6th Feb) vmax = 1 N = 15
k1 = [0.01 , 0.05 , 0.1 : 0.2 : 0.9, 0.99];
k2 = k1;    %[ 0.1, 0.5,  1.0 : 1.0 : 150 ];
cbfd = [0.01];
obs = [1.0 3.0 5.0 7.0 10.0 ]; 
Qxp = [0.1 1 10 100];
Qxw = [0.1];
Rv = [0.1 1 10];
Rw = [0.01 0.1 1];
Qtx = [10 100];
Qtw = [1 10 ];

testList = combinations(k1,k2,cbfd,obs,Qxp,Qxw,Rv,Rw,Qtx,Qtw);
testList = sortrows(testList,"obs");
alldata = [];
match_count = 0;
mpcParms = zeros(18,1);

for i = 1:size(testList,1)
    matchTest = false;
    
    % Check if test has been run already, and skip if it has
    % if existList
    %     for row = 1:size(testListOld,1)
    %         matchTest = testList.k1(i) == testListOld.k1(row) && ...
    %                     testList.k2(i) == testListOld.k2(row) && ...
    %                     testList.rcbf(i) == testListOld.rcbf(row) && ...
    %                     testList.obs(i) == testListOld.obs(row);
    % 
    %         if matchTest
    %             break
    %         end
    %     end
    % end
    % 
    % if matchTest
    %     match_count = match_count + 1;
    %     disp(fprintf("Test already run, match_count = %d",match_count));
    %     continue
    % end
    
    cbfParms = [ testList.k1(i); testList.k2(i) ; testList.rcbf(i)];
    obs_rad = testList.obs(i);
    mpcParms(1:6) = table2array(testList(i,5:10));
    simdata = simulationLoop(solver,args,f, cbfParms, obs_rad, N, DT, false, mpcParms);
    alldata = [alldata ; simdata];
    if mod(i,100)==0
        save(outname,"alldata", "testList");
        fprintf("Run %05d of %05d complete\n",i,size(testList,1))
    end

end
fprintf("Run %05d of %05d complete\nDONE\n\n",i,size(testList,1))
save(outname,"alldata", "testList");





%% Parameters Run History

%E3 (5th Feb) vmax = 10
k1 = [1:2:50 ];
k2 = [0.5 :0.5: 5.0];    %[ 0.1, 0.5,  1.0 : 1.0 : 150 ];
rcbf = [0];
obs = [10.0 ]; 

%E2 (5th Feb) : vmax=2
k1 = [0.001 0.01, 0.1, 0.5:0.5:10 ];
k2 = [0.1 0.5 1.0 1.5 2.0 3.0];    %[ 0.1, 0.5,  1.0 : 1.0 : 150 ];
rcbf = [0];
obs = [0.5 1.0 2.0 3.0 4.0 5.0 7.5 10.0 ]; 

%E1a (5th Feb)
k1 = [0.01, 0.1 0.5 :0.5: 5.0];
k2 = [0.5 1.0 1.5 2.0];    %[ 0.1, 0.5,  1.0 : 1.0 : 150 ];
rcbf = [0];
obs = [0.5 1.5 2.5 3.5 4.5 5.5 ]; 

%E1a (5th Feb)
k1 = [0.01, 0.1 0.5 :0.5: 5.0];
k2 = [0.5 1.0 1.5 2.0];    %[ 0.1, 0.5,  1.0 : 1.0 : 150 ];
rcbf = [0];
obs = [0.5 1.5 2.5 3.5 4.5 5.5 ]; 

%D1b (4th Feb) - % b = cbfk1*sepDist^cbfk2; | rcbf is sepdist safety margin
k1 = [2.1 :0.2: 5.0];
k2 = [0.5 1.0 1.5 2.0];    %[ 0.1, 0.5,  1.0 : 1.0 : 150 ];
rcbf = [0.01];
obs = [0.5 1.5 2.5 3.5 4.5 5.5 ]; 

%D1a (4th Feb) - % b = cbfk1*sepDist^cbfk2;
k1 = [0.1 :0.1: 2.0];
k2 = [0.5 1.0 1.5 2.0];    %[ 0.1, 0.5,  1.0 : 1.0 : 150 ];
rcbf = [0.01];
obs = [0.5 1.5 2.5 3.5 4.5 5.5 ];   

 


%%
%%
%%



%% >>>>>>>>>>>>>>>>>>>>>>> BATCH RUN V1 <<<<<<<<<<<<<<<<<<<<<<<<<<
addpath("./functions/");
firstrun = ~exist("solver","var") || ~exist("args","var") || ~exist("f","var");
if firstrun
    clc, close all; clearvars -except testListOld;
    addpath('/home/sm/matlab/com/casadi-3.6.7/');   % ### ADJUST PATH TO CASADI PACKAGE LOACTION ####          
    import casadi.*
    DT = 0.1; N = 20;
    [solver, args, f] = createMPCKinematicSolver(DT,N);
end
todaydate = datestr(datetime('today'), 'yymmdd');
outname = sprintf("./%s_sweep_parm3_C3c.mat",todaydate);

% >> >> >> >> >>  Test History << << << << << << 
%sweep_parm3_B* : Parameter sweep with original vehicle mpc settings
%C1 : VelMax = 10 (this might have been 15 or 12!), limited k1 range
%C2 : VelMax = 10, equal k1 k2 range
%C3a : VelMax = 10, equal k1 k2, k3 range also
%C4 : VelMax = 20, equal k1 k2 k3 range 
%C3b : same as a, more k1 k2 range, skip a tests 25.01.30
%C3 : combine a+b 25.01.30

fprintf("\n\nDid you change the output mat file name? \nSet as: %s\n\nENTER to begin simulations...\n\n",outname);
input("");
existList = false;
if exist("testListOld","var")
    fprintf("\n\nThere is an existing test list, will ignore existing tests...\n\nENTER to begin simulations...\n\n");
    input("");
    existList = true;
end


% % C3
% k1 = [ 0.1, 0.5, 1.0, 5.0 :5.0: 70 ];
% k2 = [ 0.1, 0.5, 1.0, 5.0 :5.0: 70 ];    %[ 0.1, 0.5,  1.0 : 1.0 : 150 ];
% rcbf = [0.5, 1.5, 2.5 ];
% obs = [0.5 1.5 2.5 3.5 4.5 5.5 ];       % obs = [ 0.5, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 ];

% % C4
% k1 = [ 0.1, 0.5, 1.0, 5.0 :5.0: 140 ];
% k2 = [ 0.1, 0.5, 1.0, 5.0 :5.0: 140 ];    %[ 0.1, 0.5,  1.0 : 1.0 : 150 ];
% rcbf = [0.5, 1.5, 2.5];
% obs = [0.5 1.5 2.5 3.5 4.5 5.5 ];       % obs = [ 0.5, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 ];

%C3b
% k1 = [ 0.001, 0.01, 0.1, 0.5, 1.0, 5.0 :5.0: 70, 80 :10: 300 ];
% k2 = [ 0.001, 0.01, 0.1, 0.5, 1.0, 5.0 :5.0: 70, 80 :10: 300 ];    %[ 0.1, 0.5,  1.0 : 1.0 : 150 ];
% rcbf = [0.5, 1.5, 2.5 ];
% obs = [0.5 1.5 2.5 3.5 4.5 5.5 ];       % obs = [ 0.5, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 ];

%C3c
k1 = [ 0.001, 0.01, 0.1, 0.5, 1.0, 7.5 :5.0: 70, 90 :10: 300 ];
k2 = [ 0.001, 0.01, 0.1, 0.5, 1.0, 7.5 :5.0: 70, 90 :10: 300 ];    %[ 0.1, 0.5,  1.0 : 1.0 : 150 ];
rcbf = [0.5, 1.5, 2.5 ];
obs = [0.5 1.5 2.5 3.5 4.5 5.5 ];   

testList = combinations(k1,k2,rcbf,obs);
testList = sortrows(testList,"obs");
alldata = [];
match_count = 0;

for i = 1:size(testList,1)
    matchTest = false;
    
    % Check if test has been run already, and skip if it has
    if existList
        for row = 1:size(testListOld,1)
            matchTest = testList.k1(i) == testListOld.k1(row) && ...
                        testList.k2(i) == testListOld.k2(row) && ...
                        testList.rcbf(i) == testListOld.rcbf(row) && ...
                        testList.obs(i) == testListOld.obs(row);

            if matchTest
                break
            end
        end
    end

    if matchTest
        match_count = match_count + 1;
        disp(fprintf("Test already run, match_count = %d",match_count));
        continue
    end

    cbfParms = [ testList.k1(i); testList.k2(i) ; testList.rcbf(i)];
    obs_rad = testList.obs(i);
    simdata = simulationLoop(solver,args,f, cbfParms, obs_rad, N, DT);
    alldata = [alldata ; simdata];
    if mod(i,1000)==0
        save(outname,"alldata", "testList");
    end
    if mod(i,100)==0
        fprintf("Run %05d of %05d complete\n",i,size(testList,1))
    end
end
save(outname,"alldata", "testList");


%% LOCAL FUNCTIONS

%% PLOT FUNCTIONS
%%
function plotCurrentState(vehicle, obstacle,time)
% plotCurrentState - Plots current state of vehicle and obstacle

    %Vehicle State
    yaw = vehicle(3);
    veh_radius = 0.55;
    vpos = [vehicle(1),vehicle(2)];
    yline = [50*cos(yaw), 50*sin(yaw)];
    
    % Obstacle State
    orad = obstacle(3);
    obs_x = obstacle(1);
    obs_y = obstacle(2);
    opos = [obs_x , obs_y];
    obs_bearing = atan2((obs_y - vpos(2)), (obs_x - vpos(1)));

    % Safe clearance parameters
    veh2obs_angle           = wrapToPi(yaw - obs_bearing);
    cen_sep                 = sqrt((obs_x - vpos(1))^2 + (obs_y - vpos(2))^2);
    dw = 1; %min(1/cen_sep,orad*2);
    clear_radius            = (orad + veh_radius)*dw;
    clear_obstacle_angle    = atan2(clear_radius,cen_sep);
    clear_rad_ang = wrapToPi(obs_bearing + deg2rad(90)*sign(veh2obs_angle));
    
    alpha = veh2obs_angle;
    beta  = clear_obstacle_angle;
    
    h = alpha^2 - beta^2;
    
    if h >= 0
        htxt = "SAFE";
    else
        htxt = "DANGER";
    end
    
    stxt = sprintf("h(eta) : %.04f  | %s  [%.03f]",h,htxt,time);
    fig = figure();
   
    plotCircle(vpos,veh_radius,':','k');  % Plot vehicle outline
    plotCircle(opos,orad,'-','r');  % Plot obstacle
    hold on
    
    % Obstacle vehicle center line
    plot([vpos(1), obs_x],[vpos(2),obs_y],'k:');
    
    % Plot vehicle heading line
    plot([vpos(1),yline(1)], [vpos(2),yline(2)], 'b:');
    
    % Plot vehicle yaw marker
    yaw_marker_len = veh_radius;
    dx = yaw_marker_len * cos(yaw);
    dy = yaw_marker_len * sin(yaw);
    quiver(vpos(1),vpos(2),dx,dy,0,'b',LineWidth=2,MaxHeadSize=1,DisplayName="Vehicle Yaw");
    
    % Plot clear rad marker
    dx = clear_radius * cos(clear_rad_ang);
    dy = clear_radius * sin(clear_rad_ang);
    quiver(obs_x,obs_y,dx,dy,0,'b',LineWidth=2,MaxHeadSize=1,DisplayName="Vehicle Yaw");
    
    subtitle(stxt);
    lim = 20;
    
    minxlim = min(vpos(1),obstacle(1)) - max(veh_radius,orad);
    maxxlim = max(vpos(1),obstacle(1)) + max(veh_radius,orad);
    minylim = min(vpos(2),obstacle(2)) - max(veh_radius,orad);
    maxylim = max(vpos(2),obstacle(2)) + max(veh_radius,orad);


    axis equal;
    xlim([minxlim maxxlim]);
    ylim([minylim maxylim]);

    input("Press ENTER to continue...")
    % pause(1)
    close all;
end

