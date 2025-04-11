% %% [RUN BY SECION ONLY]
% return; 
% null = 1;
%% Linux
cd /home/sm/matlab/cbfRL/nmpc_cbf_rl_2parm_veh
addpath("functions/");
addpath('/home/sm/matlab/com/casadi-3.6.7/');
clc; disp("Done")
%% Windows (Win 11 Laptop)
cd("C:\Users\14244039\OneDrive - Queen's University Belfast\win_11_qub\Documents\MATLAB\nmpc_cbf_rl_2parm_veh");
addpath("functions\");
addpath("C:\Users\14244039\AppData\Roaming\MathWorks\MATLAB Add-Ons\Collections\casadi-3.7.0-windows64-matlab2018b");
clc; disp("Done");
%% Create Solvers For variable N runs
import casadi.*
Nvals = 20;%10:5:110;
settings.DT = 0.1; 
settings.velMax = 2;
settings.accMax = 5;
settings.cbfParms = [1.0, 1.0];
settings.obs_rad = 1;
settings.veh_rad = 0.55;
solverStack = createSolvers(Nvals,settings);
fprintf("%d NMPC solvers created\n",numel(Nvals));
clearvars settings
%% Setup parameters for V4 sweep [with variance]
load("./temp_data/train_td3v2-2_results.mat","test");       % file with test output from agent validation (best parameters)
bestParms = array2table(test.results, "VariableNames",["obs","k1","kr","k2"]);   % obstacle, rl-k1, rl-kr, rl-k2
testParms = [bestParms(2:4:end,:)];
testParms.k1 = round(testParms.k1,2);
testParms.k2 = round(testParms.k2,2);
testParms.kr = round(testParms.kr,2);

testList = [];

k1_steps = 32; k1_step_ratio = 0.03125; 
kr_steps = 32; kr_step_ratio = 0.03125;
k1_mul = -(k1_steps/2):1:(k1_steps/2);
k1_mul = k1_mul*k1_step_ratio;
kr_mul = -(kr_steps/2):1:(kr_steps/2);
kr_mul = kr_mul*kr_step_ratio;

for in = 1:length(solverStack)
    for io = 1:height(testParms) % iterate over each obstacle
        for i1 = 1:numel(k1_mul)
            for ir = 1:numel(kr_mul)
                thisTest = testParms(io,:);
                thisTest.Nidx = in;
                thisTest.k1 = thisTest.k1*k1_mul(i1) + thisTest.k1;
                thisTest.kr = thisTest.kr*kr_mul(ir) + thisTest.kr;
                thisTest.k2 = thisTest.k1 / thisTest.kr;
                testList = [testList ; thisTest];
            end
        end
    end
end
fprintf("Test List created. Number of simulations to run : %d\n ", height(testList) );
clearvars test bestParms ans i1 in io ir k1_mul kr_mul k1_step* kr_step* thisTest testParms;

%% Setup parameters for V4 sweep [rl parameters only]
load("train_td3v2-2_results.mat","test");       % file with test output from agent validation (best parameters)
bestParms = array2table(test.results, "VariableNames",["obs","k1","kr","k2"]);   % obstacle, rl-k1, rl-kr, rl-k2
bestParms.kr = [];
testList = [];
for nidx = 1:height(solverStack)
    bestParms.Nidx(:) = nidx;
    testList = [ testList ; bestParms ];
end

% for i = 1:2
%     testList = [testList ; testList];
% end
clearvars i nidx Nvals test
%% >>>>>>>>>>>>>>>>>>>>>>> BATCH RUN V4 Dynamic Model With 2 CBF param and dynamic N (MPC horizon) <<<<<<<<<<<<<<<<<<<<<<<<<<

todaydate = string(datetime('today'), 'yyMMdd');
runname = "sweep_ecbf_correct_solver_1n";
outname = sprintf("./temp_data/%s_%s.mat",todaydate,runname);
input(sprintf("\n\nDid you change the output mat file name? \nSet as: %s\n\nENTER to begin simulations...\n\n",outname));

alldata = [];

for i = 1:size(testList,1)
    nmpc = solverStack{testList.Nidx(i)};
    solver = nmpc.solver;
    args = nmpc.args;
    f = nmpc.f;
    settings = nmpc.settings;
    settings.cbfParms = [ testList.k1(i) ; testList.k2(i) ];
    settings.obs_rad = testList.obs(i);
    % settings.N = nmpc.settings.N;
    % settings.DT = nmpcSolver.settings.DT;
    settings.endSepTol = 0.1;
    settings.maxSimTime = 150;
    % <<<< RUN SIMULATION >>>
    simdata = simulationLoopDyn(solver, args, f, settings);
    alldata = [alldata ; simdata];
    if mod(i,100)==0
        save(outname,"alldata", "testList");
        fprintf("Run %05d of %05d complete\n",i,size(testList,1))
    end
    disp(i);
end
fprintf("Run %05d of %05d complete\nDONE\n\n",i,size(testList,1))
save(outname,"alldata", "testList");



%%

%%
%%
%%


%% >>>>>>>>>>>>>>>>>>>>>>> BATCH RUN V3 Dynamic Model CBF Parameter Sweep <<<<<<<<<<<<<<<<<<<<<<<<<<
%{
____    ____  ____   
\   \  /   / |___ \  
 \   \/   /    __) | 
  \      /    |__ <  
   \    /     ___) |
    \__/     |____/  
%}


addpath("./functions/");
% firstrun = ~exist("solver","var") || ~exist("args","var") || ~exist("f","var");
firstrun = true;
if firstrun
    clc, close all; clearvars -except testListOld;
    addpath('/home/sm/matlab/com/casadi-3.6.7/');   % ### ADJUST PATH TO CASADI PACKAGE LOACTION ####          
    import casadi.*
    settings.DT = 0.1; 
    settings.velMax = 2;
    settings.accMax = 5;
    settings.cbfParms = [1.0, 1.0];
    settings.obs_rad = 1;
    settings.veh_rad = 0.55;
    settings.N = 20;
    [solver, args, f] = createMPCDynamicSolver(settings);
end
todaydate = datestr(datetime('today'), 'yymmdd');

runname = "sweep_ecbf_2parm_A4";
outname = sprintf("./%s_%s.mat",todaydate,runname);

% existList = false;
% if exist("testListOld","var")
%     fprintf("\n\nThere is an existing test list, will ignore existing tests...\n\nENTER to begin simulations...\n\n");
%     input("");
%     existList = true;
% end

% Create test list for simulations
%  ecbf_2parm_A4 (24th March) vmax = 2 accmax = 5 N = 20, no margin
cbf_k1      = 1:2:100;
cbf_k2      = cbf_k1;
obs         = [1.0 3.0 5.0 7.0 10.0 15.0 ]; 
testList    = combinations(cbf_k1, cbf_k2, obs);
clearvars cbf_k1 cbf_k2 obs
% User input to confirm start of batch run
input(sprintf("\n\nDid you change the output mat file name? \nSet as: %s\n\nENTER to begin %d simulations...\n\n",outname,size(testList,1)));
testList = sortrows(testList,"obs");
alldata = [];

for i = 1:size(testList,1)
    settings.cbfParms = [ testList.cbf_k1(i) ; testList.cbf_k2(i)];
    settings.obs_rad = testList.obs(i);
    simdata = simulationLoopDyn(solver,args,f, settings);
    alldata = [alldata ; simdata];
    if mod(i,100)==0
        save(outname,"alldata", "testList");
        fprintf("Run %05d of %05d complete\n",i,size(testList,1))
    end
    disp(i);
end
fprintf("Run %05d of %05d complete\nDONE\n\n",i,size(testList,1))
save(outname,"alldata", "testList");


%% >>>>>>>>>>>>>>>>>>>>>>> BATCH RUN V2 <<<<<<<<<<<<<<<<<<<<<<<<<<
addpath("./functions/");
firstrun = ~exist("solver","var") || ~exist("args","var") || ~exist("f","var");
if firstrun
    clc, close all; clearvars -except testListOld;
    addpath('/home/sm/matlab/com/casadi-3.6.7/');   % ### ADJUST PATH TO CASADI PACKAGE LOACTION ####          
    import casadi.*
    DT = 0.1; N = 15;
    velMax = 1;
    cbfParms = [-1, -1, -1]; % [gamma-obs1, gamma-obs2, margin]
    mpcParms = [-1              % Qx[x+y]       xy tracking     (REMOVE for point tracking)
                -1               % Qx[yaw]       yaw tracking    (REMOVE for point tracking)
                -1             % R[v]          linear control
                -1             % R[w]          angular control
                -1             % Q[x+y]        terminal xy
                -1             % Q[yaw]         terminal yaw
                ones(12,1)*-1];   %               spare unused parms
    obs_rad = -1;
    veh_rad = 0.55;
    % [obstacle, target] = setupObstacleScenario(obs_rad,veh_rad,[0,0,deg2rad(45)]);
    % obstacle = [1000 1000 1];
    [solver, args, f] = createMPCKinematicSolver(DT,N,velMax,1);
end
todaydate = datestr(datetime('today'), 'yymmdd');
outname = sprintf("./%s_sweep_parm5_A2.mat",todaydate);

fprintf("\n\nDid you change the output mat file name? \nSet as: %s\n\nENTER to begin simulations...\n\n",outname);
input("");
existList = false;
if exist("testListOld","var")
    fprintf("\n\nThere is an existing test list, will ignore existing tests...\n\nENTER to begin simulations...\n\n");
    input("");
    existList = true;
end

% Create test list for simulations
%parm5_A2 (13th Feb) vmax = 1 N = 15
k1 = [0.01 0.1:0.2:0.9 0.99];
k2 = k1;    %[ 0.1, 0.5,  1.0 : 1.0 : 150 ];
cbfd = [0.01];
obs = [1.0 3.0 5.0 7.0 10.0 ]; 
% Qxp = [0.1 1 10 100];
% Qxw = [0.1];
Rv = [0.1 1 10 100];
Rw = [0.01 0.1 1 10];
Qtx = [0.1 1 10 100];
Qtw = [0.01 0.1 1 10];

testList = combinations(k1,cbfd,obs,Rv,Rw,Qtx,Qtw);
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

    cbfParms = [ testList.k1(i); testList.k1(i) ; testList.cbfd(i)];
    obs_rad = testList.obs(i);
    mpcParms(3:6) = table2array(testList(i,4:7));
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

%  ecbf_2parm_A1 (20th March) vmax = 2 accmax = 5 N = 20, no margin
cbf_k1      = [1:2:30];  %[0.01 0.1:0.2:0.9 0.99];
cbf_k2      = cbf_k1;
obs         = [1.0 3.0 5.0 7.0 10.0 ]; 

%   parm2_B2 (18th March) vmax = 2 accmax = 5 N = 20, margin added to CBF within mpc
cbf_k       = [0.01 0.1:0.1:0.9 0.99];
cbf_alpha   = [ 0.1 1:2:10 10]; 
obs         = [1.0 3.0 5.0 7.0 10.0 ]; 
testList    = combinations(cbf_k,cbf_alpha,obs);

%   parm2_B1 (17th March) vmax = 2 accmax = 5 N = 20
cbf_k       = [0.01 0.1:0.1:0.9 0.99];
cbf_alpha   = [ 0.1 1:2:10 10]; 
obs         = [1.0 3.0 5.0 7.0 10.0 ]; 

%parm9_A1 (11th Feb) vmax = 1 N = 15
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
