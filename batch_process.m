%% Windows Path
cd("C:\Users\14244039\OneDrive - Queen's University Belfast\Documents\MATLAB\cbfrl\cbfrl_2param\cbf_2parm_veh")
return

%% Linux Path
cd('/home/sm/matlab/cbfRL/nmpc_cbf_rl_2parm_veh')
return

%% Linux Path (OLD files)
cd('/home/sm/matlab/cbfRL/cbf_2parm_veh_old/')
return

%%
clearvars -except alldata testList

%% Add functions path
addpath("./functions/");


%% Read in all simulated data and get reward from each run
addpath("./functions/");
rfWeights = [1 1 1]; % [path end-sep ave-vel]
[results, ~] = processResultsTable(alldata);
fprintf("\n\nResults Table Generated.\n\n\n");

clearvars rfWeights

%% Check test parameter ranges
k1s = unique(results.k1);
k2s = unique(results.k2);
rcbfs = unique(results.rcbf);

fprintf("min k1 : %.3f  | max k1 : %.2f\n", min(k1s),max(k1s));
fprintf("min k2 : %.3f  | max k2 : %.2f\n", min(k2s),max(k2s));
fprintf("rcbf : %.2f \n", rcbfs);

clearvars k1s k2s rcbfs



%% Plot Results
close all;
orads = unique(results.orad1);
%fig = figure();
obsBest = [];

rcbfs = unique(results.rcbf);
rcbfn = numel(rcbfs);
rcbf = rcbfs(1); 
fprintf("Displaying %.3f, from the set of %.0f rcbf values tested...\n",rcbf,rcbfn);

for i = 1:numel(orads)
    obs = orads(i);
    % if obs == 0 || obs == 6.0
    %     break    
    % end
    ftable = results(ismembertol(results.orad1, obs, 1e-5),:);    % Filter for different obstacle each loop
    ftable = ftable(ismembertol(ftable.rcbf, rcbf, 1e-5),:);         % Filter all loops for one rcbf
    [best figarray{i}] = plotResults(ftable,"on");                 % second arg hides all seperate obs plots, set "on" to show
    subtitle(sprintf("Obstacle Radius %.03f m",obs));
    % input("ENTER for next figure")
    %pause(1);
    obsBest = [obsBest ; [obs , best] ];
end

xobs = obsBest(:,1);
yk1  = obsBest(:,2);
yk2  = obsBest(:,3);
yk3  = obsBest(:,4); 

figure(1);
% Create tiled layout
t1 = tiledlayout(3, 1);
title(t1,sprintf('Best Reward | rcbf = %.03f',rcbf));
% Plot 1
nexttile;
scatter3(xobs, yk1, yk2);
subtitle(sprintf('Best Paramemters'));
xlabel("Obs(m)"); ylabel("k1"); zlabel("k2");
% xlim(cbfrange);
% ylim(cbfrange);
% zlim([0 1]);

% Plot 2
nexttile;
plot(xobs,yk1);
subtitle('Best K1');
xlabel("Obs(m)"); ylabel("k1");
%xlim(cbfrange); ylim(cbfrange); zlim([0 1]);

% Plot 3
nexttile;
plot(xobs,yk2);
subtitle('Best K2');
xlabel("Obs(m)"); ylabel("k2");
%xlim(cbfrange); ylim(cbfrange); zlim([0 1]);

% % Plot 4
% nexttile;
% plot(xobs,yk3);
% title('Best K3 (cbf radius)');
% xlabel("Obs(m)"); ylabel("k3");
% %xlim(cbfrange); ylim(cbfrange); zlim([0 1]);

% Plot obstacle figs
% figure(2);
% for i = 1:numel(figarray)
%     nexttile;
%     ax = copyobj(allchild(figarray{i}.CurrentAxes), gca);
%     title(sprintf("%0.3f m Obs",obsBest(i,1)));
% end

% input("ENTER to close all figures...");
% close all;
clearvars -except alldata obsBest fig* t* results*


%% Plot a single obstacle results
close all
ftable = results(ismembertol(results.obs_rad, 2.0, 1e-5),:);
plotResults(ftable)



%% Plot valid path distances
close all;
orads = unique(results.obs_rad);
cbfrange = [min(results.k1) max(results.k1)];

fig = figure();
for i = 1:numel(orads)
    obs = orads(i);                                                     % new obstacle radius each loop
    ftable = results(ismembertol(results.obs_rad, obs, 1e-5),:);        % filter table for obstacle
    avoid_ftable = ftable(ftable.minSep > 0 ,:);                        % filter obstacle table for non collisions
    %avoid_ftable = sortrows(avoid_ftable,"pathDist");                   

    x = avoid_ftable.k1;
    y = avoid_ftable.k2;
    z1 = 1 - normalize(avoid_ftable.pathDist, 'range');
    z2 = 1 - normalize(avoid_ftable.finishSep, "range");
    z3 = 1 - normalize(avoid_ftable.minSep,"range")'

    % Create tiled layout
    t = tiledlayout(2, 2);
    
    % Plot 1
    nexttile;
    scatter3(x, y, z1, 5, "blue", "filled");
    title('Path Distance');
    xlim(cbfrange);
    ylim(cbfrange);
    zlim([0 1]);
    
    % Plot 2
    nexttile;
    scatter3(x, y, z2, 5, "green", "filled");
    title('Target Seperation');
    xlim(cbfrange);
    ylim(cbfrange);
    zlim([0 1]);

    % Plot 3
    nexttile;
    scatter3(x, y, z3, 5, "red", "filled");
    title('Obstacle Clearance');
    xlim(cbfrange);
    ylim(cbfrange);
    zlim([0 1]);


    title(t, "Reward Function Components")
    subtitle(t, sprintf("Obstacle Radius %.03f m \n1 is better",obs))
    % input("ENTER for next figure")
    pause(3);
    clf(fig);
end











%% Plot Paths
close all;
fig = figure(Visible="on");

for i = 1:size(alldata,1)
    plotPath(fig, alldata(i), 4.0);
    pause(0.1);
end



%% Plot single from alldata

% i = 300;
i = 200;
% i = 30000;
% i = 33333;
simdata = alldata(i);

close all; staticPlot= true; viewOnScreen = false;
fig = visualiseSimulation(simdata,staticPlot,viewOnScreen);
figure(fig);

%% Plot multiple from alldata to gif
staticPlot= true; viewOnScreen = false;
for i = 1:1:size(alldata,1)
    simdata = alldata(i);
    % close(fig); 
    fig = visualiseSimulation(simdata,staticPlot,viewOnScreen);
    exportgraphics(fig,"/home/sm/matlab/cbfRL/nmpc_cbf_rl_2parm_veh/outB1.gif", Append=true);
    close(fig);
    disp(i);

end

%% Plot multiple from alldata to gif, separate by obstacle size
staticPlot= true; viewOnScreen = false;
allobs = [alldata.obstacle];
allObs = unique(allobs(3,:));


for o = allObs
    outfile = sprintf("/home/sm/matlab/cbfRL/nmpc_cbf_rl_2parm_veh/parmSweep_obs%04.1fm.gif",o);

    for i = 1:1:size(alldata,1)
        simdata = alldata(i);
        if simdata.obstacle(3) - o > 0.1
            break
        end
        % close(fig); 
        fig = visualiseSimulation(simdata,staticPlot,viewOnScreen);
        exportgraphics(fig,outfile, Append=true);
        close(fig);
        if mod(i,10)==0
            disp(i);
        end
    end
    fprintf("Finished image for %04.1fm obstacle.\n",o);
end

%% Plot cbf vs reward, with each obsticle
close all
obsSet      = unique(results.orad1);
cbfk1Set    = unique(results.k1);
cbfk2Set    = unique(results.k2);

figs = [];

for ii = 1:numel(obsSet)
    fig = figure(Visible="off");
    hold on;
    obsTgt = obsSet(ii);
    
    for i = 1:numel(cbfk2Set)
        k2Tgt  = cbfk2Set(i);
        filterResults  = results( ismember(results.orad1,obsTgt) & ismember(results.k2,k2Tgt),:);
        x = filterResults.k1;
        y = filterResults.reward;
        lbl = num2str(k2Tgt);
        plot(x,y,LineWidth=2,DisplayName=lbl);
    end
    
    ylim([-1 1])
    title("CBF-Settings Rewards");
    subtitle(sprintf("Obstacle Radius %4.1f m",obsTgt))
    lg = legend();
    title(lg,"CBF{\alpha}")
    xlabel("CBF_{k1}");
    ylabel("Reward [-1 1]");
    hold off;
    figs = [figs ; fig];
end

clearvars cbfk1Set cbfk2Set fig i ii k2Tgt lbl lg obsSet obsTgt x y filterResults

for i = 1:numel(figs)
    figure(figs(i));
end

input("<ENTER> to clear figures");
close all;
%% Plot data per obstacle, optional filter by cbf k2 & N
close all
obsSet      = unique(results.orad1);
cbfk1Set    = unique(results.k1);
cbfk2Set    = unique(results.k2);
nSet        = unique(results.N);

obsTgt = obsSet(4);
k2Tgt  = cbfk2Set(2);
fprintf("Showing Paths For Obstacle Radius: %.2f m and CBF-k2: %d\n", obsTgt,k2Tgt)
figs = [];

filterResults  = results( ismember(results.orad1,obsTgt) & ismember(results.k2,k2Tgt),:);

for i = 1:size(filterResults,1)
    idx = filterResults.allIdx(i);
    simdata = alldata(idx); staticPlot = true; viewOnScreen = false;
    fig = visualiseSimulation(simdata,staticPlot,viewOnScreen);
    figs = [figs ; fig];
    fprintf("%d ",i);
end
fprintf("\n");

for f = 1:numel(figs)
    figure(figs(f));
    pause(1);
    % close all;
end
close all

clearvars cbfk1Set cbfk2Set fig i ii k2Tgt lbl lg obsSet obsTgt x y idx staticPlot viewOnScreen simdata




%%
%%
%%
%%




%% LOCAL FUNCTIONS

function [results, resultsObs] = processResultsTable(alldata)
    results = [];
    for i = 1:size(alldata,1)
        cbf = alldata(i).cbf;
        obs1 = alldata(i).obstacle(3,1);
        N = alldata(i).N;
        % obs2 = alldata(i).obstacle(3,2);
        obs2 = obs1;
        rewardout = getReward(alldata(i));
        reward = rewardout.reward;
        dist = rewardout.pathDist;
        opdst = rewardout.optDist;
        fsep = rewardout.endSep;
        msep = rewardout.min_sep;
        aveVel = rewardout.aveVel;
        maxVel = rewardout.maxVel;
        stime = rewardout.simtime;

        results = [results ; array2table(   [cbf',      obs1,   obs2,       reward,     dist,       opdst,        fsep,       msep      aveVel,     maxVel,     stime,     i, N], ...
                            "VariableNames",["k1","k2", "orad1","orad2",    "reward",   "pathDist", "optimalDist","finishSep","minSep", "aveVel",   "maxVel",   "simTime", "allIdx", "N"])];
        if mod(i,1000)==0
            disp(i)
        end
    end
    %results = sortrows(results,"obs_rad");
    
    % Split Results into cell array of tables (1 table per obstacle)
    close all;
    % orads = unique(results.obs_rad);
    % for i = 1:numel(orads)
    %     resultsObs{i,1} = results(ismembertol(results.obs_rad, orads(i), 1e-5),:);
    %     resultsObs{i,2} = orads(i);
    % end
    resultsObs = [];
end



function plotPath(fig, simdata, orad)
% PLOTPATH Plot the path of a simulated run
%   result = plot on fig object of path from simdata
%   inputs:
%       fig - input figure object to draw the plot on
%       simdata - input simulation data 
%       orad - (Optional) Filter by obstacle radius
%   Example:
%       plotPath(fig,simdata,0.5)

    or = simdata.obstacle(3);
    if or < 0.01
        return
    end

    if nargin > 2
        if abs(or - orad) > 1e-3    % if our set radius matches sim radius
            return
        end

    end

    figure(fig);
    clf(fig)
    x = simdata.states(1,:);
    y = simdata.states(2,:);

    xt = simdata.target(1);
    yt = simdata.target(2);

    ox = simdata.obstacle(1);
    oy = simdata.obstacle(2);
    oc = [ox,oy];
   

    plot(x, y);
    hold on;
    scatter(xt, yt, 'MarkerEdgeColor', 'green', 'MarkerFaceColor', 'green', 'Marker', 'x');
    plotCircle(oc,or,'-','r');

    xlim([-5 , 25]);
    ylim([-5 , 25]);
    axis square;

    ttxt = sprintf('k1 = %.3f | k2 = %.3f | rcbf = %.3f',simdata.cbf(1),simdata.cbf(2),simdata.cbf(3) );
    title(ttxt);


    hold off
    
    %exportgraphics(gcf, './out.gif', 'ContentType', 'image', 'Append', true);

end


function [best, obsfig] = plotResults(ftable,show)
    
    if nargin < 2
        show = "off";
    end

    obs = unique(ftable.orad1);
    if numel(obs) > 1
        disp("Error - Need 1 obstacle in table only")
        return
    end
    x = ftable.k1;
    y = ftable.k2;
    z = ftable.reward;

    % [X, Y] = meshgrid(x, y);
    % Z = griddata(x, y, z, X, Y);
    % surf(x, y, Z, 'EdgeColor', 'none')
    % view(2)  % Set 2D view from above
    % colorbar  % Add a color scale
    % colormap('jet')  % Use the 'jet' colormap (you can change this)
    % clim([-1 1])  % Set color axis limits to match your z range
    
    bestreward = max(ftable.reward);
    bestresult = ftable(ismembertol(ftable.reward, bestreward, 1e-5),:);
    
    xb = bestresult.k1;
    yb = bestresult.k2;
    rb = bestresult.rcbf;
    zb = bestresult.reward;
    best = [ xb yb rb ];
    if numel(best) > 3
        best = best(1,:);
    end

    obsfig = figure(Visible=show);
    scatter3(x,y,z,10,"filled");
    hold on;
    scatter3(xb,yb,zb, 50, "filled", "r")

    xlabel('k1 Values');
    ylabel('k2 values');
    title(sprintf('%.03 m Obstacle Rewards',obs));

end