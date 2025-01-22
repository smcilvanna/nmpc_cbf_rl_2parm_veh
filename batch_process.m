cd("C:\Users\14244039\OneDrive - Queen's University Belfast\Documents\MATLAB\cbfrl\cbfrl_2param\cbf_2parm_veh")
return

%%
load("250121_sweep6.mat")

%% Read in all simulated data and get reward from each run

results = [];

for i = 1:size(alldata,1)
    
    cbf = alldata(i).cbf;
    obs = alldata(i).obstacle(3);
    rewardout = getReward(alldata(i));
    reward = rewardout.reward;
    dist = rewardout.pathDist;
    opdst = rewardout.optDist;
    fsep = rewardout.finishSep;
    msep = rewardout.min_sep;

    results = [results ; array2table(   [cbf',            obs,      reward,  dist,      opdst,        fsep,       msep], ...
                        "VariableNames",["k1","k2","rcbf","obs_rad","reward","pathDist","optimalDist","finishSep","minSep"])];
    
    if mod(i,1000)==0
        disp(i)
    end

end
%results = sortrows(results,"obs_rad");

clearvars -except alldata results

%% Plot Results
close all;
orads = unique(results.obs_rad);
fig = figure();
for i = 1:numel(orads)
    obs = orads(i);
    % if obs == 0 || obs == 6.0
    %     break    
    % end
    ftable = results(ismembertol(results.obs_rad, obs, 1e-5),:);
    plotResults(ftable)
    subtitle(sprintf("Obstacle Radius %.03f m",obs))
    % input("ENTER for next figure")
    pause(1);
    clf(fig);
end



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
fig = figure(Visible="off");

for i = 1:size(alldata,1)
    plotPath(fig,alldata(i),0.5);
    pause(0.1);
end






%% LOCAL FUNCTIONS

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

    ttxt = sprintf('k1 = %.3f | k2 = %.3f',simdata.cbf(1),simdata.cbf(2));
    title(ttxt);


    hold off
    exportgraphics(gcf, './out.gif', 'ContentType', 'image', 'Append', true);

end


function plotResults(ftable)

    obs = unique(ftable.obs_rad);
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
    zb = bestresult.reward;



    scatter3(x,y,z,10,"filled");
    hold on;
    scatter3(xb,yb,zb, 50, "filled", "r")




    xlabel('k1 Values')
    ylabel('k2 values')
    title('Reward Heatmap')

end

