return
%% 
disp("")

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

%% Cleanup Workspace
clearvars -except agent test*

%% Load agent from mat file to validate

[fileName, filePath] = uigetfile('*.mat', 'Select a MAT-file', './temp_data');
if isequal(fileName, 0)
    disp('File selection canceled.');   % Check if the user selected a file or canceled the operation
else
    load( fullfile(filePath, fileName) , 'agent', 'episodeInfo','verID','trainID'); % construct full path and load into workspace
    disp(['Loaded MAT-file: ', fullfile(filePath, fileName)]);
end
clearvars fileName filePath

%% Validate TD3 agent Outputs VS Obstacle observation inputs [ normalisation ]
clc;
test.ObservationsReal = (0.5:0.5:10);
test.Observations = normaliseObservation(test.ObservationsReal);
test.results = zeros(length(test.Observations), 4);

for i = 1:length(test.Observations)
    test.obs = test.Observations(i);
    test.actionN = getAction(agent, {test.obs});
    test.action = denormaliseAction(test.actionN{1});
    % Store results
    test.results(i,1) = test.ObservationsReal(i);
    test.results(i,2) = test.action(1); % k1
    test.results(i,3) = test.action(2); % k1/k2 ratio
    test.results(i,4) = test.results(i,2)/test.results(i,3);
    
    % Display
    fprintf('Obstacle Radius: %5.1f (%4.2f)  →  k1=% .2f,  k2= %.2f,     k2_ratio= %.2f\n',...
                               test.obs, ...
                                     test.Observations(i),... 
                                                 test.results(i,2),... 
                                                           test.results(i,4),... 
                                                                                    test.results(i,3));
end

clearvars i

%% Plot Obstacle Radius against RL predicted CBF Parameters
close all;
% Create figure and axis objects, not visible until tiled-layout at end
vis = 'off';
fig1 = figure(Visible=vis); ax1 = axes(fig1);
fig2 = figure(Visible=vis); ax2 = axes(fig2);
fig3 = figure(Visible=vis); ax3 = axes(fig3);

x   = test.ObservationsReal;
yk1 = test.results(:,2);
yk2 = test.results(:,4);
ykr = test.results(:,3);

scatter(ax1, x, yk1, 'LineWidth', 2, 'MarkerEdgeColor', 'blue',  'MarkerFaceColor', 'blue',  'Marker', 'o', 'SizeData', 25);
scatter(ax2, x, yk2, 'LineWidth', 2, 'MarkerEdgeColor', 'green', 'MarkerFaceColor', 'green', 'Marker', 'o', 'SizeData', 25);
scatter(ax3, x, ykr, 'LineWidth', 2, 'MarkerEdgeColor', 'red',   'MarkerFaceColor', 'red',   'Marker', 'o', 'SizeData', 36);

xlabel(ax1,'Obstacle Radius (m)');
ylabel(ax1,'RL k1');
title(ax1,'Predicted k1');

xlabel(ax2,'Obstacle Radius (m)');
ylabel(ax2,'RL k2');
title(ax2,'Best k2 vs obsTgt');

xlabel(ax3, 'Obstacle Radius (m)');
ylabel(ax3, 'RL k1/k2');
title(ax3, 'Best Ratio vs obsTgt');

% Create a tiled Layout for 3 plots
tiled_fig = figure(Visible="on");   % Create a new figure for the tiled layout
tl = tiledlayout(tiled_fig, 3, 1);

% Copy contents of fig1
t1 = nexttile(tl);
fig1_contents = get(ax1, 'Children');
copyobj(fig1_contents, t1);
% xlabel(t1, 'obsTgt');
ylabel(t1, 'k1');
title(t1, 'RL k1');
grid(t1, 'on');

% Copy contents of fig2
t2 = nexttile(tl);
fig2_contents = get(ax2, 'Children');
copyobj(fig2_contents, t2);
% xlabel(t2, 'obsTgt');
ylabel(t2, 'k2');
title(t2, 'RL k2');
grid(t2, 'on');

% Copy contents of fig3
t3 = nexttile(tl);
fig3_contents = get(ax3, 'Children');
copyobj(fig3_contents, t3);
xlabel(t3, 'Obstacle Radius (m)');
ylabel(t3, 'k1/k2');
title(t3, 'RL Ratio');
grid(t3, 'on');

tl.TileSpacing = 'compact';
tl.Padding = 'compact';
title(tl,"2 Parameter CBF-RL (TD3)",'FontWeight', 'bold', 'FontSize', 14)
% subtitle(tl,sprintf("Weights [%.2f %.2f %.2f]",weights(1),weights(2),weights(3) ) , 'FontSize', 10);
if exist("verID","var") && exist ("trainID","var") && exist("episodeInfo","var")
    subtitle(tl,sprintf("TD3%s-%s %d Episodes",verID,trainID,size(episodeInfo,1)));
end
clearvars ax* fig* t1 t2 t3 tl vis x yk* 

%%
%% Single Sample [ without manual normalisation ]
test.obs = 4.0;
test.actionOut = getAction(agent,{test.obs});
disp(test.actionOut{1});

%% Validate TD3 agent Outputs VS Obstacle observation inputs [ without manual normalisation ]
clc;
test.Observations = (0.5:0.5:10); % Edge cases
test.results = zeros(length(test.Observations), 4);

for i = 1:length(test.Observations)
    test.obs = test.Observations(i);
    test.action = getAction(agent, {test.obs});
    
    % Store results
    test.results(i,1) = test.obs;
    test.results(i,2) = test.action{1}(1); % k1
    test.results(i,3) = test.action{1}(2); % k1/k2 ratio
    test.results(i,4) = test.results(i,2)/test.results(i,3);
    
    % Display
    fprintf('Obstacle Radius: %5.1f  →  k1=% .2f,  k2= %.2f,     k2_ratio= %.2f\n',...
            test.obs, test.results(i,2), test.results(i,4), test.results(i,3));
end

clearvars i testr

%% Training Information (episodeInfo)
% episodeInfo(columns) = observation, reward, k1, k1/k2, k2
close all;
info.obs = denormaliseObservation(episodeInfo(:,1));
info.cbf = zeros(length(episodeInfo),2);

for i = 1:length(episodeInfo)
    naction = episodeInfo(i,3:4);
    info.cbf(i,:) = round(denormaliseAction(naction),2);
end

fig = figure; ax =axes(fig);
scatter3(ax, info.obs,info.cbf(:,1),info.cbf(:,2),2,"filled");
xlabel(ax,'Obstacle Radius (m)');
ylabel(ax,'k1');
zlabel(ax,'k1/k2');

clearvars action naction i 

%%

%%