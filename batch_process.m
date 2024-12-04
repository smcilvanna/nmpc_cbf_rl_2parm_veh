

%% Read in all simulated data and get reward from each run

results = [];

for i = 1:size(alldata,1)
    
    cbf = alldata(i).cbf;
    obs = alldata(i).obstacle(3);
    reward = getReward(alldata(i));
    results = [results ; array2table([cbf',obs,reward],"VariableNames",["k1","k2","rcbf","obs_rad","reward"])];
    
    if mod(i,100)==0
        disp(i)
    end

end
results = sortrows(results,"obs_rad");

clearvars -except alldata results

%% Plot Results

orads = unique(results.obs_rad);

for i = 1:numel(orads)
    obs = orads(i);


end


%%
ftable = results(ismembertol(results.obs_rad, 2.0, 1e-5),:);
plotResults(ftable)

%% LOCAL FUNCTIONS

function plotResults(ftable)

    obs = unique(ftable.obs_rad);
    if numel(obs) > 1
        disp("Error - Need 1 obstacle in table only")
        return
    end
    x = ftable.k1;
    y = ftable.k2;
    z = ftable.reward;

    [X, Y] = meshgrid(x, y);
    Z = griddata(x, y, z, X, Y);


    surf(x, y, Z, 'EdgeColor', 'none')
    view(2)  % Set 2D view from above
    colorbar  % Add a color scale
    colormap('jet')  % Use the 'jet' colormap (you can change this)
    clim([-1 1])  % Set color axis limits to match your z range
    xlabel('k1 Values')
    ylabel('k2 values')
    title('Reward Heatmap')

end
