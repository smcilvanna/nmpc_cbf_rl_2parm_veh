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

%%
clearvars -except agent test*

%%
test.obs = 1.0;
test.actionOut = getAction(agent,{test.obs});
disp(test.actionOut{1});

%% Validate TD3 agent Outputs VS Obstacle observation inputs
clc;
test.Observations = [0.5, 1.0, 5.0, 10.0]; % Edge cases
test.results = cell(length(test.Observations), 4);

for i = 1:length(test.Observations)
    test.obs = test.Observations(i);
    test.action = getAction(agent, {test.obs});
    
    % Store results
    test.results{i,1} = test.obs;
    test.results{i,2} = test.action{1}(1); % k1
    test.results{i,3} = test.action{1}(2); % k1/k2 ratio
    test.results{i,4} = test.results{i,2}/test.results{i,3};
    
    % Display
    fprintf('Obstacle Radius: %5.1f  →  k1=% .2f,  k2= %.2f,     k2_ratio= %.2f\n',...
            test.obs, test.results{i,2}, test.results{i,4}, test.results{i,3});
end

clearvars i testr

