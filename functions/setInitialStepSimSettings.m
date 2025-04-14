function simSettings = setInitialStepSimSettings(maxTime, loopSteps,nmpcSolver,env)

    simSettings.cbfParms = repmat([22 , 86],5,1);
    simSettings.N = nmpcSolver.settings.N;
    simSettings.DT = nmpcSolver.settings.DT;
    simSettings.veh_rad = nmpcSolver.settings.veh_rad;
    simSettings.loopSteps = loopSteps;
    simSettings.maxSimTime = maxTime;
    simSettings.maxEpSteps = simSettings.maxSimTime / simSettings.DT;
    simSettings.endSepTol = 0.1;
    simSettings.currentState = [0.0, 0.0, deg2rad(45), 0, 0]';
    simSettings.obstacles = env.obstacles;
    simSettings.target = [ env.targetPos , deg2rad(45) , 0, 0 ]';
    simSettings.currentTime = 0.00;
    simSettings.mpcIter = 0;
    simSettings.ctrlHistory = NaN(simSettings.maxEpSteps,2);
    simSettings.ssHistory = NaN(simSettings.maxEpSteps,1);
    simSettings.stateHistory = NaN(5,simSettings.maxEpSteps);
    simSettings.stateHistory(:,1) = simSettings.currentState';
    simSettings.simTimeHistory = zeros(simSettings.maxEpSteps,1);
    simSettings.controlHorizon = zeros(simSettings.N,2);
    simSettings.X0 = repmat(simSettings.currentState,1,simSettings.N+1)';
end