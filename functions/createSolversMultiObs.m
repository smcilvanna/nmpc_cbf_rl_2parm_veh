function solvers = createSolversMultiObs(settings)
    if ~isvector(settings.Nvals) % check there is a vector of nvals to create solvers from
        disp("settings.Nvals should be a 1-D vector");
        return
    end
    if numel(fieldnames(settings))==2 % if only nvals and num obstacles are passed use defaults below
        settings.DT = 0.1; 
        settings.velMax = 2;
        settings.accMax = 5;
        settings.cbfParms = [1.0, 1.0]; % temp values to create solver, replaced at runtime
        settings.veh_rad = 0.55;
    end

    import casadi.*
    solverStack = [];
    solverN = [];
    for i = settings.Nvals
        settings.N = i;
        nmpcSolver = createMPCDynamicObsSolver(settings);
        nmpcSolver.settings = settings;
        solverStack = [solverStack ; nmpcSolver];
        solverN = [solverN ; i ];
    end
    solvers.solverStack = solverStack;
    solvers.solverN = solverN;
end
