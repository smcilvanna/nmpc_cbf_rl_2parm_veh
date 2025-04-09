function solverStack = createSolvers(Nvals,settings)
    import casadi.*
    if size(Nvals,1) ~= 1
        disp("Error Nvals must be row vector");
        return
    end
    solverStack = {};
    for i = Nvals
        settings.N = i;
        nmpcSolver = struct();
        nmpcSolver.settings = settings; 
        [nmpcSolver.solver, nmpcSolver.args, nmpcSolver.f] = createMPCDynamicSolver(settings);
        solverStack = [solverStack ; {nmpcSolver}];
    end
end
