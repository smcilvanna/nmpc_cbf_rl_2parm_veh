function simdata = simulationStepDyn(nmpcSolver, settings)

    % check if normalisation settings have been passed
    if settings.normalisedActions
        % nActions = settings.cbfParms;
        k1s = Normalizer.denormalize01(settings.cbfParms(:,1),settings.realCbfMin(1),settings.realCbfMax(1));
        krs = Normalizer.denormalize01(settings.cbfParms(:,2),settings.realCbfMin(2),settings.realCbfMax(2));
        k2s = k1s ./ krs;
        cbfParms = [k1s , k2s ];
        simdata.cbf = settings.cbfParms;
        N = nmpcSolver.settings.N;
        simdata.N = N;
        simdata.Naction = settings.N;
    else
        cbfParms    = settings.cbfParms; 
        simdata.cbf = cbfParms;
        N = nmpcSolver.settings.N;
        simdata.N = N;
        simdata.Naction = N;
    end

    solver      = nmpcSolver.solver;
    args        = nmpcSolver.args;
    f           = nmpcSolver.f;
    
    DT          = nmpcSolver.settings.DT;
    loopSteps   = settings.loopSteps;
    veh_rad     = settings.veh_rad;    
    maxSimTime  = settings.maxSimTime;
    endSepTol   = settings.endSepTol;

    current_state   = settings.currentState(:);     % each step will pass in the start position
    obstacles       = settings.obstacles;       % static obstacle configuration in environment
    target_state    = settings.target(:);       % target pose vehicle is navigating to
    current_time    = settings.currentTime;     % time counter for current episode
    mpciter         = settings.mpcIter;         % MPC iteration counter for current episode
    ctrlHistory     = settings.ctrlHistory;     % history of control actions applied to system
    ssHistory       = settings.ssHistory;      % history of minimum obstacle seperation
    state_history   = settings.stateHistory;    % append this array at each step with current state
    simTimeHistory  = settings.simTimeHistory; % append this array at each step with sim time
    
    % control_horizon = zeros(N,2);             % Controls for N horizon steps
    control_horizon = settings.controlHorizon;  % Pass in from last step, or initalise as zeros(N,2)
    % X0 = repmat(current_state,1,N+1)';          % initialization of the states decision variables
    X0 = settings.X0;                           % Pass in from last step, or initalise as above comment

    targetReached       = false;
    obstacleCollision   = false;
    stepsComplete       = false;
    episodeTimeout      = false;

    % assemble Pvector elements for cbf/obstacles [o1-cbf(2)   obstacle1(3)]
    pObstacles = zeros(height(obstacles)*5,1);
    for i = 1:height(obstacles)
        o = (i-1)*5; % offset for each loop
        pObstacles(1+o) = cbfParms(i,1);  % k1 value
        pObstacles(2+o) = cbfParms(i,2);  % k2 value
        pObstacles(3+o) = obstacles(i,1); % obstacle x-position
        pObstacles(4+o) = obstacles(i,2); % obstacle y-position
        pObstacles(5+o) = obstacles(i,3); % obstacle radius
    end

    % Start Simulation Loop
    stepCount = 0;
    main_loop = tic;
    
    % run the loop until target is reached OR collision with obstacle OR completed set number of steps OR the episode time is exceeded
    while( ~targetReached && ~obstacleCollision && ~stepsComplete && ~episodeTimeout )
        mpciter  = mpciter + 1;       
        stepCount = stepCount + 1;
        args.p   = [ target_state ; current_state; pObstacles ];
        args.x0  = [reshape(X0',5*(N+1),1);reshape(control_horizon',2*N,1)];     % initial value of the optimization variables
        sol      = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx, 'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);
        u        = reshape(full(sol.x(5*(N+1)+1:end))',2,N)';                  % get controls only from the solution
    
        % solution_history(:,1:5,mpciter+1) = reshape(full(sol.x(1:5*(N+1)))',5,N+1)';  % get solution TRAJECTORY
        % u_mpc_history= [u_mpc_history ; u(1,:)];
        simTimeHistory(mpciter+1) = current_time;
        
        % Simulate Time Step                                                              tstep, t_now,           x0,            u, f, obstacle
        [current_time, current_state, control_horizon, ~, sep_safe] = simulateTimeStep(DT,    current_time,    current_state, u, f, obstacles);               % Apply the control and simulate the timestep
        
        state_history( : , mpciter+1) = current_state;
        % u_safe_history = [u_safe_history ; control_horizon(1,:)];
        ctrlHistory(mpciter,:) = control_horizon(1,:);
        % safe_sep_history = [safe_sep_history ; sep_safe];
        ssHistory(mpciter,:) = sep_safe;

        % X0 = solution_history(:,:,mpciter+1);               % current state horizon     % = reshape(full(sol.x(1:6*(N+1)))',6,N+1)';   
        X0 = reshape(full(sol.x(1:5*(N+1)))',5,N+1)';       % initalise state horizon for next step 
        X0 = [ current_state' ;  X0(3:end,:) ; X0(end,:)];  % state horizon for next step, replace mpc current state with sim current state, end state appears on last two horizon steps

        % Evaluate end of step conditions
        stepsComplete       = stepCount >= loopSteps;
        episodeTimeout      = mpciter*DT >= maxSimTime;
        obstacleCollision   = sep_safe <= 0.00; 
        targetReached       = norm((current_state(1:2) - target_state(1:2)),2) <= endSepTol;
    
    end
    
    main_loop_time = toc(main_loop);
    
    simdata.average_mpc_time = main_loop_time/stepCount;
    simdata.states = state_history;
    % simdata.ucbf = u_cbf_history;
    % simdata.umpc = u_mpc_history;
    simdata.usafe = ctrlHistory;
    % simdata.solutions = solution_history;
    simdata.obstacles = obstacles;
    simdata.vrad = veh_rad;
    simdata.target = target_state';
    simdata.dt = DT;
    simdata.sep = ssHistory;
    simdata.looptime = main_loop_time;
    simdata.mpcIter = mpciter;
    simdata.endAtTarget = targetReached;
    simdata.endHitObs = obstacleCollision;
    simdata.endStepsDone = stepsComplete;
    simdata.endEpTimeout = episodeTimeout;
    simdata.end_control_horizon = control_horizon;
    simdata.end_X0 = X0;
    simdata.end_current_state = current_state;
    simdata.end_current_time = current_time;
    simdata.maxEpTime = settings.maxSimTime;
    simdata.maxVel = nmpcSolver.settings.velMax;
    simdata.maxAcc = nmpcSolver.settings.accMax;
    simdata.numSteps = stepCount;
    simdata.cLevel = settings.cLevel;
end




%% LOCAL FUNCTIONS
function [t_next, x0, u0, u_qp, sep_safe] = simulateTimeStep(tstep, t_now, x0, u, f, obstacles)
    st = x0;                                
    u_nom = u(1,:)';
    u_apply = u_nom;    % otherwise use output from MPC only
    u_qp = 0;
    minSep = inf;
    for o = 1:height(obstacles)
        obspos = obstacles(o,1:2);
        vehpos = st(1:2);
        sep_safe = norm(obspos - vehpos') - obstacles(o,3) - 0.55;  % 0.55 is vehicle radius, change this to variable
        if sep_safe < minSep
            minSep = sep_safe;
        end    
    end
    sep_safe = minSep;
    st = st + (tstep*f(st,u_apply));
    x0 = full(st);
    t_next = t_now + tstep;
    u0 = [ u_apply' ; u(3:size(u,1),:) ; u(size(u,1),:)];
end

