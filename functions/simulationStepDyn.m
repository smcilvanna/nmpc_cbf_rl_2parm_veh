function simdata = simulationStepDyn(solver,args,f, settings)


    if numel(cbfParms) ~= 2
        fprintf("Error, number of cbf parameters incorrect, expected 2 got %d\n", numel(cbfParms));
    else
        cbfParms = reshape(cbfParms,2,1);
    end

    % if numel(obs_rad) ~= 1
    %     fprintf("Error, obstacle radius arg, expected 1 got %d\n", numel(obs_rad));
    % end

    cbfParms    = settings.cbfParms; 
    N           = settings.N;
    DT          = settings.DT;
    loopSteps   = settings.loopSteps;
    veh_rad     = settings.veh_rad;    
    maxSimTime  = settings.maxSimTime;
    endSepTol   = settings.endSepTol;
    maxTsteps    = settings.maxTsteps;

    current_state   = settings.vehStart(:);     % each step will pass in the start position
    obstacles       = settings.obstacles;       % static obstacle configuration in environment
    target_state    = settings.target;          % target pose vehicle is navigating to
    current_time    = settings.currentTime;     % time counter for current episode
    mpciter         = settings.mpcIter;         % MPC iteration counter for current episode
    u_safe_history  = settings.ctrlHistory;     % history of control actions applied to system
    safe_sep_history = settings.ssHistory;      % history of minimum obstacle seperation
    state_history   = settings.stateHistory;    % append this array at each step with current state
    sim_time_history = settings.simTimeHistory; % append this array at each step with sim time
    
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
        sim_time_history(mpciter+1) = current_time;
        
        % Simulate Time Step                                                              tstep, t_now,           x0,            u, f, obstacle
        [current_time, current_state, control_horizon, ~, sep_safe] = simulateTimeStep(DT,    current_time,    current_state, u, f, obstacles);               % Apply the control and simulate the timestep
        
        state_history( : , mpciter+2) = current_state;
        % u_safe_history = [u_safe_history ; control_horizon(1,:)];
        u_safe_history(mpciter,:) = control_horizon(1,:);
        % safe_sep_history = [safe_sep_history ; sep_safe];
        safe_sep_history(mpciter,:) = sep_safe;

        % X0 = solution_history(:,:,mpciter+1);               % current state horizon     % = reshape(full(sol.x(1:6*(N+1)))',6,N+1)';   
        X0 = reshape(full(sol.x(1:5*(N+1)))',5,N+1)';       % initalise state horizon for next step 
        X0 = [ current_state' ;  X0(3:end,:) ; X0(end,:)];  % state horizon for next step, replace mpc current state with sim current state, end state appears on last two horizon steps

        % Evaluate end of step conditions
        stepsComplete       = stepCount == maxTsteps;
        episodeTimeout      = mpciter*DT >= maxSimTime;
        obstacleCollision   = sep_safe <= 0.00; 
    
    end
    
    
    
    main_loop_time = toc(main_loop);
    % ss_error = norm((current_state(1:2)-target_state(1:2)),2)
    % average_mpc_time = main_loop_time/(mpciter+1)
    simdata.states = state_history;
    simdata.ucbf = u_cbf_history;
    simdata.umpc = u_mpc_history;
    simdata.usafe = u_safe_history;
    simdata.solutions = solution_history;
    simdata.obstacle = obstacle;
    simdata.N = N;
    simdata.vrad = veh_rad;
    simdata.target = target_state';
    simdata.dt = DT;
    simdata.sep = safe_sep_history;
    simdata.cbf = cbfParms;
    % simdata.mpcParms = mpcParms;
    simdata.looptime = main_loop_time;

    simdata.end_control_horizon = control_horizon;
    simdata.end_X0 = X0;
end



%%


%%
function nextTarget = getNextTarget(current_state, target_state)
    target_ahead = 2.5;
    x = current_state(1);
    y = current_state(2);
    w = atan2(y,x);
    xg = target_state(1);
    yg = target_state(2);
    tw = atan2(yg,xg);
    q = -(w - tw);
    new_x = x*cos(q) - y*sin(q) + target_ahead*cos(tw);
    new_y = x*sin(q) + y*cos(q) + target_ahead*sin(tw);
    new_x = min(new_x,xg);
    new_y = min(new_y,yg);
    nextTarget = [new_x ; new_y ; target_state(3)];
end

%%
function [t_next, x0, u0, u_qp, sep_safe] = simulateTimeStep(tstep, t_now, x0, u, f, obstacle)
    st = x0;                                
    u_nom = u(1,:)';

    % if false
    %     [u_safe, u_qp, sep_safe] = controlBarrierFunction(t_now, obstacle, u_nom, st, qpParms, r_veh, tstep)   ;
    %     u_apply = u_safe;   % if qp-cbf is enabled, use output from qp
    % else
    u_apply = u_nom;    % otherwise use output from MPC only
    u_qp = 0;
    obspos = obstacle(1:2);
    vehpos = st(1:2);
    sep_safe = norm(obspos - vehpos) - obstacle(3) - 0.55;  % 0.55 is vehicle radius, change this to variable
    % end

    st = st + (tstep*f(st,u_apply));
    x0 = full(st);
    t_next = t_now + tstep;
    u0 = [ u_apply' ; u(3:size(u,1),:) ; u(size(u,1),:)];
end

%%
function [u_safe, u_qp, sep_safe] = controlBarrierFunction(t, obs, u_nom,eta, cbfParms, r_veh, tstep)
% controlBarrierFunction Compute safe outputs for each timestep based on 

    % System states from inputs
    % Variables used in ECBF
    k1          = cbfParms(1);
    k2          = cbfParms(2);

    % earth frame positions
    pos_x       = eta(1);
    pos_y       = eta(2);
    yaw         = eta(3);

    % Obstacle parameters
    r_obs       = obs(3);
    obs_x       = obs(1);
    obs_y       = obs(2);
    
    % Safe seperation distance parameters
    sep_x          = obs_x - pos_x;
    sep_y          = obs_y - pos_y;
    rs          = r_obs + r_veh + 0.00;
    DSEP        = [2*(sep_x*cos(yaw) + sep_y*sin(yaw)) , 0 ];
    
%   Linear Motion Control Barrier function terms
    h           = sep_x^2 + sep_y^2 - rs^2 ;
  % Lfh         = 2*Cx*v*cos(yaw) + 2*Cy*v*sin(yaw) ;   
%   ECBF        = Lfh(eta,u) + k1*h(eta) >= 0;

    % Safe clearance parameters
    obstacle_bearing        = atan2(sep_y,sep_x);
    veh2obs_angle           = wrapToPi(yaw - obstacle_bearing);
    sep_centres             = sqrt(sep_x^2 + sep_y^2);
    sep_safe                = sep_centres - r_obs - r_veh;
    r_cbf = cbfParms(3);
    d = 0.01*(1/r_obs);
    d2 = sep_safe - r_cbf;
    c = max(d,d2);
    dw = 1/c; %min(1/sep_centres,r_obs*2);
    clear_radius            = r_obs*dw + r_veh;
    clear_obstacle_angle    = atan2(clear_radius,sep_centres);
    
    hh      = veh2obs_angle^2 - clear_obstacle_angle^2;
  % Lfhh    = 2*veh2obs_angle * w ;
  % ECBF    = Lfh(eta,u) + k2*h(eta) >= 0 ;  
    ASEP    = [0 , 2*veh2obs_angle];
    
    % quadprog parameters
    H       = 2*[   1 0     ; 
                    0 1 ]   ;

    f       = zeros(size(H,1),1); 

    % Setup A & b matrices
    A       =  [DSEP(1) ,  0   ;
                0    , ASEP(2) ];
    
    b       = [ h*k1  + 2*(sep_x*cos(yaw) + sep_y*sin(yaw))*u_nom(1)  ;
                hh*k2   + 2*veh2obs_angle*u_nom(2)              ];

    % A = A(2,:);
    % b = b(2,:);
    Aeq     = [];    
    beq     = []; 

    u_qp   = [0;0];
    flag = 0;
    coder.extrinsic('quadprog');
    options = optimset('Display','off');
    [output, ~, flag]   = quadprog(H,f,A,b,Aeq,beq,[],[],[],options);

    if flag == 1
        u_qp = output;
    else
        disp("qp error");
    end

    if abs(u_qp(1)) > 1
        pause(0.001);
    end

    if t > 4.9
        pause(0.0001);
    end
    
    u_safe  = u_nom - u_qp; 
    u_safe(1) = max(u_safe(1),-5);
    u_safe(1) = min(u_safe(1), 5);
    u_safe(2) = max(u_safe(2),-1);
    u_safe(2) = min(u_safe(2), 1);

    % if u_safe(2) > 0
    %     disp("lateral control error");
    % end

end

function args = dynamicHorizon(Nmax,n,args)

    % ADJUST LOWER AND UPPER BOUNDS DYNAMIC/SAFETY CONSTRAINTS ARGS 
    % enable horizon dynamics for n steps
    % start1 = 6
    end1 = (5+5*n);
    args.lbg(6: end1 ) = 0;
    args.ubg(6: end1 ) = 0;
    % disable horizon dynamics for N-n steps
    start2 = end1 + 1;
    end2   = end1 + 5*(Nmax-n);
    args.lbg(start2:end2 ) = -inf;
    args.ubg(start2:end2 ) =  inf;

    % enable obstacle constraints for n+1 steps
    start3 = end2 + 1; % start position for obstacle constraints
    end3   = end2 + (n+1);
    args.lbg(start3:end3) = 0;
    args.ubg(start3:end3) = inf;
    % disable obstacle constraints for N-n steps
    start4 = end3 + 1;
    args.lbg(start4:end ) = -inf;
    args.ubg(start4:end ) =  inf;


    % ADJUST UPPER AND LOWER BOUNDS STATE/CONTROLS LIMIT ARGS
    velMax = 2; wMax = 1; accMaxL = 5; accMaxA = 1;
    lbons  = [-10,  -10,  -inf, -velMax, -wMax ]';
    lboffs = [-inf, -inf, -inf, -inf,    -inf  ]';
    ubons  = [100, 100, inf, velMax, wMax ]';
    uboffs = [inf, inf, inf, inf,    inf  ]';

    lbonc = [-accMaxL, -accMaxA]';
    lboffc = [-inf, -inf]';
    ubonc = [ accMaxL,  accMaxA]';
    uboffc = [ inf,  inf]';

    args.lbx = vertcat(  repmat(lbons,(n+1),1), repmat(lboffs,(Nmax-n),1), repmat(lbonc,n,1), repmat(lboffc,(Nmax-n),1) );
    args.ubx = vertcat(  repmat(ubons,(n+1),1), repmat(uboffs,(Nmax-n),1), repmat(ubonc,n,1), repmat(uboffc,(Nmax-n),1) );
end