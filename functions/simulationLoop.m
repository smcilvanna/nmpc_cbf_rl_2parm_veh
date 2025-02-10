function simdata = simulationLoop(solver,args,f, cbfParms, obs_rad, N, DT, qpEnable, mpcParms)
    
    if ~exist("qpEnable","var")
        qpEnable = false;
    end

    if numel(cbfParms) ~= 3
        fprintf("Error, number of cbf parameters incorrect, expected 3 got %d\n", numel(cbfParms));
    else
        cbfParms = reshape(cbfParms,3,1);
    end

    if numel(obs_rad) ~= 1
        fprintf("Error, obstacle radius arg, expected 1 got %d\n", numel(obs_rad));
    end


    veh_rad = 0.55;         % vehicle radius
    % Static Obstacle params`
    veh_start = [0, 0, deg2rad(45)]';
    [obstacle, goal] = setupObstacleScenario(obs_rad,veh_rad,veh_start,true);    % static obstacle definintion
    
    current_time = 0;       % set initial time to zero
    mpciter = 0;            % MPC iteration counter
    solution_history = [];  % empty array for history of mpc solution for result plotting
    u_mpc_history = [];     % history of first horizon step control generated by mpc
    u_cbf_history = [];     % history of cbf action on controls
    u_safe_history = [];    % history of action applied to system
    safe_sep_history = [];
    current_state = veh_start;        % Set condition.
    target_state = goal;                            % Reference posture.
    state_history(:,1) = current_state;             % append this array at each step with current state
    time_limit = 30;                                % Maximum simulation time (seconds)
    sim_time_history(1) = current_time;             % append this array at each step with sim time
    
    control_horizon = zeros(N,2);                   % Controls for N horizon steps
    X0 = repmat(current_state,1,N+1)';              % initialization of the states decision variables

    obsParms = zeros(16,1);
    nObs = size(obstacle,2);
    obsParms(1) = nObs;
    obsParms(2:1+nObs*3) = reshape(obstacle,(nObs*3),1);

    % Start Simulation Loop
    % main_loop = tic;
    while(norm((current_state(1:3)-target_state),2) > 0.1 && mpciter < time_limit / DT)
        
                      % states(3)  target(3)    nObs     obstacles  RL-parms     
    %   P = SX.sym('P', n_states + n_pos_ref    +1     + 15         + 18      );  % 40x1 Parameter vector, updated every call
        
        args.p   = [current_state ; target_state ; obsParms ; mpcParms ];

        args.x0  = [reshape(X0',3*(N+1),1);reshape(control_horizon',2*N,1)];     % initial value of the optimization variables
        sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx, 'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);
        u = reshape(full(sol.x(3*(N+1)+1:end))',2,N)';                  % get controls only from the solution
    
        solution_history(:,1:3,mpciter+1) = reshape(full(sol.x(1:3*(N+1)))',3,N+1)';  % get solution TRAJECTORY
        u_mpc_history= [u_mpc_history ; u(1,:)];
        sim_time_history(mpciter+1) = current_time;
        
        % Simulate Time Step                                                    tstep, t_now,           x0,            u, f, obstacle, cbfParms, r_veh
        [current_time, current_state, control_horizon, u_qp, sep_safe] = simulateTimeStep(DT,    current_time,    current_state, u, f, obstacle, cbfParms, veh_rad, qpEnable );               % Apply the control and simulate the timestep
        
        state_history(:,mpciter+2) = current_state;
        u_cbf_history = [u_cbf_history ; u_qp'];
        u_safe_history = [u_safe_history ; control_horizon(1,:)];
        safe_sep_history = [safe_sep_history ; sep_safe];

        X0 = solution_history(:,:,mpciter+1);               % current state horizon     % = reshape(full(sol.x(1:6*(N+1)))',6,N+1)';   
        X0 = [ current_state' ;  X0(3:end,:) ; X0(end,:)];   % state horizon for next step, replace mpc current state with sim current state, end state appears on last two horizon steps
        mpciter;
        mpciter = mpciter + 1;

        if sep_safe < 0
            disp("CRASH!!")
            break
        end
    
        % if mod(current_time,0.2) == 0
        %     plotCurrentState(current_state, obstacle, current_time);
        %     continue;
        % end
        % if current_time > 2.2
        %     plotCurrentState(current_state, obstacle, current_time);
        % end
    
    end
    % main_loop_time = toc(main_loop);
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
  
    % disp(getReward(simdata));
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
function [t_next, x0, u0, u_qp, sep_safe] = simulateTimeStep(tstep, t_now, x0, u, f, obstacle, qpParms, r_veh, qpEnable)
    st = x0;                                
    u_nom = u(1,:)';

    if qpEnable
        [u_safe, u_qp, sep_safe] = controlBarrierFunction(t_now, obstacle, u_nom, st, qpParms, r_veh, tstep)   ;
        u_apply = u_safe;   % if qp-cbf is enabled, use output from qp
    else
        u_apply = u_nom;    % otherwise use output from MPC only
        u_qp = 0;
        sep_safe = norm(obstacle(1:2)'-st(1:2)) - obstacle(3) - r_veh;
    end

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