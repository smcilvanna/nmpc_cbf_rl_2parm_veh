cd("C:\Users\14244039\OneDrive - Queen's University Belfast\Documents\MATLAB\cbfrl\cbfrl_2param\cbf_2parm_veh")
return
%% Run for single parameter

firstrun = ~exist("solver","var") || ~exist("args","var") || ~exist("f","var");
if firstrun
    clc, close all, clear all
    addpath('/home/sm/matlab/com/casadi-3.6.7/');   % ### ADJUST PATH TO CASADI PACKAGE LOACTION ####          
    import casadi.*
    DT = 0.1; N = 20;
    [solver, args, f] = createMPCKinematicSolver(DT,N);
end

cbfParms = [1 ; 15 ; 0.25];
obs_rad = 0.5;
simdata = simulationLoop(solver,args,f, cbfParms, obs_rad, N, DT);

% input("Press ENTER to continue to Plots..")

%% Plots

visualiseSimulation(simdata)
%%
fig2 = figure();
t = tiledlayout(3, 2);
nexttile
plot(simdata.usafe(:,1));
subtitle("Applied Control Longitudinal")
nexttile
plot(simdata.usafe(:,2))
subtitle("Applied Control Yaw")

nexttile
plot(simdata.ucbf(:,1));
subtitle("CBF-QP Action Longitudinal")
nexttile
plot(simdata.ucbf(:,2))
subtitle("CBF-QP Action Yaw")
hold off

nexttile
plot(simdata.umpc(:,1));
subtitle("MPC Action Longitudinal")
nexttile
plot(simdata.umpc(:,2))
subtitle("MPC Action Yaw")
hold off

%% BATCH RUN

firstrun = ~exist("solver","var") || ~exist("args","var") || ~exist("f","var");
if firstrun
    clc, close all, clear all
    addpath('/home/sm/matlab/com/casadi-3.6.7/');   % ### ADJUST PATH TO CASADI PACKAGE LOACTION ####          
    import casadi.*
    DT = 0.1; N = 20;
    [solver, args, f] = createMPCKinematicSolver(DT,N);
end
outname = "./251221_sweep9.mat";
input("\n\nDid you change output file name?\n\nENTER to begin ...");
% k1 = [ 0.01:0.05:3];
% k2 = k1;
k1 = [0.1, 1, 10];
k2 = [0.001, 0.01, 0.1, 0.5, 1, 5, 10, 25, 50 ];
rcbf = [0.5, 1, 2, 5];
obs = [0.1, 0.5, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 10.0];
testList = combinations(k1,k2,rcbf,obs);
alldata = [];

for i = 1:size(testList,1)
    cbfParms = [ testList.k1(i); testList.k2(i) ; testList.rcbf(i)];
    obs_rad = testList.obs(i);
    simdata = simulationLoop(solver,args,f, cbfParms, obs_rad, N, DT);
    alldata = [alldata ; simdata];
    if mod(i,1000)==0
        save(outname,"alldata", "testList");
    end
    if mod(i,100)==0
        fprintf("Run %05d of %05d complete\n",i,size(testList,1))
    end
end
save(outname,"alldata", "testList");


%% LOCAL FUNCTIONS

% CONTROL FUNCTIONS
%%
function [solver, args, f] = createMPCKinematicSolver(DT,N)
%%% createMPCKinematicSolver - create the mpc optimisation objects 
import casadi.*
    x = SX.sym('x');        
    y = SX.sym('y');        
    yaw = SX.sym('yaw');
    states = [ x ; y ; yaw ]; 
    n_states = length(states);
    n_pos_ref = length(states(1:3));
    v = SX.sym('v');    
    w = SX.sym('w');
    controls = [v ; w];
    n_controls = length(controls);
    
    dyn = [ v*cos(yaw)  ;
            v*sin(yaw)  ;
            w           ];  
    
    f = Function('f',{states,controls},{dyn}); % f is symbolic representation of the system dynamics
    
    T = SX.sym('T', n_controls, N);         % 3xN Decision variables (tau) for N horizon steps
    P = SX.sym('P', n_states + n_pos_ref);  % 9x1 Parameters [initial state ; target state]
    X = SX.sym('X', n_states, (N+1));       % 6xN+1 System states initial then N horizon steps
    % S = SX.sym('s',N,1);                    %% Slack variable
    
    J = 0;                                  % Empty Objective Function
    g = [];                                 % Empty Constraints Vector
    
    Qx = diag([1 1 1]);                   % Horizon steps position error weighing matrix
    % Qv = diag([10 10 1]);                   % Horizon steps velocity error Weighing matrix
    R = diag([1 1]);                      % Horizon steps control effort Weighing matrix
    Q = 10^3*diag([1 1 1]);                 % Terminal state position error weight matrix
    
    st = X(:,1); % Initial State
    
    g = [g;st-P(1:3)]; % Initial Condition Constraints
    
    for k = 1:N
        st = X(:,k);    % vehicle states at each horizon step
        con = T(:,k);   % vehicle controls at each horizon step
    
        % append objective function at each horizon step
        %        (veh_pos - target pos)       
        J = J + (st(1:3)-P(4:6))'*Qx*(st(1:3)-P(4:6)) + con'*R*con; % calculate obj
        st_next = X(:,k+1);
        k1 = f(st, con);                % compute rk4 coefficients
        k2 = f(st + DT/2*k1, con);      %
        k3 = f(st + DT/2*k2, con);      %
        k4 = f(st + DT*k3, con);        %
        st_next_RK4 = st + DT/6*(k1 +2*k2 +2*k3 +k4); 
        g = [ g ; st_next-st_next_RK4 ]; % compute constraints % new
        %f_value = f(st,con);
        %st_next_euler = st + (DT*f_value);
        %g = [g;st_next-st_next_euler]; % compute constraints    
    end
    
    J = J + (X(1:3,N+1)-P(4:6))'*Q*(X(1:3,N+1)-P(4:6)); % Terminal state constraint
    
    OPT_variables = [reshape(X,3*(N+1),1);reshape(T,2*N,1)];
    
    nlp_prob = struct('f', J, 'x', OPT_variables, 'g', g, 'p', P);
    
    opts = struct;
    opts.ipopt.max_iter = 2000;
    opts.ipopt.print_level =0;%0,3
    opts.print_time = 0;
    opts.ipopt.acceptable_tol =1e-8;
    opts.ipopt.acceptable_obj_change_tol = 1e-6;
    
    solver = nlpsol('solver', 'ipopt', nlp_prob, opts);
    
    args = struct;
    % constraint for dynamic model
    args.lbg(1:n_states*(N+1)) = 0;  % Equality constraints
    args.ubg(1:n_states*(N+1)) = 0;  % Equality constraints
    % state limits
    args.lbx(1:n_states:n_states*(N+1),1) = -10; %state x lower bound
    args.ubx(1:n_states:n_states*(N+1),1) = 30; %state x upper bound
    args.lbx(2:n_states:n_states*(N+1),1) = -10; %state y lower bound
    args.ubx(2:n_states:n_states*(N+1),1) = 30; %state y upper bound
    args.lbx(3:n_states:n_states*(N+1),1) = -inf; %state yaw lower bound
    args.ubx(3:n_states:n_states*(N+1),1) = inf; %state yaw upper bound
    % linear velocity control limits
    args.lbx(n_states*(N+1)+1:n_controls:n_states*(N+1)+n_controls*N,1) = -5; %Tx lower bound
    args.ubx(n_states*(N+1)+1:n_controls:n_states*(N+1)+n_controls*N,1) = 5; %Tx upper bound
    % angular velocity control limits
    args.lbx(n_states*(N+1)+n_controls:n_controls:n_states*(N+1)+n_controls*N,1) = -1; %Tyaw lower bound
    args.ubx(n_states*(N+1)+n_controls:n_controls:n_states*(N+1)+n_controls*N,1) = 1; %Tyaw upper bound

end

%%
function simdata = simulationLoop(solver,args,f, cbfParms, obs_rad, N, DT)
    veh_rad = 0.55;         % vehicle radius
    % Static Obstacle params`
    veh_start = [0, 0, deg2rad(45)]';
    [obstacle, goal] = setupObstacleScenario(obs_rad,veh_rad,veh_start);    % static obstacle definintion
    
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
    time_limit = 60;                                % Maximum simulation time (seconds)
    sim_time_history(1) = current_time;             % append this array at each step with sim time
    
    control_horizon = zeros(N,2);                   % Controls for N horizon steps
    X0 = repmat(current_state,1,N+1)';              % initialization of the states decision variables
    
    % Start Simulation Loop
    % main_loop = tic;
    while(norm((current_state(1:3)-target_state),2) > 0.1 && mpciter < time_limit / DT)
        pathTarget = target_state; %getNextTarget(current_state, target_state);
        args.p   = [current_state;pathTarget];                                         % p : parameter vector 9x1 [initial state ; target state]
        args.x0  = [reshape(X0',3*(N+1),1);reshape(control_horizon',2*N,1)];     % initial value of the optimization variables
        sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx, 'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);
        u = reshape(full(sol.x(3*(N+1)+1:end))',2,N)';                  % get controls only from the solution
    
        solution_history(:,1:3,mpciter+1) = reshape(full(sol.x(1:3*(N+1)))',3,N+1)';  % get solution TRAJECTORY
        u_mpc_history= [u_mpc_history ; u(1,:)];
        sim_time_history(mpciter+1) = current_time;
        
        % Simulate Time Step                                                    tstep, t_now,           x0,            u, f, obstacle, cbfParms, r_veh
        [current_time, current_state, control_horizon, u_qp, sep_safe] = simulateTimeStep(DT,    current_time,    current_state, u, f, obstacle, cbfParms, veh_rad );               % Apply the control and simulate the timestep
        
        state_history(:,mpciter+2) = current_state;
        u_cbf_history = [u_cbf_history ; u_qp'];
        u_safe_history = [u_safe_history ; control_horizon(1,:)];
        safe_sep_history = [safe_sep_history ; sep_safe];

        X0 = solution_history(:,:,mpciter+1);               % current state horizon     % = reshape(full(sol.x(1:6*(N+1)))',6,N+1)';   
        X0 = [ current_state' ;  X0(3:end,:) ; X0(end,:)];   % state horizon for next step, replace mpc current state with sim current state, end state appears on last two horizon steps
        mpciter;
        mpciter = mpciter + 1;

        if sep_safe < 0
            % disp("CRASH!!")
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
function [obs,tgt] = setupObstacleScenario(obs_rad,veh_rad,veh_start)
% setupObstacleScenario Set position of obstacle and goal point to maintain equal seperation across different sized obstacles
    vx = veh_start(1);
    vy = veh_start(2);
    veh_yaw = veh_start(3);
    approach_sep = 10;                                       % min distance between vehicle clearance radius and obstacle at start
    after_sep = 10;                                          % min distance between perimiter of obstacle and goal point
    v2oCen = veh_rad + approach_sep + obs_rad;              % centre to centre distance between vehicle start and obstacle
    obs = [ (vx + v2oCen*cos(veh_yaw)) ,  (vy + v2oCen*sin(veh_yaw))  , obs_rad];
    obs(2) = obs(2) - 0.1;
    tgt = [ (obs(1) + obs_rad + after_sep*cos(veh_yaw)) , (obs(2) + obs_rad + after_sep*sin(veh_yaw)) , veh_yaw]';
end

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
function [t_next, x0, u0, u_qp, sep_safe] = simulateTimeStep(tstep, t_now, x0, u, f, obstacle, cbfParms, r_veh)
    st = x0;                                
    eta = st(1:3);
    u_nom = u(1,:)'; 
    [u_safe, u_qp, sep_safe] = controlBarrierFunction(t_now, obstacle, u_nom, eta, cbfParms, r_veh, tstep)   ;

    cbfEnable = 1;
    if cbfEnable
        u_apply = u_safe;
    else
        u_apply = u_nom;
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


%% PLOT FUNCTIONS
%%
function plotCurrentState(vehicle, obstacle,time)
% plotCurrentState - Plots current state of vehicle and obstacle

    %Vehicle State
    yaw = vehicle(3);
    veh_radius = 0.55;
    vpos = [vehicle(1),vehicle(2)];
    yline = [50*cos(yaw), 50*sin(yaw)];
    
    % Obstacle State
    orad = obstacle(3);
    obs_x = obstacle(1);
    obs_y = obstacle(2);
    opos = [obs_x , obs_y];
    obs_bearing = atan2((obs_y - vpos(2)), (obs_x - vpos(1)));

    % Safe clearance parameters
    veh2obs_angle           = wrapToPi(yaw - obs_bearing);
    cen_sep                 = sqrt((obs_x - vpos(1))^2 + (obs_y - vpos(2))^2);
    dw = 1; %min(1/cen_sep,orad*2);
    clear_radius            = (orad + veh_radius)*dw;
    clear_obstacle_angle    = atan2(clear_radius,cen_sep);
    clear_rad_ang = wrapToPi(obs_bearing + deg2rad(90)*sign(veh2obs_angle));
    
    alpha = veh2obs_angle;
    beta  = clear_obstacle_angle;
    
    h = alpha^2 - beta^2;
    
    if h >= 0
        htxt = "SAFE";
    else
        htxt = "DANGER";
    end
    
    stxt = sprintf("h(eta) : %.04f  | %s  [%.03f]",h,htxt,time);
    fig = figure();
   
    plotCircle(vpos,veh_radius,':','k');  % Plot vehicle outline
    plotCircle(opos,orad,'-','r');  % Plot obstacle
    hold on
    
    % Obstacle vehicle center line
    plot([vpos(1), obs_x],[vpos(2),obs_y],'k:');
    
    % Plot vehicle heading line
    plot([vpos(1),yline(1)], [vpos(2),yline(2)], 'b:');
    
    % Plot vehicle yaw marker
    yaw_marker_len = veh_radius;
    dx = yaw_marker_len * cos(yaw);
    dy = yaw_marker_len * sin(yaw);
    quiver(vpos(1),vpos(2),dx,dy,0,'b',LineWidth=2,MaxHeadSize=1,DisplayName="Vehicle Yaw");
    
    % Plot clear rad marker
    dx = clear_radius * cos(clear_rad_ang);
    dy = clear_radius * sin(clear_rad_ang);
    quiver(obs_x,obs_y,dx,dy,0,'b',LineWidth=2,MaxHeadSize=1,DisplayName="Vehicle Yaw");
    
    subtitle(stxt);
    lim = 20;
    
    minxlim = min(vpos(1),obstacle(1)) - max(veh_radius,orad);
    maxxlim = max(vpos(1),obstacle(1)) + max(veh_radius,orad);
    minylim = min(vpos(2),obstacle(2)) - max(veh_radius,orad);
    maxylim = max(vpos(2),obstacle(2)) + max(veh_radius,orad);


    axis equal;
    xlim([minxlim maxxlim]);
    ylim([minylim maxylim]);

    input("Press ENTER to continue...")
    % pause(1)
    close all;
end

