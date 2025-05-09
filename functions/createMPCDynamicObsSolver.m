function nmpcSolver = createMPCDynamicObsSolver(settings)
%%% createMPCKinematicSolver - create the mpc optimisation objects 
    import casadi.*
    % cbfEnable = false;  % default to no CBF, will be set true if parameters are passed
    % if nargin ~= 5       % set default max linear velocity if not passed as arguement
    %     fprintf("\n\n[ERROR]: Error with input args to create MPC solver function.\n\n");
    %     return
    % end

    DT      = settings.DT;
    N       = settings.N;
    velMax  = settings.velMax;
    accMax  = settings.accMax;
    nObs    = settings.nObs;
    vrad    = settings.veh_rad;
    
    x = SX.sym('x');        % x world position  
    y = SX.sym('y');        % y world position
    yaw = SX.sym('yaw');    % heading
    v = SX.sym('v');        % linear velocity
    w = SX.sym('w');        % angular velocity
    states = [ x ; y ; yaw ; v ; w ]; 
    n_states = length(states);
    % n_pos_ref = length(states(1:3));

    a = SX.sym('a');            % linear acceleration
    alpha = SX.sym('alpha');    % angular acceleration
    controls = [a ; alpha];
    n_controls = length(controls);
    
    dyn = [ v*cos(yaw)  ;       % xdot = v*cos(yaw)
            v*sin(yaw)  ;       % ydot = v*sin(yaw)
            w           ;       % yawdot = w
            a           ;       % vdot = a
            alpha       ];      % wdot = alpha
    
    f = Function('f',{states,controls},{dyn}); % f is symbolic representation of the system dynamics

                    % target(5)   current(5)   cbf(2)   obstacle(3) 
    P = SX.sym('P', n_states    + n_states   + (2     + 3)*nObs    );  % Parameter vector, updated every call

    X = SX.sym('X', n_states, (N+1));       % 3xN+1 System states initial then N horizon steps
    T = SX.sym('T', n_controls, N);         % 3xN Decision variables (tau) for N horizon steps
    % S = SX.sym('s',N,1);                  % Slack variable
    
    J = 0;                                  % Empty Objective Function
    g = [];                                 % Empty Constraints Vector
    
    Q = diag([100 100 10 10 10]);          % Terminal state position error weight matrix
    R = diag([0.1 0.1]);                % Horizon steps control effort Weighing matrix
    Qstage = diag([10 10 1]);         % Horizon steps position error weighing matrix  
    
    st = X(:,1); % Initial State    {st:3x1}
    
    g = [g ; st - P(6:10)]; % Initial Condition Constraints {g:3x1}
    
    % horizon state constraints
    for k = 1:N
        st = X(:,k);    % vehicle states at each horizon step
        con = T(:,k);   % vehicle controls at each horizon step
    
        % Stage cost (state error and control effort)
        %       (veh_pos - target pos)       
        % J = J + (st(1:3)-P(1:3))'*(Qstage*dynCost(k))*(st(1:3)-P(1:3)) + ( st(4:5) - P(4:5) )'*Q(4:5,4:5)*( st(4:5) - P(4:5))  + con'*R*con; % calculate obj ##OLD 
        J = J + (st(1:3)-P(1:3))'*(Qstage)*(st(1:3)-P(1:3)) + con'*(R)*con; % calculate obj

        % System dynamics constraint (using RK4)
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
    
    % Terminal cost
    J = J + (X(:,N+1)-P(1:5))'*Q*(X(:,N+1)-P(1:5));
    % J = J + (X(:,n+1)-P(1:5))'*Q*(X(:,n+1)-P(1:5));

%>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>><<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    % >>>> Obstacle Avoidance Constraints (eCBF-V2)
    
    %Parameter P-vector structure % target(5)   current(5)  [o1-cbf(2)   obstacle1(3)] [o2-cbf(2) obstacle2(3)]  ...
    %                               P(1:5)      P(6:10)      P(11:12)    P(13:15)       P(16:17)  P(18:20)       ...
    
    if true
        cbf_margin = 0.00;
        for obs_idx = 1:nObs
            oo = 5*(obs_idx - 1);   % obstacle offset
            %cbf_parameters
            cbf_k1 = P(11+oo);     % CBF parameter (tunable)
            cbf_k2 = P(12+oo);     % CBF parameter (tunable)
            
            % Obstacle parameters
            obs_x   = P(13+oo);     % Obstacle x position
            obs_y   = P(14+oo);     % Obstacle y position
            obs_rad = P(15+oo);     % Obstacle radius
            
            for k = 1:N+1 % Iterate over prediction horizon
                if k ~= N+1
                    ka = k;
                else
                    ka = k-1;
                end
                % Calculate distance to obstacle
                vehicle_x = X(1, k);
                vehicle_y = X(2, k);
                vehicle_yaw = X(3,k);
                vehvel_x  = X(4,k)*cos(vehicle_yaw);
                vehvel_y  = X(4,k)*sin(vehicle_yaw);
                vehacc_x  = T(1,ka)*cos(vehicle_yaw);
                vehacc_y  = T(1,ka)*sin(vehicle_yaw);
                sx = vehicle_x - obs_x;
                sy = vehicle_y - obs_y;
                dist = sqrt((sx)^2 + (sy)^2);
                h = dist - obs_rad - vrad - cbf_margin; 
                lfh = 2*sx*vehvel_x + 2*sy*vehvel_y;
                l2fh = 2*sx*vehacc_x + 2*vehvel_x^2 + 2*sy*vehacc_y + 2*vehvel_y^2;
                ecbf = l2fh + cbf_k1*lfh + cbf_k2*h; % >= 0
                g = [g ; ecbf ]; 
            end
        end
    end

    OPT_variables = [ reshape(X, n_states*(N+1), 1) ; reshape(T, n_controls*N, 1) ];
    
    nlp_prob = struct('f', J, 'x', OPT_variables, 'g', g, 'p', P);
    
    opts = struct;
    opts.ipopt.max_iter = 2000;
    opts.ipopt.print_level =0;  %0,3
    opts.print_time = 0;
    opts.ipopt.acceptable_tol =1e-8;
    opts.ipopt.acceptable_obj_change_tol = 1e-6;
    
    solver = nlpsol('solver', 'ipopt', nlp_prob, opts);
    
    args = struct;
    
    % Equality constraings for dynamics
    args.lbg(1:n_states*(N+1)) = 0;  
    args.ubg(1:n_states*(N+1)) = 0;  

    % CBF constraints (h_dot >= -gamma(h))
    if nObs > 0
        args.lbg(end+1:end+nObs*(N+1)) = 0; % Lower bound for CBF constraints, must be >= zero for safe condition
        args.ubg(end+1:end+nObs*(N+1)) = inf; % No upper bound
    end

    % state limits
    args.lbx(1:n_states:n_states*(N+1),1) = -10; % x lower bound
    args.ubx(1:n_states:n_states*(N+1),1) = 100; % x upper bound
    args.lbx(2:n_states:n_states*(N+1),1) = -10; % y lower bound
    args.ubx(2:n_states:n_states*(N+1),1) = 100; % y upper bound
    args.lbx(3:n_states:n_states*(N+1),1) = -inf; % yaw lower bound
    args.ubx(3:n_states:n_states*(N+1),1) = inf;  % yaw upper bound
    args.lbx(4:n_states:n_states*(N+1),1) = -velMax;  % v lower bound
    args.ubx(4:n_states:n_states*(N+1),1) = velMax;  % v lower bound
    args.lbx(5:n_states:n_states*(N+1),1) = -1;  % w lower bound
    args.ubx(5:n_states:n_states*(N+1),1) = 1;  % w lower bound

    % linear velocity control limits
    args.lbx(n_states*(N+1)+1:n_controls:n_states*(N+1)+n_controls*N,1) = -accMax; % linear accel lower bound
    args.ubx(n_states*(N+1)+1:n_controls:n_states*(N+1)+n_controls*N,1) = accMax;  % linear accel upper bound
    % angular velocity control limits
    args.lbx(n_states*(N+1)+n_controls:n_controls:n_states*(N+1)+n_controls*N,1) = -1; % angular accel lower bound
    args.ubx(n_states*(N+1)+n_controls:n_controls:n_states*(N+1)+n_controls*N,1) = 1;  % angular accel upper bound

    nmpcSolver = struct;
    nmpcSolver.solver = solver;
    nmpcSolver.args = args;
    nmpcSolver.f = f;
    nmpcSolver.settings = settings;
end