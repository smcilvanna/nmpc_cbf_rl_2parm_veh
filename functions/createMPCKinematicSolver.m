function [solver, args, f] = createMPCKinematicSolver(DT,N,velMax)
%%% createMPCKinematicSolver - create the mpc optimisation objects 
    import casadi.*
    if nargin < 3       % set default max linear velocity if not passed as arguement
        velMax = 10;
    end

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
    args.lbx(n_states*(N+1)+1:n_controls:n_states*(N+1)+n_controls*N,1) = -5; % Linear vel control lower bound
    args.ubx(n_states*(N+1)+1:n_controls:n_states*(N+1)+n_controls*N,1) = velMax; % Linear vel control upper bound
    % angular velocity control limits
    args.lbx(n_states*(N+1)+n_controls:n_controls:n_states*(N+1)+n_controls*N,1) = -1; %Tyaw lower bound
    args.ubx(n_states*(N+1)+n_controls:n_controls:n_states*(N+1)+n_controls*N,1) = 1; %Tyaw upper bound

end