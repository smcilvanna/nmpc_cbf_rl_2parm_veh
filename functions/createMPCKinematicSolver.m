function [solver, args, f] = createMPCKinematicSolver(DT,N,velMax,cbfParms,obstacle)
%%% createMPCKinematicSolver - create the mpc optimisation objects 
    import casadi.*
    cbfEnable = false;  % default to no CBF, will be set true if parameters are passed
    if nargin == 2       % set default max linear velocity if not passed as arguement
        fprintf("\n\nOnly 2 input args to create-solver-function provided.\nMax-vehicle-velocity set as 10.\nNO CBF!!\n\n");
        velMax = 10;
    end
    
    if exist("cbfParms","var")
        if numel(cbfParms) ~= 3     % check cbf parameters are correct
            fprintf("CBF parameters in arg4 not correct!");
            return
        else
            cbfEnable = true;
            cbfk1 = cbfParms(1);
            cbfk2 = cbfParms(2);
            cbf_d = cbfParms(3);

            if numel(obstacle) ~=3      % check the obstacle definition is correct
                fprintf("Obstacle definition in arg5 not correct!");
                return
            end
        end
    end


    vrad = 0.55;
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
    
    Qx = diag([10 10 1]);                   % Horizon steps position error weighing matrix
    % Qv = diag([10 10 1]);                   % Horizon steps velocity error Weighing matrix
    R = diag([0.1 0.1]);                      % Horizon steps control effort Weighing matrix
    Q = diag([100 100 10]);                 % Terminal state position error weight matrix
    
    st = X(:,1); % Initial State    {st:3x1}
    
    g = [g;st-P(1:3)]; % Initial Condition Constraints {g:3x1}
    
    % horizon state constraints
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
    
    % CBF constraints
    if cbfEnable
     opos = [obstacle(1);obstacle(2)];
     orad = obstacle(3);
     for k = 1:N            % if enabled, add cbf constraints        
        vpos = [X(1,k); X(2,k)];
        sepDist = norm(opos-vpos) - orad - vrad;                
        b = cbfk1*(cbf_d - sepDist)^cbfk2;
        g = [g ; b];
     end
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
    
    % append CBF constraints for N timesteps
    args.lbg(end+1:end+N) = 0;      % cbf constraint must be > 0
    args.ubg(end+1:end+N) = inf;    % cbf constraint no upper bound
    

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