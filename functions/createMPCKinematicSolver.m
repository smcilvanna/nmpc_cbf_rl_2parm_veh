function [solver, args, f] = createMPCKinematicSolver(DT,N,velMax,nObs)
%%% createMPCKinematicSolver - create the mpc optimisation objects 
    import casadi.*
    cbfEnable = false;  % default to no CBF, will be set true if parameters are passed
    if nargin ~= 4       % set default max linear velocity if not passed as arguement
        fprintf("\n\n[ERROR]: Error with input args to create MPC solver function.\n\n");
        return
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
    n_obs = 2;
    % obs_x = SX.sym('ox');
    % obs_y = SX.sym('oy');
    % obs_r = SX.sym('or');
    % obs = [obs_x ; obs_y ;obs_r];

    
    dyn = [ v*cos(yaw)  ;
            v*sin(yaw)  ;
            w           ];  
    
    f = Function('f',{states,controls},{dyn}); % f is symbolic representation of the system dynamics
    
    T = SX.sym('T', n_controls, N);             % 3xN Decision variables (tau) for N horizon steps
    
                   % states(3)  target(3)   nObs     obstacles    RL-parms     
    P = SX.sym('P', n_states + n_pos_ref    +1     + 5*n_obs   + 15      );  % 32x1 Parameter vector, updated every call
                   % P(1:3)     (4:6)       (7)     1(8:10)      (23:32) 
                                                  % 2(11:13)
                                                  % 3(14:16)
                                                  % 4(17:19)
                                                  % 5(20:22)

    X = SX.sym('X', n_states, (N+1));       % 3xN+1 System states initial then N horizon steps
    % S = SX.sym('s',N,1);                    % Slack variable
    
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
    if nObs > 0
        cbfk1 = P(13);
        cbfk2 = P(14);
        cbf_d = P(15);
        for obs = 0:(nObs-1)
            opos = [P(8+obs*3);P(9+obs*3)];
            orad = P(10+obs*3);


        %  for k = 1:N            % if enabled, add cbf constraints        
        %     vpos = [X(1,k); X(2,k)];
        %     sepDist1 = norm(opos1-vpos) - orad1 - vrad - cbf_d;
        %     sepDist2 = norm(opos2-vpos) - orad2 - vrad - cbf_d;
        %     % b = cbfk1*(cbf_d - sepDist)^cbfk2;
        %     % b = cbfk1*sepDist^cbfk2;
        %     b1 = cbfk1 * (1 - exp(-cbfk2 * sepDist1));
        %     b2 = cbfk1 * (1 - exp(-cbfk2 * sepDist2));
        %     g = [g ; b1 ; b2];
        %  end
        % end
            w=1;
            for k = 1:N
                vpNow  = [X(1,k); X(2,k)];
                vpNext = [X(1,k+1); X(2,k+1)];
                h1Now = sqrt((opos1(1) - vpNow(1))^2 + (opos1(2) - vpNow(2))^2 ) - orad1+vrad+cbf_d;
                h2Now = sqrt((opos2(1) - vpNow(1))^2 + (opos2(2) - vpNow(2))^2 ) - orad2+vrad+cbf_d;
        
                h1Next = sqrt(opos1(1) - vpNext(1))^2 + (opos1(2) - vpNext(2))^2 - (orad1+vrad+cbf_d)^2;
                h2Next = (opos2(1) - vpNext(1))^2 + (opos2(2) - vpNext(2))^2 - (orad2+vrad+cbf_d)^2;
        
        
                g = [g ; h1Next - 0.2*w*(1-cbfk1)*h1Now ; h2Next - 0.2*w*(1-cbfk2)*h2Now];
              
            end
        end
    end
    J = J + (X(1:3,N+1)-P(4:6))'*Q*(X(1:3,N+1)-P(4:6)); % Terminal state constraint
    
    OPT_variables = [reshape(X,3*(N+1),1);reshape(T,2*N,1)];
    
    nlp_prob = struct('f', J, 'x', OPT_variables, 'g', g, 'p', P);
    
    opts = struct;
    opts.ipopt.max_iter = 2000;
    opts.ipopt.print_level =0;  %0,3
    opts.print_time = 0;
    opts.ipopt.acceptable_tol =1e-8;
    opts.ipopt.acceptable_obj_change_tol = 1e-6;
    
    solver = nlpsol('solver', 'ipopt', nlp_prob, opts);
    
    args = struct;
    % constraint for dynamic model
    args.lbg(1:n_states*(N+1)) = 0;  % Equality constraints
    args.ubg(1:n_states*(N+1)) = 0;  % Equality constraints
    
    if cbfEnable
        % append CBF constraints for N timesteps and n_obs Obstacles if cbf is enabled
        % args.lbg(end+1:end+N*n_obs) = 0;      % cbf constraint must be > 0
        % args.ubg(end+1:end+N*n_obs) = inf;    % cbf constraint no upper bound
        args.lbg(end+1:end+N*n_obs) = 0;
        args.ubg(end+1:end+N*n_obs) = inf;
    end

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