
% =====================================================
%                   MPC SETUP 
% =====================================================

clc, close all, clear all
addpath('/home/sm/matlab/com/casadi-3.6.7/');   % ### ADJUST PATH TO CASADI PACKAGE LOACTION ####          
import casadi.*

DT = 0.1;               % sampling time [s]
N = 20;                 % prediction horizon
veh_rad = 0.55;         % vehicle radius
% gamma = 0.999;          % cbf parameter
cbfParms = [0.5 ; 0.5 ; 1]; 

% Static Obstacle params`
obs_rad = 0.5;
veh_start = [0, 0, deg2rad(45)]';
[obstacle, goal] = setupObstacleScenario(obs_rad,veh_rad,veh_start);    % static obstacle definintion

% n_obs =0;                % number of obstacles

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
        w           ];  % vel_dot = x2_dot = tau - C*vel - D*vel

f = Function('f',{states,controls},{dyn}); % f is symbolic representation of the system dynamics

T = SX.sym('T', n_controls, N);         % 3xN Decision variables (tau) for N horizon steps
P = SX.sym('P', n_states + n_pos_ref);  % 9x1 Parameters [initial state ; target state]
X = SX.sym('X', n_states, (N+1));       % 6xN+1 System states initial then N horizon steps
S = SX.sym('s',N,1);                    %% Slack variable

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
%constraints for SO 
% args.lbg(n_state*(N+1)+1 : n_state*(N+1)+n_obs*N) = 0;
% args.ubg(n_state*(N+1)+1 : n_state*(N+1)+n_obs*N) = inf; 

args.lbx(1:n_states:n_states*(N+1),1) = -10; %state x lower bound
args.ubx(1:n_states:n_states*(N+1),1) = 30; %state x upper bound
args.lbx(2:n_states:n_states*(N+1),1) = -10; %state y lower bound
args.ubx(2:n_states:n_states*(N+1),1) = 30; %state y upper bound
args.lbx(3:n_states:n_states*(N+1),1) = -inf; %state yaw lower bound
args.ubx(3:n_states:n_states*(N+1),1) = inf; %state yaw upper bound

min_lat = 0;

args.lbx(n_states*(N+1)+1:n_controls:n_states*(N+1)+n_controls*N,1) = -3; %Tx lower bound
args.ubx(n_states*(N+1)+1:n_controls:n_states*(N+1)+n_controls*N,1) = 3; %Tx upper bound

% args.lbx(n_state*(N+1)+2:3:n_state*(N+1)+3*N,1) = -min_lat; %Ty lower bound
% args.ubx(n_state*(N+1)+2:3:n_state*(N+1)+3*N,1) = min_lat; %Ty upper bound

args.lbx(n_states*(N+1)+n_controls:n_controls:n_states*(N+1)+n_controls*N,1) = -1; %Tyaw lower bound
args.ubx(n_states*(N+1)+n_controls:n_controls:n_states*(N+1)+n_controls*N,1) = 1; %Tyaw upper bound

%%
% =====================================================
%                   SIMULATION LOOP 
% =====================================================
current_time = 0;       % set initial time to zero
mpciter = 0;            % MPC iteration counter
solution_history = [];  % empty array for history of mpc solution for result plotting
u_mpc_history = [];     % history of first horizon step control generated by mpc
u_cbf_history = [];     % history of cbf action on controls
u_safe_history = [];    % history of action applied to system

current_state = veh_start;        % Set condition.
target_state = goal;                            % Reference posture.
state_history(:,1) = current_state;             % append this array at each step with current state
time_limit = 30;                                % Maximum simulation time (seconds)
sim_time_history(1) = current_time;             % append this array at each step with sim time

control_horizon = zeros(N,2);                   % Controls for N horizon steps
X0 = repmat(current_state,1,N+1)';              % initialization of the states decision variables



% Start Simulation Loop

main_loop = tic;
while(norm((current_state(1:3)-target_state),2) > 0.1 && mpciter < time_limit / DT)
    args.p   = [current_state;target_state];                                         % p : parameter vector 9x1 [initial state ; target state]
    args.x0  = [reshape(X0',3*(N+1),1);reshape(control_horizon',2*N,1)];     % initial value of the optimization variables
    sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx, 'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);
    u = reshape(full(sol.x(3*(N+1)+1:end))',2,N)';                  % get controls only from the solution

    solution_history(:,1:3,mpciter+1) = reshape(full(sol.x(1:3*(N+1)))',3,N+1)';  % get solution TRAJECTORY
    u_mpc_history= [u_mpc_history ; u(1,:)];
    sim_time_history(mpciter+1) = current_time;
    
    % Simulate Time Step                                                    tstep, t_now,           x0,            u, f, obstacle, cbfParms, r_veh
    [current_time, current_state, control_horizon, u_qp] = simulateTimeStep(DT,    current_time,    current_state, u, f, obstacle, cbfParms, veh_rad );               % Apply the control and simulate the timestep
    
    state_history(:,mpciter+2) = current_state;
    u_cbf_history = [u_cbf_history ; u_qp'];
    u_safe_history = [u_safe_history ; control_horizon(1,:)];

    X0 = solution_history(:,:,mpciter+1);               % current state horizon     % = reshape(full(sol.x(1:6*(N+1)))',6,N+1)';   
    X0 = [ current_state' ;  X0(3:end,:) ; X0(end,:)];   % state horizon for next step, replace mpc current state with sim current state, end state appears on last two horizon steps
    mpciter
    mpciter = mpciter + 1;

    % if mod(current_time,0.2) == 0
    %     plotCurrentState(current_state, obstacle, current_time);
    %     continue;
    % end
    % if current_time > 2.2
    %     plotCurrentState(current_state, obstacle, current_time);
    % end

end
main_loop_time = toc(main_loop);
ss_error = norm((current_state(1:2)-target_state(1:2)),2)
average_mpc_time = main_loop_time/(mpciter+1)


%% Plots

%
vehicle_positions = state_history(1:3,:);
solution_horiozons = solution_history(:,1:3,:);
visualiseSimulation(vehicle_positions, solution_horiozons, [], obstacle, target_state', N, veh_rad, DT)
%%
fig2 = figure();
t = tiledlayout(3, 2);
nexttile
plot(u_safe_history(:,1));
subtitle("Applied Control Longitudinal")
nexttile
plot(u_safe_history(:,2))
subtitle("Applied Control Yaw")

nexttile
plot(u_cbf_history(:,1));
subtitle("CBF-QP Action Longitudinal")
nexttile
plot(u_cbf_history(:,2))
subtitle("CBF-QP Action Yaw")
hold off

nexttile
plot(u_mpc_history(:,1));
subtitle("MPC Action Longitudinal")
nexttile
plot(u_mpc_history(:,2))
subtitle("MPC Action Yaw")
hold off

%%

plotCurrentState(current_state, obstacle, current_time);


%% LOCAL FUNCTIONS

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
function [t_next, x0, u0, u_qp] = simulateTimeStep(tstep, t_now, x0, u, f, obstacle, cbfParms, r_veh)
    st = x0;                                
    eta = st(1:3);
    u_nom = u(1,:)'; 
    [u_safe, u_qp] = controlBarrierFunction(t_now, obstacle, u_nom, eta, cbfParms, r_veh, tstep)   ;

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






function [u_safe, u_qp] = controlBarrierFunction(t, obs, u_nom,eta, cbfParms, r_veh, tstep)
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
    rs          = (r_obs + r_veh + 0.00)^2;
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
    Aeq     = [];     % equality constraints, set the non-existant lateral control to be zero in the qp output
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

    u_safe  = u_nom - u_qp; 


    u_safe(1) = max(u_safe(1),-3);
    u_safe(1) = min(u_safe(1), 3);

    u_safe(2) = max(u_safe(2),-1);
    u_safe(2) = min(u_safe(2), 1);

    % if u_safe(2) > 0
    %     disp("lateral control error");
    % end

end

%%

function plotCurrentState(vehicle, obstacle,time)
%% plotCurrentState - Plots current state of vehicle and obstacle

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

%%

function plotCircle(center, radius, lineStyle, color)
    % plotCircle Plots a circle on the current figure with specified line style and color
    %   plotCircle(center, radius, lineStyle, color) plots a circle with the specified center, radius,
    %   line style, and color.
    %   center - A 1x2 vector specifying the [x, y] coordinates of the circle's center.
    %   radius - A scalar specifying the radius of the circle.
    %   lineStyle - A character vector or string scalar specifying the line style (e.g., '-', '--').
    %   color - A character vector or string scalar specifying the color ('r', 'g', 'b', 'k', 'gray').

    % Validate inputs
    if length(center) ~= 2
        error('Center must be a 1x2 vector specifying [x, y] coordinates.');
    end
    if ~isscalar(radius) || radius <= 0
        error('Radius must be a positive scalar.');
    end
    validLineStyles = {'-', '--', ':', '-.'};
    if ~ismember(lineStyle, validLineStyles)
        error('Invalid line style. Choose from: ''-'', ''--'', '':'' or ''-.''.');
    end
    validColors = {'r', 'g', 'b', 'k', 'gray'};
    if ~ismember(color, validColors)
        error('Invalid color. Choose from: ''r'', ''g'', ''b'', ''k'', ''gray''.');
    end
    
    % Create the circle using the rectangle function
    rectangle('Position', [center - radius, 2*radius, 2*radius], 'Curvature', [1, 1], ...
              'EdgeColor', color, 'LineStyle', lineStyle);
    
    % Ensure the aspect ratio is equal to make the circle look like a circle
    axis equal;
    grid on; % Optional: Add some grid lines for better visualization
end