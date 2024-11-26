
% =====================================================
%                   MPC SETUP 
% =====================================================

clc, close all, clear all
addpath('/home/sm/matlab/com/casadi-3.6.7/');   % ### ADJUST PATH TO CASADI PACKAGE LOACTION ####          
import casadi.*

DT = 0.1;               % sampling time [s]
N = 15;                 % prediction horizon
veh_rad = 0.55;         % vehicle radius
% gamma = 0.999;          % cbf parameter
cbfParms = [10 ; 5]; 

% Static Obstacle params`
obs_rad = 0.5;
veh_start = [0, 0, deg2rad(45)]';
[obstacle, goal] = setupObstacleScenario(obs_rad,veh_rad,veh_start);    % static obstacle definintion
n_obs =0;                % number of obstacles

x = SX.sym('x');        y = SX.sym('y');        yaw = SX.sym('yaw');
v_x = SX.sym('v_x');    v_y = SX.sym('v_y');    v_yaw = SX.sym('v_yaw'); 

states = [x ; y ; yaw ;  v_x ; v_y ; v_yaw ]; 
n_states = length(states);
n_pos_ref = length(states(1:3));

tau_x = SX.sym('tau_x');    tau_y = SX.sym('tau_y');    tau_yaw = SX.sym('tau_yaw');
controls = [tau_x;tau_y;tau_yaw];
n_controls = length(controls);

M = [       25.800, 0,          0;
            0,      24.6612,    0;
            0,      0,          2.7600];

C = zeros(3);
D = zeros(3);
Cr = [      0,                              0,         -24.6612*v_y     ; 
            0,                              0,          25.8*v_x        ; 
            24.6612*v_y + 1.0948*v_yaw,    -25.8*v_x,   0               ];

Dr = [      0.7225 + 1.3274*abs(v_x) + 5.8664*v_x^2,    0,                                              0                                           ;
            0,                                          0.8612 + 36.2823*abs(v_y) + 8.05*abs(v_yaw),    0.845*abs(v_y) + 3.45*abs(v_yaw) - 0.1079   ; 
            0,                                         -0.1025 - 5.0437*abs(v_y) - 0.13*abs(v_yaw),     0.845*abs(v_y) + 3.45*abs(v_yaw) - 0.1079   ];

case_CD = 0;    

switch case_CD
    case 1
        C = Cr;
    case 2
        D = Dr;
    case 3
        C = Cr;
        D = Dr;
end

J_n = [     cos(yaw),  -sin(yaw),       0   ;
            sin(yaw),   cos(yaw),       0   ;
            0,          0,              1   ];

dyn = [ J_n*states(4:6)                                 ;   % pos_dot = x1_dot = J*vel
        inv(M)*(controls-C*states(4:6)-D*states(4:6))   ];  % vel_dot = x2_dot = tau - C*vel - D*vel

f = Function('f',{states,controls},{dyn}); % f is symbolic representation of the system dynamics

T = SX.sym('T', n_controls, N);         % 3xN Decision variables (tau) for N horizon steps
P = SX.sym('P', n_states + n_pos_ref);  % 9x1 Parameters [initial state ; target state]
X = SX.sym('X', n_states, (N+1));       % 6xN+1 System states initial then N horizon steps
J = 0;                                  % Empty Objective Function
g = [];                                 % Empty Constraints Vector
Qx = diag([10 10 1]);                   % Horizon steps position error weighing matrix
Qv = diag([10 10 1]);                   % Horizon steps velocity error Weighing matrix
R = diag([1 0 1]);                      % Horizon steps control effort Weighing matrix
Q = 10^5*diag([1 1 1]);                 % Terminal state position error weight matrix

st = X(:,1); % Initial State

g = [g;st-P(1:6)]; % Initial Condition Constraints

for k = 1:N
    st = X(:,k);    % vehicle states at each horizon step
    con = T(:,k);   % vehicle controls at each horizon step

    % append objective function at each horizon step
    %        (veh_pos - target pos)       
    J = J + (st(1:3)-P(7:9))'*Qx*(st(1:3)-P(7:9)) + (st(4:6)-P(4:6))'*Qv*(st(4:6)-P(4:6)) + con'*R*con; % calculate obj
    st_next = X(:,k+1);
    k1 = f(st, con);                % compute rk4 coefficients
    k2 = f(st + DT/2*k1, con);      %
    k3 = f(st + DT/2*k2, con);      %
    k4 = f(st + DT*k3, con);        %
    st_next_RK4 = st + DT/6*(k1 +2*k2 +2*k3 +k4); % new
    g = [ g ; st_next-st_next_RK4 ]; % compute constraints % new
    %f_value = f(st,con);
    %st_next_euler = st + (DT*f_value);
    %g = [g;st_next-st_next_euler]; % compute constraints    
end


% % Constraints on SO ##### OLD #####
% for k = 1:N      
%     for i = 1:n_obs
%         h =         ((X(1,k)-obstacle(i,1))^2         + (X(2,k)-obstacle(i,2))^2 )      - (veh_rad + obstacle(i,3))^2;
%         h_next =    ((X(1,k+1)-obstacle(i,1))^2       + (X(2,k+1)-obstacle(i,2))^2)     - (veh_rad + obstacle(i,3))^2;
%         g = [g ; h_next - (1-gamma)*h];
%     end
% end

% CBF Constraints
% rs = (veh_rad + obs_rad + 0.01)^2;


% for n = 1:N      
%     for i = 1:n_obs % for this example, only 1 obstacle
% 
%         sepx = X(1,n) - obstacle(i,1);
%         sepy = X(2,n) - obstacle(i,2);
% 
%         h = sepx^2 + sepy^2 - rs^2;
%         lfh = 
% 
% 
% 
%         g = [g ; h_next - (1-gamma)*h];
%     end
% end


J = J + (X(1:3,N+1)-P(7:9))'*Q*(X(1:3,N+1)-P(7:9));
OPT_variables = [reshape(X,6*(N+1),1);reshape(T,3*N,1)];

nlp_prob = struct('f', J, 'x', OPT_variables, 'g', g, 'p', P);

opts = struct;
opts.ipopt.max_iter = 2000;
opts.ipopt.print_level =0;%0,3
opts.print_time = 0;
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;

solver = nlpsol('solver', 'ipopt', nlp_prob,opts);

args = struct;
% constraint for dynamic model
args.lbg(1:6*(N+1)) = 0;  % Equality constraints
args.ubg(1:6*(N+1)) = 0;  % Equality constraints
%constraints for SO 
args.lbg(6*(N+1)+1 : 6*(N+1)+n_obs*N) = 0;
args.ubg(6*(N+1)+1 : 6*(N+1)+n_obs*N) = inf; 

args.lbx(1:6:6*(N+1),1) = -10; %state x lower bound
args.ubx(1:6:6*(N+1),1) = 30; %state x upper bound
args.lbx(2:6:6*(N+1),1) = -10; %state y lower bound
args.ubx(2:6:6*(N+1),1) = 30; %state y upper bound
args.lbx(3:6:6*(N+1),1) = -inf; %state yaw lower bound
args.ubx(3:6:6*(N+1),1) = inf; %state yaw upper bound
args.lbx(4:6:6*(N+1),1) = -3; %state vx lower bound
args.ubx(4:6:6*(N+1),1) = 3; %state vx upper bound
args.lbx(5:6:6*(N+1),1) = -3; %state vy lower bound
args.ubx(5:6:6*(N+1),1) = 3; %state vy upper bound
args.lbx(6:6:6*(N+1),1) = -1; %state vyaw lower bound
args.ubx(6:6:6*(N+1),1) = 1; %state vyaw upper bound

min_lat = 0;

args.lbx(6*(N+1)+1:3:6*(N+1)+3*N,1) = -30; %Tx lower bound
args.ubx(6*(N+1)+1:3:6*(N+1)+3*N,1) = 30; %Tx upper bound

args.lbx(6*(N+1)+2:3:6*(N+1)+3*N,1) = -min_lat; %Ty lower bound
args.ubx(6*(N+1)+2:3:6*(N+1)+3*N,1) = min_lat; %Ty upper bound

args.lbx(6*(N+1)+3:3:6*(N+1)+3*N,1) = -10; %Tyaw lower bound
args.ubx(6*(N+1)+3:3:6*(N+1)+3*N,1) = 10; %Tyaw upper bound

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

current_state = [veh_start ; 0 ; 0 ; 0];        % Set condition.
target_state = goal;                            % Reference posture.
state_history(:,1) = current_state;             % append this array at each step with current state
time_limit = 50;                                % Maximum simulation time (seconds)
sim_time_history(1) = current_time;             % append this array at each step with sim time

control_horizon = zeros(N,3);                   % Controls for N horizon steps
X0 = repmat(current_state,1,N+1)';              % initialization of the states decision variables



% Start Simulation Loop

main_loop = tic;
while(norm((current_state(1:3)-target_state),2) > 0.1 && mpciter < time_limit / DT)
    args.p   = [current_state;target_state];                                         % p : parameter vector 9x1 [initial state ; target state]
    args.x0  = [reshape(X0',6*(N+1),1);reshape(control_horizon',3*N,1)];     % initial value of the optimization variables
    sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx, 'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);
    u = reshape(full(sol.x(6*(N+1)+1:end))',3,N)';                  % get controls only from the solution

    solution_history(:,1:6,mpciter+1) = reshape(full(sol.x(1:6*(N+1)))',6,N+1)';  % get solution TRAJECTORY
    u_mpc_history= [u_mpc_history ; u(1,:)];
    sim_time_history(mpciter+1) = current_time;

                                                                    %       tstep, t_now, x0, u, f, obstacle, cbfParms, r_veh
    [current_time, current_state, control_horizon, u_qp] = simulateTimeStep(DT,    current_time,    current_state, u, f, obstacle, cbfParms, veh_rad, M );               % Apply the control and simulate the timestep
    
    state_history(:,mpciter+2) = current_state;
    X0 = reshape(full(sol.x(1:6*(N+1)))',6,N+1)';   % get solution TRAJECTORY
    u_cbf_history = [u_cbf_history ; u_qp];
    % Shift trajectory to initialize the next step
    X0 = [X0(2:end,:);X0(end,:)];
    mpciter
    mpciter = mpciter + 1;
end
main_loop_time = toc(main_loop);
ss_error = norm((current_state(1:2)-target_state(1:2)),2)
average_mpc_time = main_loop_time/(mpciter+1)

%%
xxx = state_history(1:3,:);
xxx1 = solution_history(:,1:3,:);
visualiseSimulation(xxx,xxx1,[],obstacle,target_state',N,veh_rad)



%% LOCAL FUNCTIONS
function [obs,tgt] = setupObstacleScenario(obs_rad,veh_rad,veh_start)
% setupObstacleScenario Set position of obstacle and goal point to maintain equal seperation across different sized obstacles
    vx = veh_start(1);
    vy = veh_start(2);
    veh_yaw = veh_start(3);
    approach_sep = 5;                                       % min distance between vehicle clearance radius and obstacle at start
    after_sep = 5;                                          % min distance between perimiter of obstacle and goal point
    v2oCen = veh_rad + approach_sep + obs_rad;              % centre to centre distance between vehicle start and obstacle
    obs = [ (vx + v2oCen*cos(veh_yaw)) ,  (vy + v2oCen*sin(veh_yaw))  , obs_rad];
    obs(2) = obs(2) - 0.1;
    tgt = [ (obs(1) + obs_rad + after_sep*cos(veh_yaw)) , (obs(2) + obs_rad + after_sep*sin(veh_yaw)) , veh_yaw]';
end

function [t_next, x0, u0, u_qp] = simulateTimeStep(tstep, t_now, x0, u, f, obstacle, cbfParms, r_veh, M)
    st = x0;                                % 6x1
    earth_position = st(1:3);
    u_nom = u(1,:)';                          % 3x1
    %f_value = f(st,con);

    CDG = zeros(3,1);
    yaw = st(3);
    J   = [     cos(yaw),  -sin(yaw),       0   ;
                sin(yaw),   cos(yaw),       0   ;
                0,          0,              1   ];

    [u_safe, u_qp] = controlBarrierFunction(t_now, obstacle, u_nom, earth_position, J, CDG, M, cbfParms, r_veh, tstep)   ;

    st = st + (tstep*f(st,u_safe));
    x0 = full(st);
    
    t_next = t_now + tstep;
    u0 = [u(2:size(u,1),:) ; u(size(u,1),:)];
end






function [u_safe, u_qp] = controlBarrierFunction(t, obs, u_nom,e_psn,J,CDG,M, cbfParms,r_veh,tstep)
% controlBarrierFunction Compute safe outputs for each timestep based on 

    % % debug - mute output first few timesteps
    % if t < 0.005;
    %     mute    =0;
    % else
    %     mute    =1;
    % end

    % Persistent Variable for derivatives
    persistent Jprev pos_prev;
    if isempty(Jprev)
        Jprev = J;
        pos_prev = [0,0,0];
    end

    % Derivative Variables
    Jdot = (J - Jprev)/ tstep;
    Jprev = J;

    % System states from inputs
    % Variables used in ECBF
    m           = inv(M);
    k1          = cbfParms(1);
    k2          = cbfParms(2);

    % earth frame positions
    pos_x       = e_psn(1);
    pos_y       = e_psn(2);
    yaw         = e_psn(3);
            
    % earth frame velocities [eta_dot]        
    e_vel_x     = ( pos_x - pos_prev(1) ) / tstep ;     % calculate earth frame velocity components
    e_vel_y     = ( pos_y - pos_prev(2) ) / tstep ; 
    e_vel_w     = ( yaw   - pos_prev(3) ) / tstep ;
    pos_prev    = [ pos_x, pos_y, yaw];                  % set current position for next step previous value
    e_vel       = [ e_vel_x ; e_vel_y ; e_vel_w];     

    % earth frame accelerations [eta_dot_dot]
    % ctrl        = u_nom;
    % earth_acc   = J*m*(ctrl - CDG) + Jdot*(Jinv*e_vel);
    % 
    % e_acc_x     = earth_acc(1);
    % e_acc_y     = earth_acc(2);
   
    % Obstacle parameters
    r_obs       = obs(3);
    obs_x       = obs(1);
    obs_y       = obs(2);

    % a           = 1; %2*r_obs;      % for circular obstacle
    % b           = a;                % for circular obstacle
    
    sep_x       = pos_x - obs_x;
    sep_y       = pos_y - obs_y;

    rs          = (r_obs + r_veh + 0.01)^2;
    
    SEP         = [2*sep_x ; 2*sep_y ; 0 ]';
%   S           = [2*a*Sx   2*b*Sy  ];
%   S2          = [2*a*S2x  2*b*S2y ]; 
    
%   Control Barrier function terms
    h           = sep_x^2 + sep_y^2 - rs^2 ;
    Lfh         = 2*(sep_x)*(e_vel_x) + 2*(sep_y)*(e_vel_y); 
%   L2fh        = 2*(sep_x)*e_acc_x + 2*(e_vel_x^2) + 2*(sep_y)*e_acc_y + 2*(e_vel_y^2)     
%   ECBF        = L2fh*mute + k1*Lfh*mute + k2*h;

    % quadprog parameters
    H       = 2*[   1 0 0   ; ...
                    0 1 0   ; ... 
                    0 0 1   ] ;
        
    f       = [0;0;0]; 

    % Setup A & b matrices
    A       =  SEP*J*m ;
    b       =  SEP*(J*m*(u_nom - CDG) + Jdot*e_vel) + 2*e_vel_x^2 + 2*e_vel_y^2 + k1*Lfh  + k2*h       ;

    Aeq     = [ 0 1 0 ];     % equality constraints, set the non-existant lateral control to be zero in the qp output
    beq     = 0; 

    u_qp   = [0;0;0];
    flag = 0;
    coder.extrinsic('quadprog');
    options = optimset('Display','off');
    [output, ~, flag]   = quadprog(H,f,A,b,Aeq,beq,[],[],[],options);

    if flag == 1
        u_qp = output;
    end

    u_safe  = u_nom - u_qp; 

    u_safe(1) = max(u_safe(1),-30);
    u_safe(1) = min(u_safe(1), 30);

    u_safe(3) = max(u_safe(3),-10);
    u_safe(3) = min(u_safe(3), 10);

    if u_safe(2) > 0
        disp("lateral control error");
    end

end



%% OLD


% %%
% x_ref = repmat(xs,1,length(xxx));
% error = abs(x_ref-xxx);
% figure(1)
% subplot(3,2,1)
% plot(t,x_ref(1,1:length(t))),hold on,grid on,plot(t,xx(1,1:length(t)))
% title('Tracking performance in x'),xlabel('Time (s)'),ylabel('x (m)')
% subplot(3,2,2)
% plot(t,error(1,1:length(t))),hold on,grid on,
% title('Tracking error in x'),xlabel('Time (s)'),ylabel('Error x (m)')
% subplot(3,2,3)
% plot(t,x_ref(2,1:length(t))),hold on,grid on,plot(t,xx(2,1:length(t)))
% title('Tracking performance in y'),xlabel('Time (s)'),ylabel('y (m)')
% subplot(3,2,4)
% plot(t,error(2,1:length(t))),hold on,grid on,
% title('Tracking error in y'),xlabel('Time (s)'),ylabel('Error y (m)')
% subplot(3,2,5)
% plot(t,x_ref(3,1:length(t))),hold on,grid on,plot(t,xx(3,1:length(t)))
% title('Tracking performance in yaw'),xlabel('Time (s)'),ylabel('yaw (rad)')
% subplot(3,2,6)
% plot(t,error(3,1:length(t))),hold on,grid on,
% title('Tracking error in yaw'),xlabel('Time (s)'),ylabel('Error yaw (rad)')
% 
% figure(2)
% subplot(2,2,1)
% plot(t,u_cl(1:length(t),1)),hold on,grid on,
% title('Control signal in x'),xlabel('Time (s)'),ylabel('tau-x (Nm)')
% subplot(2,2,2)
% plot(t,u_cl(1:length(t),2)),hold on,grid on,
% title('Control signal in y'),xlabel('Time (s)'),ylabel('tau-y (Nm)')
% subplot(2,2,3)
% plot(t,u_cl(1:length(t),2)),hold on,grid on,
% title('Control signal in yaw'),xlabel('Time (s)'),ylabel('tau-yaw (Nm)')
% 
% figure(3)
% subplot(2,2,1)
% plot(t,xx(4,1:length(t))),hold on,grid on,
% title('Velocity x'),xlabel('Time (s)'),ylabel('v-x (m/s)')
% subplot(2,2,2)
% plot(t,xx(5,1:length(t))),hold on,grid on,
% title('Velocity y'),xlabel('Time (s)'),ylabel('v-y (m/s)')
% subplot(2,2,3)
% plot(t,xx(6,1:length(t))),hold on,grid on,
% title('Velocity yaw'),xlabel('Time (s)'),ylabel('v-yaw (rad/s)')
