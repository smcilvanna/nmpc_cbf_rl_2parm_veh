function visualiseSimulation(simdata)

vehicle_positions = simdata.states(1:3,:);
solution_horiozons = simdata.solutions(:,1:3,:);
obstacle = reshape(simdata.obstacle,1,3);
x_ref = simdata.target;
N = simdata.N;
veh_radius = simdata.vrad;
timeStep = simdata.dt;


%% Figure setup
fig = figure(100);
% fig = gcf; %Current figure handle
fig.Color = 'w';
fig.Units = 'normalized';
fig.OuterPosition = [0 0 1 1];
fig.PaperPositionMode = 'auto';


%% Draw simulation

% Footprint of the robot
draw_ang=0:0.005:2*pi;               % angles for to draw the robot
x_robot = veh_radius*cos(draw_ang);
y_robot = veh_radius*sin(draw_ang);

% arrow triangle parameters
h_t = 0.14; w_t=0.09; % triangle parameters
x_driven = [];
y_driven = [];
x_mo = [];
y_mo = [];
step_size = 1;

%pause(10)
for k = 1:step_size:size(vehicle_positions,2)-1 % go through the open loop
    % Plot SO (Static Obstacles)
    for i = 1:size(obstacle,1)
        x_obs_fp = obstacle(i,3)*cos(draw_ang);
        y_obs_fp = obstacle(i,3)*sin(draw_ang);
        plot(obstacle(i,1)+x_obs_fp, obstacle(i,2)+y_obs_fp,'k', 'LineWidth', 0.5)  % circle around SO
        hold on
    end
    
    
    %Plot reference trajectory
    if size(x_ref,1) > 1
        plot(x_ref(:,1), x_ref(:,2), '--.g', 'LineWidth', 0.5)
    else 
        xr = x_ref(1); yr = x_ref(2); thr = x_ref(3);
        x_r = [ xr+h_t*cos(thr), xr+(w_t/2)*cos((pi/2)-thr), xr-(w_t/2)*cos((pi/2)-thr), xr+h_t*cos(thr)];
        y_r = [ yr+h_t*sin(thr), yr-(w_t/2)*sin((pi/2)-thr), yr+(w_t/2)*sin((pi/2)-thr), yr+h_t*sin(thr)];
        plot(x_r, y_r, 'g','linewidth',2); % plot reference
    end    
    hold on


    % Plot the driven (executed) trajectory
    x1 = vehicle_positions(1,k,1); y1 = vehicle_positions(2,k,1); th1 = vehicle_positions(3,k,1);
    x_driven = [x_driven x1];
    y_driven = [y_driven y1];
    plot(x_driven,y_driven,'r','LineWidth', 1) % plot exhibited trajectory
    hold on

    % Plot MPC solution horizion
    if k < size(vehicle_positions,2)
        plot(solution_horiozons(1:N,1,k), solution_horiozons(1:N,2,k), 'r--*')
        hold on
        for i = 2:N+1
            %plot(x_cl(i,1,k)+x_robot, x_cl(i,2,k)+y_robot,'--r')     % plot robot footprint in predictions
            hold on
        end
    end
      
    % Plot Robot footprint
    x_f = [x1+h_t*cos(th1), x1+(w_t/2)*cos((pi/2)-th1), x1-(w_t/2)*cos((pi/2)-th1), x1+h_t*cos(th1)];
    y_f = [y1+h_t*sin(th1), y1-(w_t/2)*sin((pi/2)-th1), y1+(w_t/2)*sin((pi/2)-th1), y1+h_t*sin(th1)];
    plot(x_f,y_f,'r','linewidth',1); 
    hold on
    plot(x1+x_robot,y1+y_robot,'--r')      % plot robot circle
    
    yaw_marker_len = veh_radius;
    dx = yaw_marker_len * cos(th1);
    dy = yaw_marker_len * sin(th1);
    quiver(x1,y1,dx,dy,0,'b',LineWidth=2,MaxHeadSize=1,DisplayName="Vehicle Yaw");


    hold off
    
    % ylabel('$y$-position [m]','interpreter','latex','FontSize', 16)
    % xlabel('$x$-position [m]','interpreter','latex','FontSize', 16)
    ylabel("y-position (m)");
    xlabel("x-position (m)");
    %axis([-4 4 -1.5 6.5])
    axis([0 18 0 18])
    axis square
    grid minor

    tnow = (k-1)*timeStep;
    title("Mobile Robot Trajectory");
    subtitle(sprintf("Time : %.02f",tnow));
    
    ytxt = sprintf("Yaw Angle %0.2f",rad2deg(th1));
    text(10,1,ytxt);




    % box on;
    grid on;
    
    drawnow
end