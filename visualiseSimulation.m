function visualiseSimulation(vehicle_positions, solution_horiozons, o_cl, obstacle, x_ref, N, veh_radius)
%% Figure setup
figure(100)
fig = gcf; %Current figure handle
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
    
    % % Plot MO (Moving Obstacles) predictions
    % if k < size(x_ol,2)
    %     for i = 1:size(o_cl,1)
    %         x_obs_fp = o_cl(i,1,5,1)*cos(draw_ang);
    %         y_obs_fp = o_cl(i,1,5,1)*sin(draw_ang);
    %         %plot(o_cl(i,2:(N+1),1,k), o_cl(i,2:(N+1),2,k), 'c--*')
    %         hold on
    %         for j = 2:(N+1)
    %             %plot(o_cl(i,j,1,k)+x_obs_fp, o_cl(i,j,2,k)+y_obs_fp,'--c', 'LineWidth', 0.5)     % plot robot footprint in predictions
    %             hold on
    %         end
    %     end
    % end
    % 
    % % Plot MO current position
    % for i = 1:size(o_cl,1)
    %     ox1 = o_cl(i,1,1,k);
    %     ox2 = o_cl(i,1,2,k);
    %     ox3 = o_cl(i,1,3,k);
    %     x_r = [ ox1+h_t*cos(ox3), ox1+(w_t/2)*cos((pi/2)-ox3), ox1-(w_t/2)*cos((pi/2)-ox3), ox1+h_t*cos(ox3)];
    %     y_r = [ ox2+h_t*sin(ox3), ox2-(w_t/2)*sin((pi/2)-ox3), ox2+(w_t/2)*sin((pi/2)-ox3), ox2+h_t*sin(ox3)];
    %     plot(x_r, y_r, 'm','linewidth',1); % plot MO triangle
    % 
    %     x_mo(i,k) =  o_cl(i,1,1,k);
    %     y_mo(i,k) = o_cl(i,1,2,k);
    %     plot(x_mo(i,:),y_mo(i,:),'--m','LineWidth', 0.3) % plot exhibited trajectory
    % 
    %     %plotArrow(ox1, ox2, o_cl(i,1,3,k), o_cl(i,1,5,k), arrow_h, arrow_w, 'k'); % arrow for MO
    %     hold on
    %     x_obs_fp = o_cl(i,1,5,1)*cos(draw_ang);
    %     y_obs_fp = o_cl(i,1,5,1)*sin(draw_ang);
    %     plot(o_cl(i,1,1,k)+x_obs_fp, o_cl(i,1,2,k)+y_obs_fp,'--k') % circle around MO
    %     hold on
    % end
    
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

    % plot legend
    %plot([-3.7 -3.2],[-0.4 -0.4],'-.g','linewidth',1)   % plot legend 
    %text(-3.1,-0.35,'Reference trajectory','FontSize',15)
    %plot([-3.7 -3.2],[-0.7 -0.7],'--m','linewidth',1)   % plot legend 
    %text(-3.1,-0.65,'Moving obstacle trajectory','FontSize',15)
    %plot([-3.7 -3.2],[-1 -1],'r','linewidth',1)   % plot legend 
    %text(-3.1,-0.95,'NMPC-CBF (N = 5; \gamma = 0.2)','FontSize',15)   
    
    % Plot reference trajectory until horizon ends
    % Plot positon on reference trajcetory
    %if (k+N <= size(x_ref,1))
        %plot(x_ref(k:k+N,1), x_ref(k:k+N, 2), 'g*')
        %hold on
        %fitted_trajectory = BuiltIn_trajectory_fitting(x_ref(k:k+N,1:2),7);
        %plot(fitted_trajectory(:,1), fitted_trajectory(:,2), 'b', 'LineWidth', 1.5)
    %else
        %plot(x_ref(k:end,1), x_ref(k:end, 2), 'g*')
        %hold on
        %fitted_trajectory = BuiltIn_trajectory_fitting(x_ref(k:end,1:2),7);
        %plot(fitted_trajectory(:,1), fitted_trajectory(:,2), 'b', 'LineWidth', 1.5)
    %end
    %hold on
    % Plot goal position
    %plotArrow(x_ref(end,1), x_ref(end,2), x_ref(end,3), veh_radius, arrow_h, arrow_w, 'g');
    %hold on
%%%%%%%%%%%%%%%%%%%%%%%     Robot  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    % Plot the driven (executed) trajectory
    x1 = vehicle_positions(1,k,1); y1 = vehicle_positions(2,k,1); th1 = vehicle_positions(3,k,1);
    x_driven = [x_driven x1];
    y_driven = [y_driven y1];
    plot(x_driven,y_driven,'r','LineWidth', 1) % plot exhibited trajectory
    hold on

    % Plot prediction
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
    %plotArrow(x1, y1, th1, veh_radius, arrow_h, arrow_w, 'k');
    hold on
    plot(x1+x_robot,y1+y_robot,'--r')      % plot robot circle
    
    hold off
    
    ylabel('$y$-position [m]','interpreter','latex','FontSize', 16)
    xlabel('$x$-position [m]','interpreter','latex','FontSize', 16)
    %axis([-4 4 -1.5 6.5])
    axis([0 18 0 18])
    axis square
    grid minor
    
    % box on;
    grid on;
    
    drawnow
end