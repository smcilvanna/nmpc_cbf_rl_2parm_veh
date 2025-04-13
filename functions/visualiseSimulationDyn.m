function fig = visualiseSimulationDyn(simdata,staticPlot,view,alim)
    arguments
        simdata struct
        staticPlot logical = 0;
        view logical = 1;
        alim = [0 0];
    end
    
    showFig='off';
    if view
        showFig='on';
    end
    
    vehicle_positions = simdata.states(1:3,:);    
    % nObs = size(simdata.obstacle,2);
    % obstacle1  = reshape(simdata.obstacle(1:3),1,3);
    % if nObs > 1
    %     obstacle2 = reshape(simdata.obstacle(4:6),1,3);
    % else
    %     obstacle2 = obstacle1;
    % end
    x_ref = simdata.target;
    N = simdata.N;
    veh_radius = simdata.vrad;
    timeStep = simdata.dt;
    axmax = ceil(max(x_ref(1:2)));
    
    
    % Figure setup
    fig = figure(Visible=showFig);
    % set(fig,'Visible',);
    % fig = gcf; %Current figure handle
    fig.Color = 'w';
    fig.Units = 'normalized';
    fig.OuterPosition = [0 0 1 1];
    fig.PaperPositionMode = 'auto';
    draw_ang=0:0.005:2*pi;               % angles for plotting circles
    
    if ~staticPlot
        solution_horiozons = simdata.solutions(:,1:3,:);
        % Animate simulation
        % Footprint of the robot
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
            % main obstacle
            x_obs_fp = obstacle1(3)*cos(draw_ang);
            y_obs_fp = obstacle1(3)*sin(draw_ang);
            plot(obstacle1(1)+x_obs_fp, obstacle1(2)+y_obs_fp,'k', 'LineWidth', 0.5)  % circle around SO    
            hold on
            % second obstacle to make gap
            x_obs_fp2 = obstacle2(3)*cos(draw_ang);
            y_obs_fp2 = obstacle2(3)*sin(draw_ang);
            plot(obstacle2(1)+x_obs_fp2, obstacle2(2)+y_obs_fp2,'k', 'LineWidth', 0.5)  % circle around SO    
        
            
            
            %Plot reference trajectory
            if size(x_ref,1) > 1
                plot(x_ref(:,1), x_ref(:,2), '--.g', 'LineWidth', 0.5)
            else 
                % xr = x_ref(1); yr = x_ref(2); thr = x_ref(3);
                % x_r = [ xr+h_t*cos(thr), xr+(w_t/2)*cos((pi/2)-thr), xr-(w_t/2)*cos((pi/2)-thr), xr+h_t*cos(thr)];
                % y_r = [ yr+h_t*sin(thr), yr-(w_t/2)*sin((pi/2)-thr), yr+(w_t/2)*sin((pi/2)-thr), yr+h_t*sin(thr)];
                % plot(x_r, y_r, 'g','linewidth',2); % plot reference
                scatter(x_ref(1),x_ref(2),75,[0 0.2 1],'x', LineWidth=2);
    
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
            if sum(alim) == 0
                axis([0 axmax 0 axmax])
            else
                axis([0 alim(1) 0 alim(2)])
            end
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
    else
        % Static Overall Plot
        tl = tiledlayout(3,6);
        title(tl,"Simulation Results")
        tl.TileSpacing = "compact";
        tl.Padding = "compact";
        
        ax1 = nexttile([3 3]);
    
        nObs = height(simdata.obstacles);

        for o = 1:nObs
            % Plot Obstacle 1
            x_obs_fp = simdata.obstacles(o,3)*cos(draw_ang);
            y_obs_fp = simdata.obstacles(o,3)*sin(draw_ang);
            plot(ax1,simdata.obstacles(o,1)+x_obs_fp, simdata.obstacles(o,2)+y_obs_fp,'k', 'LineWidth', 0.5)  % circle around SO    
            hold(ax1,"on")
        end
    
        % Plot vehicle path 
        x1 = vehicle_positions(1,:); 
        y1 = vehicle_positions(2,:); 
        plot(ax1,x1,y1,'r','LineWidth', 1) % plot exhibited trajectory
    
        % Plot Target Point
        scatter(ax1,x_ref(1),x_ref(2),75,[0 0.2 1],'x', LineWidth=2);
    
        ylabel("y-position (m)");
        xlabel("x-position (m)");
        %axis([-4 4 -1.5 6.5])
        if sum(alim) == 0
            axis([0 axmax 0 axmax])
        else
            axis([0 alim(1) 0 alim(2)])
        end
        axis square
        grid minor
    
        tsteps = size(vehicle_positions,2) -1;
        tend = tsteps*timeStep;
        title(ax1,"Mobile Robot Trajectory");
        cbf_k1 = simdata.cbf(1);
        cbf_alpha = simdata.cbf(2);
        % mpcQxy = simdata.mpcParms(5);
        % mpcQw = simdata.mpcParms(6);
        % mpcRv = simdata.mpcParms(3);
        % mpcRw = simdata.mpcParms(4);
        mpcN = simdata.N;
        ltime = simdata.looptime;
        steptime = ltime/tsteps * 1000;
        yawend = rad2deg(vehicle_positions(3,end));
        % cbfk2 = simdata.cbf(2);
        % cbfd = simdata.cbf(3);
        subtitle(ax1,sprintf("cbf_{k1}= [%.3f]  cbf_{\\alpha}= [%.3f]   |   N= [%d] \nYaw_{end} [%05.1f]  |  EndTime [%05.2f s]  SimTime [%05.2f s]  StepTime [%05.2f ms]", ...
                                         cbf_k1,                cbf_alpha,      mpcN,            yawend,              tend,               ltime,               steptime));
        
        % ytxt = sprintf("Final Yaw : %0.2f\nRuntime: %.02f",rad2deg(vehicle_positions(3,end)),tend );
        % text(ax1,10,1,ytxt);
        % box on;
        % grid on;
        % drawnow
    
        % fig2 = figure(2);
    
        xyvels = [zeros(2,1), diff(vehicle_positions(1:2,:),1,2)/simdata.dt ];  % calculate linear velocity from positions
        speed = sqrt(sum(xyvels.^2,1));                                         % convert to lin vel for plot
        linvCtrl = [ simdata.usafe(:,1)];                                   % Linear velocity controls
        angCtrl  = [ simdata.usafe(:,2)];                                   % Angular velocity controls
        t  = 0:simdata.dt:(simdata.dt*(size(vehicle_positions,2)-1));           % timesteps
    


        dlv = [0 ; diff(linvCtrl)];     % change in linvel
        dav = [0 ; diff(angCtrl)];      % change in steering
    
        dlv_sum = sum(abs(dlv));
        dav_sum = sum(abs(dav));
    
        ax2 = nexttile(4,[1 3]);
        plot(ax2, t,speed,LineWidth=2, Color=[0.1 0.1 0.9],DisplayName="Linear Velocity");
        % hold(ax2,"on");
        % plot(ax2, t,linvCtrl, LineWidth=1, Color=[0.1 0.1 0.1],DisplayName="u_{v}")
        title(ax2,"Linear Velocity"); ylabel("v (m/s)"); grid on
    
        ax3 = nexttile(10,[1 3]);
        plot(ax3, t,angCtrl,LineWidth=2, Color=[0.8 0.1 0.1] ,DisplayName="Angular Velocity");
        title(ax3,"Angular Velocity"); ylabel("w (rad/s)"); grid on
    
        ax4 = nexttile(16,[1 3]);
        yyaxis left;
        plot(ax4, t,dlv,LineWidth=2, Color=[0.1 0.1 0.9] ,DisplayName="\Delta u_v ");
        ylabel("v (m/s^2)");
        hold(ax4,"on");
        yyaxis right
        plot(ax4, t,dav,LineWidth=2, Color=[0.8 0.1 0.1] ,DisplayName="\Delta u_{\omega}");
        ylabel("\Delta\omega (rad/s^2)");
        legend(ax4);
        title(ax4,"Control Changes"); xlabel("Time (seconds)")  ; grid on
    
        ctxt = sprintf("\\Sigma\\Deltav : %0.2f | \\Sigma\\Delta\\omega : %0.2f",dlv_sum,dav_sum);
        subtitle(ax4,ctxt);
        
    end
end