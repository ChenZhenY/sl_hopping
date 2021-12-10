function animate(tspan, x, p, interval)
    % Prepare plot handles
    hold on
    h_OB = plot([0],[0],'LineWidth',2);
    h_AC = plot([0],[0],'LineWidth',2);
    h_BD = plot([0],[0],'LineWidth',2);
    h_CE = plot([0],[0],'LineWidth',2);
    h_OF = plot([0],[0],'LineWidth',2);
    h_body = plot([0],[0],'LineWidth',3);
    
    xlabel('x'); ylabel('y');
    h_title = title('t=0.0s');
    
    axis equal
    x_min = -.2;
    x_max = .8;
    axis([x_min x_max -.3 .1]*3); % -.2 .2 -.3 .1
    height = p(end);
    h_ground = plot([x_min x_max]*3,[height height],'k-','LineWidth',1);

    %Step through and update animation
    for i = 1:length(tspan)
        % skip frame.
        if mod(i,10)
            continue;
        end
        t = tspan(i);
        z = x(:,i); 
        keypoints = keypoints_jumping_leg(z,p);

        rA = keypoints(:,1); % Vector to base of cart
        rB = keypoints(:,2);
        rC = keypoints(:,3); % Vector to tip of pendulum
        rD = keypoints(:,4);
        rE = keypoints(:,5);
        rO = keypoints(:,6);
        rF = keypoints(:,7);

        set(h_title,'String',  sprintf('t=%.2f',t) ); % update title
        
        set(h_OB,'XData',[rO(1) rB(1)]);
        set(h_OB,'YData',[rO(2) rB(2)]);
        
        set(h_AC,'XData',[rA(1) rC(1)]);
        set(h_AC,'YData',[rA(2) rC(2)]);
        
        set(h_BD,'XData',[rB(1) rD(1)]);
        set(h_BD,'YData',[rB(2) rD(2)]);
        
        set(h_CE,'XData',[rC(1) rE(1)]);
        set(h_CE,'YData',[rC(2) rE(2)]);
        
        set(h_OF,'XData',[rO(1) rF(1)]);
        set(h_OF,'YData',[rO(2) rF(2)]);
        
        set(h_body,'XData',[rO(1) rO(1)]);
        set(h_body,'YData',[rO(2)+0.05 rO(2)]);
        
        hold on
        
        if mod(i,50) == 0
            hold on
            plot()

        end

        pause(interval) % pause .01
    end