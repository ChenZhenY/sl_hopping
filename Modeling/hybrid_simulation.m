function [tout, zout, uout, indices, slip_out] = hybrid_simulation(z0,ctrlpts,p,tspan, option)
% Model hopping with swinging pendulum leg and fixed rotation
%Inputs:
% z0 - the initial state
% ctrl- control structure
% p - simulation parameters
% tspan - [t0 tf] where t0 is start time, tf is end time
% option - 1 for hopping leg simulation, 2 for swinging leg
% 
% Outputs:
% tout - vector of all time points
% zout - vector of all state trajectories
% uout - vector of all control trajectories
% indicies - 2 x phase change matrix of the phase id and the index when
% each phase ended (after the last phase)
%   for instance, tout( indices(1) ) provides the time at which
%   the first phase (stance) ended
% 0: stance
% 1: flight
% slip_out - vector of differences between fx and friction cone limits.
% Positive values indicate slip.

    restitution_coeff = 0.;
    friction_coeff = .5;    % 0.3 and 10
    ground_height = p(end-1);
    tend = p(end);          %simulation time
    ctrl.T = ctrlpts;       % control points + tf
    ctrl.tf = tend;         % control time period(decide if throughout simulation)
    ctrl.num = 2;           % control repeating number: how many repeat curves happen within simulation
    
    t0 = tspan(1); %tend = tspan(end);   % set initial and final times
    dt = 0.001;
    num_step = floor((tend-t0)/dt);
    tout = linspace(t0, tend, num_step);
    zout(:,1) = z0;
    uout = zeros(3,1);
    iphase_list = 1;
    slip_out = 0;
    
    % start simulation yay
    for i = 1:num_step-1
        t = tout(i);
        
        iphase = iphase_list(i);
        [dz, u] = dynamics_continuous(t,zout(:,i),ctrl,p,iphase, option);
        zout(:,i+1) = zout(:,i) + dz*dt; % continuous dynamic determine status for contact determination

        [zout(6:10,i+1), slip] = discrete_impact_contact(zout(:,i+1),p, restitution_coeff,...
            friction_coeff, ground_height); %determine contact or not
        
        zout(1:5,i+1) = zout(1:5,i) + zout(6:10, i+1)*dt;% use velocity to update position
        uout(:,i+1) = u; 
        slip_out(i+1) = slip;
        
        % do not let anything except the hopping for touch the ground
        pos = position_foot(zout(:, i+1),p);
        Cy = pos(2) - ground_height;
       
        % determine phase
        if(Cy > 0 && iphase == 1)      % switch to jump
            iphase = 2;
        elseif(Cy <= 0 && iphase == 2) % switch to stance
            iphase = 1;
        end
        
        iphase_list(i+1) = iphase;    
    end % simulation loop
    
    % TODO: fix the indices logic and determine how to use
    j=1;
    indices = 0;
    for i = 1:num_step-1
        if (iphase_list(i+1) == iphase_list(i))
            indices(j) = indices(j)+1;
        else
            j = j+1;
            indices(j) = 0;
        end
    end
    
end

%% Discrete Contact
function [qdot, slip] = discrete_impact_contact(z,p,rest_coeff, fric_coeff, yC)
    % compute height of the foot
    pos = position_foot(z,p);
    vel = velocity_foot(z,p);
    Cy = pos(2)-yC;
    dCy = vel(2);
    slip = 0; % difference between required horizontal force and closest friction cone bound (>0 means slip)
    
    % only update dq when constraints violated
    if Cy<0 && dCy<0 % hopping foot
        J = jacobian_foot(z,p);
        J = J(1:2,:);
        Jcx = J(1,:);
        Jcy = J(2,:);
        M = A_jumping_leg(z,p);           % mass matrix in configuration space
        A = inv(J*inv(M)*J');     % mass matrix in operational space
        Acy = A(2,2);
        Acx = A(1,1);
        dq = z(6:10);
        % update vertical force
        Fcy = Acy*(-rest_coeff*dCy-Jcy*dq);
        q_dot = dq + inv(M)*Jcy'*Fcy;
        % update horizontal force
        Fcx = Acx*(0-Jcx*q_dot);
        if Fcx > fric_coeff*Fcy
            slip = Fcx - fric_coeff*Fcy;
            Fcx = fric_coeff*Fcy;
        end
        if Fcx < -fric_coeff*Fcy
            slip = Fcx - fric_coeff*Fcy;
            Fcx = -fric_coeff*Fcy;
        end
        qdot = q_dot + inv(M)*Jcx'*Fcx;
    else
        qdot = z(6:10);
    end
end

%% Continuous dynamics
function [dz, u] = dynamics_continuous(t,z,ctrl,p,iphase, option)

    u = control_laws(t,z,ctrl,iphase, option);  % get 3 motor torque controls at this instant
    % TODO: need a test strategy
    % TODO: we don't handle constraints Fc here?
    % u = [0; 0; 0];
    Fc = [0;0;0];
    
    A = A_jumping_leg(z,p);                 % get full A matrix
    b = b_jumping_leg(z,u,Fc,p);               % get full b vector
    
    x = A\b;                % solve system for accelerations (and possibly forces)
    dz(1:5,1) = z(6:10);   % assign velocities to time derivative of state vector
    dz(6:10,1) = x(1:5);   % assign accelerations to time derivative of state vector
end

%% Control law of BezierCurve SISO
% TODO: to be changed for three variables control
function u = control_laws(t,z,ctrl,iphase, option)
    tf = ctrl.tf;
    ctrlpts = ctrl.T;
    repeat_num = ctrl.num;
    if option == 1 % optimizing hopping
        motors = 1:2;
    elseif option == 2 % optimizing swinging
        motors = 3;
    end
    
    if iphase ~= -1  
        for i=motors
            
            timeslice = floor(tf/repeat_num);
            t_control = mod(t,timeslice)/tf;
            u(i,1) = BezierCurve(ctrlpts(i,:), t_control);% control at t instant, input is t/tf due to normalization demand within Bezier function
            %repeat control curve
            
        end
    else
        % to be fixed
        % PD Control in flight
        th = z(1:3);            % leg angle
        dth = z(6:8);           % leg angular velocity
        com = COM_jumping_leg(z,p); %COM position&speed with respect to O
        
        COM_yvel = z(10) + com(4); %z(10) is y axis velocity of O
        %flight time
        t_fly = 2*COM_yvel/g;
        t_control = t/t_fly;
        %control mid-phase slip length
        mid_l = 0.12;  
        %calculate desired joint angle
        thd = flight_jointcontrol(th,mid_l,t_control)
        %thd = pi/4;             % desired leg angle
        k = 5;                  % stiffness (N/rad)
        b = .5;                 % damping (N/(rad/s))

        u = -k*(th-thd) - b*dth;% apply PD control
    end

    if size(u,1) == 2
        u = vertcat(u,zeros(1,size(u,2)));
    end
end