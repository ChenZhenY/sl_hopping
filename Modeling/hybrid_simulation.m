function [tout, zout, uout, indices] = hybrid_simulation(z0,ctrlpts,p,tspan)
% Model hopping with swinging pendulum leg and fixed rotation
%Inputs:
% z0 - the initial state
% ctrl- control structure
% p - simulation parameters
% tspan - [t0 tf] where t0 is start time, tf is end time
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
% -1: illegal configuration

    restitution_coeff = 0.;
    friction_coeff = 10; % 0.3 and 10
    ground_height = p(end-1);
    tend = p(end);
    ctrl.T = ctrlpts; % control points + tf
    ctrl.tf = tend;
    
    t0 = tspan(1); %tend = tspan(end);   % set initial and final times
    dt = 0.001;
    num_step = floor((tend-t0)/dt);
    tout = linspace(t0, tend, num_step);
    zout(:,1) = z0;
    uout = zeros(3,1);
    iphase_list = 1;
    for i = 1:num_step-1
        t = tout(i);
        
        iphase = iphase_list(i);
        [dz, u] = dynamics_continuous(t,zout(:,i),ctrl,p,iphase);
        zout(:,i+1) = zout(:,i) + dz*dt;
        [zout(6:10,i+1), slip] = discrete_impact_contact(zout(:,i+1), p, restitution_coeff, friction_coeff, ground_height);
        zout(1:5,i+1) = zout(1:5,i) + zout(6:10, i+1)*dt;
        uout(:,i+1) = u; 
        
        % do not let anything except the hopping for touch the ground
        pos = position_foot(zout(:, i+1),p);
        Cy = pos(2) - ground_height;
        
        if slip % hopping leg slipped
            iphase = -1;
        elseif(Cy > 0 && iphase == 1) % switch to jump
            iphase = 2;
        elseif(Cy <= 0 && iphase == 2) % switch to stance
            iphase = 1;
        end
        iphase_list(i+1) = iphase;
    end
    
    j=1; % keeps track of phase changes
    indices = [0;0];
    for i = 1:num_step-1
        if (iphase_list(i+1) == iphase_list(i))
            indices(2, j) = indices(2, j)+1;
        else
            indices(1,j) = iphase_list(i); % grab id of phase that just ended
            j = j+1;
            indices(2,j) = 0;
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
    slip = false;
    
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
            Fcx = fric_coeff*Fcy;
            slip = true;
        end
        if Fcx < -fric_coeff*Fcy
            Fcx = -fric_coeff*Fcy;
            slip = true;
        end
        qdot = q_dot + inv(M)*Jcx'*Fcx;
    else
        qdot = z(6:10);
    end
end

%% Continuous dynamics
function [dz, u] = dynamics_continuous(t,z,ctrl,p,iphase)

    u = control_laws(t,z,ctrl,iphase);  % get controls at this instant
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
function u = control_laws(t,z,ctrl,iphase)

    tf = ctrl.tf;
    ctrlpts = ctrl.T;
    
    if iphase ~= -1
        for i=1:3
            u(i,1) = BezierCurve(ctrlpts(i,:), t/tf);
        end
    else
        % to be fixed
        % PD Control in flight
        th = z(1:3);            % leg angle
        dth = z(6:8);           % leg angular velocity

        thd = pi/4;             % desired leg angle
        k = 5;                  % stiffness (N/rad)
        b = .5;                 % damping (N/(rad/s))

        u = -k*(th-thd) - b*dth;% apply PD control
    end

end