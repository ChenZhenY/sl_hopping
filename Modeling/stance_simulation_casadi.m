function [tout,zout,uout,slip] = stance_simulation_casadi(z0,ctrl,p,N,option)
% simulate stance phase for hop leg or swing leg
%Inputs:
% z0 - the initial state
% ctrl- control structure, which will be optimized
% p - simulation parameters
% N - number of dynamics timesteps where ctrl is applied
% option - 1 for hopping leg simulation, 2 for swinging leg
    
    % simulation parameters
    restitution_coeff = 0.;
    friction_coeff = 1.5;    % 0.3 and 10
    ground_height = p(end);

    % Declare casadi symbolic variables
    tout = casadi.MX(zeros(1,N.ctrl));
    zout = casadi.MX(zeros(10,N.ctrl));
    uout = casadi.MX(zeros(3,N.ctrl-1));
    slip = casadi.MX(zeros(1,N.ctrl-1));

    % Time discretization
    tout(1) = 0;
    dt = ctrl.tf/(N.ctrl-1);
    for i = 2:N.ctrl
        tout(i) = tout(i-1) + dt;
    end
 
    % Simulate
    zout(:,1) = z0;
    for i = 1:N.ctrl-1
        t = tout(i);
        % Dynamics timestep
        [dz, u] = dynamics_continuous(t,zout(:,i),ctrl,p,option);
        zout(:,i+1) = zout(:,i) + dz*dt;
        [zout(6:10,i+1), slip(i)] = discrete_impact_contact(zout(:,i+1), p, restitution_coeff,...
            friction_coeff, ground_height);
        zout(1:5,i+1) = zout(1:5,i) + zout(6:10, i+1)*dt;
        % Log control input
        uout(:,i) = u; 
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
    slip = abs(Fcx) - fric_coeff*Fcy; 
%     Fcx = min(max(Fcx, - fric_coeff*Fcy), fric_coeff*Fcy);
    qdot = q_dot + inv(M)*Jcx'*Fcx;
 
end

%% Continuous dynamics
function [dz, u] = dynamics_continuous(t,z,ctrl,p,option)

    u = control_laws(t,ctrl,option);  % get controls at this instant

    Fc = casadi.MX(zeros(3,1));
    A = A_jumping_leg(z,p);      % get full A matrix
    b = b_jumping_leg(z,u,Fc,p);  % get full b vector
   
    x = A\b;                     % solve system for accelerations (and possibly forces)

    dz = casadi.MX(zeros(10,1));
    dz(1:5,1) = z(6:10);   % assign velocities to time derivative of state vector
    dz(6:10,1) = x(1:5);   % assign accelerations to time derivative of state vector
end

%% Control
function u = control_laws(t,ctrl,option)

    ctrlpts = ctrl.T;
    u = casadi.MX(zeros(3,1));
    if option == 1 % optimizing hopping
        u(1) = BezierCurve(ctrlpts(1,:), t/ctrl.tf);
        u(2) = BezierCurve(ctrlpts(2,:), t/ctrl.tf);
    elseif option == 2 % optimizing swinging
        u(3) = BezierCurve(ctrlpts(3,:), t/ctrl.tf);
    end
    
end
