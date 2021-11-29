function [tout, zout, uout, indices, slip_out] = hybrid_simulation_hop(z0,ctrl,p,tspan, option)
% Model hopping with swinging pendulum leg and fixed rotation
%Inputs:
% z0 - the initial state
% ctrl- control structure, with flight time and ctrl pts
% p - simulation parameters
% tspan - [t0 tf] where t0 is start time, tf is end time
% option - struct
%   leg: 1 for hopping leg simulation, 2 for swinging leg
%          (TBD) num_repeat: number of repeat of simulation
%   mid_l: desired SLIP model length of leg at midpoint of flight phase
% 
% Outputs:
% tout - vector of all time points
% zout - vector of all state trajectories
% uout - vector of all control trajectories
% indicies - 2 x phase change matrix of the phase id and the index when
% each phase ended (after the last phase)
%   for instance, tout( indices(1) ) provides the time at which
%   the first phase (stance) ended
% 1: stance
% 2: flight
% slip_out - vector of differences between fx and friction cone limits.
% Positive values indicate slip.

    restitution_coeff = 0.;
    friction_coeff = .5;    % 0.3 and 10
    ground_height = p(end);

    t0 = tspan(1); %tend = tspan(end);   % set initial and final times
    tend = tspan(2);
    dt = 0.001;
    num_step = floor((tend-t0)/dt);
    tout = linspace(t0, tend, num_step);
    zout(:,1) = z0;
    uout = zeros(3,1);
    iphase_list = 1;
    slip_out = 0;
    t_phase_start = 0;
    t_flight = 0; % duration of flight phase from phase start time (relative time)
    the_begin = zeros(3,1); % send the state at phase change (from stance2flight) to control_law

    
    % start simulation yay
    for i = 1:num_step-1
        t_global = tout(i);
        t_relative = t_global - t_phase_start;
        
        iphase = iphase_list(i);
        [dz, u] = dynamics_continuous(t_relative,zout(:,i),ctrl,p,iphase, option, t_flight, the_begin);
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
        indices = 0;
        if(Cy > 0 && iphase == 1)      % switch to jump
            iphase = 2;
            indices = 1;
            the_begin = zout(1:3);
            t_phase_start = t_global;
            disp('iphase == 2');
            disp(t_phase_start);
            % calculate flight time
            g = p(end-1);
            com = COM_jumping_leg(zout(:, i+1),p); %COM position & speed with respect to O
            t_flight = 2*com(4)/g;
        elseif(Cy <= 0 && iphase == 2) % switch to stance
            iphase = 1;
            t_phase_start = t_global;
            disp('iphase == 1');
            disp(t_phase_start);
        end
        
        iphase_list(i+1) = iphase;    
    end % simulation loop
    
    % TODO: fix the indices logic and determine how to use
    j=1;
    indices = 0;
%     for i = 1:num_step-1
%         if (iphase_list(i+1) == iphase_list(i))
%             indices(j) = indices(j)+1;
%         else
%             j = j+1;
%             indices(j) = 0;
%         end
%     end
%     
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
function [dz, u] = dynamics_continuous(t,z,ctrl,p,iphase, option, t_flight, the_begin)

    u = control_laws(t, z, ctrl, iphase, p, option, t_flight, the_begin);  % get 3 motor torque controls at this instant
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
function u = control_laws(t,z,ctrl,iphase, p, option, t_flight, the_begin)
    
    if iphase == 1 % stance
        ctrlpts = ctrl.T;
        u = zeros(3,1);
        if option.leg == 1 % optimizing hopping
            u(1) = BezierCurve(ctrlpts(1,:), t/ctrl.tf);
            u(2) = BezierCurve(ctrlpts(2,:), t/ctrl.tf);
        elseif option.leg == 2 % optimizing swinging
            u(3) = BezierCurve(ctrlpts(3,:), t/ctrl.tf);
        end
    else
        % PD Control in flight
%         global th_begin;
%         if indices == 1
%            th_begin = z(1:3)            % joint angles
%         end
        
%         COM_yvel = z(10) + com(4); %z(10) is y axis velocity of O
        
%         if t > t_flight
%             u = zeros(3,1);
%         else
        %control mid-phase slip length
        %calculate desired joint angle
        t_control = t/t_flight;
        dth = z(6:8);           % joint angular velocities
        th = z(1:3);            % joint angles
        thd = flight_trajectory(the_begin,option.mid_l,t_control);
        if option.leg == 1
            thd = vertcat(thd, [0]);
        end
        k = 5;                  % stiffness (N/rad)
        b = .5;                 % damping (N/(rad/s))
        u = -k*(th-thd) - b*dth;% apply PD control
%         end

    end
end