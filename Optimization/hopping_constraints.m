function [cineq ceq] = hopping_constraints(x,z0,p)
% Inputs:
% x - an array of decision variables.
% z0 - the initial state
% p - simulation parameters
% 
% Outputs:
% cineq - an array of values of nonlinear inequality constraint functions.  
%         The constraints are satisfied when these values are less than zero.
% ceq   - an array of values of nonlinear equality constraint functions.
%         The constraints are satisfied when these values are equal to zero.
%
% Note: fmincon() requires a handle to an constraint function that accepts 
% exactly one input, the decision variables 'x', and returns exactly two 
% outputs, the values of the inequality constraint functions 'cineq' and
% the values of the equality constraint functions 'ceq'. It is convenient 
% in this case to write an objective function which also accepts z0 and p 
% (because they will be needed to evaluate the objective function).  
% However, fmincon() will only pass in x; z0 and p will have to be
% provided using an anonymous function, just as we use anonymous
% functions with ode45().

% todo: add slip constraints

    tf = p(end);
    ctrl = x;
    [tout, zout, uout, indices, slip_out] = hybrid_simulation(z0, ctrl, p, [0 tf]);
    theta1 = zout(1,:); 
    theta2 = zout(2, :);
    COM = COM_jumping_leg(zout,p);
    ground_height = p(end-1);
    
    % check if anything other than the hopping foot touches the ground

    sw_Cy = zeros(numel(tout),1); k_Cy = zeros(numel(tout),1); h_Cy = zeros(numel(tout),1);
    for i = 1:numel(tout)
        sw_pos = position_swinging_foot(zout(:, i),p);
        sw_Cy(i) = sw_pos(2) - ground_height;      % foot height from ground
        k_pos = position_knee(zout(:, i),p);
        k_Cy(i) = k_pos(2) - ground_height;        % knee height from ground
        h_pos = position_hip(zout(:, i),p);
        h_Cy(i) = h_pos(2) - ground_height;        % hip height from ground
    end
    
    cineq = [-min(theta2) + pi/6;... % joint limits
        max(theta1 + theta2)-2*pi/3;...
        max(theta2) - 2*pi/3;...
        -min(theta1) - pi/3;...
        -min(sw_Cy); ...            % swinging leg does not contact floor
        -min(k_Cy); ...             % knee does not contact floor
        -min(h_Cy);...              % hip does not contact floor
        max(slip_out);...           % foot does not slip
        -(zout(4,end) - zout(4,1) - .1) ]; % leg moves forward in x direction
    
    ceq = [min(zout(3,:)) - max(zout(3,:))]; % swing leg angle stays fixed
    
end