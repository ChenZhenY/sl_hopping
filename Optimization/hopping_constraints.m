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

    tf = p(end);
    ctrl = x;
    [tout, zout, uout, indices] = hybrid_simulation(z0, ctrl, p, [0 tf]);
    theta1 = zout(1,:); 
    theta2 = zout(2, :);
    COM = COM_jumping_leg(zout,p);
    violation_count = sum(indices(1,:) == -1);
    
    cineq = [-min(theta2) + pi/6;... % leg geometry constraints
        max(theta1 + theta2)-2*pi/3;... 
        max(theta2) - 2*pi/3;...
        -min(theta1) - pi/3;...
        % violation_count;...        % indices checks for non-foot parts hitting ground, slip
        -(zout(4,end) - zout(4,1) - .1) ]; % leg moves forward in x direction
    
    ceq = [min(zout(3,:)) - max(zout(3,:))]; % swing leg angle stays fixed
    
end