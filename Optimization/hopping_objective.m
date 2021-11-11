function f = hopping_objective(x,z0,p)
% Inputs:
% x - an array of decision variables.
% z0 - the initial state
% p - simulation parameters
% 
% Outputs:
% f - scalar value of the function (to be minimized) evaluated for the
%     provided values of the decision variables.
%
% Note: fmincon() requires a handle to an objective function that accepts 
% exactly one input, the decision variables 'x', and returns exactly one 
% output, the objective function value 'f'.  It is convenient for this 
% assignment to write an objective function which also accepts z0 and p 
% (because they will be needed to evaluate the objective function).  
% However, fmincon() will only pass in x; z0 and p will have to be
% provided using an anonymous function, just as we use anonymous
% functions with ode45().

    % numerically integrate to the final height
    tf = p(end);
    ctrl = x;
    [tout, zout, uout, indices, slip_out] = hybrid_simulation(z0, ctrl, p, [0 tf]);
    COM = COM_jumping_leg(zout,p);
    % f = -COM_end;                                           % negative of COM height
    % f = -max(COM(2,:));
    
    
%    t0 = 0; tend = tf;   % set initial and final times
%    dt = 0.001;
%    num_step = floor((tend-t0)/dt);
%     torque = zeros(1, num_step);
%     for i = 1:num_step-1
%         torque(i) = BezierCurve(ctrl.T, i*dt/ctrl.tf);
%     end
%     work = torque*torque.';
%     f = work;                                         % minimize T^2 integral
f = -(zout(5,end) - zout(5,1));
    
end