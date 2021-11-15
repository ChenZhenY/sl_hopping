clear all; close all; clc;

% 2 parts:
% - Fix swing leg and optimize over hopping leg control.
% - Given hopping leg control, optimize over swing leg control.

% We can organize our code by filing things in different folders.  These
% folders need to be added to the Matlab path so that it can run the files
% inside them even when they are not the current folder listed at the top
% of the Matlab window.  For more information about the current folder, see
% http://www.mathworks.com/help/matlab/matlab_env/understanding-file-locations-in-matlab.html
% For more information about the Matlab path, see
% http://www.mathworks.com/help/matlab/matlab_env/what-is-the-matlab-search-path.html
setpath                                     % add AutoDerived, Modeling, and Visualization folders to Matlab path

% set to run optimization over hopping leg or over swing leg
run_hopping = true;

z0 = [-pi/4; pi/2; 0; 0; 0; 0; 0; 0; 0; 0];    
p = parameters();                           % get parameters from file
pos_foot0 = position_foot(z0, p);  
ground_height = pos_foot0(2);
tf = 0.6;                                   %simulation time
p = [p; ground_height; tf];

% An equation has been added to dynamics_continuous and dynamics_discrete
% to integrate this new state.

%__________________________ run hopping leg_________________________________________
if run_hopping
    
%     ctrl_rd = rand(2,4)*2;
    ctrl = [-.5 .5 0 0.0 ; 1 .5 0 0];                     % control values
    %ctrl(1:3,1:3) = 0;
                                            % one row for one motor control points
% optimization start                                            
    x = ctrl;
    % % setup and solve nonlinear programming problem
    problem.objective = @(x) hopping_objective(x,z0,p);     % create anonymous function that returns objective
    problem.nonlcon = @(x) hopping_constraints(x,z0,p);     % create anonymous function that returns nonlinear constraints
    problem.x0 = [ctrl];                   % initial guess for decision variables
    problem.lb = [-2*ones(size(ctrl))];     % lower bound on decision variables
    problem.ub = [2*ones(size(ctrl))];     % upper bound on decision variables
    problem.Aineq = []; problem.bineq = [];         % no linear inequality constraints
    problem.Aeq = []; problem.beq = [];             % no linear equality constraints
    problem.options = optimset('Display','iter');   % set options
    problem.solver = 'fmincon';                     % required
    x = fmincon(problem);                           % solve nonlinear programming problem

% optimization end 
    ctrl = x
    [t, z, u, indices] = hybrid_simulation(z0,ctrl,p,[0 tf], 1); % run simulation

%_______________________________ run swinging leg_____________________________
else
    % set guess
    tf = 1.0;     % 0.5                                   % simulation final time
    ctrl.tf = .4;    % 0.35                              % control time points
    ctrl.T = [1.0 1.0 1.0];                               % control values

    x = ctrl.T;
    % % % setup and solve nonlinear programming problem
    % problem.objective = @(x) swinging_objective(x,z0,p);     % create anonymous function that returns objective
    % problem.nonlcon = @(x) swinging_constraints(x,z0,p);     % create anonymous function that returns nonlinear constraints
    % TODO: input precomputed hopping leg control
    % problem.x0 = [ctrl.T];                   % initial guess for decision variables
    % problem.lb = [-2*ones(size(ctrl.T))];     % lower bound on decision variables
    % problem.ub = [2*ones(size(ctrl.T))];     % upper bound on decision variables
    % problem.Aineq = []; problem.bineq = [];         % no linear inequality constraints
    % problem.Aeq = []; problem.beq = [];             % no linear equality constraints
    % problem.options = optimset('Display','iter');   % set options
    % problem.solver = 'fmincon';                     % required
    % x = fmincon(problem);                           % solve nonlinear programming problem
    % 
    % % Note that once you've solved the optimization problem, you'll need to 
    % % re-define tf, tfc, and ctrl here to reflect your solution.
    % tf = x(1);
    % ctrl.tf = x(2);
    % ctrl.T = x(3:end)

    [t, z, u, indices] = hybrid_simulation(z0,ctrl,p,[0 tf], 2); % run simulation
end

%% Plot COM for your submissions
figure(1)
COM = COM_jumping_leg(z,p);
plot(t,COM(2,:))
xlabel('time (s)')
ylabel('CoM Height (m)')
title('Center of Mass Trajectory')

figure(2)
plot(t,COM(1,:))
xlabel('time (s)')
ylabel('CoM Horizontal (m)')
title('Center of Mass Trajectory')

% figure(2)  % control input profile
% ctrl_t = linspace(0, ctrl.tf, 50);
% ctrl_pt_t = linspace(0, ctrl.tf, length(ctrl.T));
% n = length(ctrl_t);
% ctrl_input = zeros(1,n);

% for i=1:n
%     ctrl_input(i) = BezierCurve(x(3:end),ctrl_t(i)/ctrl.tf);
% end
% 
% hold on
% plot(ctrl_t, ctrl_input);
% plot(ctrl_pt_t, x(3:end), 'o');
% hold off
% xlabel('time (s)')
% ylabel('torque (Nm)')
% title('Control Input Trajectory')

%%
% Run the animation
figure(3)                          % get the coordinates of the points to animate
speed = .25;                                 % set animation speed
clf                                         % clear fig
% animate_simple(t,z,p,speed)                 % run animation
animate(t, z, p)
