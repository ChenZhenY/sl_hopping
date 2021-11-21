clear all; close all; clc;
%%

% We can organize our code by filing things in different folders.  These
% folders need to be added to the Matlab path so that it can run the files
% inside them even when they are not the current folder listed at the top
% of the Matlab window.  For more information about the current folder, see
% http://www.mathworks.com/help/matlab/matlab_env/understanding-file-locations-in-matlab.html
% For more information about the Matlab path, see
% http://www.mathworks.com/help/matlab/matlab_env/what-is-the-matlab-search-path.html
setpath                                     % add AutoDerived, Modeling, and Visualization folders to Matlab path

%% Step 0: Setup Casadi
% 1) Download the casadi package for your operating system from https://web.casadi.org/get/
% 2) Unzip the folder, rename it "casadi", and place it in this directory
% 3) Familiarize yourself with casadi by reading (at least) Chapters 1-3, 9
% of https://web.casadi.org/docs/
addpath(genpath('casadi'))

%% Step 1: Setup the Optimization
% Create the optimization object
opti = casadi.Opti();

% Declare Optimization variables
num_ctrlpts = 4;
ctrl.tf = opti.variable();    % Duration of control
ctrl.T  = opti.variable(2, num_ctrlpts); % Control values

% Time discretization
N.ctrl   = 50; % number of dynamics timesteps where ctrl is applied
% only for hopping leg
option = 1;

% Set parameters
init_angle = pi/3;
init_length = 0.18;
[th1, th2] = initial_condition_convert(init_angle, init_length);
z0 = [th1; th2; 0; 0; 0; 0; 0; 0; 0.3; -0.3*1.732];    

p = parameters();                           % get parameters from file
pos_foot0 = position_foot(z0, p);  
ground_height = pos_foot0(2);
p = [p; ground_height];

%% Step 3: Build Objective function
% It is very easy to add an objective function to a casadi optimization.
% Simply use the opti.minimize() function
% Note: Using these function overwrites the last time it was used, so only use it once 
%
% Also note that we are only optimizing over the stance portion (not
% flight), so we set up an equivalent cost function to maximum COM height:
% maximum COM velocity at takeoff
[tout,zout,uout,slip] = stance_simulation_casadi(z0,ctrl,p,N,option);

% Maximize the COM velocity at takeoff
COM = COM_jumping_leg(zout,p); 
opti.minimize(-COM(4,N.ctrl)); % maxmize the y velocity of final time


%% Step 2: Add constraints
% Adding constraints is likewise very simple, just use the
% opti.subject_to() function

% Add lower and upper bounds
opti.subject_to(ctrl.tf >= 0.1);
opti.subject_to(ctrl.tf <= 0.6);
opti.subject_to(ctrl.T(:) >= -2);
opti.subject_to(ctrl.T(:) <= 2 );

% make sure lift off with velocity angle of 45
diff_ang = COM(4, N.ctrl) - 1.732*COM(3, N.ctrl);
opti.subject_to(diff_ang == 0);

% Leg angle must stay within bounds
for i = 1:N.ctrl
    % joint limit constraints
    opti.subject_to(zout(2,i) >= pi/6);
    opti.subject_to(zout(2,i) <= 2*pi/3);
    opti.subject_to(zout(1,i) >= -pi/3);
    opti.subject_to(zout(1,i)+zout(2,i) <= 2*pi/3);
    opti.subject_to(zout(3,i) <= pi/2);
    opti.subject_to(zout(3,i) >= -pi/2);
    
    % TODO: all joints above the ground (not working now)
    sw_pos = position_swinging_foot(zout(:, i),p);
    sw_Cy = sw_pos(2) - ground_height;      % foot height from ground
    k_pos = position_knee(zout(:, i),p);
    k_Cy = k_pos(2) - ground_height;        % knee height from ground
    h_pos = position_hip(zout(:, i),p);
    h_Cy = h_pos(2) - ground_height;
    opti.subject_to(sw_Cy >= 0.0);
    opti.subject_to(k_Cy >= 0.0);
    opti.subject_to(h_Cy >= 0.0);
    
    % always moving forward along x
    if i>1
        opti.subject_to(zout(4,i)-zout(4,i-1) > 0);
    end
    
    % TODO: no slipping, fail in discrete_contact_dynamic
    % opti.subject_to(slip(i) == 0); % no slip
end

% %% 
% figure(4)
% for i = 1:length(t)
% hh_pos = position_hip(z(:, i),p);
% aa(i) = hh_pos(2) - ground_height;
% end
% plot(t,aa)

%% Step 3: Setup the solver options
% (only the ambitious student needs to worry about changing these settings,
% for the most part, you should be able to leave them as is)

p_opts = struct('expand',false);
opti.solver('ipopt',p_opts);

%% Step 4: Provide Initial Guess & Run the optimization
% Setting the initial guess is simple, just use opti.set_initial() for each
% of your optimization variables

% Initial guess
opti.set_initial(ctrl.tf,0.25);
opti.set_initial(ctrl.T,[0. 1.0 1.0 0; 0. 1.0 1.0 0]);

% Solve the Optimization
sol = opti.solve();

%% Step 5: Simulate and Visualize the Result (same as before mostly)
% Parse solution
tf = sol.value(ctrl.tf)+0.3;          % simulation final time, no flight
optimal_ctrl.tf = sol.value(ctrl.tf); % control final time
optimal_ctrl.T  = sol.value(ctrl.T);  % control values

[t z u indices] = hybrid_simulation_hop(z0,optimal_ctrl,p,[0 tf],option); % run simulation

figure(1)
COM = COM_jumping_leg(z,p);
subplot(1,2,1)
plot(t,COM(1,:))
xlabel('time (s)')
ylabel('CoM x (m)')
title('Center of Mass Trajectory')
subplot(1,2,2)
plot(t,COM(4,:))
xlabel('time (s)')
ylabel('CoM Vertical Velocity (m/s)')
title('Center of Mass Vel. Trajectory')

% figure(2)  % control input profile
% ctrl_t = linspace(0, optimal_ctrl.tf, 50);
% ctrl_pt_t = linspace(0, optimal_ctrl.tf, length(optimal_ctrl.T));
% 
% for i=1:length(ctrl_t)
%     ctrl_input(i) = BezierCurve(optimal_ctrl.T,ctrl_t(i)/optimal_ctrl.tf);
% end
% hold on
% plot(ctrl_t, ctrl_input);
% plot(ctrl_pt_t, optimal_ctrl.T, 'o');
% hold off
% xlabel('time (s)')
% ylabel('torque (Nm)')
% title('Control Input Trajectory')
% axis([0 0.45 0 2]);

%%
% Run the animation
figure(3)                          % get the coordinates of the points to animate
speed = .25;                                 % set animation speed
cla                                         % clear axes
animate_hop(t,z,p)                 % run animation
