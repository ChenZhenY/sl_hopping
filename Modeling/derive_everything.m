function derive_everything() 
name = 'jumping_leg';

% Define variables for time, generalized coordinates + derivatives, controls, and parameters
% new: th3 for swing leg; x y for torso position; m5, I5 for swing leg
syms t th1 th2 th3 dth1 dth2 dth3 ddth1 ddth2 ddth3 x dx ddx y dy ddy real
syms m1 m2 m3 m4 m5 I1 I2 I3 I4 I5 l_O_m1 l_B_m2 l_A_m3 l_C_m4 l_O_m5 g real
syms l_OA l_OB l_AC l_DE real 
syms tau1 tau2 tau3 Fx Fy real
syms Ir N real

% Group them
q   = [th1  ; th2  ; th3  ; x  ; y  ];      % generalized coordinates
dq  = [dth1 ; dth2 ; dth3 ; dx ; dy ];    % first time derivatives
ddq = [ddth1;ddth2 ; ddth3; ddx; ddy];  % second time derivatives
u   = [tau1 ; tau2 ; tau3];     % controls
Fc  = [Fx ; Fy];

p   = [m1 m2 m3 m4 m5 I1 I2 I3 I4 I5 Ir N l_O_m1 l_B_m2 l_A_m3 l_C_m4 l_O_m5 l_OA l_OB l_AC l_DE g]';        % parameters

% Generate Vectors and Derivativess
ihat = [0; -1; 0];
jhat = [1; 0; 0];
khat = cross(ihat,jhat); % 0 0 1

xhat = [1; 0; 0];
yhat = [0; 1; 0];

e1hat =  cos(th1)*ihat + sin(th1)*jhat;
e2hat =  cos(th1+th2)*ihat + sin(th1+th2)*jhat;
e3hat =  cos(th3)*ihat + sin(th3)*jhat;

ddt = @(r) jacobian(r,[q;dq])*[dq;ddq]; % a handy anonymous function for taking time derivatives

rO = x*xhat + y*yhat;
rA = rO + l_OA * e1hat;
rB = rO + l_OB * e1hat;
rC = rA  + l_AC * e2hat;
rD = rB  + l_AC * e2hat;
rE = rD  + l_DE * e1hat;

r_m1 = rO + l_O_m1 * e1hat;
r_m2 = rB + l_B_m2 * e2hat;
r_m3 = rA + l_A_m3 * e2hat;
r_m4 = rC + l_C_m4 * e1hat;
r_m5 = rO + l_O_m5 * e3hat;

drA = ddt(rA);
drB = ddt(rB);
drC = ddt(rC);
drD = ddt(rD);
drE = ddt(rE);

dr_m1 = ddt(r_m1);
dr_m2 = ddt(r_m2);
dr_m3 = ddt(r_m3);
dr_m4 = ddt(r_m4);
dr_m5 = ddt(r_m5);

% Calculate Kinetic Energy, Potential Energy, and Generalized Forces
F2Q = @(F,r) simplify(jacobian(r,q)'*(F));    % force contributions to generalized forces
M2Q = @(M,w) simplify(jacobian(w,dq)'*(M));   % moment contributions to generalized forces

omega1 = dth1;
omega2 = dth1 + dth2;
omega3 = dth1 + dth2;
omega4 = dth1;
omega5 = dth3;

T1 = (1/2)*m1 * dot(dr_m1,dr_m1) + (1/2) * I1 * omega1^2;
T2 = (1/2)*m2 * dot(dr_m2,dr_m2) + (1/2) * I2 * omega2^2;
T3 = (1/2)*m3 * dot(dr_m3,dr_m3) + (1/2) * I3 * omega3^2;
T4 = (1/2)*m4 * dot(dr_m4,dr_m4) + (1/2) * I4 * omega4^2;
T5 = (1/2)*m5 * dot(dr_m5,dr_m5) + (1/2) * I5 * omega5^2;
T1r = (1/2)*Ir*(N*dth1)^2;
T2r = (1/2)*Ir*(dth1 + N*dth2)^2;
T3r = (1/2)*Ir*(N*dth3)^2;
T = simplify(T1 + T2 + T3 + T4 + T5 + T1r + T2r + T3r);

% kinetic energy not influenced by the ref frame, but potential energy does
Vg1 = m1*g*dot(r_m1, -ihat);
Vg2 = m2*g*dot(r_m2, -ihat);
Vg3 = m3*g*dot(r_m3, -ihat);
Vg4 = m4*g*dot(r_m4, -ihat);
Vg5 = m5*g*dot(r_m5, -ihat);
Vg = Vg1 + Vg2 + Vg3 + Vg4 + Vg5;

Q_tau1 = M2Q(tau1*khat,omega1*khat);
Q_tau2 = M2Q(tau2*khat,omega2*khat); 
Q_tau2R= M2Q(-tau2*khat,omega1*khat); % internal moment
Q_tau3 = M2Q(tau3*khat, omega5*khat);
Q_tau = Q_tau1 + Q_tau2 + Q_tau2R + Q_tau3;

Q_fx = F2Q(Fx*xhat, rE);
Q_fy = F2Q(Fy*yhat, rE);
Q_f = Q_fx+Q_fy;

Q = Q_tau + Q_f;

% Assemble the array of cartesian coordinates of the key points
keypoints = [rA(1:2) rB(1:2) rC(1:2) rD(1:2) rE(1:2) rO(1:2) r_m5(1:2)];

%% All the work is done!  Just turn the crank...
% Derive Energy Function and Equations of Motion
E = T+Vg;
L = T-Vg;
eom = ddt(jacobian(L,dq).') - jacobian(L,q).' - Q;
% Rearrange Equations of Motion
A = jacobian(eom,ddq);
b = A*ddq - eom;

% Equations of motion are
% eom = A *ddq + (coriolis term) + (gravitational term) - Q = 0
Mass_Joint_Sp = A;
Grav_Joint_Sp = simplify(jacobian(Vg, q)');
Corr_Joint_Sp = simplify( eom + Q - Grav_Joint_Sp - A*ddq);

% Compute foot jacobian
J = jacobian(rE,q);

% Compute ddt( J )
dJ= reshape(ddt(J(:)) , size(J) );

% Write Energy Function and Equations of Motion
z  = [q ; dq];

% Calculate rcm, the location of the center of mass
rcm = (m1*r_m1 + m2*r_m2 + m3*r_m3 + m4*r_m4 + m5*r_m5)/(m1+m2+m3+m4+m5);

% Write functions to a separate folder because we don't usually have to see them
directory = '../AutoDerived/';
% Write a function to evaluate the energy of the system given the current state and parameters
matlabFunction(E,'file',[directory 'energy_' name],'vars',{z p});
% Write a function to evaluate the A matrix of the system given the current state and parameters
matlabFunction(A,'file',[directory 'A_' name],'vars',{z p});
% Write a function to evaluate the b vector of the system given the current state, current control, and parameters
matlabFunction(b,'file',[directory 'b_' name],'vars',{z u Fc p});

matlabFunction(keypoints,'file',[directory 'keypoints_' name],'vars',{z p});

% Write a function to denote all the state needed by the contact foot and
% leg necessary for contact
matlabFunction(rE,'file',[directory 'position_foot'],'vars',{z p});
matlabFunction(drE,'file',[directory 'velocity_foot'],'vars',{z p});
matlabFunction(r_m5,'file',[directory 'position_swinging_foot'],'vars',{z p});
matlabFunction(dr_m5,'file',[directory 'velocity_swinging_foot'],'vars',{z p});
matlabFunction(rC,'file',[directory 'position_knee'],'vars',{z p});
matlabFunction(drC,'file',[directory 'velocity_knee'],'vars',{z p});
matlabFunction(rB,'file',[directory 'position_hip'],'vars',{z p});
matlabFunction(drB,'file',[directory 'velocity_hip'],'vars',{z p});
matlabFunction(J ,'file',[directory 'jacobian_foot'],'vars',{z p});
matlabFunction(dJ ,'file',[directory 'jacobian_dot_foot'],'vars',{z p});

matlabFunction(Grav_Joint_Sp ,'file', [directory 'Grav_leg'] ,'vars',{z p});
matlabFunction(Corr_Joint_Sp ,'file', [directory 'Corr_leg'] ,'vars',{z p});

% Write a function to evaluate the X and Y coordinates and speeds of the center of mass given the current state and parameters
drcm = ddt(rcm);             % Calculate center of mass velocity vector
COM = [rcm(1:2); drcm(1:2)]; % Concatenate x and y coordinates and speeds of center of mass in array
matlabFunction(COM,'file',[directory 'COM_' name],'vars',{z p});
