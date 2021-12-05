function [thd_2]= flight_trajectory_linear(th,mid_l,t)
% Evaluates th1_d, th2_d at flight trajectory given initial joint angles
% using linear interpolation
% th = [th1, th1] and mid_l at time t
% u_joint  2x1 each row
% mid_l: slip leg len at mid point

[init_angle,slip_l] = joint2slip(th); % current slip model para
%initial joint angle
[th1_init th2_init] = initial_condition_convert(init_angle, slip_l);
% [th1_init th2_init] = initial_condition_convert(pi/3, 0.18);
%intermediate joint angle
[th1_mid th2_mid] = initial_condition_convert(pi/2, mid_l);
%land-phase joint angle
[th1_end th2_end] = initial_condition_convert(pi-init_angle, slip_l);
% [th1_end th2_end] = initial_condition_convert(2*pi/3, 0.18);
joint1_ctrlpts=[th1_init,th1_mid,th1_end];
joint2_ctrlpts=[th2_init,th2_mid,th2_end];
times = [0 .5 1];
thd_2(1,1) = interp1(times, joint1_ctrlpts, t);
thd_2(2,1) = interp1(times, joint2_ctrlpts, t);
end