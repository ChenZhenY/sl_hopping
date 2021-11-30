function [thd_2]= flight_trajectory(th,mid_l,t)
% Evaluates th1_d, th2_d at flight trajectory given initial joint angles
% th = [th1, th1] and mid_l at time t
% u_joint  2x1 each row
% mid_l: slip leg len at mid point

[init_angle,slip_l] = joint2slip(th); % current slip model para
%initial joint angle

if t > 1 % avoid error in the flight time inaccuracy
    t = 1
end

%% SLIP SPACE BEZIER TRAJECTORIES
slip_bezier_angle = BezierCurve([init_angle, pi/2, pi - init_angle], t);
slip_bezier_l = BezierCurve([slip_l, mid_l, slip_l], t);
% plot([0 cos(slip_bezier_angle)*slip_bezier_l], [0, -sin(slip_bezier_angle)*slip_bezier_l]);
% hold on

[thd_2(1,1), thd_2(2,1)] = initial_condition_convert(slip_bezier_angle, slip_bezier_l);

p = parameters();
foot = position_foot(vertcat(thd_2(1,1), thd_2(2,1), zeros(8,1)), p);
plot([0, foot(1)], [0, foot(2)]);
hold on

%% JOINT SPACE BEZIER TRAJECTORIES
% [th1_init th2_init] = initial_condition_convert(init_angle, slip_l);
% % [th1_init th2_init] = initial_condition_convert(pi/3, 0.18);
% %intermediate joint angle
% % [th1_mid th2_mid] = initial_condition_convert((pi  )/2, mid_l);
% %apex joint angle
% [th1_mid th2_mid] = initial_condition_convert(pi/2-init_angle/2, mid_l);
% %land-phase joint angle
% [th1_end th2_end] = initial_condition_convert(pi-init_angle, slip_l);
% [th1_end th2_end] = initial_condition_convert(2*pi/3, 0.18);
% joint1_ctrlpts=[th1_init,th1_end];
% joint2_ctrlpts=[th2_init,th2_end];
% thd_2(1,1) = BezierCurve(joint1_ctrlpts, t);
% thd_2(2,1) = BezierCurve(joint2_ctrlpts, t);
end