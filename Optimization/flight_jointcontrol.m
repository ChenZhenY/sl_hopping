function [thd_2]= flight_jointcontrol(th,mid_l,t)
% u_joint  2x1 each ro


[init_angle,slip_l] = joint2slip(th);

%initial joint angle
[th1_init th2_init] = initial_condition_convert(init_angle, slip_l);
%mid-phase joint angle
[th1_mid th2_mid] = initial_condition_convert(pi/2, mid_l);
%land-phase joint angle
[th1_end th2_end] = initial_condition_convert(pi-init_angle, slip_l);

joint1_ctrlpts=[th1_init,th1_mid,th1_end];
joint2_ctrlpts=[th2_init,th2_mid,th2_end];

thd_2(1,1) = BezierCurve(joint1_ctrlpts(i,:), t);
thd_2(2,1) = BezierCurve(joint1_ctrlpts(i,:), t);

end
