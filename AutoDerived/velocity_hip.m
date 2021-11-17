function drB = velocity_hip(in1,in2)
%VELOCITY_HIP
%    DRB = VELOCITY_HIP(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.7.
%    16-Nov-2021 21:14:02

dth1 = in1(6,:);
dx = in1(9,:);
dy = in1(10,:);
l_OB = in2(19,:);
th1 = in1(1,:);
drB = [dx+dth1.*l_OB.*cos(th1);dy+dth1.*l_OB.*sin(th1);0.0];
