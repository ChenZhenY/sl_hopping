function dr_m5 = velocity_swinging_foot(in1,in2)
%VELOCITY_SWINGING_FOOT
%    DR_M5 = VELOCITY_SWINGING_FOOT(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.7.
%    16-Nov-2021 21:14:01

dth3 = in1(8,:);
dx = in1(9,:);
dy = in1(10,:);
l_O_m5 = in2(17,:);
th3 = in1(3,:);
dr_m5 = [dx+dth3.*l_O_m5.*cos(th3);dy+dth3.*l_O_m5.*sin(th3);0.0];
