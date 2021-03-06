function dJ = jacobian_dot_foot(in1,in2)
%JACOBIAN_DOT_FOOT
%    DJ = JACOBIAN_DOT_FOOT(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.7.
%    07-Nov-2021 00:16:32

dth1 = in1(6,:);
dth2 = in1(7,:);
l_AC = in2(20,:);
l_DE = in2(21,:);
l_OB = in2(19,:);
th1 = in1(1,:);
th2 = in1(2,:);
t2 = cos(th1);
t3 = sin(th1);
t4 = th1+th2;
t5 = cos(t4);
t6 = sin(t4);
t7 = dth2.*l_AC.*t5;
t8 = dth2.*l_AC.*t6;
t9 = -t8;
dJ = reshape([t9-dth1.*(l_AC.*t6+l_DE.*t3+l_OB.*t3),t7+dth1.*(l_AC.*t5+l_DE.*t2+l_OB.*t2),0.0,t9-dth1.*l_AC.*t6,t7+dth1.*l_AC.*t5,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],[3,5]);
