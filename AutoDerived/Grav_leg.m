function Grav_Joint_Sp = Grav_leg(in1,in2)
%GRAV_LEG
%    GRAV_JOINT_SP = GRAV_LEG(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.7.
%    04-Dec-2021 21:27:08

g = in2(22,:);
l_AC = in2(20,:);
l_A_m3 = in2(15,:);
l_B_m2 = in2(14,:);
l_C_m4 = in2(16,:);
l_OA = in2(18,:);
l_OB = in2(19,:);
l_O_m1 = in2(13,:);
l_O_m5 = in2(17,:);
m1 = in2(1,:);
m2 = in2(2,:);
m3 = in2(3,:);
m4 = in2(4,:);
m5 = in2(5,:);
th1 = in1(1,:);
th2 = in1(2,:);
th3 = in1(3,:);
t2 = sin(th1);
t3 = th1+th2;
t4 = l_OA.*t2;
t5 = sin(t3);
Grav_Joint_Sp = [g.*m2.*(l_B_m2.*t5+l_OB.*t2)+g.*m3.*(t4+l_A_m3.*t5)+g.*m4.*(t4+l_AC.*t5+l_C_m4.*t2)+g.*l_O_m1.*m1.*t2;g.*t5.*(l_AC.*m4+l_A_m3.*m3+l_B_m2.*m2);g.*l_O_m5.*m5.*sin(th3);0.0;g.*(m1+m2+m3+m4+m5)];
