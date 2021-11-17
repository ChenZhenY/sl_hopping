function rE = position_foot(in1,in2)
%POSITION_FOOT
%    RE = POSITION_FOOT(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.7.
%    16-Nov-2021 22:57:22

l_AC = in2(20,:);
l_DE = in2(21,:);
l_OB = in2(19,:);
th1 = in1(1,:);
th2 = in1(2,:);
x = in1(4,:);
y = in1(5,:);
t2 = cos(th1);
t3 = sin(th1);
t4 = th1+th2;
rE = [x+l_DE.*t3+l_OB.*t3+l_AC.*sin(t4);y-l_DE.*t2-l_OB.*t2-l_AC.*cos(t4);0.0];
