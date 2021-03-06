function rC = position_knee(in1,in2)
%POSITION_KNEE
%    RC = POSITION_KNEE(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.7.
%    07-Nov-2021 00:16:32

l_AC = in2(20,:);
l_OA = in2(18,:);
th1 = in1(1,:);
th2 = in1(2,:);
x = in1(4,:);
y = in1(5,:);
t2 = th1+th2;
rC = [x+l_AC.*sin(t2)+l_OA.*sin(th1);y-l_AC.*cos(t2)-l_OA.*cos(th1);0.0];
