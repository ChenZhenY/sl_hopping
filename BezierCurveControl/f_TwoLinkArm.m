function dz = f_TwoLinkArm(t,in2,in3,in4)
%F_TWOLINKARM
%    DZ = F_TWOLINKARM(T,IN2,IN3,IN4)

%    This function was generated by the Symbolic Math Toolbox version 5.10.
%    23-Oct-2013 01:04:34

Fx = in3(3,:);
Fy = in3(4,:);
I1 = in4(4,:);
I2 = in4(5,:);
dth1 = in2(2,:);
dth2 = in2(4,:);
l = in4(1,:);
m1 = in4(2,:);
m2 = in4(3,:);
tau1 = in3(1,:);
tau2 = in3(2,:);
th1 = in2(1,:);
th2 = in2(3,:);
t2 = cos(th1);
t3 = cos(th2);
t4 = sin(th1);
t5 = sin(th2);
t6 = t2.^2;
t7 = l.^2;
t8 = t4.^2;
t9 = t5.^2;
t10 = m2.^2;
t11 = t3.^2;
t12 = dth2.^2;
t13 = dth1.^2;
t14 = I1.*I2.*1.6e1;
t15 = I2.*m1.*t6.*4.0;
t16 = I1.*m2.*t11.*4.0;
t17 = I2.*m1.*t8.*4.0;
t18 = I1.*m2.*t9.*4.0;
t19 = I2.*m2.*t6.*t7.*1.6e1;
t20 = I2.*m2.*t7.*t8.*1.6e1;
t21 = t6.*t7.*t9.*t10.*4.0;
t22 = t7.*t8.*t10.*t11.*4.0;
t23 = m1.*m2.*t6.*t11;
t24 = m1.*m2.*t6.*t9;
t25 = m1.*m2.*t8.*t11;
t26 = m1.*m2.*t8.*t9;
t27 = t14+t15+t16+t17+t18+t19+t20+t21+t22+t23+t24+t25+t26-t2.*t3.*t4.*t5.*t7.*t10.*8.0;
t28 = 1.0./t27;
dz = [dth1;t28.*(I2.*tau1.*8.0+m2.*t9.*tau1.*2.0+m2.*t11.*tau1.*2.0-Fx.*I2.*l.*t4.*8.0+Fy.*I2.*l.*t2.*8.0-Fx.*l.*m2.*t4.*t9.*2.0-Fx.*l.*m2.*t4.*t11.*2.0+Fy.*l.*m2.*t2.*t9.*2.0+Fy.*l.*m2.*t2.*t11.*2.0+Fx.*m2.*t4.*t7.*t9.*4.0-Fy.*m2.*t2.*t7.*t11.*4.0-l.*m2.*t2.*t3.*tau2.*4.0-l.*m2.*t4.*t5.*tau2.*4.0+l.*t2.*t5.*t9.*t10.*t12-l.*t3.*t4.*t9.*t10.*t12+l.*t2.*t5.*t10.*t11.*t12-l.*t3.*t4.*t10.*t11.*t12+t3.*t5.*t6.*t7.*t10.*t13.*2.0+t2.*t4.*t7.*t9.*t10.*t13.*2.0-t3.*t5.*t7.*t8.*t10.*t13.*2.0-t2.*t4.*t7.*t10.*t11.*t13.*2.0+I2.*l.*m2.*t2.*t5.*t12.*4.0-I2.*l.*m2.*t3.*t4.*t12.*4.0+Fx.*m2.*t2.*t3.*t5.*t7.*4.0-Fy.*m2.*t3.*t4.*t5.*t7.*4.0).*2.0;dth2;t28.*(I1.*tau2.*8.0+m1.*t6.*tau2.*2.0+m1.*t8.*tau2.*2.0+m2.*t6.*t7.*tau2.*8.0+m2.*t7.*t8.*tau2.*8.0-Fx.*I1.*l.*t5.*8.0+Fy.*I1.*l.*t3.*8.0-Fx.*l.*m1.*t5.*t6.*2.0-Fx.*l.*m1.*t5.*t8.*2.0+Fy.*l.*m1.*t3.*t6.*2.0+Fy.*l.*m1.*t3.*t8.*2.0+Fx.*m2.*t5.*t7.*t8.*4.0-Fy.*m2.*t3.*t6.*t7.*4.0-l.*m2.*t2.*t3.*tau1.*4.0-l.*m2.*t4.*t5.*tau1.*4.0-t3.*t5.*t6.*t7.*t10.*t12.*2.0-t2.*t4.*t7.*t9.*t10.*t12.*2.0+t3.*t5.*t7.*t8.*t10.*t12.*2.0+t2.*t4.*t7.*t10.*t11.*t12.*2.0-Fx.*l.*m2.*t5.*t6.*t7.*8.0-Fx.*l.*m2.*t5.*t7.*t8.*8.0+Fy.*l.*m2.*t3.*t6.*t7.*8.0+Fy.*l.*m2.*t3.*t7.*t8.*8.0-I1.*l.*m2.*t2.*t5.*t13.*4.0+I1.*l.*m2.*t3.*t4.*t13.*4.0+Fx.*m2.*t2.*t3.*t4.*t7.*4.0-Fy.*m2.*t2.*t4.*t5.*t7.*4.0-l.*m1.*m2.*t2.*t5.*t6.*t13+l.*m1.*m2.*t3.*t4.*t6.*t13-l.*m1.*m2.*t2.*t5.*t8.*t13+l.*m1.*m2.*t3.*t4.*t8.*t13-l.*t2.*t5.*t6.*t7.*t10.*t13.*4.0+l.*t3.*t4.*t6.*t7.*t10.*t13.*4.0-l.*t2.*t5.*t7.*t8.*t10.*t13.*4.0+l.*t3.*t4.*t7.*t8.*t10.*t13.*4.0).*2.0];