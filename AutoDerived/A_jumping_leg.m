function A = A_jumping_leg(in1,in2)
%A_JUMPING_LEG
%    A = A_JUMPING_LEG(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.7.
%    16-Nov-2021 22:57:19

I1 = in2(6,:);
I2 = in2(7,:);
I3 = in2(8,:);
I4 = in2(9,:);
I5 = in2(10,:);
Ir = in2(11,:);
N = in2(12,:);
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
t2 = cos(th1);
t3 = cos(th3);
t4 = sin(th1);
t5 = sin(th3);
t6 = th1+th2;
t7 = N.^2;
t8 = l_AC.^2;
t9 = l_A_m3.^2;
t10 = l_B_m2.^2;
t11 = l_O_m1.^2;
t12 = l_O_m5.^2;
t13 = Ir.*N;
t38 = m1+m2+m3+m4+m5;
t14 = l_C_m4.*t2;
t15 = l_OA.*t2;
t16 = l_OB.*t2;
t17 = cos(t6);
t18 = l_C_m4.*t4;
t19 = l_OA.*t4;
t20 = l_OB.*t4;
t21 = sin(t6);
t22 = l_O_m1.*m1.*t2;
t23 = l_O_m5.*m5.*t3;
t24 = l_O_m1.*m1.*t4;
t25 = l_O_m5.*m5.*t5;
t26 = Ir.*t7;
t27 = t14.*2.0;
t28 = t15.*2.0;
t29 = t16.*2.0;
t30 = t18.*2.0;
t31 = t19.*2.0;
t32 = t20.*2.0;
t33 = t17.^2;
t34 = t21.^2;
t35 = l_AC.*t17;
t36 = l_A_m3.*t17;
t37 = l_B_m2.*t17;
t39 = l_AC.*t21;
t40 = l_A_m3.*t21;
t41 = l_B_m2.*t21;
t42 = t35.*2.0;
t43 = t36.*2.0;
t44 = t37.*2.0;
t45 = t39.*2.0;
t46 = t40.*2.0;
t47 = t41.*2.0;
t48 = m4.*t35;
t49 = m3.*t36;
t50 = m2.*t37;
t51 = m4.*t39;
t52 = m3.*t40;
t53 = m2.*t41;
t54 = t15+t36;
t55 = t16+t37;
t56 = t19+t40;
t57 = t20+t41;
t62 = t14+t15+t35;
t63 = t18+t19+t39;
t58 = t28+t43;
t59 = t29+t44;
t60 = t31+t46;
t61 = t32+t47;
t64 = t27+t28+t42;
t65 = t46.*t56;
t66 = t47.*t57;
t69 = t30+t31+t45;
t72 = t43.*t54;
t73 = t44.*t55;
t74 = t48+t49+t50;
t75 = t51+t52+t53;
t76 = t42.*t62;
t77 = t45.*t63;
t67 = (m3.*t58)./2.0;
t68 = (m2.*t59)./2.0;
t70 = (m3.*t60)./2.0;
t71 = (m2.*t61)./2.0;
t78 = (m4.*t69)./2.0;
t79 = (m4.*t64)./2.0;
t80 = t65+t72;
t81 = t66+t73;
t84 = t76+t77;
t82 = (m3.*t80)./2.0;
t83 = (m2.*t81)./2.0;
t85 = (m4.*t84)./2.0;
t86 = t24+t70+t71+t78;
t87 = t22+t67+t68+t79;
t88 = I2+I3+t13+t82+t83+t85;
A = reshape([I1+I2+I3+I4+Ir+t26+(m1.*(t2.^2.*t11.*2.0+t4.^2.*t11.*2.0))./2.0+(m3.*(t54.^2.*2.0+t56.^2.*2.0))./2.0+(m2.*(t55.^2.*2.0+t57.^2.*2.0))./2.0+(m4.*(t62.^2.*2.0+t63.^2.*2.0))./2.0,t88,0.0,t87,t86,t88,I2+I3+t26+(m4.*(t8.*t33.*2.0+t8.*t34.*2.0))./2.0+(m3.*(t9.*t33.*2.0+t9.*t34.*2.0))./2.0+(m2.*(t10.*t33.*2.0+t10.*t34.*2.0))./2.0,0.0,t74,t75,0.0,0.0,I5+t26+(m5.*(t3.^2.*t12.*2.0+t5.^2.*t12.*2.0))./2.0,t23,t25,t87,t74,t23,t38,0.0,t86,t75,t25,0.0,t38],[5,5]);
