function [th1, th2] = initial_condition_convert(init_angle, init_length)
% intro: convert SLIP angle and length of BE to joint angles
% input: initial angle [0, pi] and length [0, 0.186] of BE 
% output: the corresponding joint angle

p = parameters();

% x = p(19)+p(21); % OB + DE
% y = p(20);       % BD=AC

% syms a b c d len ang real
% eqns = [x*cos(a)+y*cos(b)==c, x*sin(a)+y*sin(b)==d, b-a>0];
% 
% S = solve(eqns, [a b], 'ReturnConditions', true)
% %%
% fun = @ik;
% x0 = [0, 0.0001];
% [a b] = fsolve(fun, x0);
%%
AC = p(20);
DE = p(21);
costh2 = -(AC^2 + DE^2 - init_length^2)/(2*AC*DE);
if abs(costh2) > 1
    disp('blah');
end
th2 = acos(costh2);

cos_th1_a = sin(th2)*AC/init_length;
if cos_th1_a >= 0
   th1_a = acos(cos_th1_a);
   th1 = th1_a - init_angle;
elseif cos_th1_a <0
   th1_a = -acos(cos_th1_a);
   th1 = th1_a - init_angle;
end

end