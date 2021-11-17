function [th1, th2] = initial_condition_convert(init_angle, init_length)

% input: initial angle and length of BE
% output: the corresponding joint anlge
% note: the1 and the2 > 0, this function can be only used in landing
% position.
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
th2 = acos(costh2)

cos_th1_a = sin(th2)*AC/init_length;
th1_a = acos(cos_th1_a);
th1 = th1_a - init_angle

end