function F = ik(x)

init_angle = pi/4;
init_length = 0.3;

p = parameters();

a = x(1);
b = x(2);

x = p(19)+p(21); % OB + DE
y = p(20);       % BD=AC

F(1) = x*cos(a)+y*cos(b)-init_length*sin(init_angle);
F(2) = x*sin(a)+y*sin(b)-init_length*cos(init_angle);

end
