ctrlpts = [   -0.8471    0.1358   -2.0000    2.0000
    1.7853   -2.0000   -2.0000   -2.0000];
tf = .2368;
t = 0:.05:1;
joint1 = [];
joint2 = [];
for i = t
    joint1(end+1) = BezierCurve(ctrlpts(1,:),i);
    joint2(end+1) = BezierCurve(ctrlpts(2,:),i);
end
t = t / .2368;
figure(5)
plot(t,joint1);
hold on;
plot(t,joint2);