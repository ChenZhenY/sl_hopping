t  = 60;

[th1, th2] = initial_condition_convert(pi/3, 0.18);
mid_l = 0.02;
thd_1_array = 0;
thd_2_array = 0;
z0 = [th1; th2; 0; 0; 0; 0; 0; 0; 0.3; -0.3*1.732];
foot = zeros(3,60);
p = parameters();  
pos_foot0 = position_foot(z0, p);  
ground_height = pos_foot0(2);
p = [p; ground_height];

for i = 1:t
    temp = flight_trajectory([th1, th2],    mid_l,i/t);
    thd_1_array(i) = temp(1,1);
    thd_2_array(i) = temp(2,1);
    foot(:, i) = position_foot(vertcat(temp, zeros(8,1)), p);
    plot([0, foot(1,i)], [0, foot(2,i)]);
    hold on
end
t = linspace(0,0.6,60);
% plot(t,thd_2_array) % blue
% hold on;
% plot(t,thd_1_array); % orange

% plot(foot(1,:), foot(2,:));
% hold on;
% plot(t, foot(1,:));