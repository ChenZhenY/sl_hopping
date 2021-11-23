t  = 60;

[th1, th2] = initial_condition_convert(2*pi/3, 0.18);
mid_l = 0.05;
thd_1_array = 0;
thd_2_array = 0;
foot = zeros(3,60);
p = parameters();  
p = [p; ground_height];

for i = 1:t
    temp = flight_trajectory([th1, th2],mid_l,i/t);
    thd_1_array(i) = temp(1,1);
    thd_2_array(i) = temp(2,1);
    foot(:, i) = position_foot(vertcat(temp, zeros(8,1)), p);
end
t = linspace(0,0.6,60);
% plot(t,thd_2_array) % blue
% hold on;
% plot(t,thd_1_array); % orange

plot(foot(1,:), foot(2,:)); %yellow