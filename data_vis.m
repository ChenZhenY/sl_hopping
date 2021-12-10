T = readtable('jump_data - Phase_shift.csv');
T2 = readtable('jump_2');

%h = heatmap(T2, 'Swing_ratio', 'Phase_shift', 'ColorVariable', 'SingleHopD');

figure

%scatter(T, "Phase_shift", "SingleHopD")

% duration = 0.3    
dura3_phase = [-0.6 -0.3 -0.1 0.3 0.6 0.9]; % 0, 0.8 fail
dura3_time = [1.2806 1.2776  1.2696 1.2736  1.3297 1.2706 ];
dura3_dist = [1.7085 1.3423 1.4011 0.7569 0.9229 1.3736 ];
% duration = 0.4
dura4_phase = [-0.8 -0.6 -0.3 -0.1 0.3 0.6 0.9]; % 0, 0.8 fail
dura4_time = [1.2566 1.3427 1.1516 1.2636  1.3047 1.2156 1.2656];
dura4_dist = [1.5068 1.8884 1.2185 1.3960  0.8538 0.9802 1.3733];
% duration = 0.6
dura6_phase = [-0.8  -0.7   -0.6   -0.4    -0.3    -0.2 -0.1 0.3 0.6 0.9]; % 0, 0.8 fail
dura6_time = [1.3157 1.3387 1.2926 1.1776 1.1556  1.1916 1.2686 1.3627 1.1616 1.2726 ];
dura6_dist = [1.7422 1.8464 1.7627 1.3515 1.2289 1.2778 1.4021 1.1554 1.0535 1.3792];
% duration = 0.7
dura7_phase = [-0.8 -0.6 -0.3 0.3 0.6 0.9]; % 0,0.9 fail
dura7_time = [1.3387 1.2496 1.1706 1.3007 1.1516 1.2676];
dura7_dist = [1.8019 1.6543 1.2498 1.1393 1.0808 1.3724];

swing_ratio = table2array(T2(:,1)); % swing
phase_shift = table2array(T2(:,2));
hop_d = table2array(T2(:,4));

p = polyfit(dura3_phase, dura3_dist/3, 4);
xx = linspace(-1, 1);
yy = polyval(p,xx);
plot(xx,yy,'LineWidth', 3);
hold on
% plot(hop_phase_1, hop_d_1, 'x');
plot(dura3_phase, dura3_dist/3, '*','MarkerSize',10);

p = polyfit(dura4_phase, dura4_dist/3, 4);
xx = linspace(-1, 1);
yy = polyval(p,xx);
plot(xx,yy,'LineWidth', 3);
hold on
% plot(hop_phase_1, hop_d_1, 'x');
plot(dura4_phase, dura4_dist/3, 's','MarkerSize',10);


a = 1;
for i=1:16
    if swing_ratio(i) == 0.5
        hop_phase_5(a) = phase_shift(i);
        hop_d_5(a) = hop_d(i);
        a = a+1;
    end
end
[x, i] = sort(hop_phase_5);
y = hop_d_5(i);
p = polyfit(x, y, 4);
xx = linspace(-1, 1);
yy = polyval(p,xx);
plot(xx,yy,'LineWidth', 3);
hold on
% plot(hop_phase_1, hop_d_1, 'x');
plot(x,y, 'o','MarkerSize',10);

% p = polyfit(dura6_phase, dura6_dist/3, 4);
% xx = linspace(-1, 1);
% yy = polyval(p,xx);
% plot(xx,yy,'LineWidth', 3);
% hold on
% % plot(hop_phase_1, hop_d_1, 'x');
% plot(dura6_phase, dura6_dist/3, 'd','MarkerSize',10);
% 
% p = polyfit(dura7_phase, dura7_dist/3, 4);
% xx = linspace(-1, 1);
% yy = polyval(p,xx);
% plot(xx,yy,'LineWidth', 3);
% hold on
% % plot(hop_phase_1, hop_d_1, 'x');
% plot(dura7_phase, dura7_dist/3, '.','MarkerSize',10);

% a = 1;
% for i=1:16
%     if swing_ratio(i) == 0.9
%         hop_phase_9(a) = phase_shift(i);
%         hop_d_9(a) = hop_d(i);
%         a = a+1;
%     end
% end
% [x, i] = sort(hop_phase_9);
% y = hop_d_9(i);
% p = polyfit(x, y, 4);
% xx = linspace(-1, 1);
% yy = polyval(p,xx);
% hold on
% plot(xx,yy,'LineWidth', 3);
% hold on
% % plot(hop_phase_1, hop_d_1, 'x');
% plot(x,y, '*','MarkerSize',10);

% a = 1;
% for i=1:16
%     if swing_ratio(i) == 0.1
%         hop_phase_1(a) = phase_shift(i);
%         hop_d_1(a) = hop_d(i);
%         a = a+1;
%     end
% end
% [x, i] = sort(hop_phase_1);
% hopdd = hop_d_1(i);
% % plot(hop_phase_1, hop_d_1, 'x');
% p = polyfit(x, hopdd, 4);
% xx = linspace(-1, 1);
% yy = polyval(p,xx)
% plot(xx,yy,'LineWidth', 3);
% hold on
% plot(x,hopdd, 'x', 'MarkerSize',15);
% hold on

plot([-1 1], [0.459 0.459], 'LineWidth', 3);

grid on

hold off
xlabel('Phase Shift');
ylabel('Distance Per Jump');
lgd = legend('Duration 0.3', '0.3','Duration 0.4', '0.4','Duration 0.5', '0.5', 'No swing leg')
% ,'Duration 0.6', '0.6', 'Duration 0.7','0.7'
lgd.FontSize = 12;


% plot(table2array(x),table2array(y), 'o')