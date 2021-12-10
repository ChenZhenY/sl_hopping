T = readtable('jump_data - Phase_shift.csv');
T2 = readtable('jump_2');

%h = heatmap(T2, 'Swing_ratio', 'Phase_shift', 'ColorVariable', 'SingleHopD');

figure

%scatter(T, "Phase_shift", "SingleHopD")

swing_ratio = table2array(T2(:,1)) % swing
phase_shift = table2array(T2(:,2));
hop_d = table2array(T2(:,4));

a = 1;
for i=1:16
    if swing_ratio(i) == 0.1
        hop_phase_1(a) = phase_shift(i);
        hop_d_1(a) = hop_d(i);
        a = a+1;
    end
end
[x, i] = sort(hop_phase_1);
hopdd = hop_d_1(i);
% plot(hop_phase_1, hop_d_1, 'x');
p = polyfit(x, hopdd, 4);
xx = linspace(-1, 1);
yy = polyval(p,xx)
plot(xx,yy,'LineWidth', 3);
hold on
plot(x,hopdd, 'x', 'MarkerSize',15);
hold on

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
yy = polyval(p,xx)
plot(xx,yy,'LineWidth', 3);
hold on
% plot(hop_phase_1, hop_d_1, 'x');
plot(x,y, 'o','MarkerSize',10);

a = 1;
for i=1:16
    if swing_ratio(i) == 0.9
        hop_phase_9(a) = phase_shift(i);
        hop_d_9(a) = hop_d(i);
        a = a+1;
    end
end
[x, i] = sort(hop_phase_9);
y = hop_d_9(i);
p = polyfit(x, y, 4);
xx = linspace(-1, 1);
yy = polyval(p,xx);
hold on
plot(xx,yy,'LineWidth', 3);
hold on
% plot(hop_phase_1, hop_d_1, 'x');
plot(x,y, '*','MarkerSize',10);

plot([-1 1], [0.459 0.459], 'LineWidth', 3);

grid on

hold off
xlabel('Phase Shift');
ylabel('Distance Per Jump');
legend('Duration 0.1', '0.1','Duration 0.5', '0.5','Duration 0.9','0.9', 'No swing leg')



% plot(table2array(x),table2array(y), 'o')