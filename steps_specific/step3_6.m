clear; close all; clc;

sigma = 0.25488;
x = -1:1e-3:50;
y = 1/2 * erfc(x./(sigma*sqrt(2)));

figure;
p = semilogy(x, y, 'LineWidth', 2);
hold on;
% Add data tips close to y = 0.5, y = 0.05, and y = 0.01
y_targets = [0.5, 0.05, 0.01];
maxX = 0;
minX = 0;
for i = 1:length(y_targets)
    [~, idx] = min(abs(y - y_targets(i)));
    x_val = x(idx);
    y_val = y(idx);
    % Update maxX and maxY for the datatip
    maxX = max(maxX, x_val);
    minX = min(minX, x_val);
    datatip(p, x_val, y_val);
end
ylim([1e-3 1]);
varX = (maxX - minX);
xlim([minX - varX/5, maxX + varX/5]);
xlabel('Fade margin (dB)');
ylabel('Probability of outage');
