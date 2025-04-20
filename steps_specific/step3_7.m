clear; close all; clc;

sigma = 0.30236;
x = -0.3:1e-4:0.9;
y = 1/2 * erfc(x./(sigma*sqrt(2)));

figure;
semilogy(x, y, 'LineWidth', 2);
hold on;
xlabel('M (dB)');
ylabel('probability of outage');
title('Outage Probability vs Fade Margin');
