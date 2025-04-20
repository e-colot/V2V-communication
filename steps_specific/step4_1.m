% getting access to the project
addpath(genpath('./..'));

clear; close all; clc;

cfg = config(); 

cfg.environment_params.road_length = 1000;

cfg.TX_pos = [-cfg.environment_params.road_length/2; -cfg.environment_params.road_width/4];
cfg.RX_pos = [cfg.environment_params.road_length/2; -cfg.environment_params.road_width/4];
cfg.bounce_limit = 0;

rays = createRays(cfg);
rays.voltages = rayVoltage(rays, cfg);

%visualize(cfg, rays, 1);

% construction of h(t)
t = 0:1e-10:5e-5;
h = zeros(1, length(t));
for i = 1:length(rays.voltages)
    timeOfFlight = rays.lengths(i) / cfg.transmit_params.c;
    closestBin = find(t >= timeOfFlight, 1);
    h(closestBin) = h(closestBin) + rays.voltages(i);
end

figure;
plot(t, abs(h), 'LineWidth', 2);
hold on;
xlabel('Time (s)');
ylabel('Amplitude');
title('Impulse Response |h(t)|');
grid on;
xlim([t(1) t(end)]);
ylim([-max(abs(h))/5 max(abs(h))*6/5]);






