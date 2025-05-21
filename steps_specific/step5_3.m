% getting access to the project
addpath(genpath('./..'));

clear; close all; clc;

cfg = config(); 

cfg.environment_params.road_length = 2000;
distance = 500; % m distance between TX and RX
cfg.obstacles = createObstacles(cfg); % recompute obstacles

tmp_obstacles(:, :, 1) = cfg.obstacles(:, :, 6);
tmp_obstacles(:, :, 2) = cfg.obstacles(:, :, 8);
cfg.obstacles = tmp_obstacles;

cfg.TX_pos = [50; -cfg.environment_params.road_width/4];
cfg.RX_pos = [50+distance; -cfg.environment_params.road_width/4];
cfg.bounce_limit = 3;

rays = createRays(cfg);

alpha = rays.reflexionAttenuation .* exp(-j*2*pi*cfg.transmit_params.fc.*rays.lengths./cfg.transmit_params.c)./rays.lengths;

%visualize(cfg, rays, 1);

timeOfFlight = rays.lengths ./ cfg.transmit_params.c;

deltaFc = 1/(max(timeOfFlight) - min(timeOfFlight));
disp(['Coherence bandwidth: ' num2str(round(deltaFc / 1e6)) ' MHz']);
disp(['RF bandwidth: ' num2str(round(cfg.transmit_params.BW / 1e6)) ' MHz']);

% construction of h(t)
T = 15e-6; % time window
t = 0:1/cfg.transmit_params.BW:T;
h = zeros(1, length(t));
for i = 1:length(alpha)
    % find closest bin
    closestBin = find(t >= timeOfFlight(i), 1)-1;
    % add the voltage to the closest bin
    h(closestBin) = h(closestBin) + alpha(i);
end

% Plot h(t) as an impulse function with a finer time vector
t_fine = min(t):1e-10:max(t);
h_step = zeros(size(t_fine));
for i = 1:length(h)
    % Find the index of the closest time point in t_fine
    [~, idx] = min(abs(t_fine - t(i)));
    % Assign the value of h(i) to the corresponding index in h_step
    h_step(idx) = h(i);
end

figure;
plot(t_fine*1e6, abs(h_step), 'LineWidth', 2);
xlabel('Time (µs)');
ylabel('|h_{TDL}(t)|');
grid on;
xlim([1.63 1.72]);
ylim([-max(abs(h_step))/5 max(abs(h_step))*6/5]);





