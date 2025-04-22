% getting access to the project
addpath(genpath('./..'));

clear; close all; clc;

cfg = config(); 

cfg.environment_params.road_length = 2000;
distance = 1000; % m distance between TX and RX
cfg.obstacles = createObstacles(cfg); % recompute obstacles

tmp_obstacles(:, :, 1) = cfg.obstacles(:, :, 6);
tmp_obstacles(:, :, 2) = cfg.obstacles(:, :, 8);
cfg.obstacles = tmp_obstacles;

cfg.TX_pos = [0; -cfg.environment_params.road_width/4];
cfg.RX_pos = [distance; -cfg.environment_params.road_width/4];
cfg.bounce_limit = 15;

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
    h = h + alpha(i) * sinc(cfg.transmit_params.BW * (t - timeOfFlight(i)));
end

figure;
plot(t*1e6, abs(h), 'LineWidth', 2);
hold on;
xlabel('Time (ns)');
ylabel('Amplitude');
title('Impulse Response |h_{TDL}(t)|');
grid on;
xlim([t(1) t(end)]*1e6);
ylim([-max(abs(h))/5 max(abs(h))*6/5]);



%% construction of H(f)

% f = -(length(t)-1)/(2*T):1/T:(length(t)-1)/(2*T);
% H = fftshift(fft(h));

% figure;
% plot(f*1e-6, abs(H), 'LineWidth', 2);
% xlabel('Frequency (MHz)');
% ylabel('Amplitude');
% title('Frequency Response |H(f)|');
% grid on;
% ylim([-max(abs(H))/5 max(abs(H))*6/5]);





