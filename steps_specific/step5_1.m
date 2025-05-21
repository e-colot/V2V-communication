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
t = 0:1e-11:T;
h = zeros(1, length(t));
for i = 1:length(alpha)
    closestBin = find(t >= timeOfFlight(i), 1);
    h(closestBin) = h(closestBin) + alpha(i);
end

figure;
plot(t*1e6, abs(h), 'LineWidth', 2);
hold on;
xlabel('Time (µs)');
ylabel('|h(t)|');
grid on;
xlim([1.63 1.72]);
ylim([-max(abs(h))/5 max(abs(h))*6/5]);

% construction of H(f)

f = -(length(t)-1)/(2*T):1/T:(length(t)-1)/(2*T);
H = fftshift(fft(h));

figure;

% Top left: Amplitude response
% subplot(2, 2, 1);
plot(f*1e-9, abs(H), 'LineWidth', 2);
xlabel('Frequency (GHz)');
ylabel('|H(f)|');
grid on;
ylim([-max(abs(H))/5 max(abs(H))*6/5]);

% % Top right: Phase response
% subplot(2, 2, 2);
% plot(f*1e-9, angle(H), 'LineWidth', 2);
% xlabel('Frequency (GHz)');
% ylabel('Phase (rad)');
% title('Phase Response \angle H(f)');
% grid on;

% % Bottom: zoomed-in view of the phase response
% subplot(2, 2, [3, 4]);
% plot(f(1:100)*1e-9, angle(H(1:100)), 'LineWidth', 2);
% xlabel('Frequency (GHz)');
% ylabel('Phase (rad)');
% title('Phase Response \angle H(f) - zoomed in');
% grid on;






