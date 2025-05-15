% getting access to the project
addpath(genpath('./..'));

clear; close all; clc;

cfg = config(); 

distance = 1:0.1:1000; % distance from TX to the RX in meters
LOSPower = zeros(length(distance), 1);
MPCPower = zeros(length(distance), 1);

%extend the walls to reach 1km
cfg.obstacles = zeros(2, 2, 2);
cfg.obstacles(1, :, 1) = [0, -cfg.environment_params.road_width/2];
cfg.obstacles(2, :, 1) = [1100, -cfg.environment_params.road_width/2];

cfg.obstacles(1, :, 2) = [0, cfg.environment_params.road_width/2];
cfg.obstacles(2, :, 2) = [1100, cfg.environment_params.road_width/2];

for i = 1:length(distance)
    d = distance(i); % current distance
    cfg.TX_pos = [50 + d; -cfg.environment_params.road_width/4];
    cfg.RX_pos = [50; -cfg.environment_params.road_width/4];


    rays = createRays(cfg);
    rays.voltages = rayVoltage(rays, cfg); % calculate the voltages

    LOSPower(i) = norm(rays.voltages(1).^2); 
    MPCPower(i) = sum(norm(rays.voltages(2:end).^2));
end

% visualize(cfg, rays, 2);
% xlim([30 170]);
% ylim([-cfg.environment_params.road_width cfg.environment_params.road_width]);
% view(114, 47);

% Calculate the Rice factor (K) and plot it as a function of distance
K = LOSPower ./ MPCPower;

figure;
p = plot(distance, 10*log10(K), 'LineWidth', 1.5);
hold on;
% Add a data tip close to 10*log10(K) = 0
[~, idx] = min(abs(10*log10(K))); % Find the index closest to 0 dB
datatip(p, distance(idx), 10*log10(K(idx)), 'Location', 'northwest');
ylabel('Rice Factor (K) [dB]');
title('Rice Factor (K) vs Distance');
grid on;


