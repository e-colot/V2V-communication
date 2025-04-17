% getting access to the project
addpath(genpath('./..'));

clear; close all; clc;

cfg = config(); 

cfg.TX_pos = [150; -cfg.environment_params.road_width/4];
cfg.RX_pos = [50; -cfg.environment_params.road_width/4];


rays = createRays(cfg);
rays.voltages = rayVoltage(rays, cfg); % calculate the voltages

visualize(cfg, rays, 2);
xlim([30 170]);
ylim([-cfg.environment_params.road_width cfg.environment_params.road_width]);
view(114, 47);


sum_voltages = sum(rays.voltages(:)); % Sum of all elements in rays.voltages
disp(['Total received voltage: ', num2str(norm(sum_voltages)*1e6), ' µV']); % Display the sum of voltages
disp(['phase: ', num2str(angle(sum_voltages)*180/pi), ' degrees']); % Display the phase of the sum of voltages

