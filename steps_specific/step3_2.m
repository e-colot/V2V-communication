% getting access to the project
addpath(genpath('./..'));

clear; close all; clc;

cfg = config(); 

cfg.TX_pos = [150; 0];
cfg.RX_pos = [50; 0];


rays = createRays(cfg);
rays.voltages = rayVoltage(rays, cfg); % calculate the voltages

visualize(cfg, rays, 2);
xlim([30 170]);
ylim([-cfg.environment_params.road_width cfg.environment_params.road_width]);
view(114, 47);


% display for every ray the norm and phase of the voltage
for i = 1:length(rays.voltages)
    disp(['Ray ', num2str(i), ': ', num2str(norm(rays.voltages(:, i))*1e6), 'µV ', num2str(angle(rays.voltages(:, i))*180/pi), '°']);
end

sum_voltages = sum(rays.voltages(:)); % Sum of all elements in rays.voltages
disp(['Total received voltage: ', num2str(norm(sum_voltages)*1e6), ' µV']); % Display the sum of voltages
disp(['phase: ', num2str(angle(sum_voltages)*180/pi), ' degrees']); % Display the phase of the sum of voltages
