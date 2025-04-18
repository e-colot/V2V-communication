% getting access to the project
addpath(genpath('./..'));

clear; close all; clc;

cfg = config(); 

distance = 1:0.1:1000; % distance from TX to the RX in meters
receivedVoltage = zeros(length(distance), 1); % initialize received voltage array

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

    receivedVoltage(i) = norm(sum(rays.voltages(:))); % Sum of all elements in rays.voltages
end
% visualize(cfg, rays, 2);
% xlim([30 170]);
% ylim([-cfg.environment_params.road_width cfg.environment_params.road_width]);
% view(114, 47);

% P_RX = V_OC^2 / (Z_a + Z_L)
%      = (2*V_RX)^2 / (Z_a + Z_L)
simulatedPower = (2*receivedVoltage).^2 / (720 * pi / 16);

figure;
semilogy(distance, simulatedPower, 'LineWidth', 2);
xlabel('Distance (m)');
ylabel('Received Power (W)');
title('Received Power vs Distance');
hold on;
grid on;

theoreticalPower = cfg.transmit_params.TX_power * 16/(9 * pi^2) * (cfg.transmit_params.c ./ (cfg.transmit_params.fc*pi*distance)).^2; % theoretical voltage decay
semilogy(distance, theoreticalPower, 'LineWidth', 2);
legend('Simulated Power', 'Theoretical Power');
hold off;

