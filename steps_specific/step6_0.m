% getting access to the project
addpath(genpath('./..'));

clear; close all; clc;
cfg = config();

%% Setup
cfg.environment_params.road_length = 2000;

%recompute obstacles
cfg.obstacles = zeros(2, 2, 2);
cfg.obstacles(1, :, 1) = [-1100, -cfg.environment_params.road_width/2];
cfg.obstacles(2, :, 1) = [1100, -cfg.environment_params.road_width/2];

cfg.obstacles(1, :, 2) = [-1100, cfg.environment_params.road_width/2];
cfg.obstacles(2, :, 2) = [1100, cfg.environment_params.road_width/2];


x = (-cfg.environment_params.road_length+cfg.environment_params.local_area_len)/2:cfg.environment_params.local_area_len:(cfg.environment_params.road_length-cfg.environment_params.local_area_len)/2;
y = (-cfg.environment_params.road_width+cfg.environment_params.local_area_len)/2:cfg.environment_params.local_area_len:(cfg.environment_params.road_width-cfg.environment_params.local_area_len)/2;

cfg.TX_pos = [0; 0];

T = 3.5e-6; % time window
t = 0:1/cfg.transmit_params.BW:T;
P = zeros(1, length(t));

hWait = waitbar(0, 'Processing...'); % Initialize waitbar
cnt = 0;
maxTimeOfFlight = 0;
for xi = 1:length(x)
    for yi = 1:length(y)
        cfg.RX_pos = [x(xi); y(yi)];

        rays = createRays(cfg);
        timeOfFlight = rays.lengths ./ cfg.transmit_params.c;
        maxTimeOfFlight = max(maxTimeOfFlight, max(timeOfFlight));
        alpha = rays.reflexionAttenuation .* exp(-j*2*pi*cfg.transmit_params.fc.*rays.lengths./cfg.transmit_params.c)./rays.lengths;

        % PDP construction
        for i = 1:length(alpha)
            % find closest bin
            closestBin = find(t >= timeOfFlight(i), 1)-1;
            % add the voltage to the closest bin
            P(closestBin) = P(closestBin) + abs(alpha(i)^2);
        end
        cnt = cnt + 1;
    end
    waitbar(xi / length(x), hWait); % Update waitbar
end
close(hWait); % Close waitbar

% because P is defined as the sum of the expected value of |h(t)|^2
P = P/cnt; 

%% plotting
%replace 0 with NaN
Pnanned = P;
Pnanned(P == 0) = NaN;
PdBm = 10*log10(Pnanned*1e3); % convert to dBm

PdBm_shifted = PdBm + 60; % Shift values up by 60 dBm

figure;
stem(t*1e6, PdBm_shifted, 'filled', 'LineWidth', 2);
xlabel('Time (µs)');
ylabel('PDP (dBm)');

yticks = get(gca, 'YTick');
set(gca, 'YTickLabel', yticks - 60); % Display y-axis 60 dBm lower
xlim([0 1]);

disp(['Max time of flight: ' num2str(maxTimeOfFlight*1e6) ' µs']);

% total power
Ptot = sum(P);
disp(['Total power: ' num2str(10*log10(Ptot*1e3)) ' dBm']);

% mean delay
meanDelay = sum(t .* P) / Ptot;
disp(['Mean delay: ' num2str(meanDelay*1e6) ' µs']);

% delay spread
delaySpread = sqrt(sum((t - meanDelay).^2 .* P) / Ptot);
disp(['Delay spread: ' num2str(delaySpread*1e6) ' µs']);
