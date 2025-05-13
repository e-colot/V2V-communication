% getting access to the project
addpath(genpath('./..'));

clear; close all; clc;

cfg = config(); 

x = (-cfg.environment_params.road_length+cfg.environment_params.local_area_len)/2:cfg.environment_params.local_area_len:(cfg.environment_params.road_length-cfg.environment_params.local_area_len)/2;
y = (-cfg.environment_params.road_width+cfg.environment_params.local_area_len)/2:cfg.environment_params.local_area_len:(cfg.environment_params.road_width-cfg.environment_params.local_area_len)/2;

cfg.TX_pos = [0; 0];

avgPower = zeros(length(x), length(y)); % initialize power matrix
distance = zeros(length(x), length(y)); % initialize distance matrix
avgPowerPerp = zeros(length(y), length(x)); % initialize power matrix
distancePerp = zeros(length(y), length(x)); % initialize distance matrix

for xi = 1:length(x)
    for yi = 1:length(y)
        cfg.RX_pos = [x(xi); y(yi)];
        distance(xi, yi) = norm(cfg.RX_pos - cfg.TX_pos); % calculate distance

        rays = createRays(cfg);
        rays.voltages = rayVoltage(rays, cfg); % calculate the voltages

        avgPower(xi, yi) = sum(abs(rays.voltages).^2)/(45*pi);
        if avgPower(xi, yi) == 0
            avgPower(xi, yi) = NaN; % avoid log(0)
        end

        cfg.RX_pos = [y(yi); x(xi)];
        distancePerp(yi, xi) = norm(cfg.RX_pos - cfg.TX_pos); % calculate distance

        rays = createRays(cfg);
        rays.voltages = rayVoltage(rays, cfg); % calculate the voltages

        avgPowerPerp(yi, xi) = sum(abs(rays.voltages).^2)/(45*pi);
        if avgPowerPerp(yi, xi) == 0
            avgPowerPerp(yi, xi) = NaN; % avoid log(0)
        end
    end
end

% Normalize the log of avgPower
logPower = 10*log10(avgPower);
logPowerPerp = 10*log10(avgPowerPerp);
minVal = min(min(min(logPower)), min(min(logPowerPerp)));
maxVal = max(max(max(logPower)), max(max(logPowerPerp)));
normalizedPower = (logPower - minVal) / (maxVal - minVal);
normalizedPowerPerp = (logPowerPerp - minVal) / (maxVal - minVal);

visualize(cfg);
hold on;


% Create 3D blocks
cmap = colormap; % Get the colormap once
for xi = 1:length(x)
    for yi = 1:length(y)
        height = 50 * normalizedPower(xi, yi);
        if (~isnan(height))
            colorIdx = max(1, min(size(cmap, 1), round(normalizedPower(xi, yi) * size(cmap, 1))));
            color = cmap(colorIdx, :); % Choose color based on normalized power
            plot3DCube(x(xi), y(yi), height, color, cfg.environment_params.local_area_len);
        end
        height = 50 * normalizedPowerPerp(yi, xi);
        if (~isnan(height))
            colorIdx = max(1, min(size(cmap, 1), round(normalizedPowerPerp(yi, xi) * size(cmap, 1))));
            color = cmap(colorIdx, :); % Choose color based on normalized power
            plot3DCube(y(yi), x(xi), height, color, cfg.environment_params.local_area_len);
        end
    end
end

% Add a colorbar with true avgPower values
cbar = colorbar;
cbar.FontSize = 12; % Set font size for colorbar
cbar.Ticks = linspace(0, 1, 5); % Set ticks for normalized values
cbar.TickLabels = arrayfun(@(v) sprintf('%.2f dBm', v), linspace(min(logPower(:)), max(logPower(:)), 5), 'UniformOutput', false);
cbar.Label.String = 'Average Power (dBm)';
title('Average Power Distribution in the Road Area', 'FontSize', 18);
view(-40, 45);

%% path loss model

% Combine logPower and logPowerPerp into a single vector L
L = -[logPower(:); logPowerPerp(:)] + 10*log10(cfg.transmit_params.TX_power); % remove TX power
d = [distance(:); distancePerp(:)]; % combine distances

validIdx = ~isnan(L); % find indices where L is not NaN
L = L(validIdx); % keep only valid entries in L
d = d(validIdx); % keep corresponding entries in d

L_0 = L + 20*log10(16/(3*pi)); % removing the loss due to the antennas

figure;
scatter(d, L_0, 'x');
xlabel('Distance (m)');
ylabel('Path Loss (dB)');
title('Path Loss vs Distance');
grid on;
hold on;

% path loss model:
% L = L_0 + 10*n*log10(d/d_0)
% where d_0 is the reference distance (1m)

% --> LLS estimator to find d_0 and n
H = [ones(length(d), 1), 10*log10(d)]; % regression matrix
y = L_0;
params = H\y; % least squares solution

x = linspace(0, max(d), 1000); % x values for the fitted line
y_fit = params(1) + params(2)*10*log10(x); % fitted line
plot(x, y_fit, 'r--', 'LineWidth', 1.5); % plot the fitted line

% Friis path loss:
% L_friis = (lambda/(4*pi*d))^2 
% --> in dB:
y_friis = -20*log10(cfg.transmit_params.c ./ (cfg.transmit_params.fc*4*pi*x));
plot(x, y_friis, 'g--', 'LineWidth', 1.5); % plot the fitted line

legend('Data', 'LLS model', 'Friis model', 'Location', 'Best');
hold off;

disp('Estimated model:');
disp(['L_0(d) = ', num2str(params(1)), ' + ' num2str(params(2)), '*10*log(d)']);


%% 3.6
% sigma_l is the standard deviation of the experimental path loss around the fitted one
sigma_l = std(L_0 - (params(1) + params(2)*10*log10(d)));
shadowingMean = mean(L_0 - (params(1) + params(2)*10*log10(d)));
disp(' ');
disp(['Mean shadowing: ', num2str(shadowingMean), ', which should be close to 0']);
disp(' ');
disp(['Estimated variability: sigma_l = ', num2str(sigma_l)]);

function plot3DCube(x, y, height, color, width)
    % Define the vertices of the cube
    vertices = [
        x - width/2, y - width/2, 0; % Bottom face
        x + width/2, y - width/2, 0;
        x + width/2, y + width/2, 0;
        x - width/2, y + width/2, 0;
        x - width/2, y - width/2, height; % Top face
        x + width/2, y - width/2, height;
        x + width/2, y + width/2, height;
        x - width/2, y + width/2, height;
    ];

    % Define the faces of the cube
    faces = [
        1, 2, 6, 5; % Side faces
        2, 3, 7, 6;
        3, 4, 8, 7;
        4, 1, 5, 8;
        5, 6, 7, 8; % Top face
        1, 2, 3, 4; % Bottom face
    ];

    % Plot the cube
    patch('Vertices', vertices, 'Faces', faces, 'FaceColor', color, 'EdgeColor', 'none');
end
