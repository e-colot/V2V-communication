% getting access to the project
addpath(genpath('./..'));

clear; close all; clc;

cfg = config(); 

cfg.bounce_limit = 3;
cfg.local_area_len = 5;

cfg.environment_params.road_length = 2000;
cfg.obstacles = createObstacles(cfg); % create obstacles


x = (-cfg.environment_params.road_length+cfg.environment_params.local_area_len)/2:cfg.environment_params.local_area_len:(cfg.environment_params.road_length-cfg.environment_params.local_area_len)/2;
y = (-cfg.environment_params.road_width+cfg.environment_params.local_area_len)/2:cfg.environment_params.local_area_len:(cfg.environment_params.road_width-cfg.environment_params.local_area_len)/2;

cfg.TX_pos = [0; 0];

avgPower = zeros(length(x), length(y)); % initialize power matrix
distance = zeros(length(x), length(y)); % initialize distance matrix
avgPowerPerp = zeros(length(y), length(x)); % initialize power matrix
distancePerp = zeros(length(y), length(x)); % initialize distance matrix

hWait = waitbar(0, 'Processing...'); % Initialize waitbar
for xi = 1:length(x)
    for yi = 1:length(y)
        cfg_local = cfg; % Create a local copy of cfg for each worker
        cfg_local.RX_pos = [x(xi); y(yi)];
        distance(xi, yi) = norm(cfg_local.RX_pos - cfg_local.TX_pos); % calculate distance

        rays = createRays(cfg_local);
        rays.voltages = rayVoltage(rays, cfg_local); % calculate the voltages

        avgPower(xi, yi) = sum(abs(rays.voltages).^2)/(45*pi);
        if avgPower(xi, yi) == 0
            avgPower(xi, yi) = NaN; % avoid log(0)
        end

        cfg_local.RX_pos = [y(yi); x(xi)];
        distancePerp(yi, xi) = norm(cfg_local.RX_pos - cfg_local.TX_pos); % calculate distance

        rays = createRays(cfg_local);
        rays.voltages = rayVoltage(rays, cfg_local); % calculate the voltages

        avgPowerPerp(yi, xi) = sum(abs(rays.voltages).^2)/(45*pi);
        if avgPowerPerp(yi, xi) == 0
            avgPowerPerp(yi, xi) = NaN; % avoid log(0)
        end
    end
    waitbar(xi / length(x), hWait); % Update waitbar
end
close(hWait); % Close waitbar

% Normalize the log of avgPower
% multiplied by 1e3 to convert to mW and then to dBm
logPower = 10*log10(avgPower*1e3);
logPowerPerp = 10*log10(avgPowerPerp*1e3);
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

xlim([-250 250]);
ylim([-250 250]);

%% path loss model

% Combine logPower and logPowerPerp into a single vector L
L = -[logPower(:); logPowerPerp(:)] + 10*log10(cfg.transmit_params.TX_power*1e3); % remove TX power in dBm
d = [distance(:); distancePerp(:)]; % combine distances

validIdx = ~isnan(L); % find indices where L is not NaN
L = L(validIdx); % keep only valid entries in L
d = d(validIdx); % keep corresponding entries in d

L_0 = L + 20*log10(16/(3*pi)); % removing the loss due to the antennas


% path loss model:
% L = L_0 + 10*n*log10(d/d_0)
% where d_0 is the reference distance (1m)

% --> LLS estimator to find d_0 and n
H = [ones(length(d), 1), 10*log10(d)]; % regression matrix
y = L_0;
params = H\y; % least squares solution

x = linspace(0, max(d), 1000); % x values for the fitted line
y_fit = params(1) + params(2)*10*log10(x); % fitted line

figure;
plot(x, y_fit, 'r', 'LineWidth', 1.5); % plot the fitted line
xlabel('Distance (m)');
ylabel('Path Loss (dB)');
title('Path Loss vs Distance');
grid on;
hold on;

%plot the data points
scatter(d, L_0, 'x', 'b', 'LineWidth', 1.2); 

% Friis path loss:
% L_friis = (lambda/(4*pi*d))^2 
% --> in dB:
y_friis = -20*log10(cfg.transmit_params.c ./ (cfg.transmit_params.fc*4*pi*x));
plot(x, y_friis, 'g--', 'LineWidth', 1.5); % plot the fitted line

legend('LLS model', 'Data points', 'Friis model', 'Location', 'Best');
hold off;

disp('Estimated model:');
disp(['L_0(d) = ', num2str(params(1)), ' + ' num2str(params(2)), '*10*log(d)']);


%% 3.6
% sigma_l is the standard deviation of the experimental path loss around the fitted one
sigma_l = std(L_0 - (params(1) + params(2)*10*log10(d)));
shadowingMean = mean(L_0 - (params(1) + params(2)*10*log10(d)));
disp(['Estimated variability: sigma_l = ', num2str(sigma_l)]);
disp(' ');

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

%% 3.7 range
x = -1:1e-3:50;
y = 1/2 * erfc(x./(sigma_l*sqrt(2)));

figure;
p = semilogy(x, y, 'LineWidth', 2);
hold on;
% Add data tips close to y = 0.5, y = 0.05, and y = 0.01
y_targets = [0.5, 0.05, 0.01];
margins = zeros(3, 1);
maxX = 0;
minX = 0;
for i = 1:length(y_targets)
    [~, idx] = min(abs(y - y_targets(i)));
    x_val = x(idx);
    y_val = y(idx);
    % Update maxX and maxY for the datatip
    maxX = max(maxX, x_val);
    minX = min(minX, x_val);
    datatip(p, x_val, y_val);
    margins(i) = x_val;
end
ylim([1e-3 1]);
varX = (maxX - minX);
xlim([minX - varX/5, maxX + varX/5]);
xlabel('Fade margin (dB)');
ylabel('Probability of outage');

for i = 1:length(margins)

    cellRange = 10^((10*log10(cfg.transmit_params.TX_power*1e3)-cfg.transmit_params.RX_sensitivity-margins(i)+20*log10(16/(3*pi))-params(1))/(10*params(2)));
    disp(['Range for outage probability ', num2str(y_targets(i)), ': ', num2str(cellRange), ' m']);

end
