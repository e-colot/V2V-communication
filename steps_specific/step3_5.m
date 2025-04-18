% getting access to the project
addpath(genpath('./..'));

clear; close all; clc;

cfg = config(); 

x = (-cfg.environment_params.road_length+cfg.environment_params.local_area_len)/2:cfg.environment_params.local_area_len:(cfg.environment_params.road_length-cfg.environment_params.local_area_len)/2;
y = (-cfg.environment_params.road_width+cfg.environment_params.local_area_len)/2:cfg.environment_params.local_area_len:(cfg.environment_params.road_width-cfg.environment_params.local_area_len)/2;

cfg.TX_pos = [0; 0];

avgPower = zeros(length(x), length(y)); % initialize power matrix
avgPowerPerp = zeros(length(y), length(x)); % initialize power matrix

for xi = 1:length(x)
    for yi = 1:length(y)
        cfg.RX_pos = [x(xi); y(yi)];

        rays = createRays(cfg);
        rays.voltages = rayVoltage(rays, cfg); % calculate the voltages

        avgPower(xi, yi) = sum(abs(rays.voltages).^2)/(45*pi);
        if avgPower(xi, yi) == 0
            avgPower(xi, yi) = NaN; % avoid log(0)
        end

        cfg.RX_pos = [y(yi); x(xi)];

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

% Add a colorbar with true avgPower values
cbar = colorbar;
cbar.FontSize = 12; % Set font size for colorbar
cbar.Ticks = linspace(0, 1, 5); % Set ticks for normalized values
cbar.TickLabels = arrayfun(@(v) sprintf('%.2f dBm', v), linspace(min(logPower(:)), max(logPower(:)), 5), 'UniformOutput', false);
cbar.Label.String = 'Average Power (dBm)';
title('Average Power Distribution in the Road Area', 'FontSize', 18);
view(140, 45);
