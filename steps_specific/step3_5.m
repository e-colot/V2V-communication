% getting access to the project
addpath(genpath('./..'));

clear; close all; clc;

cfg = config(); 

cfg.bounce_limit = 3;
cfg.environment_params.local_area_len = 5;

cfg.environment_params.road_length = 2000;

%recompute obstacles
cfg.obstacles = zeros(2, 2, 2);
cfg.obstacles(1, :, 1) = [-1100, -cfg.environment_params.road_width/2];
cfg.obstacles(2, :, 1) = [1100, -cfg.environment_params.road_width/2];

cfg.obstacles(1, :, 2) = [-1100, cfg.environment_params.road_width/2];
cfg.obstacles(2, :, 2) = [1100, cfg.environment_params.road_width/2];


x = (-cfg.environment_params.road_length+cfg.environment_params.local_area_len)/2:cfg.environment_params.local_area_len:(cfg.environment_params.road_length-cfg.environment_params.local_area_len)/2;
y = (-cfg.environment_params.road_width+cfg.environment_params.local_area_len)/2:cfg.environment_params.local_area_len:(cfg.environment_params.road_width-cfg.environment_params.local_area_len)/2;

TX_y_pos = -cfg.environment_params.road_width/2+1:1:cfg.environment_params.road_width/2-1;

avgPower = zeros(length(x), length(y), length(TX_y_pos)); % initialize power matrix
distance = zeros(length(x), length(y), length(TX_y_pos)); % initialize distance matrix

hWait = waitbar(0, 'Processing...'); % Initialize waitbar
for TX_y = 1:length(TX_y_pos)
    cfg.TX_pos = [0; TX_y_pos(TX_y)];
    for xi = 1:length(x)
        for yi = 1:length(y)
            cfg.RX_pos = [x(xi); y(yi)];
            distance(xi, yi, TX_y) = norm(cfg.RX_pos - cfg.TX_pos); % calculate distance

            rays = createRays(cfg);
            rays.voltages = rayVoltage(rays, cfg); % calculate the voltages

            avgPower(xi, yi, TX_y) = sum(abs(rays.voltages).^2)/(45*pi);
            if avgPower(xi, yi, TX_y) == 0
                avgPower(xi, yi, TX_y) = NaN; % avoid log(0)
            end
        end
        waitbar((xi + length(x)*TX_y) / (length(x)*length(TX_y_pos)), hWait); % Update waitbar
    end
end
close(hWait); % Close waitbar

% Calculate the average power over the TX positions
TX_avgPower = mean(avgPower, 3, 'omitnan');


% Normalize the log of TX_avgPower
% multiplied by 1e3 to convert to mW and then to dBm
logPower = 10*log10(TX_avgPower*1e3);
minVal = min(min(logPower));
maxVal = max(max(logPower));
normalizedPower = (logPower - minVal) / (maxVal - minVal);


% draw buildings
    road_width = cfg.environment_params.road_width;          % Width of the road
    road_length = cfg.environment_params.road_length;        % Length of the road
    perp_road_length = cfg.environment_params.perpendicular_road_length; % Length of the perpendicular road
    avg_building_depth = cfg.graphical_params.avg_building_depth;  % Depth of buildings
    min_building_width = cfg.graphical_params.min_building_width;   % Minimum width of buildings
    max_building_width = cfg.graphical_params.max_building_width;  % Maximum width of buildings
    min_building_height = cfg.graphical_params.min_building_height; % Minimum height of buildings
    max_building_height = cfg.graphical_params.max_building_height; % Maximum height of buildings
    building_transparency = cfg.graphical_params.building_transparency;

x_pos = -perp_road_length/2;
while x_pos < perp_road_length/2
    % Random building dimensions
    building_width = min_building_width + (max_building_width - min_building_width) * rand;
    building_heightT = min_building_height + (max_building_height - min_building_height) * rand;
    building_heightB = min_building_height + (max_building_height - min_building_height) * rand;

    % Top-side buildings
    drawBuilding(x_pos, road_width/2, avg_building_depth, building_width, building_heightT, building_transparency);

    % Bottom-side buildings
    drawBuilding(x_pos, -road_width/2-avg_building_depth, avg_building_depth, building_width, building_heightB, building_transparency);

    % Update x_pos for next building
    x_pos = x_pos + building_width;
end
zlim([0, 70]); % Set z-axis limits
ylim([-cfg.environment_params.road_width/2-avg_building_depth, cfg.environment_params.road_width/2+avg_building_depth]);
xlim([-250, 250]);
axis equal;
    
% Camera Settings
view(-50, 15);
camlight;
lighting gouraud;

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
    end
end

% Add a colorbar with true avgPower values
cbar = colorbar;
cbar.FontSize = 12; % Set font size for colorbar
cbar.Ticks = linspace(0, 1, 5); % Set ticks for normalized values
cbar.TickLabels = arrayfun(@(v) sprintf('%.2f dBm', v), linspace(min(logPower(:)), max(logPower(:)), 5), 'UniformOutput', false);


%% path loss model

logPower = 10*log10(avgPower*1e3);
% Combine logPower into a single vector L
L = -[logPower(:)] + 10*log10(cfg.transmit_params.TX_power*1e3); % remove TX power in dBm
d = [distance(:)]; % combine distances

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
xlim([0 250]);
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

% Function to Draw Buildings (3D Box)
function drawBuilding(x, y, depth, width, height, building_transparency)
    % x, y: Coordinates of the bottom corner of the building base with the lowest values along the x and y axes
    % depth: Extends along the y-axis
    % width: Extends along the x-axis
    % height: Extends along the z-axis

    % Define the vertices of the building (3D box)
    vertices = [
        x, y, 0;  % Bottom front-left (origin corner)
        x + width, y, 0;  % Bottom front-right
        x + width, y + depth, 0;  % Bottom back-right
        x, y + depth, 0;  % Bottom back-left
        x, y, height;  % Top front-left
        x + width, y, height;  % Top front-right
        x + width, y + depth, height;  % Top back-right
        x, y + depth, height  % Top back-left
    ];

    % Define the faces of the building using the vertices
    faces = [
        1 2 6 5; % Front face
        2 3 7 6; % Right face
        3 4 8 7; % Back face
        4 1 5 8; % Left face
        5 6 7 8; % Top face
        1 2 3 4; % Bottom face
    ];

    % Draw the building as a 3D patch
    patch('Vertices', vertices, 'Faces', faces, 'FaceColor', [0.6 0.6 0.9], 'EdgeColor', 'k', 'FaceAlpha', building_transparency);
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
