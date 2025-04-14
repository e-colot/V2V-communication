function visualize(cfg, rays, angles, lengths)
    
    %% Parameters
    
    legendFull = 1;
    legendEntries = {};
    if nargin < 4
        legendFull = 0; 
        t = tiledlayout('flow','TileSpacing','compact');
        nexttile;
    end

    road_width = cfg.environment_params.road_width;          % Width of the road
    road_length = cfg.environment_params.road_length;        % Length of the road
    perp_road_length = cfg.environment_params.perpendicular_road_length; % Length of the perpendicular road
    avg_building_depth = cfg.graphical_params.avg_building_depth;  % Depth of buildings
    min_building_width = cfg.graphical_params.min_building_width;   % Minimum width of buildings
    max_building_width = cfg.graphical_params.max_building_width;  % Maximum width of buildings
    min_building_height = cfg.graphical_params.min_building_height; % Minimum height of buildings
    max_building_height = cfg.graphical_params.max_building_height; % Maximum height of buildings
    building_transparency = cfg.graphical_params.building_transparency;

    %% Figure Setup
    figure;
    hold on;
    axis equal;
    grid off;
    xlim([-perp_road_length/2, perp_road_length/2]);
    ylim([-road_length/2, road_length/2]);
    zlim([0, max_building_height + 2]);
    xlabel('X-axis (m)');
    ylabel('Y-axis (m)');
    zlabel('Z-axis (m)');

    %% Place Cars
    legendHandles = []; % Initialize an array to store handles for legend

    legendHandles = [legendHandles, scatter3(cfg.TX_pos(1), cfg.TX_pos(2), cfg.graphical_params.car_size, 200, 'r', 'filled')]; % Red car (left lane)
    legendHandles = [legendHandles, scatter3(cfg.RX_pos(1), cfg.RX_pos(2), cfg.graphical_params.car_size, 200, 'b', 'filled')];   % Blue car (left lane)

    %% Draw Rays

    

    colors = lines(size(rays, 3)); % Generate distinct colors for each ray
    if isempty(rays)
        % no rays to draw
    else 
        for i = 1:size(rays, 3)
            ray = rays(:, :, i);
            index = 1;
            while (index < (cfg.bounce_limit+2) && ~isnan(ray(index+1, 1)))
                % draw a line between ray(index, :) and ray(index+1, :)
                line_x = [ray(index, 1), ray(index+1, 1)];
                line_y = [ray(index, 2), ray(index+1, 2)];
                z_pos = [cfg.graphical_params.car_size, cfg.graphical_params.car_size]; % Set z position to car size
                % Draw the line with a unique color
                if index == 1
                    % First line, add to legend
                    legendHandles = [legendHandles, plot3(line_x, line_y, z_pos, 'Color', colors(i, :), 'LineWidth', 2)];
                else
                    plot3(line_x, line_y, z_pos, 'Color', colors(i, :), 'LineWidth', 2);
                end
                index = index + 1;
            end
        end
    end

    % Add legend entries for the rays
    if legendFull
        legendEntries = {'TX', 'RX'};
        if ~isempty(rays)
            % Add legend entries for each ray
            for i = 1:size(rays, 3)
                legendEntries{end+1} = ['Ray ' num2str(i) ' (Angle: ' num2str(angles(i)) '°, Length: ' num2str(lengths(i)) ' m)'];
            end
        end
        lgd = legend(legendHandles, legendEntries, 'AutoUpdate','off', 'Location','eastoutside', 'FontSize', 15);
    end

    %% Draw Road
    road_x = [-road_width/2, road_width/2, road_width/2, -road_width/2];
    road_y = [-road_length/2, -road_length/2, road_length/2, road_length/2];
    road_z = [0, 0, 0, 0]; % Flat road
    fill3(road_x, road_y, road_z, [0.2, 0.2, 0.2], 'EdgeColor', 'none'); % Dark gray

    %% Draw Dotted Line in the Middle of the Road
    line_y = linspace(-road_length/2, road_length/2, 100);
    line_z = zeros(size(line_y));
    line_x = zeros(size(line_y));
    plot3(line_x, line_y, line_z, 'w--', 'LineWidth', 2); % White dotted line

    %% Draw Perpendicular Road
    % Draw Perpendicular Road
    perp_road_x = [-perp_road_length/2, perp_road_length/2, perp_road_length/2, -perp_road_length/2];
    perp_road_y = [-road_width/2, -road_width/2, road_width/2, road_width/2];
    perp_road_z = [0, 0, 0, 0]; % Flat road
    fill3(perp_road_x, perp_road_y, perp_road_z, [0.2, 0.2, 0.2], 'EdgeColor', 'none'); % Dark gray

    % Draw Dotted Line in the Middle of the Perpendicular Road
    perp_line_x = linspace(-perp_road_length/2, perp_road_length/2, 100);
    perp_line_y = zeros(size(perp_line_x));
    perp_line_z = zeros(size(perp_line_x));
    plot3(perp_line_x, perp_line_y, perp_line_z, 'w--', 'LineWidth', 2); % White dotted line

    %% Draw Buildings (3D Boxes)
    y_pos = -road_length/2;
    midFlag = 0;

    while y_pos < road_length/2
        % Random building dimensions
        building_width = min_building_width + (max_building_width - min_building_width) * rand;
        building_heightL = min_building_height + (max_building_height - min_building_height) * rand;
        building_heightR = min_building_height + (max_building_height - min_building_height) * rand;

        if (midFlag == 0) & (y_pos + max_building_width > -road_width/2)
            % Adjust the building width to fit the perpendicular road
            building_width = -road_width/2 - y_pos;
            midFlag = 1; % Set flag to indicate that the middle building has been drawn
        elseif (midFlag == 1)
            y_pos = road_width/2;
            midFlag = 2; % Set flag to indicate that the middle building has been drawn
        end

        % Left-side buildings
        drawBuilding(-road_width/2 - avg_building_depth, y_pos, building_width, avg_building_depth, building_heightL, building_transparency);

        % Right-side buildings
        drawBuilding(road_width/2, y_pos, building_width, avg_building_depth, building_heightR, building_transparency);

        % Update y_pos for next building
        y_pos = y_pos + building_width;
    end

    %% Draw Buildings Around Perpendicular Road
    x_pos = -perp_road_length/2;
    midFlag = 0;
    while x_pos < perp_road_length/2
        % Random building dimensions
        building_width = min_building_width + (max_building_width - min_building_width) * rand;
        building_heightT = min_building_height + (max_building_height - min_building_height) * rand;
        building_heightB = min_building_height + (max_building_height - min_building_height) * rand;

        if (midFlag == 0) & (x_pos + max_building_width > -road_width/2-avg_building_depth)
            % Adjust the building width to fit the perpendicular road
            building_width = -road_width/2-avg_building_depth - x_pos;
            midFlag = 1; % Set flag to indicate that the middle building has been drawn
        elseif (midFlag == 1)
            x_pos = avg_building_depth+road_width/2;
            midFlag = 2; % Set flag to indicate that the middle building has been drawn
        end

        % Top-side buildings
        drawBuilding(x_pos, road_width/2, avg_building_depth, building_width, building_heightT, building_transparency);

        % Bottom-side buildings
        drawBuilding(x_pos, -road_width/2-avg_building_depth, avg_building_depth, building_width, building_heightB, building_transparency);

        % Update x_pos for next building
        x_pos = x_pos + building_width;
    end

    %% Camera Settings
    view(111, 45);
    camlight;
    lighting gouraud;
    title('Road');
    rotate3d on; % Enable rotation
    set(gca, 'CameraViewAngleMode', 'manual'); % Prevent displacement
    %set(gca,'XTick',[],'YTick',[],'ZTick',[])

    hold off;
end

%% Function to Draw Buildings (3D Box)
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
