function visualize(cfg, rays)
    %% Parameters
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
    while y_pos < road_length/2
        % Random building dimensions
        building_width = min_building_width + (max_building_width - min_building_width) * rand;
        building_heightL = min_building_height + (max_building_height - min_building_height) * rand;
        building_heightR = min_building_height + (max_building_height - min_building_height) * rand;

        % Left-side buildings
        drawBuilding(-road_width/2 - avg_building_depth, y_pos + building_width/2, building_width, avg_building_depth, building_heightL, building_transparency);

        % Right-side buildings
        drawBuilding(road_width/2, y_pos + building_width/2, building_width, avg_building_depth, building_heightR, building_transparency);

        % Update y_pos for next building
        y_pos = y_pos + building_width;
    end

    %% Draw Buildings Around Perpendicular Road
    x_pos = -perp_road_length/2;
    while x_pos < perp_road_length/2
        % Random building dimensions
        building_width = min_building_width + (max_building_width - min_building_width) * rand;
        building_heightT = min_building_height + (max_building_height - min_building_height) * rand;
        building_heightB = min_building_height + (max_building_height - min_building_height) * rand;

        % Top-side buildings
        drawBuilding(x_pos + building_width/2, road_width/2 + avg_building_depth, avg_building_depth, building_width, building_heightT, building_transparency);

        % Bottom-side buildings
        drawBuilding(x_pos + building_width/2, -road_width/2, avg_building_depth, building_width, building_heightB, building_transparency);

        % Update x_pos for next building
        x_pos = x_pos + building_width;
    end

    %% Place Cars
    scatter3(cfg.TX_pos(1), cfg.TX_pos(2), cfg.graphical_params.car_size, 200, 'r', 'filled'); % Red car (left lane)
    scatter3(cfg.RX_pos(1), cfg.RX_pos(2), cfg.graphical_params.car_size, 200, 'b', 'filled');   % Blue car (left lane)

    %% Draw Rays
    colormap(jet); % Set the colormap to 'jet'
    colorbar; % Display the colorbar

    for i = 1:length(rays)
        ray = rays(i);
        if ray.distance == -1
            continue;
        end
        % Normalize the distance to a value between 0 and 1 for color mapping
        normalized_distance = (ray.distance - min([rays.distance])) / (max([rays.distance]) - min([rays.distance]));
        % Map the normalized distance to a color using the colormap
        ray_color = jet(256);
        color_idx = round(normalized_distance * 255) + 1;
        plot3(ray.points(1, :), ray.points(2, :), cfg.graphical_params.car_size*ones(1, length(ray.points)), 'Color', ray_color(color_idx, :), 'LineWidth', 1);
    end

    %% Camera Settings
    view(15, 8);
    camlight;
    lighting gouraud;
    title('Environment');
    rotate3d on; % Enable rotation
    set(gca, 'CameraViewAngleMode', 'manual'); % Prevent displacement
    set(gca,'XTick',[],'YTick',[],'ZTick',[])

    hold off;
end

%% Function to Draw Buildings (3D Box)
function drawBuilding(x, y, depth, width, height, building_transparency)
    vertices = [
        x, y - depth/2, 0;  % Bottom front-left
        x + width, y - depth/2, 0;  % Bottom front-right
        x + width, y + depth/2, 0;  % Bottom back-right
        x, y + depth/2, 0;  % Bottom back-left
        x, y - depth/2, height;  % Top front-left
        x + width, y - depth/2, height;  % Top front-right
        x + width, y + depth/2, height;  % Top back-right
        x, y + depth/2, height  % Top back-left
    ];

    faces = [
        1 2 6 5; % Front
        2 3 7 6; % Right
        3 4 8 7; % Back
        4 1 5 8; % Left
        5 6 7 8; % Top
        1 2 3 4; % Bottom
    ];

    patch('Vertices', vertices, 'Faces', faces, 'FaceColor', [0.6 0.6 0.9], 'EdgeColor', 'k', 'FaceAlpha', building_transparency);
end
