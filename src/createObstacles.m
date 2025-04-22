function obst = createObstacles(cfg)
    
    obst = zeros(2, 2, 8); % Initialize obstacles array
    % 2 points per wall, 2D coordinates, 8 walls
    % obst(:, :, 1) = [x1, y1; x2, y2] for wall 1

    % vertical walls
    % Vertical wall on the left side (bottom to middle)
    obst(1, :, 1) = [-cfg.environment_params.road_width/2, -cfg.environment_params.road_length/2];
    obst(2, :, 1) = [-cfg.environment_params.road_width/2, -cfg.environment_params.road_width/2];

    % Vertical wall on the left side (middle to top)
    obst(1, :, 2) = [-cfg.environment_params.road_width/2, cfg.environment_params.road_width/2];
    obst(2, :, 2) = [-cfg.environment_params.road_width/2, cfg.environment_params.road_length/2];

    % Vertical wall on the right side (bottom to middle)
    obst(1, :, 3) = [cfg.environment_params.road_width/2, -cfg.environment_params.road_length/2];
    obst(2, :, 3) = [cfg.environment_params.road_width/2, -cfg.environment_params.road_width/2];

    % Vertical wall on the right side (middle to top)
    obst(1, :, 4) = [cfg.environment_params.road_width/2, cfg.environment_params.road_width/2];
    obst(2, :, 4) = [cfg.environment_params.road_width/2, cfg.environment_params.road_length/2];

    % Horizontal wall at the bottom (left to middle)
    obst(1, :, 5) = [-cfg.environment_params.road_length/2, -cfg.environment_params.road_width/2];
    obst(2, :, 5) = [-cfg.environment_params.road_width/2, -cfg.environment_params.road_width/2];

    % Horizontal wall at the bottom (middle to right)
    obst(1, :, 6) = [cfg.environment_params.road_width/2, -cfg.environment_params.road_width/2];
    obst(2, :, 6) = [cfg.environment_params.road_length/2, -cfg.environment_params.road_width/2];

    % Horizontal wall at the top (left to middle)
    obst(1, :, 7) = [-cfg.environment_params.road_length/2, cfg.environment_params.road_width/2];
    obst(2, :, 7) = [-cfg.environment_params.road_width/2, cfg.environment_params.road_width/2];

    % Horizontal wall at the top (middle to right)
    obst(1, :, 8) = [cfg.environment_params.road_width/2, cfg.environment_params.road_width/2];
    obst(2, :, 8) = [cfg.environment_params.road_length/2, cfg.environment_params.road_width/2];

    % VERIFICATION
    % figure;
    % hold on;
    % for i = 1:size(obst, 3)
    %     x = squeeze(obst(:, 1, i));
    %     y = squeeze(obst(:, 2, i));
    %     plot(x, y, 'k-', 'LineWidth', 2); % Plot each wall as a black line
    % end
    % axis equal;
    % xlabel('X (m)');
    % ylabel('Y (m)');
    % title('Obstacles');
    % grid on;
    % hold off;

end