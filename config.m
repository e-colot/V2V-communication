% constants and configuration of the project

function cfg = config()
    % setup the path
    project_root = fileparts(mfilename('fullpath')); % Get project root
    src_path = fullfile(project_root, 'src');
    if ~contains(path, src_path)
        addpath(genpath(src_path));
    end

    cfg = struct();

    cfg.transmit_params = struct();
    cfg.transmit_params.fc = 5.9e9; % Hz carrier frequency
    cfg.transmit_params.c = 3e8; % m/s speed
    cfg.transmit_params.BW = 100e6; % RF
    cfg.transmit_params.RX_sensitivity = -70; % dBm
    cfg.transmit_params.TX_power = 0.1; % W

    cfg.environment_params = struct();

    cfg.environment_params.bld_rel_perm = 4; % relative permittivity of buildings
    cfg.environment_params.local_area_len = 5; % m length of the local area
    cfg.environment_params.road_width = 20; 
    cfg.environment_params.road_length = 500; % m length of the road

    cfg.environment_params.perpendicular_road_length = cfg.environment_params.road_length; % m length of the perpendicular road

    cfg.graphical_params = struct();
    cfg.graphical_params.avg_building_depth = 30; % m depth of buildings
    cfg.graphical_params.min_building_width = 15; % m minimum width of buildings
    cfg.graphical_params.max_building_width = 30; % m maximum width of buildings
    cfg.graphical_params.min_building_height = 10; % m minimum height of buildings
    cfg.graphical_params.max_building_height = 30; % m maximum height of buildings
    cfg.graphical_params.building_transparency = 0.3; % transparency of buildings
    cfg.graphical_params.car_size = 5; % size of the cars

    cfg.TX_pos = [-cfg.environment_params.road_width/4; -50];
    cfg.RX_pos = [50; -cfg.environment_params.road_width/4];
    
    cfg.bounce_limit = 3; % bounce limit

    cfg.obstacles = createObstacles(cfg); % create obstacles

end
