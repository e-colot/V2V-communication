% constants and configuration of the project

function cfg = config()
    % setup the path
    project_root = fileparts(mfilename('fullpath')); % Get project root
    src_path = fullfile(project_root, 'src');
    if ~contains(path, src_path)
        addpath(genpath(src_path));
    end

    cfg = struct();
    cfg.fc = 5.9e9; % Hz carrier frequency
    cfg.c = 3e8; % m/s speed of light
    cfg.BW = 100e6; % RF bandwidth
    cfg.RX_sensitivity = -70; % dBm receiver sensitivity
    cfg.TX_power = 0.1; % W transmitter power
    cfg.bld_rel_perm = 4; % relative permittivity of buildings
    cfg.local_area_len = 5; % m length of the local area

end
