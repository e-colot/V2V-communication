% getting access to the project
addpath(genpath('./..'));

clear; close all; clc;

cfg = config(); 

cfg.TX_pos = [150; -cfg.environment_params.road_width/4];
cfg.RX_pos = [50; -cfg.environment_params.road_width/4];


[rays, angles, lengths] = createRays(cfg);

visualize(cfg, rays, angles, lengths);

xlim([30 170]);
ylim([-cfg.environment_params.road_width cfg.environment_params.road_width]);
view(114, 47);


