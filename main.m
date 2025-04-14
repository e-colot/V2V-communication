clear; close all; clc;

cfg = config(); 


[rays, angles, lengths] = createRays(cfg);

visualize(cfg, rays);
