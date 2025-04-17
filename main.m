clear; close all; clc;

cfg = config(); 


rays = createRays(cfg);
rays.voltages = rayVoltage(rays, cfg);

visualize(cfg, rays, 0);
