clear; close all; clc;

cfg = config();

% Create rays
    fakeObstacle = struct();
    fakeObstacle.start = [0; 0];
    fakeObstacle.finish = [0; 0];
    fakeObstacle.dir = [0; 0];

    TX = struct();
    TX.pos = cfg.TX_pos;
    TX.mirrors = [fakeObstacle];

    RX = struct();
    RX.pos = cfg.RX_pos;
    RX.mirrors = [fakeObstacle];

    wall1 = struct();
    wall1.start = [-cfg.environment_params.road_width/2; -cfg.environment_params.road_length/2];
    wall1.finish = [-cfg.environment_params.road_width/2; cfg.environment_params.road_length/2];
    wall1.dir = [0; 1];

    wall2 = struct();
    wall2.start = [cfg.environment_params.road_width/2; -cfg.environment_params.road_length/2];
    wall2.finish = [cfg.environment_params.road_width/2; cfg.environment_params.road_length/2];
    wall2.dir = [0; 1];

    obstacles = [wall1, wall2];

    % Create reflected antennas
    antennas = cell(1, cfg.bounce_limit);
    antennas{1} = TX;
    rays(1) = ray(TX, RX);
    for i = 2:cfg.bounce_limit+1
        antennas{i} = [];
        for j = 1:length(antennas{i-1})
            for k = 1:length(obstacles)
                if (compareObstacles(antennas{i-1}(j).mirrors(end), obstacles(k)))
                    continue;
                end
                % disp(i*100 + j*10 + k);
                new_antenna = reflectAntenna(antennas{i-1}(j), obstacles(k));
                antennas{i} = [antennas{i}, new_antenna];
                new_ray = ray(new_antenna, RX);
                % % DEBUG
                % if (i*100 + j*10 + k == 312)
                %     new_ray = ray(antennas{i}(j), RX);
                %     if (new_ray.distance == -1)
                %         disp(new_ray.points);
                %         disp(antennas{i}(j).pos);
                %         continue;
                %     end
                % end
                if (new_ray.distance == -1)
                    disp(new_ray.points);
                    disp(antennas{i}(j).pos);
                    continue;
                end
                rays = [rays, new_ray];
            end
        end
    end



    visualize(cfg, rays);

function boolean = compareObstacles(obst1, obst2)
    boolean = obst1.start(1) == obst2.start(1) && obst1.start(2) == obst2.start(2) && obst1.finish(1) == obst2.finish(1) && obst1.finish(2) == obst2.finish(2) && obst1.dir(1) == obst2.dir(1) && obst1.dir(2) == obst2.dir(2);
end
