function ret_ant = reflectAntenna(ant, obstacle)
    % ant has 2 attributes:
    % pos (nx(2x1)), list of all the successive positions of the antenna 
    % mirrors ((n-1)x1), list of all obstacles on which the antenna has been mirrored

    % obstacle has 3 attributes:
    % start (1x2)
    % finish (1x2)
    % dir (1x2)
    ret_ant = ant;
    
    if (ret_ant.mirrors(1).start(1) == 0 && ret_ant.mirrors(1).start(2) == 0)
        ret_ant.mirrors(1) = obstacle;
    else
        ret_ant.mirrors(length(ant.mirrors) + 1) = obstacle;    
    end
    
    normal = [1, 0];
    if obstacle.dir(1) == 1 && obstacle.dir(2) == 0
        normal = [0, 1];
    end
    pos_ant = size(ant.pos);
    distance = dot(obstacle.start - ant.pos(:, pos_ant(2)), normal);

    ret_ant.pos(:, pos_ant(2) + 1) = ant.pos(:, pos_ant(2)) + (2*distance*normal).';
end

