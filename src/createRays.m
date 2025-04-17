function rays = createRays(cfg)
    rays = struct();
    rays.points = [];
    rays.angles = [];
    rays.lengths = [];
    rays.reflexionAttenuation = [];
    raysCnt = 1;
    obstID = zeros(1, cfg.bounce_limit);

    while(norm(obstID - ones(1, cfg.bounce_limit) * length(cfg.obstacles)) ~= 0)
        % create the ray path (virtual antenna to virtual antenna)
        ray = nan(cfg.bounce_limit+2, 2);
        ray(1,:) = cfg.TX_pos';
        obst = [];
        i = 1;
        attenuationToCheck = 1;
        while(i <= length(obstID) && obstID(i) ~= 0)
            % adding reflexions on obstacles
            ray(i+1, :) = mirrorCoord(ray(i, :), cfg.obstacles(:, :, obstID(i)));
            obst(:,:,end+1) = cfg.obstacles(:, :, obstID(i));
            i = i + 1;
        end
        ray(i+1, :) = cfg.RX_pos;
        angleToCheck = atan2(ray(i+1, 2) - ray(i, 2), ray(i+1, 1) - ray(i, 1))*180/pi;
        lengthToCheck = norm(ray(i+1, :) - ray(i, :));
        [rayToCheck, attenuationToCheck] = instersectRay(obst, ray, size(obst, 3), attenuationToCheck);
        if ~isnan(rayToCheck(1))
            % check if the ray intersects an obstacle
            valid = 1;
            for j = 1:size(rayToCheck, 1)-1
                % for each segment of the ray
                segment = [rayToCheck(j, :); rayToCheck(j+1, :)];
                if isnan(rayToCheck(j+1, :))
                    break;
                end
                for k = 1:size(cfg.obstacles, 3)
                    % for each obstacle
                    obst = cfg.obstacles(:, :, k);
                    [coord, ~] = intersectVectors(segment, obst);
                    if (~isnan(coord(1)) && ~isnan(coord(2)))
                        % if the ray intersects with the obstacle
                        valid = 0;
                        break;
                    end
                end
            end
            if valid
                rays.points(:,:,raysCnt) = rayToCheck;
                rays.angles(raysCnt) = angleToCheck;
                rays.lengths(raysCnt) = lengthToCheck;
                rays.reflexionAttenuation(raysCnt) = attenuationToCheck;
                raysCnt = raysCnt + 1;
            end
        end

        % update obstID
        % used to get the obstacles for the next ray
        valid = 0;
        while(~valid)
            j = length(obstID);
            obstID(j) = obstID(j) + 1;
            while (obstID(j) > length(cfg.obstacles))
                obstID(j) = 0;
                j = j - 1;
                if j == 0
                    break;
                end
                obstID(j) = obstID(j) + 1;
            end
    
            % Check for validity of obstID
            valid = 1;
            j = 1;
            while (j <= length(obstID))
                if (obstID(j) == 0)
                    while (j+1 <= length(obstID))
                        j = j + 1;
                        if (obstID(j) ~= 0)
                            valid = 0;
                        end
                    end
                end
                j = j + 1;
            end
        end

    end
end

function [ray, attenuation] = instersectRay(obstacles, ray, index, attenuation)
    if index <= 1
        % no more bounces
        return;
    end
    % check if the ray intersects with the obstacle
    [coord, angle] = intersectVectors([ray(index, :); ray(index+1, :)], obstacles(:, :, index));
    if isnan(coord(1))
        ray = NaN;
        return;
    end
    % if the ray intersects with the obstacle
    ray(index, :) = coord;
    attenuation = attenuation * reflexion(angle);

    [ray, attenuation] = instersectRay(obstacles, ray, index-1, attenuation);
    return
end

function img = mirrorCoord(initial, obstacle)
    n = [obstacle(2, 2) - obstacle(1, 2), obstacle(1, 1) - obstacle(2, 1)]/norm([obstacle(2, 2) - obstacle(1, 2), obstacle(1, 1) - obstacle(2, 1)]);

    img = initial + 2*dot(obstacle(1, :) - initial, n)*n;
end
