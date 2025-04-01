function rays = createRays(cfg)
    % a ray will be a (cfg.bounce_limit + 2)x2 matrix
    % 1st row: 1st element is the attenuation, 2nd element is the nbr of bounces
    % 2nd row: initial position of the ray
    % and so on until the last row which is the final position of the ray

    prevAntenna = cfg.TX_pos;
    RX = cfg.RX_pos;
    obstacles = [];

    rays = zeros(cfg.bounce_limit + 2, 2);
    rays(1, :) = [1, 0]; % initial attenuation and bounces
    rays(2, :) = prevAntenna.'; % initial position of the ray
    rays(3, :) = RX.'; % final position of the ray


end

function ray = createSingleRay(start, obstacles, finish)
    N = size(obstacles, 3);
    
end
