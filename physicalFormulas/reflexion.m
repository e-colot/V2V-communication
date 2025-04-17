function gamma = reflexion(angle, cfg)

    % Calculate the reflexion coefficient gamma based on the angle of incidence
    % and the relative permittivity of the material.
    % angle: angle of incidence in degrees
    % cfg: configuration structure containing environment parameters

    % Convert angle to radians
    angle_rad = deg2rad(angle);

    % gamma = cos(angle_rad) - y / (cos(angle_rad) + y)

    eps = cfg.environment_params.bld_rel_perm; % relative permittivity of the buildings
    y = sqrt(eps*(1-sin(angle_rad)^2/eps)); 

    gamma = cos(angle_rad) - y / (cos(angle_rad) + y); % reflexion coefficient

end
