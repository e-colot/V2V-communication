function gamma = reflexion(angle, epsilon)

    % Calculate the reflexion coefficient gamma based on the angle of incidence
    % and the relative permittivity of the material.
    % angle: angle of incidence in radians
    % cfg: configuration structure containing environment parameters

    % init
    persistent eps;
    if nargin == 2 % if cfg is passed, use it
        eps = epsilon;
    end

    % gamma = cos(angle_rad) - y / (cos(angle_rad) + y)
    y = sqrt(eps * (1 - sin(angle)^2 / eps)); 

    gamma = (cos(angle) - y) / (cos(angle) + y); % reflexion coefficient

end
