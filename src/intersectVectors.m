function [coord, angle] = intersectVectors(v1, v2)

    d1 = (v1(2,:) - v1(1,:))/norm(v1(2,:) - v1(1,:));
    d2 = (v2(2,:) - v2(1,:))/norm(v2(2,:) - v2(1,:));
    n1 = [d1(2), -d1(1)];
    n2 = [d2(2), -d2(1)];

    angle = acos(dot(d1, n2));

    perpDRatio = dot(v1(1,:) - v2(1,:), n1)/dot(d2, n1);
    coord = v2(1,:) + perpDRatio*d2;
    projd1 = dot(coord-v1(1,:), d1);
    projd2 = dot(coord-v2(1,:), d2);
    if (projd1 < 1e-3) || (projd1 > norm(v1(2,:) - v1(1,:)) - 1e-3) || (projd2 < 1e-3) || (projd2 > norm(v2(2,:) - v2(1,:)) - 1e-3)
        % The intersection point is outside the segments
        coord = [NaN, NaN];
    end

    % VERIFICATION
    % clf;
    % plot(v1(:,1), v1(:,2), 'r', 'LineWidth', 2); % Plot the first vector in red
    % hold on;
    % plot(v2(:,1), v2(:,2), 'b', 'LineWidth', 2); % Plot the second vector in blue

end

