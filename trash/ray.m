classdef ray

    properties
        distance
        angle
        points
    end

    methods
        function obj = ray(start, finish)
            % start and finish are antennas, who have 2 attributes:
            % pos (nx(2x1)), list of all the successive positions of the antenna 
            % mirrors ((n-1)x1), list of all obstacles on which the antenna has been mirrored

            % obstacles have 3 attributes:
            % start (1x2)
            % finish (1x2)
            % dir (1x2)
            
            s = start.pos(:, end);
            f = finish.pos(:, end);
            vect = f - s;

            obj.distance = norm(vect);
            obj.angle = atan2(vect(2), vect(1));

            if (length(start.mirrors) == 1 && start.mirrors(1).start(1) == 0 && start.mirrors(1).start(2) == 0)
                obj.points = zeros(2, 2);
                obj.points(:, 1) = s;
                obj.points(:, 2) = f;
                return;
            end

            obj.points = zeros(2, length(start.mirrors) + 2);
            obj.points(:, length(start.mirrors) + 2) = f;
            obj.points(:, 1) = start.pos(:, 1);

            err = 0;
            for i = length(start.mirrors):-1:1

                pos_size = size(start.pos);
                s = start.pos(:, i + 1);
                vect = f - s;

                mirror = start.mirrors(i);

                normal = [1, 0];
                if mirror.dir(1) == 1 && mirror.dir(2) == 0
                    normal = [0, 1];
                end

                dist_norm = dot(mirror.start - s, normal);
                t = dot(s - mirror.start, mirror.dir) + dot(vect, mirror.dir) * dist_norm / dot(vect,normal);
                if t > 0 && t < dot(mirror.finish - mirror.start, mirror.dir)
                    obj.points(:, i + 1) = mirror.start + t * mirror.dir;
                    f = mirror.start + t * mirror.dir;
                else
                    err = 1;
                    break;
                end
            end

            if err == 1
                obj.distance = -1;
            end

        end

        function obj = show(obj)
            figure;
            hold on;
            for i = 1:size(obj.points, 2) - 1
                plot([obj.points(1, i), obj.points(1, i + 1)], [obj.points(2, i), obj.points(2, i + 1)], 'b-');
            end
            hold off;
        end
    end

end