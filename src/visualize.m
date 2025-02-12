function visualize(T)
    % T is the simulation time in seconds

    %% Simulation Parameters
    dt = 0.1;             % Time step (s)
    speed = 15;           % Speed of vehicles (m/s)
    distance_between = 800;
    road_length = 200 + distance_between + speed * T;
    road_width = 20;     

    %% Vehicle Initialization
    car1.x = 100;        % Initial position of Car 1
    car2.x = car1.x + distance_between; % Car 2 starts ahead
    car1.y = 0;        % Both cars on the same lane
    car2.y = 0;

    %% Simulation Loop
    figure;
    for t = 0:dt:T
        % Update vehicle positions (moving at constant speed)
        car1.x = car1.x + speed * dt;
        car2.x = car2.x + speed * dt;
        
        % Visualization
        clf; hold on; grid on;
        axis([0 road_length -20 20]);
        
        % Plot road
        fill([0 road_length road_length 0], [-road_width/2 -road_width/2 road_width/2 road_width/2], 'k');

        % Plot buildings
        plot([0 road_length], [road_width/2 road_width/2], 'Color', [0.6 0.6 0.6], 'LineWidth', 10);
        plot([0 road_length], [-road_width/2 -road_width/2], 'Color', [0.6 0.6 0.6], 'LineWidth', 10);
        
        % Plot vehicles
        plot(car1.x, car1.y, 'bs', 'MarkerSize', 15, 'MarkerFaceColor', 'b');
        plot(car2.x, car2.y, 'rs', 'MarkerSize', 15, 'MarkerFaceColor', 'r');
        
        
        pause(0.05);
    end

end
