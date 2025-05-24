% getting access to the project
addpath(genpath('./..'));

clear; close all; clc;

cfg = config(); 
cfg.bounce_limit = 3;

cfg.environment_params.road_length = 2000;
cfg.obstacles = createObstacles(cfg); % recompute obstacles

tmp_obstacles(:, :, 1) = cfg.obstacles(:, :, 6);
tmp_obstacles(:, :, 2) = cfg.obstacles(:, :, 8);
cfg.obstacles = tmp_obstacles;

cfg.TX_pos = [50; -cfg.environment_params.road_width/4];
distances = (0:5:1000)+1;

T = 5e-6; % time window
t = 0:1e-11:T;
f = -(length(t)-1)/(2*T):1/T:(length(t)-1)/(2*T);

% H will only contain frequencies between 5.5 GHz and 6.5 GHz
fkept = f(f >= 5.5e9 & f <= 6.5e9); % filter

H = zeros(length(distances), length(fkept));

waitB = waitbar(0, 'Processing...'); % Initialize waitbar
for iDist = 1: length(distances)
    distance = distances(iDist);
    cfg.RX_pos = [50+distance; -cfg.environment_params.road_width/4];

    rays = createRays(cfg);

    alpha = rays.reflexionAttenuation .* exp(-j*2*pi*cfg.transmit_params.fc.*rays.lengths./cfg.transmit_params.c)./rays.lengths;

    %visualize(cfg, rays, 1);

    timeOfFlight = rays.lengths ./ cfg.transmit_params.c;

    % construction of h(t)
    h = zeros(1, length(t));
    for i = 1:length(alpha)
        closestBin = find(t >= timeOfFlight(i), 1);
        h(closestBin) = h(closestBin) + alpha(i);
    end

    % construction of H(f)
    Htotal = fftshift(fft(h)); % FFT of h(t)
    H(iDist, :) = Htotal(f >= 5.5e9 & f <= 6.5e9); % Keep only the frequencies of interest
    waitbar(iDist / length(distances), waitB); % Update waitbar
end

close(waitB); % Close waitbar

%% Plots

% plot in 3D the amplitude of H(f)
figure;
[X, Y] = meshgrid(distances, fkept*1e-9);
surf(X', Y', db(H), 'EdgeColor', 'none');
xlabel('Distance (m)');
ylabel('Frequency (GHz)');
zlabel('|H(f)| (dB)');

%% Optimal frequency

meanH = mean(abs(H), 1); % mean over the distances
[~, fOptIndex] = max(meanH); % index of the optimal frequency

fOpt = fkept(fOptIndex); % optimal frequency

figure;
plot(fkept*1e-9, meanH, 'LineWidth', 2);
hold on;
plot(fOpt*1e-9, meanH(fOptIndex), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
xlabel('Frequency (GHz)');
ylabel('Mean |H(f)|');

H_corrected = H .* (distances)';

figure;
[Xc, Yc] = meshgrid(distances, fkept*1e-9);
surf(Xc', Yc', db(abs(H_corrected)), 'EdgeColor', 'none');
xlabel('Distance (m)');
ylabel('Frequency (GHz)');
zlabel('|H_{corrected}(f)| (dB)');
colorbar;

% Find the minimal value of |H_corrected| across distances for each frequency
minHcorr = min(abs(H_corrected), [], 1);

% Find the threshold for the top 10% frequencies
nTop = ceil(0.10 * length(fkept));
[~, sortedIdx] = sort(minHcorr, 'descend');
topIdx = sortedIdx(1:nTop);

% Extract the corresponding frequencies
topFrequencies = fkept(topIdx);

% Remove 90% of the remaining frequencies with the highest variance across distances
remainingIdx = setdiff(1:length(fkept), topIdx); % indices not in top 10%
H_remaining = H(:, remainingIdx);

% Compute variance across distances for each remaining frequency
varH = var(abs(H_remaining), 0, 1);

% Find indices of the 10% lowest variance frequencies
nKeep = ceil(0.10 * length(remainingIdx));
[~, varSortIdx] = sort(varH, 'ascend');
lowestVarIdx = remainingIdx(varSortIdx(1:nKeep));

% The final set of frequencies after both selections
finalFrequencies = fkept(lowestVarIdx);

% Find the frequency with the highest maximum |H(f)| across all distances
[maxH, maxIdx] = max(max(abs(H), [], 1));
optimalFrequency = fkept(maxIdx);

fprintf('The optimal frequency is %.3f GHz\n', optimalFrequency * 1e-9);

