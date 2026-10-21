% 2D radar target tracking with a linear Kalman filter
% Georg Müller, 2026
%
% A simulated radar measures range and bearing of one target.
% The Kalman filter (constant velocity model) estimates position and speed.

clear; clc; close all;
rng(42);   % fixed seed so every run gives the same noise

%% settings
dt = 1;                       % radar scan interval [s]
N  = 120;                     % number of scans
radar_pos = [0; 0];           % radar at the origin

sigma_r     = 25;             % range noise [m]
sigma_theta = deg2rad(0.4);   % bearing noise [rad]
q = 20;                        % process noise, acceleration variance [m^2/s^4]

%% truth and measurements
X = simulate_target(N, dt);

Z  = zeros(2, N);
Rk = zeros(2, 2, N);
for k = 1:N
    [Z(:,k), Rk(:,:,k)] = radar_measurement(X(1:2,k), radar_pos, sigma_r, sigma_theta);
end

%% Kalman filter
F = [1 0 dt 0;
     0 1 0 dt;
     0 0 1  0;
     0 0 0  1];
G = [dt^2/2 0; 0 dt^2/2; dt 0; 0 dt];
Q = q * (G*G');
H = [1 0 0 0;
     0 1 0 0];

% two-point init: position from scan 2, velocity from the difference
x = [Z(:,2); (Z(:,2) - Z(:,1))/dt];
P = blkdiag(Rk(:,:,2), 2*Rk(:,:,2)/dt^2);

Xest = nan(4, N);
Xest(:,2) = x;
for k = 3:N
    [x, P] = kalman_step(x, P, Z(:,k), Rk(:,:,k), F, Q, H);
    Xest(:,k) = x;
end

%% results
idx = 3:N;
err_meas = vecnorm(Z(:,idx) - X(1:2,idx));
err_kf   = vecnorm(Xest(1:2,idx) - X(1:2,idx));

fprintf('RMSE raw measurements: %6.1f m\n', sqrt(mean(err_meas.^2)));
fprintf('RMSE Kalman filter:    %6.1f m\n', sqrt(mean(err_kf.^2)));

if ~exist('results', 'dir'), mkdir('results'); end

figure;
plot(X(1,:), X(2,:), 'k-', 'LineWidth', 1.5); hold on;
plot(Z(1,:), Z(2,:), 'r.');
plot(Xest(1,:), Xest(2,:), 'b-', 'LineWidth', 1.2);
plot(radar_pos(1), radar_pos(2), 'ks', 'MarkerFaceColor', 'k');
axis equal; grid on;
xlabel('x [m]'); ylabel('y [m]');
legend('true path', 'radar measurements', 'Kalman estimate', 'radar', 'Location', 'best');
title('2D target tracking');
saveas(gcf, 'results/track.png');

figure;
t = idx*dt;
plot(t, err_meas, 'r.'); hold on;
plot(t, err_kf, 'b-', 'LineWidth', 1.2);
grid on;
xlabel('time [s]'); ylabel('position error [m]');
legend('measurement', 'Kalman filter');
title('Position error');
saveas(gcf, 'results/error.png');
