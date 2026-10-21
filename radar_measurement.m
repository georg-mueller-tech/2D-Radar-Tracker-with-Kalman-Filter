function [z, R] = radar_measurement(pos, radar_pos, sigma_r, sigma_theta)
% Simulated radar: measures range and bearing with noise.
% Converted to x/y so the linear Kalman filter can use it directly.

d = pos - radar_pos;
r     = norm(d) + sigma_r*randn;
theta = atan2(d(2), d(1)) + sigma_theta*randn;

z = radar_pos + [r*cos(theta); r*sin(theta)];

% noise covariance in x/y, linearized at the measurement
J = [cos(theta) -r*sin(theta);
     sin(theta)  r*cos(theta)];
R = J * diag([sigma_r^2, sigma_theta^2]) * J';
end
