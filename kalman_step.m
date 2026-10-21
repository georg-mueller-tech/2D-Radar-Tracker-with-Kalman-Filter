function [x, P] = kalman_step(x, P, z, R, F, Q, H)
% One Kalman filter cycle: predict to the next scan, then correct with z.

% predict
x = F*x;
P = F*P*F' + Q;

% update
y = z - H*x;          % innovation (measurement minus prediction)
S = H*P*H' + R;       % innovation covariance
K = P*H' / S;         % Kalman gain
x = x + K*y;
P = (eye(length(x)) - K*H) * P;
end
