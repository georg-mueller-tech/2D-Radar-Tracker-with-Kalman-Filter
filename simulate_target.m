function X = simulate_target(N, dt)
% State per column: [x; y; vx; vy]

X = zeros(4, N);
X(:,1) = [9000; 3000; -100; 10];

w1 = deg2rad(4);        % turn rate [rad/s], negative = clockwise
Rot1 = [cos(w1*dt) -sin(w1*dt);
       sin(w1*dt)  cos(w1*dt)];


w2 = deg2rad(-4);  
Rot2 = [cos(w2*dt) -sin(w2*dt);
       sin(w2*dt)  cos(w2*dt)];


for k = 2:N
    v = X(3:4, k-1);
    if k > 15 && k <= 40
        v = Rot1 * v;
    elseif k > 50 && k <= 70
        v = Rot2 * v;
    end
    X(3:4, k) = v;
    X(1:2, k) = X(1:2, k-1) + v*dt;
end
end
