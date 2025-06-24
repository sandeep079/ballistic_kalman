clear; clc; clf;

%% Load and preprocess data
data = csvread('projectile_log/projectile_log2.csv');
ball = csvread('projectile_log/ball_size2.csv');
Vel = 7.5;

% Extract ball dimensions and compute statistics
ball_x = ball(3:end, 2);
ball_y = ball(3:end, 3);
all_ball = [ball_x; ball_y];

mean_x = mean(ball_x);
std_x = std(ball_x);
mean_y = mean(ball_y);
std_y = std(ball_y);
mean_all_ball = mean(all_ball);
std_all_ball = std(all_ball);

%% Convert pixel coordinates to physical measurements (meters)
x_data = (data(3:end, 2) - 104) * (0.24 / mean_x);
y_data = ((480 - data(3:end, 3)) - 192) * (0.24 / mean_y);
time = data(3:end, 1) - 5.86;
new_data = [time, x_data, y_data];

%% Estimate initial conditions using linear regression
F = 0;          % Offset if needed
N = 2;          % Number of points for initial fit
tx = time(1+F:N+F);
xx = x_data(1+F:N+F);
yy = y_data(1+F:N+F);

% Linear fits: x(t) ≈ x0 + vx*t, y(t) ≈ y0 + vy*t
p_x = polyfit(tx, xx, 1);  % Returns [vx, x0]
p_y = polyfit(tx, yy, 1);  % Returns [vy, y0]

vx = p_x(1);    % Initial x-velocity (m/s)
vy = p_y(1);    % Initial y-velocity (m/s)
x0 = p_x(2);    % Initial x-position (m)
y0 = p_y(2);    % Initial y-position (m)

% Compute initial velocity magnitude and angle
v0 = sqrt(vx^2 + vy^2);
angle_rad = atan2(vy, vx);

% Display initial conditions
fprintf('x0 = %.2f m, y0 = %.2f m\n', x0, y0);
fprintf('vx = %.2f m/s, vy = %.2f m/s\n', vx, vy);
fprintf('v0 = %.2f m/s\n', v0);
fprintf('angle: %.2f radians\n', angle_rad);

%% Physical constants
rho = 1.225;    % Air density (kg/m^3)
r = 0.24/2;     % Ball radius (m)
A = pi * r^2;   % Cross-sectional area (m^2)
m = 0.62;       % Mass (kg)

% Compute velocity components from magnitude and angle
vel_x = Vel * cos(angle_rad);
vel_y = Vel * sin(angle_rad);

%% Parameter estimation (drag coefficient)
CDs = (0:0.001:0.05);      % Test range of drag coefficients
smallest_error = 10000;     % Initialize with large value
error_graph = zeros(length(CDs), 2);

for i = 1:length(CDs)
    Dr = CDs(i);
    
    % Simulate trajectory with current drag coefficient
    [x_sim, y_sim] = simulate_with_ode45(x0, y0, vel_x, vel_y, Dr);
    
    % Interpolate simulated y-values at measured x-positions
    y_interp = interp1(x_sim, y_sim, x_data, 'spline', 'extrap');
    
    % Compute and store error
    error_graph(i, 1) = Dr;
    error_graph(i, 2) = sum((y_interp - y_data).^2);
    
    % Track best fit
    if error_graph(i, 2) < smallest_error
        smallest_error = error_graph(i, 2);
        corresponding_Dr = Dr;
    end
end

fprintf('Optimal drag coefficient: %.4f\n', corresponding_Dr);
fprintf('Minimum error: %.4f m^2\n', smallest_error);

%% Generate final simulation with optimal parameters
[x_sim, y_sim] = simulate_with_ode45(x0, y0, vel_x, vel_y, corresponding_Dr);

%% Plot results
subplot(1, 2, 1);
plot(x_sim, y_sim, 'r-', 'LineWidth', 2);
hold on;
plot(x_data, y_data, 'bo');
xlabel('x (m)');
ylabel('y (m)');
title('Projectile Trajectory with Drag');
legend('Simulation', 'Experimental Data');
grid on;

subplot(1, 2, 2);
plot(error_graph(:,1), error_graph(:,2));
hold on;
plot(corresponding_Dr, smallest_error, 'ro', 'MarkerSize', 8);
xlabel('Drag Coefficient');
ylabel('Error (m^2)');
title('Drag Coefficient vs. Error');
grid on;

%% Simulation function (would be in separate file)
function [x_sim, y_sim] = simulate_with_ode45(x0, y0, vx0, vy0, Dr)
    % Define ODE system with quadratic drag
    function dy = projectile_ode(~, y)
        v = sqrt(y(2)^2 + y(4)^2);  % Velocity magnitude
        dy = zeros(4,1);
        dy(1) = y(2);               % dx/dt = vx
        dy(2) = -Dr * v * y(2);     % dvx/dt = -D*v*vx
        dy(3) = y(4);               % dy/dt = vy
        dy(4) = -9.81 - Dr * v * y(4); % dvy/dt = -g - D*v*vy
    end

    % Solve ODE
    [~, Y] = ode45(@projectile_ode, [0 3], [x0; vx0; y0; vy0]);
    x_sim = Y(:,1);
    y_sim = Y(:,3);
end