% =========================================================================
% PROJECTILE MOTION ANALYSIS WITH AIR DRAG ESTIMATION
% =========================================================================
%
% Description:
%   This script analyzes projectile motion data by:
%   1. Processing experimental trajectory measurements
%   2. Estimating initial conditions
%   3. Determining optimal drag coefficient through parameter fitting
%   4. Comparing simulated and experimental trajectories
%
% Input Files:
%   - projectile_log2.csv: Experimental trajectory data (time, x, y pixels)
%   - ball_size2.csv: Ball dimension calibration data
%
% Output:
%   - Optimal drag coefficient estimate
%   - Visual comparison of simulated and experimental trajectories
%   - Error analysis of drag coefficient fitting
%
% Physics Model:
%   Uses quadratic drag model: F_drag = 0.5*ρ*v^2*C_d*A
%
% Author: Sandeep Yadav
% Date: 2082-03-14
% Version: 1.0
% =========================================================================

%% Initialization
clear; clc; close all;

%% Data Loading and Preprocessing
% -------------------------------------------------------------------------
% Load experimental data and calibration data
data = csvread('projectile_log/projectile_log2.csv');
ball = csvread('projectile_log/ball_size2.csv');

% Extract ball dimensions for pixel-to-meter conversion
ball_x = ball(3:end, 2);
ball_y = ball(3:end, 3);
all_ball = [ball_x; ball_y];

% Calculate statistics for conversion
mean_x = mean(ball_x);
std_x = std(ball_x);
mean_y = mean(ball_y);
std_y = std(ball_y);
mean_all_ball = mean(all_ball);
std_all_ball = std(all_ball);

% Convert pixel coordinates to physical measurements (meters)
x_data = (data(3:end, 2) - 104) * (0.24 / mean_x);
y_data = ((480 - data(3:end, 3)) - 192) * (0.24 / mean_y);
time = data(3:end, 1) - 5.86;  % Time zero adjustment
new_data = [time, x_data, y_data];

%% Initial Condition Estimation
% -------------------------------------------------------------------------
% Using linear regression on first N points to estimate initial conditions
F = 0;          % Offset if needed
N = 2;          % Number of points for initial fit
tx = time(1+F:N+F);
xx = x_data(1+F:N+F);
yy = y_data(1+F:N+F);

% Linear fits: x(t) ≈ x0 + vx*t, y(t) ≈ y0 + vy*t
p_x = polyfit(tx, xx, 1);  % Returns [vx, x0]
p_y = polyfit(tx, yy, 1);  % Returns [vy, y0]

% Extract initial conditions
vx = p_x(1);    % Initial x-velocity (m/s)
vy = p_y(1);    % Initial y-velocity (m/s)
x0 = p_x(2);    % Initial x-position (m)
y0 = p_y(2);    % Initial y-position (m)

% Compute initial velocity magnitude and angle
v0 = sqrt(vx^2 + vy^2);
angle_rad = atan2(vy, vx);

% Display initial conditions
fprintf('\n=== Initial Conditions ===\n');
fprintf('x0 = %.4f m\n', x0);
fprintf('y0 = %.4f m\n', y0);
fprintf('vx = %.4f m/s\n', vx);
fprintf('vy = %.4f m/s\n', vy);
fprintf('v0 = %.4f m/s\n', v0);
fprintf('Launch angle = %.4f radians (%.2f degrees)\n', ...
        angle_rad, rad2deg(angle_rad));

%% Physical Constants Setup
% -------------------------------------------------------------------------
rho = 1.225;    % Air density (kg/m^3)
r = 0.24/2;     % Ball radius (m)
A = pi * r^2;   % Cross-sectional area (m^2)
m = 0.62;       % Mass (kg)
g = 9.81;       % Gravitational acceleration (m/s^2)

% User-specified initial velocity magnitude
Vel = 7.5;      % m/s

% Compute velocity components from magnitude and angle
vel_x = Vel * cos(angle_rad);
vel_y = Vel * sin(angle_rad);

%% Drag Coefficient Estimation
% -------------------------------------------------------------------------
% Parameter sweep to find optimal drag coefficient
CDs = (0:0.001:0.05);      % Test range of drag coefficients
smallest_error = inf;       % Initialize with infinity
error_graph = zeros(length(CDs), 2);

fprintf('\n=== Running Parameter Estimation ===\n');
for i = 1:length(CDs)
    Dr = CDs(i);
    
    % Simulate trajectory with current drag coefficient
    [x_sim, y_sim] = simulate_projectile(x0, y0, vel_x, vel_y, Dr);
    
    % Interpolate simulated y-values at measured x-positions
    y_interp = interp1(x_sim, y_sim, x_data, 'spline', 'extrap');
    
    % Compute and store error (sum of squared differences)
    current_error = sum((y_interp - y_data).^2);
    error_graph(i, 1) = Dr;
    error_graph(i, 2) = current_error;
    
    % Track best fit
    if current_error < smallest_error
        smallest_error = current_error;
        optimal_Dr = Dr;
    end
    
    % Progress feedback
    if mod(i,10) == 0
        fprintf('Completed %d/%d iterations (Current CD: %.4f)\n', ...
                i, length(CDs), Dr);
    end
end

fprintf('\n=== Optimal Parameters ===\n');
fprintf('Drag coefficient (CD): %.6f\n', optimal_Dr);
fprintf('Minimum error: %.6f m^2\n', smallest_error);

%% Final Simulation with Optimal Parameters
% -------------------------------------------------------------------------
[x_sim, y_sim] = simulate_projectile(x0, y0, vel_x, vel_y, optimal_Dr);

%% Visualization
% -------------------------------------------------------------------------
figure('Name', 'Projectile Motion Analysis', 'Position', [100 100 1200 500]);

% Trajectory comparison plot
subplot(1, 2, 1);
plot(x_sim, y_sim, 'r-', 'LineWidth', 2);
hold on;
plot(x_data, y_data, 'bo', 'MarkerFaceColor', 'b');
xlabel('Horizontal Position (m)');
ylabel('Vertical Position (m)');
title('Trajectory Comparison');
legend('Simulated', 'Experimental', 'Location', 'best');
grid on;
axis equal;

% Error analysis plot
subplot(1, 2, 2);
semilogy(error_graph(:,1), error_graph(:,2), 'b-');
hold on;
plot(optimal_Dr, smallest_error, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
xlabel('Drag Coefficient (C_D)');
ylabel('Error (log scale)');
title('Parameter Estimation Error');
grid on;

%% Projectile Simulation Function
% -------------------------------------------------------------------------
function [x_sim, y_sim] = simulate_projectile(x0, y0, vx0, vy0, Dr)
    % PROJECTILE_SIMULATION Solves ODE for projectile motion with drag
    %
    % Inputs:
    %   x0, y0: Initial position (m)
    %   vx0, vy0: Initial velocity (m/s)
    %   Dr: Drag coefficient (Dr = 0.5*ρ*C_d*A/m)
    %
    % Outputs:
    %   x_sim, y_sim: Simulated trajectory coordinates
    
    % Define ODE system with quadratic drag
    function dy = projectile_ode(~, y)
        v = sqrt(y(2)^2 + y(4)^2);  % Velocity magnitude
        dy = zeros(4,1);
        dy(1) = y(2);               % dx/dt = vx
        dy(2) = -Dr * v * y(2);     % dvx/dt = -D*v*vx
        dy(3) = y(4);               % dy/dt = vy
        dy(4) = -9.81 - Dr * v * y(4); % dvy/dt = -g - D*v*vy
    end

    % Solve ODE with event detection for ground impact
    options = odeset('Events', @ground_events);
    [~, Y] = ode45(@projectile_ode, [0 3], [x0; vx0; y0; vy0], options);
    
    % Extract position components
    x_sim = Y(:,1);
    y_sim = Y(:,3);
    
    % Nested event function for ground impact
    function [value, isterminal, direction] = ground_events(~, y)
        value = y(3);       % Detect when y = 0 (ground)
        isterminal = 1;     % Stop integration
        direction = -1;     % Detect decreasing through zero
    end
end