function [x_vals, y_vals] = simulate_with_ode45(x0, y0, vx0, vy0, D)
    % SIMULATE_WITH_ODE45 - Simulates projectile motion with quadratic drag
    % 
    % Inputs:
    %   x0   - Initial x-position (m)
    %   y0   - Initial y-position (m)
    %   vx0  - Initial x-velocity (m/s)
    %   vy0  - Initial y-velocity (m/s)
    %   D    - Drag coefficient (1/m)
    %
    % Outputs:
    %   x_vals - Array of x-positions (m)
    %   y_vals - Array of y-positions (m)
    
    %% Physical Constants
    g = 9.81;  % Acceleration due to gravity (m/s^2)
    
    %% Initial Conditions
    % State vector: [x; y; vx; vy]
    y_init = [x0; y0; vx0; vy0];
    
    %% Time Span for Simulation
    tspan = [0, 10];  % Simulation time (s) - will stop early if projectile hits ground
    
    %% Define ODE System
    % Differential equations for projectile motion with quadratic drag
    f = @(t, y) [
        y(3);                          % dx/dt = vx
        y(4);                          % dy/dt = vy
        -D * norm([y(3), y(4)]) * y(3);  % dvx/dt = -D*v*vx
        -g - D * norm([y(3), y(4)]) * y(4)  % dvy/dt = -g - D*v*vy
    ];
    
    %% Integration Options
    % Stop integration when projectile hits ground (y < 0)
    opts = odeset('Events', @stop_when_ground_hit);
    
    %% Solve ODE System
    [T, Y] = ode45(f, tspan, y_init, opts);
    
    %% Extract Results
    x_vals = Y(:,1);  % x-positions
    y_vals = Y(:,2);  % y-positions
end

%% Event Function
function [value, isterminal, direction] = stop_when_ground_hit(t, y)
    % STOP_WHEN_GROUND_HIT - ODE event function to stop simulation when projectile hits ground
    %
    % Inputs:
    %   t - Current time (unused)
    %   y - Current state vector [x; y; vx; vy]
    %
    % Outputs:
    %   value - The value we're monitoring (y-position)
    %   isterminal - Whether to stop integration (1 = yes)
    %   direction - Direction of zero crossing to detect (-1 = decreasing through zero)
    
    value = y(2);        % Monitor y-position
    isterminal = 1;      % Stop integration when event occurs
    direction = -1;      % Only trigger when y is decreasing through zero
end