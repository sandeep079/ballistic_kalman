function [x_vals, y_vals] = simulate_with_ode45(x0, y0, vx0, vy0, D)
  g = 9.81;

  % Initial state: [x, y, vx, vy]
  y_init = [x0; y0; vx0; vy0];

  % Time span to simulate (will stop early if y < 0)
  tspan = [0, 10];
%oc
  % Define ODE function
  f = @(t, y) [
    y(3);  % dx/dt = vx
    y(4);  % dy/dt = vy
    -D * norm([y(3), y(4)]) * y(3);  % dvx/dt
    -g - D * norm([y(3), y(4)]) * y(4)  % dvy/dt
  ];

  % Integrate using ode45 (adaptive RK)
  opts = odeset("Events", @stop_when_ground_hit);
  [T, Y] = ode45(f, tspan, y_init, opts);

  % Extract x and y
  x_vals = Y(:,1);
  y_vals = Y(:,2);
endfunction

% Stop integration when y < 0
function [value, isterminal, direction] = stop_when_ground_hit(t, y)
  value = y(2);       % we're watching y-position
  isterminal = 1;     % stop the integration
  direction = -1;     % only stop when y is decreasing through zero
endfunction

