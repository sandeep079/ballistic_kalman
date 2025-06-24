clear
clc


data = csvread('projectile_log/projectile_log2.csv');
ball = csvread('projectile_log/ball_size2.csv');



ball_x = ball(3:end, 2);
ball_y = ball(3:end, 3);

all_ball = [ball_x ; ball_y];

mean_x = mean(ball_x)
std_x = std(ball_x)
mean_y = mean(ball_y)
std_y = std(ball_y)
mean_all_ball = mean(all_ball)
std_all_ball = std(all_ball)


#time = data(1:end,1) - 47.41;
#x_data = (848 - data(1:end,2))*(0.24 / mean_x);
#y_data = (480 - data(1:end,3) - 67)*(0.24 / mean_x);

x_data = (data(3:end, 2)) * (0.24 / mean_x);
y_data = ((480 - data(3:end, 3)) - 192) *( 0.24 / mean_x);
time = data(3:end, 1) - 5.86;

new_data = [time, x_data, y_data];




% Use first few points to estimate initial velocity via linear regression
F = 0;
N = 2;  % number of points to fit
tx = time(1+F:N+F);
xx = x_data(1+F:N+F);
yy = y_data(1+F:N+F);

% Linear fits: x(t) ≈ x0 + vx * t, y(t) ≈ y0 + vy * t
p_x = polyfit(tx, xx, 1);  % [vx, x0]
p_y = polyfit(tx, yy, 1);  % [vy, y0]

vx = p_x(1);  % m/s
vy = p_y(1);  % m/s

% Initial position
x0 = p_x(2)
y0 = p_y(2)

% Initial velocity magnitude and angle
v0 = sqrt(vx^2 + vy^2);
angle_deg = atan2(vy, vx) #* 180 / pi;

% Display results
printf(" x0 = %.2f m, y0 = %.2f m\n", x0, y0);
printf(" vx = %.2f m/s, vy = %.2f m/s\n", vx, vy);
printf(" v0 = %.2f m/s\n", v0);
printf("angle: %.2f degrees\n", angle_deg);


#plot(x_data, y_data, '-o')

% Constants
Cd = 0.25^3 ;
rho = 1.225;
r = 0.24/2;
A = pi * r^2;
m = 0.62;
D0 = 0.5 * Cd * rho * A / m;

D = (4.0402) * (10 ^ -2)
k = 3.0959


[x, y] = simulate_with_ode45(x0, y0, vx*k, vy*k, D);
plot(x, y, 'r-', 'LineWidth', 2);
xlabel('x (m)');
ylabel('y (m)');
title('Projectile Trajectory with ode45 (Quadratic Drag)');
grid on;
hold on
plot(x_data, y_data, '-o')
