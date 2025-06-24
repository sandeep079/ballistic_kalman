clear
clc

data = csvread('../projectile_log/projectile_log2.csv');
ball = csvread('../projectile_log/ball_size2.csv');

ball_x=ball(2:end,2);
ball_y=ball(2:end,3);

all_ball=[ball_x;ball_y];

mean_x = mean(ball_x)
std_x = std(ball_x)
mean_y = mean(ball_y)
std_y = std(ball_y)
mean_all_ball = mean(all_ball)
std_all_ball = std(all_ball)

ball_mean=1;
time0=0;
x0=data(2,2)
#x0=0

x_data=(data(2:end,2)-x0)#*(ball_mean/mean_x)
y_data=(480-data(2:end,3))#*(ball_mean/mean_x)
time=data(2:end,1)-time0

new_data=[time,x_data,y_data]
plot(x_data,y_data,'-o-')
xlabel('X (scaled)')
ylabel('Y (scaled)')
title('Projectile Path')
grid on
