clc;
clear;
z_init= zeros(10,1);
z_init(1) = 27;
%Vx = z(:,1);
t_span =  [0 16];
Forces = zeros(1,4);
[ t, z ] =ode45(@(t,z) vehicle_dynamics(t,z, u(t)), t_span , z_init);
%dy_dt = diff(z(:,3)/ t);
pos_x = z(:,4);
pos_y = z(:, 5);
yaw = z(:, 6);
vel_x = z(:,1);
vel_y = z(:, 2);
yaw_rate = z(:,3);
lat_accel =vel_x .* yaw_rate./9.81;
% figure(1);
% plot(  t,dy_dt , 'b-');
% xlabel('x');
% ylabel('y');
% title('yaw_accel');
% grid on;
% axis equal;
% figure(2);
% plot(  t,sin(t) , 'b-');
% xlabel('t');
% ylabel('delta');
% title('delta plot');
figure(3);

plot(  t,z(:,3) , 'b-');
xlabel('t');
ylabel('yaw rate');
title('yae rate plot');
hold 
plot(  t,lat_accel , '-');
xlabel('t');
ylabel('lat_accel');
title('lat accel plot');

figure(4);
plot(  t,z(:,1) , 'b-');
xlabel('t');
ylabel('Vx');
title('Vx');

figure(5);
plot(  t,z(:,2) , 'b-');
xlabel('t');
ylabel('Vy');
title('Vy');

figure(6);
plot(  pos_x,pos_y , 'b-');
xlabel('x');
ylabel('y');
title('pos');
axis equal;  % Sets the aspect ratio to be equal


