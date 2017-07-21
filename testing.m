%clear all
%close all
%clc

% load camera data
cam_file = fopen('stereoboard_database/Track3/result.csv');
cam = textscan(cam_file,'%f,%f,%f,%f,%f,%f,%f,%f,%f');
fclose(cam_file);

x_kirk = cam{1}/100.;
y_kirk = cam{2}/100.;
z_kirk = cam{3}/100.;

x_pos = cam{7}/100.;
y_pos = cam{8}/100.;
z_pos = cam{9}/100.;

time = 0:1/8:numel(x_kirk)/8 - 1/8;

cam_file = fopen('stereoboard_database/Track3/result_kim.csv');
cam = textscan(cam_file,'%f,%f,%f,%f,%f,%f,%f,%f,%f');
fclose(cam_file);

x_pixelwise_kim = cam{1}/100;
z_pixelwise_kim = cam{2}/100;
x_global_kim = cam{3}/100;
y_global_kim = cam{4}/100;
z_global_kim = cam{5}/100;

[Vx, Vz, yaw_frame, t_frame, X, Y, Z] = getOptiTrack('/home/kirk/mavlab/stereoboard/ext/stereoboard_testing/stereoboard_database/Track3');

figure(2)
subplot(3,1,1); hold on;
plot(time, x_kirk, 'r'); plot(time, x_pixelwise_kim, 'm'); ylim([-1,1]); hold off;
subplot(3,1,2); hold on;
plot(time, y_kirk, 'r'); plot(time, y_global_kim, 'm'); ylim([-1,1]); hold off;
subplot(3,1,3); hold on;
plot(time, z_kirk, 'r'); plot(time, -z_pixelwise_kim, 'm'); ylim([-1,1]); 
plot(time, zeros(size(time))); hold off;

SAD(1) = sum(abs(Vx - x_kirk));
SAD(2) = sum(abs(Vz - z_kirk))

max_error(1) = max(abs(Vx - x_kirk));
max_error(2) = max(abs(Vz - z_kirk));

figure(6);
subplot(3,1,1); 
plot(x_pos,'r'); hold on; title('Position');ylabel('x [m]')
plot(X - X(1)); hold off;
subplot(3,1,2);
plot(y_pos,'r'); hold on; ylabel('y [m]')
plot(Y - Y(1)); hold off;
subplot(3,1,3);
plot(z_pos,'r'); hold on; ylabel('z [m]'); xlabel('time [s]')
plot(Z - Z(1)); hold off;
axis tight

figure(7)
plot(time,180*7*atan2(x_kirk, z_kirk)/22); hold on;
plot(time,180*7*atan2(y_kirk, z_kirk)/22); hold off;

figure(8)
plot(time,smooth(x_kirk./z_kirk)); hold on;
plot(time,smooth(y_kirk./z_kirk)); hold off;

%%
% load camera data
[Vx, Vz, yaw_frame, t_frame, X, Y, Z] = getOptiTrack('/home/kirk/mavlab/stereoboard/ext/stereoboard_testing/stereoboard_database/Take16');

cam_file = fopen('stereoboard_database/Take16/result.csv');
cam = textscan(cam_file,'%f,%f,%f,%f,%f,%f,%f,%f,%f');
fclose(cam_file);

x_kirk = cam{1}/100.;
y_kirk = cam{2}/100.;
z_kirk = cam{3}/100.;

time = 0:1/7:numel(x_kirk)/7 - 1/7;

figure(2);
subplot(3,1,1); hold on;
plot(time,x_kirk,'r'); ylim([-1,1]); hold off;
subplot(3,1,2); hold on;
plot(time, y_kirk,'r'); ylim([-1,1]); hold off;
subplot(3,1,3); hold on;
plot(time,z_kirk,'r'); ylim([-1,1]); hold off;

SAD(1) = sum(abs(Vx(2:end) - x_kirk));
SAD(2) = sum(abs(Vz(2:end) - z_kirk))

max_error(1) = max(abs(Vx(2:end) - x_kirk));
max_error(2) = max(abs(Vz(2:end) - z_kirk));

x_pos = cam{7}/100.;
y_pos = cam{8}/100.;
z_pos = cam{9}/100.;

figure(5)
plot3(x_pos, y_pos, z_pos, '.-');
xlabel('x')
ylabel('y')

figure(6);
subplot(3,1,1); 
plot(x_pos,'r'); hold on; title('Position');ylabel('x [m]')
plot(X - X(1)); hold off;
subplot(3,1,2);
plot(y_pos,'r'); hold on; ylabel('y [m]')
plot(Y - Y(1)); hold off;
subplot(3,1,3);
plot(z_pos,'r'); hold on; ylabel('z [m]'); xlabel('time [s]')
plot(Z - Z(1)); hold off;

figure(7)
plot(time,180*7*atan2(x_kirk, z_kirk)/22); hold on;
plot(time,180*7*atan2(y_kirk, z_kirk)/22); hold off;

figure(8)
plot(time,smooth(x_kirk./z_kirk)); hold on;
plot(time,smooth(y_kirk./z_kirk)); hold off;