%clear all
%close all
%clc

% load camera data
cam_file = fopen('stereoboard_database/Track3/result.csv');
cam = textscan(cam_file,'%f,%f,%f,%f,%f,%f');
fclose(cam_file);

x_kirk = cam{1}/100.;
y_kirk = cam{2}/100.;
z_kirk = cam{3}/100.;

time = 0:1/8:numel(x_kirk)/8 - 1/8;

cam_file = fopen('stereoboard_database/Track3/result_kim.csv');
cam = textscan(cam_file,'%f,%f,%f,%f,%f,%f,%f,%f,%f');
fclose(cam_file);

x_pixelwise_kim = cam{1}/100;
z_pixelwise_kim = cam{2}/100;
x_global_kim = cam{3}/100;
y_global_kim = cam{4}/100;
z_global_kim = cam{5}/100;

[cam_Vx_frame, cam_Vz_frame, yaw_frame, t_frame] = getOptiTrack('/home/kirk/mavlab/stereoboard/ext/stereoboard_testing/stereoboard_database/Track3');

figure(2)
subplot(3,1,1); hold on;
plot(time, x_kirk, 'r'); plot(time, x_pixelwise_kim, 'm'); ylim([-1,1]); hold off;
subplot(3,1,2); hold on;
plot(time, y_kirk, 'r'); plot(time, y_global_kim, 'm'); ylim([-1,1]); hold off;
subplot(3,1,3); hold on;
plot(time, z_kirk, 'r'); plot(time, z_pixelwise_kim, 'm'); ylim([-1,1]); hold off;

SAD(1) = sum(abs(cam_Vx_frame - x_kirk));
SAD(2) = sum(abs(cam_Vz_frame - z_kirk));

max_error(1) = max(abs(cam_Vx_frame - x_kirk));
max_error(2) = max(abs(cam_Vz_frame - z_kirk));

%%
% load camera data
[cam_Vx_frame, cam_Vz_frame, yaw_frame, t_frame] = getOptiTrack('/home/kirk/mavlab/stereoboard/ext/stereoboard_testing/stereoboard_database/Take16');

cam_file = fopen('stereoboard_database/Take16/result.csv');
cam = textscan(cam_file,'%f,%f,%f,%f,%f,%f');
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

SAD(1) = sum(abs(cam_Vx_frame(2:end) - x_kirk));
SAD(2) = sum(abs(cam_Vz_frame(2:end) - z_kirk));

max_error(1) = max(abs(cam_Vx_frame(2:end) - x_kirk));
max_error(2) = max(abs(cam_Vz_frame(2:end) - z_kirk));