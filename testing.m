clear all
%close all
clc

% load camera data
cam_file = fopen('/home/kirk/mavlab/stereoboard/stereoboard_testing/stereoboard_database/Track3/result.csv');
cam = textscan(cam_file,'%f,%f,%f,%f,%f,%f,%f,%f,%f');
fclose(cam_file);

x_pixelwise_kirk = cam{1}/256;
z_pixelwise_kirk = cam{2}/256;
x_global_kirk = cam{3}/256;
y_global_kirk = cam{4}/256;
z_global_kirk = cam{5}/256;

cam_file = fopen('/home/kirk/mavlab/stereoboard/stereoboard_testing/stereoboard_database/Track3/result_kim.csv');
cam = textscan(cam_file,'%f,%f,%f,%f,%f,%f,%f,%f,%f');
fclose(cam_file);

x_pixelwise_kim = cam{1}/100;
z_pixelwise_kim = cam{2}/100;
x_global_kim = cam{3}/100;
y_global_kim = cam{4}/100;
z_global_kim = cam{5}/100;

figure(2)
subplot(3,1,1)
plot([x_pixelwise_kirk, x_global_kirk, x_global_kim]); ylim([-1,1])
subplot(3,1,2)
plot(y_global_kirk); ylim([-1,1])
subplot(3,1,3)
plot([z_pixelwise_kirk, z_global_kirk]); ylim([-1,1])
