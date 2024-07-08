location = [xxx_1; yyy_1];
range = sqrt(location(1, :).^2 + location(2, :).^2);
angle = atan2(location(1,:), location(2,:));

xx = range .* sin(angle);
yy = range .* cos(angle);

load param.mat

radar_axis = Radar(1).ProcessParam.range_axis;
angle_axis = Radar(1).ProcessParam.an_axis_az;
name = "2024-06-20-22-22-06.txt";

writeDatatoDeepLearning(name, range, angle,...
        radar_axis, angle_axis)