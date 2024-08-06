% location = [xxx_1; yyy_1];
% range = sqrt(location(1, :).^2 + location(2, :).^2);
% angle = atan2(location(1,:), location(2,:));
% 
% xx = range .* sin(angle);
% yy = range .* cos(angle);


load MultiTarget\Label10_S.mat
xxxx1 = xxx_1;
yyyy1 = yyy_1;
load MultiTarget\Label10_SS.mat
xxxx2 = xxx_1;
yyyy2 = yyy_1;

range_ = [];
angle_ = [];
xxx = [];
yyy = [];
for tt = 1:length(xxxx1)
    location = [];
    if xxxx1(tt) ~= 0 || yyyy1(tt) ~= 0
        location = [location xxxx1(tt); yyyy1(tt)];
    end
    if xxxx2(tt) ~= 0 || yyyy2(tt) ~= 0
        location = [location [xxxx2(tt); yyyy2(tt)]];
    end
    range = sqrt(location(1, :).^2 + location(2, :).^2);
    angle = atan2(location(1,:), location(2,:));

    range_{tt} = range;
    angle_{tt} = angle;
    
    xx = range .* sin(angle);
    yy = range .* cos(angle);
    xxx = [xxx xx];
    yyy = [yyy yy];
end

load param.mat

radar_axis = Radar(1).ProcessParam.range_axis;
angle_axis = Radar(1).ProcessParam.an_axis_az;
name = "2024-06-20-22-29-31.txt";

writeDatatoDeepLearning(name, range_, angle_,...
        radar_axis, angle_axis)