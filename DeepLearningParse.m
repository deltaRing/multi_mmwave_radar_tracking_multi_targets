load ./RadarFiles/ground_truth3.mat
ground_Truth = TracksRecord;

groundTruth_arr = [];
for ff = 1:length(ground_Truth)
    gT = ground_Truth{ff};
    gTloc = gT{1}(2:3);
    groundTruth_arr = [groundTruth_arr; gTloc];
end

xx = [];
yy = [];
err_DP = [];

for tt = 1:length(Target)
    if isempty(Target{tt}), continue; end
    xxx = Target{tt}(:, 1);
    yyy = Target{tt}(:, 2);
    
    diff = Target{tt} - groundTruth_arr(tt,:);
    diff = sqrt(diff(:,1).^2 + diff(:,2).^2);
    errDP = min(diff);
    err_DP = [err_DP errDP];
    
    xx = [xx; xxx];
    yy = [yy; yyy];
end

% scatter(xx, yy, 1, 'filled', 'm')
% hold on
% scatter(groundTruth_arr(:, 1), groundTruth_arr(:, 2), 5, 'filled', 'b')
% legend('Deep Learning Method', 'The Groundtruth')
% plot(groundTruth_arr(:, 1), groundTruth_arr(:, 2), 'b--')

% xlabel('X-Axis')
% ylabel('Y-Axis')
% title('The positioning result in scenario 1')

mean(err_DP)