% Compare and Plot the Difference
load ./main_2.mat
main_Track = TracksRecord;
load ./main_2_KF.mat
main_KF_Track = TracksRecord;
load ./main_2_PDA.mat
main_PDA_Track = TracksRecord;
load ./main_2_JPDA.mat
main_JPDA_Track = TracksRecord;
load ./RadarFiles/ground_truth3.mat
ground_Truth = TracksRecord;
load('./RadarFiles/DeepLearningResult/3/track_res.mat')

addpath ./Utils
addpath ./SimulationResults
addpath ./SimulationResults/PlotFunctions

close all

err_MT = [];
err_MTKF = [];
err_MTPDA = [];
err_MTJPDA = [];
groundTruth_arr = [];
for ff = 1:length(ground_Truth)
    gT = ground_Truth{ff};
    gTloc = gT{1}(2:3);
    groundTruth_arr = [groundTruth_arr; gTloc];
end

for ff = 1:length(main_Track)
    minIndex = ff - 10;
    maxIndex = ff + 10;
    if maxIndex > length(main_Track), maxIndex = length(main_Track); end
    if minIndex < 1, minIndex = 1; end

    mT = main_Track{ff};
    mTKF = main_KF_Track{ff};
    mTPDA = main_PDA_Track{ff};
    mTJPDA = main_JPDA_Track{ff};
    
%     if isempty(mT) || isempty(mTKF) || isempty(mTPDA) || isempty(mTJPDA)
%         continue
%     end
    
    gT = ground_Truth{ff};
    gTloc = gT{1}(2:3);

    errMT = inf;
    errMTKF = inf;
    errMTPDA = inf;
    errMTJPDA = inf;
    if ~isempty(mT)
        loc = [];
        for tt = 1:length(mT)
            if isempty(mT{tt}) || mT{tt}(4) == 0, continue; end
            if isempty(loc)
                loc = mT{tt}(2:3);
            else 
                loc = [loc; mT{tt}(2:3)];
            end
        end

        for tt = 1:size(loc, 1)
            diff = loc(tt,:) - groundTruth_arr(minIndex:maxIndex,:);
            diff = sqrt(diff(:,1).^2 + diff(:,2).^2);
            errMT = min(diff);
            if errMT > 3.0, continue; end
            err_MT = [err_MT errMT];
        end
    else
        err_MT = [err_MT inf];
    end
    
    if ~isempty(mTJPDA)
        loc = [];
        for tt = 1:length(mTJPDA)
            if isempty(mTJPDA{tt}) || mTJPDA{tt}(4) == 0, continue; end
            if isempty(loc)
                loc = mTJPDA{tt}(2:3);
            else 
                loc = [loc; mTJPDA{tt}(2:3)];
            end
        end

        for tt = 1:size(loc, 1)
            diff = loc(tt,:) - groundTruth_arr(minIndex:maxIndex,:);
            diff = sqrt(diff(:,1).^2 + diff(:,2).^2);
            errMTJPDA = min(diff);
            if errMTJPDA > 3.0, continue; end
            err_MTJPDA = [err_MTJPDA errMTJPDA];
        end
    else
        err_MTJPDA = [err_MTJPDA inf];
    end

    if ~isempty(mTPDA)
        loc = [];
        for tt = 1:length(mTPDA)
            if isempty(mTPDA{tt}) || mTPDA{tt}(4) == 0, continue; end
            if isempty(loc)
                loc = mTPDA{tt}(2:3);
            else 
                loc = [loc; mTPDA{tt}(2:3)];
            end
        end

        for tt = 1:size(loc, 1)
            diff = loc(tt,:) - groundTruth_arr(minIndex:maxIndex,:);
            diff = sqrt(diff(:,1).^2 + diff(:,2).^2);
            errMTPDA = min(diff);
            if errMTPDA > 3.0, continue; end
            err_MTPDA = [err_MTPDA errMTPDA];
        end
    else
        err_MTPDA = [err_MTPDA inf];
    end
    
    if ~isempty(mTKF)
        loc = [];
        for tt = 1:length(mTKF)
            if isempty(mTKF{tt}) || mTKF{tt}(4) == 0, continue; end
            if isempty(loc) 
                loc = mTKF{tt}(2:3);
            else 
                loc = [loc; mTKF{tt}(2:3)];
            end
        end
        
        for tt = 1:size(loc, 1)
            diff = loc(tt,:) - groundTruth_arr(minIndex:maxIndex,:);
            diff = sqrt(diff(:,1).^2 + diff(:,2).^2);
            errMTKF = min(diff);
            if errMTKF > 3.0, continue; end
            err_MTKF = [err_MTKF errMTKF];
        end
    else
        err_MTKF = [err_MTKF inf];
    end
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

fig = figure(10000);
set(fig, 'Position', [100 100 640 400]); %单位为英寸

maxLength = max([length(err_MT), length(err_MTKF), length(err_MTPDA), length(err_MTJPDA), length(err_DP)]);
z = [];
for ll = 1:maxLength
    
    
end

bar(linspace(1,length(err_MT),length(err_MT)), err_MT, 0.2)
hold on
bar(linspace(1,length(err_MTKF),length(err_MTKF)), err_MTKF, 0.4)
bar(linspace(1,length(err_MTPDA),length(err_MTPDA)), err_MTPDA, 0.6)
bar(linspace(1,length(err_MTJPDA),length(err_MTJPDA)), err_MTJPDA, 0.8)
legend('Error of Proposed Method', 'Error of the KF', 'Error of the PDA', 'Error of the JPDA')
axis on

err_MT(find(err_MT == inf)) = [];
err_MT(find(err_MT == inf)) = [];
err_MT(find(err_MT == inf)) = [];
err_MT(find(err_MT == inf)) = [];
err_MT(find(err_MT == inf)) = [];

mean(err_MT)
mean(err_MTKF)
mean(err_MTPDA)
mean(err_MTJPDA)
mean(err_DP)

ID = 200;

fig = figure(10004);
set(fig, 'Position', [100 100 640 400]); %单位为英寸
plotRadarTracks(main_Track, ID, 10004, 'r', 0, 2, 'r-.');
plotRadarTracks(main_KF_Track, ID, 10004, 'g', 1, 10, '+');
plotRadarTracks(main_PDA_Track, ID, 10004, 'c', 1, 10, '*');
plotRadarTracks(main_JPDA_Track, ID, 10004, 'k', 1, 1, 'x');

hold on
scatter(xx, yy, 1, 'filled', 'm')
plotRadarTracks(ground_Truth, ID, 10004, 'b', 0, 1, 'b-.');
legend({'The Proposed method', 'The KF method', 'The PDA method', ...
    'The JPDA method', 'The DL method', 'The Ground Truth'},'Location','southeast')
fontSize = 16;
xlabel('X-Axis (m)', 'fontsize',fontSize,'FontName','Times New Roman')
ylabel('Y-Axis (m)', 'fontsize',fontSize,'FontName','Times New Roman')
title(strcat('The positioning result in Scenario ', num2str(2)), 'fontsize',fontSize,'FontName','Times New Roman')

% plotRadarTracks(main_KF_Track, ID, 10005, 'r');
% plotRadarTracks(ground_Truth, ID, 10005, 'b');
% legend('The KF method', 'The Ground Truth')
% plotRadarTracks(main_PDA_Track, ID, 10006, 'r');
% plotRadarTracks(ground_Truth, ID, 10006, 'b');
% legend('The PDA method', 'The Ground Truth')
% plotRadarTracks(main_JPDA_Track, ID, 10007, 'r');
% plotRadarTracks(ground_Truth, ID, 10007, 'b');
% legend('The JPDA method', 'The Ground Truth')