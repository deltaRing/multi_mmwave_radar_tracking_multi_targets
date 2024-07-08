% Compare and Plot the Difference
load ./RadarFiles/runout_main_new83.mat
main_Track = TracksRecord;
load ./RadarFiles/runout_main_KF_new83.mat
main_KF_Track = TracksRecord;
load ./RadarFiles/ground_truth8.mat
ground_Truth = TracksRecord;

err_MT = [];
err_MTKF = [];
groundTruth_arr = [];
for ff = 1:length(ground_Truth)
    gT = ground_Truth{ff};
    gTloc = gT{1}(2:3);
    groundTruth_arr = [groundTruth_arr; gTloc];
end

for ff = 30:length(main_Track)
    minIndex = ff - 15;
    maxIndex = ff + 15;
    if maxIndex > length(main_Track), maxIndex = length(main_Track); end
    if minIndex < 1, minIndex = 1; end

    mT = main_Track{ff};
    mTKF = main_KF_Track{ff};
    gT = ground_Truth{ff};
    gTloc = gT{1}(2:3);

    if isempty(mT) || isempty(mTKF) continue; end

    errMT = inf;
    errMTKF = inf;
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
            if errMT > 2.0, continue; end
            err_MT = [err_MT errMT];
        end
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
            if errMTKF > 2.0, continue; end
            err_MTKF = [err_MTKF errMTKF];
        end
    end
end

figure(10000)
plot(err_MT)
hold on
plot(err_MTKF)
mean(err_MT)
mean(err_MTKF)

plotRadarTracks(main_Track, ID, 10004);
plotRadarTracks(ground_Truth, ID, 10004);
plotRadarTracks(main_KF_Track, ID, 10005);
plotRadarTracks(ground_Truth, ID, 10005);