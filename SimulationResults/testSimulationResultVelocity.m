% For Test Velocity
addpath PlotFunctions
addpath GeometryTransform
addpath ../Utils
addpath ../RadarRelated/DataAssociation
addpath ../RadarRelated/TrackUtils
addpath ../LocationOptimal
addpath ../LocationOptimal/Estimate

clear all
warning off

% 测试模拟环境生成
%% 雷达参数
radarLoc = [10 10; -10 60; 60 -10];
radarVelo = [0 0; 0 0; 0 0];
radarAttitude = [0 0 pi / 4; 0 0 0; 0 0 pi / 2];
radarSearchAngle = [ 2 * pi / 3; 2 * pi / 3; 2 * pi / 3 ];
radarSearchRange = [150; 150; 150];
radarQ  = [0.5; 0.5; 0.5];
radarvQ = [0.005; 0.005; 0.005];
radarNum = 3;

%% 目标参数
targetLoc = [100 100; 10 100; 100 10];
targetVelo = [-3.5 -1; 5 -5; -1 5];
targetQ = [0.15; 0.15; 0.15];
targetType = [0; 1; 0];
targetNum = 3;

Nz = 2;

radar = {};
for rr = 1:radarNum
    radar{rr} = Radarsim(radarLoc(rr, :), radarVelo(rr, :), ...
        radarAttitude(rr, :), radarSearchAngle(rr, :), ...
        radarSearchRange(rr, :), radarQ(rr, :), radarvQ(rr, :), Nz);
end

target = {};
for tt = 1:targetNum
    target{tt} = Targetsim(targetLoc(tt, :), ...
        targetVelo(tt, :), ...
        targetQ(tt, :), Nz, targetType(tt));
end

%% 定义帧数和步长
frames = 500;
dt = 0.1;
deltaStep = 5;

Tracks = {};
TracksRecord = {};
ID = 1;

%%
singleMeasures = {};
Error = {};

for ff = 1:frames
    [dTargets, vTargets] = GenerateSimulationResults(radar, target, ff, dt);
    plotGenerateSimulationFrames(target, radar, dTargets, ff * dt, vTargets);
    plotRadarDetectArea(radar);
    
    if isempty(dTargets)
        
        
    else
        singleMeasures = RecordMeasureInfo(singleMeasures, ff * dt, dTargets, ...
            vTargets, radar);
        [optiVelo, valid_measure, ValidEstimatedVelo, posiX, posiY, RadarInfo, velo] = ...
            EstimateTraceVelocity(singleMeasures, Nz, dt);
        [W, Vpre, En, VpreValid] = VeloPrediction(velo, RadarInfo, posiX, posiY, ...
            optiVelo, valid_measure, ValidEstimatedVelo, 0.0001, 12, deltaStep, 1000, dt);

        % for ttt = 1:size(Vpre, 2)
        %     optiVelo(:, end + 1, :) = Vpre(:, ttt, :);
        % end

        if isempty(Vpre), continue; end

        % get the latest frame
        LocX = posiX{end}; LocY = posiY{end};
        Velo = squeeze(Vpre(:, end, :));
        if size(Velo, 2) == 1, Velo = Velo'; end

        % 初始化航迹
        if isempty(Tracks)
            for tt = 1:length(LocX)
                if valid_measure(tt) > length(LocX), continue; end
                locX_ = mean(LocX{valid_measure(tt)});
                locY_ = mean(LocY{valid_measure(tt)});
                index = find(tt == VpreValid);
                if ~isempty(index)
                    velocity = Velo(index, :);
                else
                    velocity = [0 0];
                end
                if isnan(locX_) || isnan(locY_)
                    continue; 
                elseif norm(velocity) > 10.0
                    Tracks{end + 1} = trackInit([locX_ locY_], [0 0], dt, ff, ID);
                else 
                    Tracks{end + 1} = trackInit([locX_ locY_], velocity, dt, ff, ID);
                end
                ID = ID + 1;
            end
        else
            measures = [];
            for tt = 1:length(valid_measure)
                if valid_measure(tt) > length(LocX), continue; end
                locX_ = mean(LocX{valid_measure(tt)});
                locY_ = mean(LocY{valid_measure(tt)});
                index = find(tt == VpreValid);
                if ~isempty(index)
                    velocity = Velo(index, :);
                else
                    velocity = [0 0];
                end
                if isnan(locX_) || isnan(locY_)
                   continue; 
                elseif norm(velocity) > 10.0
                    measures = [measures; locX_ locY_ 0 0];
                else 
                    measures = [measures; locX_ locY_ velocity];
                end
            end

            [newTracks, redundantMeasures, observed, observed_track] = TraceUpdate(measures, Tracks);
            Error = plotErrorCurve(newTracks, observed_track, 10001, Error, ff);
            % 更新航迹
            newTracks = TrackUpdate(newTracks, observed);
            if ~isempty(redundantMeasures)
                for ttt = 1:size(redundantMeasures, 1)
                    location = redundantMeasures(ttt, 1:2);
                    velocity = redundantMeasures(ttt, 3:4);
                    if isnan(location(1)) || isnan(location(2)), continue;
                    elseif norm(velocity) > 10.0
                        newTracks{end + 1} = trackInit(location, [0 0], dt, ff, ID);
                    else 
                        newTracks{end + 1} = trackInit(location, velocity, dt, ff, ID);
                    end
                    ID = ID + 1;
                end
            end
            Tracks = newTracks;
        end

        % for tt = 1:length(target)
        %     targetVelo = [target{tt}.vx(ff * dt) target{tt}.vy(ff * dt)];
        %     targetLocs = [target{tt}.x target{tt}.y] + ...
        %             targetVelo * ff * dt;
        %     fprintf("Target %d, px: %f, py: %f vx: %f vy: %f\n", tt, targetLocs(1), targetLocs(2), ...
        %         targetVelo(1), targetVelo(2));
        % end

        TrackIndex = 1;
        for tt = 1:ID - 1
            if TrackIndex <= length(Tracks) && Tracks{TrackIndex}.ID == tt
                TracksRecord{ff}{tt} = [Tracks{TrackIndex}.ID Tracks{TrackIndex}.X(1) ...
                    Tracks{TrackIndex}.X(3) Tracks{TrackIndex}.Type]; % 记录Tracks
                TrackIndex = TrackIndex + 1;
            else
                TracksRecord{ff}{tt} = [];
            end
        end
        plotRadarTracks(TracksRecord, ID);
    end
    
end
