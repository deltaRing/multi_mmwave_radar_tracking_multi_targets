addpath PlotFunctions
addpath GeometryTransform
addpath ../Utils
addpath ../RadarRelated/DataAssociation
addpath ../RadarRelated/TrackUtils

clear all

% 测试模拟环境生成
%% 雷达参数
radarLoc = [80 90; 100 50; 30 80; -10 0];
radarVelo = [0 0; 0 0; 0 0; 0 0];
radarAttitude = [0 0 -pi; 0 0 pi / 2; 0 0 -pi / 4; 0 0 0];
radarSearchAngle = [pi / 2; pi / 2; pi / 2; pi / 2];
radarSearchRange = [100; 75; 100; 125];
radarQ  = [0.5; 0.5; 0.5; 0.5];
radarvQ = [0.25; 0.25; 0.25; 0.25];
radarNum = 4;

%% 目标参数
targetLoc = [10 10; 10 100; 100 10];
targetVelo = [5 2.5; 2 -4.5; -5 3.5];
targetQ = [0.25; 0.35; 0.15];
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
        targetQ(tt, :), Nz);
end

%% 定义帧数和步长
frames = 350;
dt = 0.1;
Tracks = {};
TracksRecord = {};
ID = 1;

for ff = 1:frames
    [dTargets, vTargets] = GenerateSimulationResults(radar, target, ff, dt);
    plotGenerateSimulationFrames(target, radar, dTargets, ff * dt);
    plotRadarDetectArea(radar);
    
    if isempty(dTargets)
        
        
    else
        if isempty(Tracks)
            % 无航迹
            ddTargets = {}; dvTargets = {};
            %% 需要对其进行矫正
            for rrr = 1:length(dTargets)
                % 获取雷达位置
                radarPos = radarLoc(rrr, :);
                radarYaw = -radarAttitude(rrr, 3);
                rotMatrix  = rotz(radarYaw / pi * 180); 
                rotMatrix  = rotMatrix(1:2, 1:2);
                for ttt = 1:length(dTargets{rrr})
                    ddTargets{length(ddTargets) + 1} = rotMatrix' * dTargets{rrr}{ttt} + radarPos';
                    dvTargets{length(dvTargets) + 1} = vTargets{rrr}{ttt};
                end
            end
            [Tracks, ID] = TrackInitFunc(ddTargets, dvTargets, ...
                                                ff, dt, ID, Tracks);
            % 初始化完毕后
        else
            % 请计算前后航迹之间的速度
            measures = [];

            for rrr = 1:length(dTargets)
                for ttt = 1:length(dTargets{rrr})
                    weights = rand(size(dTargets{rrr}{ttt}));
                    % 获取雷达位置
                    radarPos = radarLoc(rrr, :);
                    radarYaw = -radarAttitude(rrr, 3);
                    rotMatrix  = rotz(radarYaw / pi * 180); 
                    rotMatrix  = rotMatrix(1:2, 1:2);
                    if length(dTargets{rrr}{ttt}) == 2
                        
                        ddTargets = rotMatrix' * dTargets{rrr}{ttt} + radarPos';
                        
                        measures(:,size(measures, 2) + 1) = ...
                            [ddTargets(1); vTargets{rrr}{ttt} * weights(1); ...
                            ddTargets(2); vTargets{rrr}{ttt} * weights(2)];
                    elseif length(dTargets{rrr}{ttt}) == 3
                        measures(:,size(measures, 2) + 1) = ...
                            [dTargets{rrr}{ttt}(1); vTargets{rrr}{ttt} * weights(1); ...
                            dTargets{rrr}{ttt}(2); vTargets{rrr}{ttt} * weights(2); ...
                            dTargets{rrr}{ttt}(3); vTargets{rrr}{ttt} * weights(3)];
                    else
                        error("Simulation Result Test: Not Support High Dimension")
                    end
                end
            end
            
            [newTracks, observed, empty_measurements] = JPDA(Tracks, measures);
            
            % 估计速度
            %newTracks = VeloUpdate(newTracks, Tracks, observed); % 不好用 %
            %错误容易导致估计
            
            % 更新航迹
            newTracks = TrackUpdate(newTracks, observed);
            
            % 如果有新航迹
            if ~isempty(empty_measurements)
                ddTargets = {}; dvTargets = {};
                for rrr = 1:size(empty_measurements, 2)
                    if size(empty_measurements, 1) == 4
                        ddTargets{rrr} = [empty_measurements(1, rrr) empty_measurements(3, rrr)];
                        dvTargets{rrr} = [empty_measurements(2, rrr) empty_measurements(4, rrr)];
                    elseif size(empty_measurements, 1) == 6
                        ddTargets{rrr} = [empty_measurements(1, rrr) empty_measurements(3, rrr) empty_measurements(5, rrr)];
                        dvTargets{rrr} = [empty_measurements(2, rrr) empty_measurements(4, rrr) empty_measurements(6, rrr)];
                    end
                end
                [newTracks, ID] = TrackInitFunc(ddTargets, dvTargets, ...
                                                ff, dt, ID, newTracks);
            end
            Tracks = newTracks;
            
            TrackIndex = 1;
            for ii = 1:ID - 1
                if TrackIndex <= length(Tracks) && Tracks{TrackIndex}.ID == ii
                    TracksRecord{ff}{ii} = [Tracks{TrackIndex}.ID Tracks{TrackIndex}.X(1) Tracks{TrackIndex}.X(3)]; % 记录Tracks
                    TrackIndex = TrackIndex + 1;
                else
                    TracksRecord{ff}{ii} = [];
                end
            end
            plotRadarTracks(TracksRecord, ID);
        end
        
    end
    
end