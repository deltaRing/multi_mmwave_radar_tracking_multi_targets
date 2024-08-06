%% Radar联合工作程序
clear all; close all;
addpath Utils
addpath RadarRelated
addpath RadarRelated/TrackUtils
addpath RadarRelated/DataProcessing
addpath RadarRelated/DataAssociation
addpath RadarRelated/SignalProcessing
addpath LocationOptimal
addpath LocationOptimal/Estimate
addpath SimulationResults
addpath SimulationResults/GeometryTransform
addpath SimulationResults/PlotFunctions

radarInit;
warning off

lamb = 0.25;
fid = []; 
sdata = {};  
fnumber = Radar(1).fnumber;
dt  = 0.1;
vv = [];

Tracks = {};
TracksRecord = {};
ID = 1;
deltaStep = 5;
Nz = 2;

for rr = 1:Net_RadarNum
    fid(rr) = fopen(Radar(rr).fname,'rb');
    if fid(rr) == -1
        error('Main Program: File Not Found')
    end
    sdata{rr} = fread(fid(rr),'int16');
    if fnumber ~= Radar(rr).fnumber
        error('Main Program: frame numbers are not unified') 
    end
end
% 读取数据

xxx_1 = []; xxx_2 = [];
yyy_1 = []; yyy_2 = [];
singleMeasures = {};
for xx = 1:fnumber - 1
    RadarEchos = {}; % 原始回波成分
    RadarDetection = {}; % 距离多普勒检测结果
    RadarMVDRs = {}; % 距离-角度谱
    new_data = [];   % 探测数据
    radar_info = []; % 雷达信息
    for rr = 1:Net_RadarNum
        n_samples = Radar(radar).RadarParam.n_samples;
        n_chirps  = Radar(radar).RadarParam.n_chirps;
        n_RX      = Radar(radar).FFTParam.n_RX;
        n_TX      = Radar(radar).FFTParam.n_TX;
        
        sdata2 = sdata{rr}((xx-1) * ...
            n_samples * ...
            n_chirps * ...
            n_RX * ...
            n_TX * 2 + ...
            1:xx * n_samples * ...
            n_chirps * ... 
            n_RX * ...
            n_TX * 2);
        
        fileSize = size(sdata2, 1);
        lvds_data = zeros(1, fileSize/2);
        count = 1;
        for i=1:4:fileSize-5
           lvds_data(1,count) =1i* sdata2(i) + sdata2(i+2); 
           lvds_data(1,count+1) =1i* sdata2(i+1)+sdata2(i+3); 
           count = count + 2;
        end
        lvds_data = reshape(lvds_data, n_TX * ...
                            n_samples * n_RX, ...
                            n_chirps);

        t_lvds_data = [];
        for tr = 1:n_TX * n_RX
            t_lvds_data(tr, :, :) = lvds_data((tr - 1) * n_samples+1:tr * n_samples, :);
        end
        
        for ttxx = 1:n_TX
            data_radar_1 = squeeze(t_lvds_data(ttxx * n_RX - 3, :, :));   %RX1
            data_radar_2 = squeeze(t_lvds_data(ttxx * n_RX - 2, :, :));   %RX2
            data_radar_3 = squeeze(t_lvds_data(ttxx * n_RX - 1, :, :));   %RX3
            data_radar_4 = squeeze(t_lvds_data(ttxx * n_RX, :, :));       %RX4

            data_radar((ttxx - 1) * n_RX + 1, :, :) = data_radar_1;
            data_radar((ttxx - 1) * n_RX + 2, :, :) = data_radar_2;
            data_radar((ttxx - 1) * n_RX + 3, :, :) = data_radar_3;
            data_radar((ttxx - 1) * n_RX + 4, :, :) = data_radar_4;
        end
        
        % 执行MTI
        for cc = 1: n_TX * n_RX
            data_radar_(cc, :, :) = MTI(squeeze(data_radar(cc, :, :)), 40);
        end
        
        % 得到距离像
        for cc = 1: n_TX * n_RX
            RangeProfile_(cc, :, :) = PulseCompression(squeeze(data_radar_(cc, :, :)), N);
        end
        
        % 进一步执行 MTD
        MotionTargetDetection = [];
        for cc = 1: n_TX * n_RX
            MotionTargetDetection_(cc, :, :) = MTD(squeeze(RangeProfile_(cc, :, :)), M);
            if isempty(MotionTargetDetection), MotionTargetDetection = abs(MotionTargetDetection_(cc, :, :));
            else MotionTargetDetection = MotionTargetDetection + abs(MotionTargetDetection_(cc, :, :)); end 
        end

        figure(10000)
        imagesc(Radar(rr).ProcessParam.velo_axis, ...
            Radar(rr).ProcessParam.range_axis, ...
            abs(squeeze(MotionTargetDetection)))
        title('MTD结果')
        xlabel('速度(m/s)')
        ylabel('距离(m)')
        
        % 执行CA-CFAR
        [detect_map, detect_result, detect_threshold] = ...
                                            CA_CFAR(squeeze(MotionTargetDetection));

        figure(10001)
        imagesc(Radar(rr).ProcessParam.velo_axis, ...
            Radar(rr).ProcessParam.range_axis, ...
            abs(detect_map))
        title('CA-CFAR结果')
        xlabel('速度(m/s)')
        ylabel('距离(m)')
                                        
         % 执行角度FFT
        azMap = [];

        if Radar(rr).Type == "1843"
            for cc = 1: n_TX
            RangeProfiles(1, :, :) = RangeProfile_((cc - 1) * n_RX + 1, :, :);
            RangeProfiles(2, :, :) = RangeProfile_((cc - 1) * n_RX + 2, :, :);
            RangeProfiles(3, :, :) = RangeProfile_((cc - 1) * n_RX + 3, :, :);
            RangeProfiles(4, :, :) = RangeProfile_((cc - 1) * n_RX + 4, :, :);
            for chirps = 1:size(RangeProfiles, 3)
                RangeProfiles_ = squeeze(RangeProfiles(:, :, chirps));
                AngleMap      = fft(RangeProfiles_.', Q, 2);
                if isempty(azMap), azMap = abs(AngleMap);
                else azMap = azMap + abs(AngleMap); end
            end
            clear RangeProfiles
            end
        else
            Antenna_index = [1 4 5 8; 2 3 6 7];
            for cc = 1: n_TX
            RangeProfiles(1, :, :) = RangeProfile_(Antenna_index(cc, 1), :, :);
            RangeProfiles(2, :, :) = RangeProfile_(Antenna_index(cc, 2), :, :);
            RangeProfiles(3, :, :) = RangeProfile_(Antenna_index(cc, 3), :, :);
            RangeProfiles(4, :, :) = RangeProfile_(Antenna_index(cc, 4), :, :);
            for chirps = 1:size(RangeProfiles, 3)
                RangeProfiles_ = squeeze(RangeProfiles(:, :, chirps));
                AngleMap      = fft(RangeProfiles_.', Q, 2);
                if isempty(azMap), azMap = abs(AngleMap);
                else azMap = azMap + abs(AngleMap); end
            end
            clear RangeProfiles
            end
        end

        azMap = fftshift(azMap, 2);
        figure(10002)
        imagesc(Radar(rr).ProcessParam.an_axis_az, ...
            Radar(rr).ProcessParam.range_axis, ...
            abs(azMap))
        title('距离-角度谱')
        xlabel('角度(rad)')
        ylabel('距离(m)')

        % 进行定位
        res = RangeCentroid(detect_result);
        [range, angle, velo_] = getAngleInfo(res, azMap);

        if isempty(range)
            xxxx = [];
            yyyy = [];
            velo = [];
        else
            xxxx = [];
            yyyy = [];
            velo = Radar(rr).ProcessParam.velo_axis(fix(velo_ + 1));
            for aaa = 1:length(angle) 
                angles = Radar(rr).ProcessParam.an_axis_az(angle(aaa));
                if abs(angles) > Radar(rr).ProcessParam.DisableAngle, continue; end
                ranges = Radar(rr).ProcessParam.range_axis(range(aaa));
                radarLoc = Radar(rr).Geometry.Location;
                xxxx = [xxxx ranges .* sin(angles) + radarLoc(1)];
                yyyy = [yyyy ranges .* cos(angles) + radarLoc(2)];
                radar_info = [radar_info; Radar(rr).Geometry.Location(1:2) ...
                    Radar(rr).Geometry.Angle];
                new_data = [new_data; xx * Radar(rr).RadarParam.PRI ...
                ranges .* sin(angles) + radarLoc(1) ...
                ranges .* cos(angles) + radarLoc(2) ...
                velo(aaa) rr];
            end
        end
        % 转换为世界坐标系
        if rr == 1
            xxx_1 = [xxx_1 xxxx];
            yyy_1 = [yyy_1 yyyy];
        else
            xxx_2 = [xxx_2 xxxx];
            yyy_2 = [yyy_2 yyyy];
        end
    end
    
    figure(10003)
    scatter(xxx_1, yyy_1, 5, 'filled', 'b');
    hold on
    scatter(xxx_2, yyy_2, 5, 'filled', 'g');
    axis([-10 10 -1 20])
    
    if isempty(new_data)
        if ~isempty(Tracks)
            Tracks = TrackUpdate(Tracks, zeros(1, length(Tracks)));
        end
    else
        singleMeasures = RecordRealMeasureInfo(singleMeasures, new_data, ...
            xx * dt, radar_info);
        [optiVelo, valid_measure, ValidEstimatedVelo, posiX, posiY, RadarInfo, velo] = ...
            EstimateTraceVelocity(singleMeasures, Nz, dt);
        [W, Vpre, En, VpreValid] = VeloPrediction2(velo, RadarInfo, posiX, posiY, ...
            optiVelo, valid_measure, ValidEstimatedVelo, 0.0001, 12, deltaStep, 1000, dt);

        % for ttt = 1:size(Vpre, 2)
        %     optiVelo(:, end + 1, :) = Vpre(:, ttt, :);
        % end

        if isempty(Vpre), continue; end

        % get the latest frame
        LocX = posiX{end}; LocY = posiY{end};
        Velo = squeeze(Vpre(:, end, :));
        if size(Velo, 2) == 1, Velo = Velo'; end
        valid_measure = valid_measure(1:min([length(LocX) length(valid_measure)]));

        % 初始化航迹
        if isempty(Tracks)
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
                elseif norm(velocity) > 7.5
                    Tracks{end + 1} = trackInit([locX_ locY_], [0 0], dt, xx, ID, lamb);
                else 
                    Tracks{end + 1} = trackInit([locX_ locY_], velocity, dt, xx, ID, lamb);
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
                vv = [vv; velocity];
                if isnan(locX_) || isnan(locY_)
                   continue; 
                elseif norm(velocity) > 7.5
                    measures = [measures; locX_ locY_ 0 0];
                else 
                    measures = [measures; locX_ locY_ velocity];
                end
            end

            [newTracks, redundantMeasures, observed, observed_track] = ...
                TraceUpdate(measures, Tracks);
            % 更新航迹
            newTracks = TrackUpdate(newTracks, observed);
            if ~isempty(redundantMeasures)
                for ttt = 1:size(redundantMeasures, 1)
                    location = redundantMeasures(ttt, 1:2);
                    velocity = redundantMeasures(ttt, 3:4);
                    if isnan(location(1)) || isnan(location(2)), continue;
                    elseif norm(velocity) > 7.5
                        newTracks{end + 1} = trackInit(location, [0 0], dt, xx, ID, lamb);
                    else 
                        newTracks{end + 1} = trackInit(location, velocity, dt, xx, ID, lamb);
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
                TracksRecord{xx}{tt} = [Tracks{TrackIndex}.ID Tracks{TrackIndex}.X(1) ...
                    Tracks{TrackIndex}.X(3) Tracks{TrackIndex}.Type xx]; % 记录Tracks
                TrackIndex = TrackIndex + 1;
            else
                TracksRecord{xx}{tt} = [];
            end
        end
        plotRadarTracks(TracksRecord, ID, 10003);
    end
end