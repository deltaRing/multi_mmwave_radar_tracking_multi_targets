%% Radar���Ϲ�������
clear all
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

fid = []; 
sdata = {};  
fnumber = Radar(1).fnumber;
dt  = 0.1;

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
% ��ȡ����

xxx = [];
yyy = [];
singleMeasures = {};
for xx = 1:fnumber - 1
    RadarEchos = {}; % ԭʼ�ز��ɷ�
    RadarDetection = {}; % ��������ռ����
    RadarMVDRs = {}; % ����-�Ƕ���
    new_data = [];   % ̽������
    radar_info = []; % �״���Ϣ
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
        
        % ִ��MTI
        for cc = 1: n_TX * n_RX
            data_radar_(cc, :, :) = MTI(squeeze(data_radar(cc, :, :)), 40);
        end
        
        % �õ�������
        for cc = 1: n_TX * n_RX
            RangeProfile_(cc, :, :) = PulseCompression(squeeze(data_radar_(cc, :, :)), N);
        end
        
        % ��һ��ִ�� MTD
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
        title('MTD���')
        xlabel('�ٶ�(m/s)')
        ylabel('����(m)')
        
        % ִ��CA-CFAR
        [detect_map, detect_result, detect_threshold] = ...
                                            CA_CFAR(squeeze(MotionTargetDetection));

        figure(10001)
        imagesc(Radar(rr).ProcessParam.velo_axis, ...
            Radar(rr).ProcessParam.range_axis, ...
            abs(detect_map))
        title('CA-CFAR���')
        xlabel('�ٶ�(m/s)')
        ylabel('����(m)')
                                        
         % ִ�нǶ�FFT
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
        title('����-�Ƕ���')
        xlabel('�Ƕ�(rad)')
        ylabel('����(m)')

        % ���ж�λ
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
                if abs(angles) > (69 / 180 * pi), continue; end
                ranges = Radar(rr).ProcessParam.range_axis(range(aaa));
                radarLoc = Radar(rr).Geometry.Location;
                xxxx = [xxxx ranges .* sin(angles) + radarLoc(1)];
                yyyy = [yyyy ranges .* cos(angles) + radarLoc(2)];
                radar_info = [radar_info; Radar(rr).Geometry.Location(1:2) ...
                    Radar(rr).Geometry.Angle];
                new_data = [new_data; xx * Radar(rr).RadarParam.PRI ...
                ranges .* sin(angles) + radarLoc(1) ...
                ranges .* cos(angles) + radarLoc(2) ...
                velo(aaa)];
            end
        end
        % ת��Ϊ��������ϵ
        figure(10003)
        xxx = [xxx xxxx];
        yyy = [yyy yyyy];
        scatter(xxx, yyy, 5, 'filled', 'b');
        axis([-10 10 -1 20])
    end
    
    if isempty(new_data)

    else

        if isempty(Tracks)
            ddTargets = {}; dvTargets = {};
            for ttt = 1:size(new_data, 1)
                ddTargets{length(ddTargets) + 1} = new_data(1, 2:3);
                dvTargets{length(dvTargets) + 1} = new_data(1, 4);
            end
            [Tracks, ID] = TrackInitFunc(ddTargets, dvTargets, ...
                                                xx, dt, ID, Tracks);
        else
            measures = [new_data(:, 2)';
                    0 * new_data(:, 4)';
                    new_data(:, 3)';
                    0 * new_data(:, 4)'];
            [newTracks, observed, empty_measurements, observed_data] = JPDA(Tracks, measures);

            % ���º���
            newTracks = TrackUpdate(newTracks, observed);
            
            % ������º���
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
                                                xx, dt, ID, newTracks);
            end
            Tracks = newTracks;

            TrackIndex = 1;
            for tt = 1:ID - 1
                if TrackIndex <= length(Tracks) && Tracks{TrackIndex}.ID == tt
                    TracksRecord{xx}{tt} = [Tracks{TrackIndex}.ID Tracks{TrackIndex}.X(1) ...
                        Tracks{TrackIndex}.X(3) Tracks{TrackIndex}.Type]; % ��¼Tracks
                    TrackIndex = TrackIndex + 1;
                else
                    TracksRecord{xx}{tt} = [];
                end
            end
            plotRadarTracks(TracksRecord, ID, 10003);
        end
    end
end
