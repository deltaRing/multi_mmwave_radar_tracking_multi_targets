%% Radar联合工作程序

addpath Utils
addpath RadarRelated
addpath RadarRelated/TrackUtils
addpath RadarRelated/DataProcessing
addpath RadarRelated/DataAssociation
addpath RadarRelated/SignalProcessing
addpath LocationOptimal
addpath SimulationResults
addpath SimulationResults/GeometryTransform

radarInit;

fid = []; 
sdata = {};  
fnumber = Radar(1).fnumber;

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

for xx = 1:fnumber - 1
    RadarEchos = {}; % 原始回波成分
    RadarDetection = {}; % 距离多普勒检测结果
    RadarMVDRs = {}; % 距离-角度谱
    for rr = 1:Net_RadarNum
        n_samples = Radar(radar).RadarParam.n_samples;
        n_chirps  = Radar(radar).RadarParam.n_chirps;
        n_RX      = Radar(radar).RadarParam.n_RX;
        n_TX      = Radar(radar).RadarParam.n_TX;
        
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
        
        rx_select_tx = 1;
        for ttxx = 1:n_TX
            if ttxx == rx_select_tx
                data_radar_1 = squeeze(t_lvds_data(rx_select_tx * n_RX - 3, :, :));   %RX1
                data_radar_2 = squeeze(t_lvds_data(rx_select_tx * n_RX - 2, :, :));   %RX2
                data_radar_3 = squeeze(t_lvds_data(rx_select_tx * n_RX - 1, :, :));   %RX3
                data_radar_4 = squeeze(t_lvds_data(rx_select_tx * n_RX, :, :));       %RX4
               break; 
            end
        end
        data_radar(1, :, :) = data_radar_1;
        data_radar(2, :, :) = data_radar_2;
        data_radar(3, :, :) = data_radar_3;
        data_radar(4, :, :) = data_radar_4;
        
        % 执行MTI
        data_radar_1 = MTI(data_radar_1, 2);
        data_radar_2 = MTI(data_radar_2, 2);
        data_radar_3 = MTI(data_radar_3, 2);
        data_radar_4 = MTI(data_radar_4, 2);
        
        % 得到距离像
        RangeProfile_1 = PulseCompression(data_radar_1, N);
        RangeProfile_2 = PulseCompression(data_radar_2, N);
        RangeProfile_3 = PulseCompression(data_radar_3, N);
        RangeProfile_4 = PulseCompression(data_radar_4, N);
        
        % 进一步执行 MTD
        RangeProfiles = RangeProfile_1 + ...
                        RangeProfile_2 + ...
                        RangeProfile_3 + ...
                        RangeProfile_4;
                    
        MotionTargetDetection = MTD(RangeProfiles, M);
        
        % 执行CA-CFAR
        [detect_map, detect_result, detect_threshold] = ...
                                            CA_CFAR(MotionTargetDetection);
                                        
        % 执行MVDR
        RangeProfiles(1, :, :) = RangeProfile_1;
        RangeProfiles(2, :, :) = RangeProfile_2;
        RangeProfiles(3, :, :) = RangeProfile_3;
        RangeProfiles(4, :, :) = RangeProfile_4;
        AngleMap               = MVDR(RangeProfiles, Q, ...
                                        Radar(rr).RadarParam.f0);
    end
    
end