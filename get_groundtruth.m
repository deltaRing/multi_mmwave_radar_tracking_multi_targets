% align_main
clear all; close all;
file_name = ["./RadarFiles/label2.mat", "./RadarFiles/label2_2.mat";
    "./RadarFiles/label3.mat", "./RadarFiles/label3_2.mat";
    "./RadarFiles/label4.mat", "./RadarFiles/label4_2.mat";
    "./RadarFiles/label5.mat", "./RadarFiles/label5_2.mat";
    "./RadarFiles/label6.mat", "./RadarFiles/label6_2.mat";
    "./RadarFiles/label7.mat", "./RadarFiles/label7_2.mat";
    "./RadarFiles/label8.mat", "./RadarFiles/label8_2.mat";
    "./RadarFiles/label9.mat", "./RadarFiles/label9_2.mat";
    ];

RadarNum = 2;
disableAngle = 69 / 180.0 * pi;

xxx_1 = []; xxx_2 = [];
yyy_1 = []; yyy_2 = [];

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
for rr = 1:RadarNum
    fid(rr) = fopen(Radar(rr).fname,'rb');
    if fid(rr) == -1
        error('Main Program: File Not Found')
    end
    sdata{rr} = fread(fid(rr),'int16');
end

for ff = 1:size(file_name, 1)
    for rr = 1:RadarNum
        load(file_name(ff, rr));
    end
    dt  = 0.1;
    Tracks = {};
    TracksRecord = {};
    ID = 1;

    for fff = 1:length(xxx_1)
        ang1 = atan2(xxx_1(fff), yyy_1(fff));
        ang2 = atan2(xxx_2(fff), yyy_2(fff));
        dataX = []; dataY = [];
        if abs(ang1) < disableAngle
            dataX = [dataX xxx_1(fff)];
            dataY = [dataY yyy_1(fff)];
        end
        if abs(ang2) < disableAngle
            dataX = [dataX xxx_2(fff)];
            dataY = [dataY yyy_2(fff)];
        end
            
        LocX = {dataX};
        LocY = {dataY};

        % 初始化航迹
        if isempty(Tracks)
            for tt = 1:length(LocX)
                locX = mean(LocX{tt});
                locY = mean(LocY{tt});
                if isnan(locX) || isnan(locY)
                    continue; 
                else 
                    Tracks{end + 1} = trackInit([locX locY], [0 0], dt, fff, ID);
                end
                ID = ID + 1;
            end
        else
            measures = [];
            for tt = 1:length(LocX)
                locX = mean(LocX{tt});
                locY = mean(LocY{tt});
                if isnan(locX) || isnan(locY), continue; 
                else measures = [measures; locX locY 0 0]; end
            end
   
            [newTracks, redundantMeasures, observed, observed_track] = TraceUpdate(measures, Tracks);
            % 更新航迹
            newTracks = TrackUpdate(newTracks, observed);
            if ~isempty(redundantMeasures)
                for ttt = 1:size(redundantMeasures, 1)
                    location = redundantMeasures(ttt, 1:2);
                    if isnan(location(1)) || isnan(location(2)), continue;
                    else 
                    newTracks{end + 1} = trackInit(location, [0 0], dt, fff, ID);
                    end
                    ID = ID + 1;
                end
            end
            Tracks = newTracks;
        end
    
        TrackIndex = 1;
        for tt = 1:ID - 1
            if TrackIndex <= length(Tracks) && Tracks{TrackIndex}.ID == tt
                TracksRecord{fff}{tt} = [Tracks{TrackIndex}.ID Tracks{TrackIndex}.X(1) ...
                    Tracks{TrackIndex}.X(3) Tracks{TrackIndex}.Type fff]; % 记录Tracks
                    TrackIndex = TrackIndex + 1;
            else
                TracksRecord{fff}{tt} = [];
            end
        end
        plotRadarTracks(TracksRecord, ID, 10003);
        axis([-10 10 -1 20])
    end
    return
end