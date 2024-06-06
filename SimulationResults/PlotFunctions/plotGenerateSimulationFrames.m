% 绘制真实目标 真实雷达 以及 检测结果
% 输入1：真实目标位置
% 输入2：真实雷达位置
% 输入3：检测结果
% 输入4：当前时间
function plotGenerateSimulationFrames(realTargets, realRadars, ...
                                        rdetections, time, figureIndex, ...
                                        targetPlot, RadarPlot, measurePlot)
    if nargin == 4
        figureIndex = 10000;
        targetPlot = 1;
        RadarPlot = 1;
        measurePlot = 1;
    end
    targetLocs = {};
    radarLocs  = {};
    detections = {};
    figure(figureIndex)
    hold off
    if targetPlot
        for tt = 1:length(realTargets)
            if realTargets{tt}.Dimension == 2
                targetLocs{tt} = [realTargets{tt}.x realTargets{tt}.y] + ...
                    [realTargets{tt}.vx realTargets{tt}.vy] * time;
                scatter(targetLocs{tt}(1), targetLocs{tt}(2), 20, 'filled', 'b')
                hold on
            elseif realTargets{tt}.Dimension == 3
                targetLocs{tt} = [realTargets{tt}.x realTargets{tt}.y realTargets{tt}.z] + ...
                    [realTargets{tt}.vx realTargets{tt}.vy realTargets{tt}.vz] * time;
            end
        end
    end
    
    if RadarPlot
        for rr = 1:length(realRadars)
            if realRadars{rr}.Dimension == 2
                radarLocs{rr} = [realRadars{rr}.x realRadars{rr}.y] + ...
                    [realRadars{rr}.vx realRadars{rr}.vy] * time;
                scatter(radarLocs{rr}(1), radarLocs{rr}(2), 20, 'filled', 'r')
            elseif realRadars{rr}.Dimension == 3
                radarLocs{rr} = [realRadars{rr}.x realRadars{rr}.y realRadars{rr}.z] + ...
                    [realRadars{rr}.vx realRadars{rr}.vy realRadars{rr}.vz] * time;
            end
        end
    end
    
    if measurePlot
        for rr = 1:length(rdetections)
            if realRadars{rr}.Dimension == 2
                radarAngle = -realRadars{rr}.yaw;
                rotMatrix  = rotz(radarAngle / pi * 180); 
                rotMatrix  = rotMatrix(1:2, 1:2);
                radar_loc  = radarLocs{rr};
                for tt = 1:length(rdetections{rr})
                    detections{rr}{tt} = rdetections{rr}{tt}; % 是delta_loc 需要进行修正
                    delta_loc = rotMatrix' * detections{rr}{tt};
                    detections{rr}{tt} = radar_loc + delta_loc';
                    scatter(detections{rr}{tt}(1), detections{rr}{tt}(2), 10, 'filled', 'b')
                end
            end
        end
    end

    axis([-30 150 -30 150])
end