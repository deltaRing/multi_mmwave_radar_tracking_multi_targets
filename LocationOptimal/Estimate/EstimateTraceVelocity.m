% 通过观察到的时间估计速度值
% 输入1：singleMeasures 观测值
% 输入2：dt 步进时间
% 输入3：维度 Nz
% 输出1：optimized_measures 速度优化后的观测
% 输出2：posiX
% 输出3：posiY
% 输出4：RadarInfo
% 输出5：Velo
function [optimized_measures, valid_measures, ValidEstimatedVelo, posiX, posiY, radarInfo, velo] ...
    = EstimateTraceVelocity(singleMeasures, ...
    Nz, dt)
    timeIndex = getFrameTimeIndex(singleMeasures);
    optimized_measures = {};
    velo = []; posiX = []; posiY = []; radarInfo = []; % 相关变量 
    for tt = 1:length(singleMeasures)
        observedNum = length(singleMeasures{tt});
        for ttt = 1:length(singleMeasures{tt})
            info = [];
            for tttt = 1:length(singleMeasures{tt}{ttt})
                info(tttt, :) = singleMeasures{tt}{ttt}{tttt};
            end
            if Nz == 2
                index                = find(info(tttt, 1) == timeIndex);
                posiX{index}{tt}     = info(:, 2); % radars x observeTimes
                posiY{index}{tt}     = info(:, 3);
                velo{index}{tt}      = info(:, 4);
                radarInfo{index}{tt} = info(:, 5:end);
            end
        end
        
    end
    
    [optimized_measures, valid_measures, ValidEstimatedVelo] = VelocityEstimate2(velo, posiX, posiY, radarInfo, dt);
end

function timeIndex = getFrameTimeIndex(singleMeasures)
    timeIndex = [];
    for tt = 1:length(singleMeasures)
        for ttt = 1:length(singleMeasures{tt})
            for tttt = 1:length(singleMeasures{tt}{ttt})
                time = singleMeasures{tt}{ttt}{tttt}(1);
                [~, index] = find(time == timeIndex);
                if isempty(index)
                    timeIndex = [timeIndex time];
                end
            end
        end
    end
    timeIndex = sort(timeIndex);
end