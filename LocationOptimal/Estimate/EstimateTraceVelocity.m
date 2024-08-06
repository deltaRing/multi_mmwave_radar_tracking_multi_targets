% ͨ���۲쵽��ʱ������ٶ�ֵ
% ����1��singleMeasures �۲�ֵ
% ����2��dt ����ʱ��
% ����3��ά�� Nz
% ���1��optimized_measures �ٶ��Ż���Ĺ۲�
% ���2��posiX
% ���3��posiY
% ���4��RadarInfo
% ���5��Velo
function [optimized_measures, valid_measures, ValidEstimatedVelo, posiX, posiY, radarInfo, velo] ...
    = EstimateTraceVelocity(singleMeasures, ...
    Nz, dt)
    timeIndex = getFrameTimeIndex(singleMeasures);
    optimized_measures = {};
    velo = []; posiX = []; posiY = []; radarInfo = []; % ��ر��� 
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