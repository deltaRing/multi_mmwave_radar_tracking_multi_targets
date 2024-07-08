% EstimateTraceLocate 估计当前航迹的参数
% 输入1：根据多部雷达可关联的观测结果进行优化 dTargets
% 输入2：根据多部雷达观测的速度值
% 输入3：多部雷达的位置 dRadar
% 输入4：关联值 dAssociateRange
% 输入4：维度 Nz
% 输出1：优化后的观测值 1 x N {xx yy (zz)}
function dMeasures = EstimateTraceLocate(singleMeasures, Nz)
    if nargin == 1
        Nz = 2;
    end
    dMeasures = {};
    posiX = []; posiY = []; radarInfo = []; % 相关变量 
    for tt = 1:length(singleMeasures)
        observedNum = length(singleMeasures{tt});
        for ttt = 1:length(singleMeasures{tt})
            info = [];
            for tttt = 1:length(singleMeasures{tt}{ttt})
                info(tttt, :) = singleMeasures{tt}{ttt}{tttt};
            end
            if Nz == 2
                posiX{ttt}{tt}     = info(:, 2); % radars x observeTimes
                posiY{ttt}{tt}     = info(:, 3);
            end
        end
    end
    dMeasures = PositionEstimate(posiX, posiY);
end