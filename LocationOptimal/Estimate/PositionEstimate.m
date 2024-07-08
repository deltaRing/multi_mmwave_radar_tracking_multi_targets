% 位置估计函数
function PosiEstimated = PositionEstimate(PosiX, PosiY)
    frameLength = length(VeloVec);
    TarNum = -1;
    for ff = 1:frameLength
        VeloFrame = VeloVec{ff};
        if TarNum < length(VeloFrame)
            TarNum = length(VeloFrame);
        end
    end

    PosiEstimated = [];
    for tt = 1:TarNum
        for ff = 1:frameLength
            PosiEstimated(ff, :) = [mean(PosiX{ff}{tt}) mean(PosiY{ff}{tt})];
        end
    end

end