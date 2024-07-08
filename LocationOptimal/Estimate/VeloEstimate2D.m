% 速度估计子函数
% 输入1：获取的速度子向量 Frame x Observes x Tar
% 输入2：获取的X位置子向量 Frame x Observes x Tar
% 输入3：获取的Y位置子向量 Frame x Observes x Tar
% 输入4：子雷达向量 Frame x Observes x 3 (PosiX PosiY radarYaw)
% 输入5：单位步进时间 dt
% 输出1：精确估计的速度向量
function Velo = VeloEstimate2D(VeloVec, PosiX, PosiY, ...
    TarNum, radarInfo, dt, deltaStep, multiStepNum)
    frameLength = length(VeloVec);
    
    if frameLength <= deltaStep
        Velo = [];
        return;
    end
    
    Nz = 2;
    tempVelo = randn(length(1:deltaStep:frameLength), TarNum, Nz);
    % 最大允许迭代300次数据
    % 最大函数值的终止容差 0.01
    options = optimset('MaxIter',3000, 'TolFun', 1e-2);
    Velo = fminsearch(@SingleStepDiff, tempVelo, options); % 估计初始值
    % 精确估计速度
    Velo = fminsearch(@MultiStepDiff, Velo); % 
    % 精确估计速度
    Velo = fminsearch(@SameVeloDiff, Velo); %

    % 单步速度估计子函数
    % 输入tempVelo randn[frames x Tar x Nz]
    function costSSD = SingleStepDiff(tempVelo)
        costSSD = 0;
        index = 0;
        for tt = 1:deltaStep:frameLength - deltaStep
            index = index + 1;
            for pp = 1:length(PosiX{tt + deltaStep})
                meanPosiX = mean(PosiX{tt + deltaStep}{pp}); 
                meanPosiY = mean(PosiY{tt + deltaStep}{pp}); 
                diffInfo = [meanPosiX meanPosiY] - [PosiX{tt}{pp} PosiY{tt}{pp}];
                costSSD = costSSD + norm(diffInfo - ...
                    squeeze(tempVelo(index, pp, :))' * deltaStep * dt);
            end
        end
    end
    
    % 多步子函数
    % 输入tempVelo randn[frames x Tar x Nz]
    function costMSD = MultiStepDiff(tempVelo)
        costMSD = 0;
        index = 0;
        for tt = 1:deltaStep:frameLength
            index = index + 1; multiStepIndex = 0;
            for ttt = tt + deltaStep:deltaStep:frameLength
                if multiStepIndex >= multiStepNum, break; end
                multiStepIndex = multiStepIndex + 1;
                for pp = 1:length(PosiX{ttt})
                    meanPosiX = mean(PosiX{ttt}{pp}); 
                    meanPosiY = mean(PosiY{ttt}{pp}); 
                    diffInfo = [meanPosiX meanPosiY] - [PosiX{tt}{pp} PosiY{tt}{pp}];
                    costMSD = costMSD + norm(diffInfo - ...
                        squeeze(tempVelo(index, pp, :))' * (ttt - tt) * dt);
                end
            end
        end
    end

    % 速度是否符合子函数观测值
    % 输入tempVelo randn[frames x Tar x Nz]
    function costSVD = SameVeloDiff(tempVelo)
        costSVD = 0;
        index = 0;
        for tt = 1:deltaStep:frameLength - 1
            index = index + 1;
            for pp = 1:length(PosiX{tt})
                targetVelo = squeeze(tempVelo(index, pp, :));
                radarAngle = -radarInfo{tt}{pp}(:, 3); % 获得偏航角
                deltaVelo = norm(targetVelo) * ...
                    cos(atan2(targetVelo(2), targetVelo(1)) + radarAngle);
                costSVD = costSVD + norm(deltaVelo - VeloVec{tt}{pp});
            end
        end
    end

    for tttt = 1:TarNum
        disp(strcat('目标 ', num2str(tttt) ,' 速度：'))
        squeeze(Velo(:, tttt, :))
    end
end
