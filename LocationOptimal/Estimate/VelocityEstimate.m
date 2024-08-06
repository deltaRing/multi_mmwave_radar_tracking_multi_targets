% 速度估计子函数
% 输入1：获取的速度子向量 Frame x Observes x Tar
% 输入2：获取的X位置子向量 Frame x Observes x Tar
% 输入3：获取的Y位置子向量 Frame x Observes x Tar
% 输入4：子雷达向量 Frame x Observes x 3 (PosiX PosiY radarYaw)
% 输入5：单位步进时间 dt
% 可选输入1：
% 可选输入2：
% 可选输入3：
% 可选输入4：
% 输出1：精确估计的速度向量
function [VeloEstimated, ValidTar, ValidEstimatedVelo] = VelocityEstimate(VeloVec, PosiX, PosiY, ...
                            RadarVec, dt, deltaStep, minimalStep, ...
                            iternum, sigma)
    if nargin == 5
        deltaStep = 5;
        minimalStep = 10; % Origin:10
        iternum = 1000;
        sigma = 1e-4;
    end

    frameLength = length(VeloVec);
    if frameLength <= minimalStep || frameLength <= deltaStep
        VeloEstimated = [];
        ValidTar = [];
        ValidEstimatedVelo = [];
        return;
    end

    if deltaStep >= minimalStep, deltaStep = minimalStep; end
    
    TarNum = -1;
    for ff = 1:frameLength
        VeloFrame = VeloVec{ff};
        if TarNum < length(VeloFrame)
            TarNum = length(VeloFrame);
        end
    end

    VeloEstimated = [];
    ValidEstimatedVelo = [];
    ValidTar = [];
    for tt = 1:TarNum
        VeloMatrix = []; PosiMatrix = [];
        for ff = 1:frameLength
            if length(VeloVec{ff}) < tt
                VeloMatrix{ff} = [];
                PosiMatrix(ff, :) = zeros(1, 2);
                continue;
            end % 无该目标
            if isempty(PosiX{ff}{tt}) || isempty(VeloVec{ff}{tt})
                if ff == 1 || isempty(VeloMatrix{ff - 1})
                    VeloMatrix{ff} = [];
                    PosiMatrix(ff, :) = zeros(1, 2);
                else
                    VeloMatrix{ff} = VeloMatrix{ff - 1};
                    PosiMatrix(ff, :) = PosiMatrix(ff - 1, :);
                end
            else
                VeloMatrix{ff} = VeloVec{ff}{tt};
                PosiMatrix(ff, :) = [mean(PosiX{ff}{tt}) mean(PosiY{ff}{tt})];
            end
        end
        
        ObsvTimeMatrix = [];
        for ttt = 1:deltaStep
            ObsvTimeMatrix(ttt, :) = [ttt * dt 1];
        end

        Velox = []; Veloy = [];
        deltaMatrixs = [];
        for ttt = 1:frameLength - deltaStep
            deltaMatrix = [];
            validTimes = [];
            if isempty(VeloMatrix{ttt})
                ValidEstimatedVelo(ttt, tt) = 0;
                continue;
            end
            for dd = 1:deltaStep
                if isempty(VeloMatrix{dd + ttt}), continue; end
                deltaMatrix(length(validTimes) + 1, :) = PosiMatrix(dd + ttt, :) - PosiMatrix(ttt, :);
                validTimes = [validTimes; dd];
            end
            deltaMatrixs{ttt} = PosiMatrix(dd + ttt, :) - PosiMatrix(ttt, :);
            if length(validTimes) < 1
                ValidEstimatedVelo(ttt, tt) = 0;
                continue;
            end
            ValidEstimatedVelo(ttt, tt) = 1;
            obMatrixTime = ObsvTimeMatrix(validTimes);
            % 最小二乘法
            estimateVeloX = inv(obMatrixTime' * obMatrixTime) * obMatrixTime' * deltaMatrix(:, 1);
            estimateVeloY = inv(obMatrixTime' * obMatrixTime) * obMatrixTime' * deltaMatrix(:, 2);
            Velox = [Velox estimateVeloX(1)];
            Veloy = [Veloy estimateVeloY(1)];
        end
        Velo_ = [Velox; Veloy]';
        
        % 高斯牛顿法 
        for ff = 1:length(Velox)
            if ~ValidEstimatedVelo(ff, tt), continue; end
            if isempty(RadarVec{ff}{tt}), continue; end

            err = inf;
            minV1 = inf; minV2 = inf;
            V1 = Velo_(ff, 1); V2 = Velo_(ff, 2);
            VeloMeasured = VeloVec{ff}{tt};
            radarAngle = -RadarVec{ff}{tt}(:, 3);
            for it = 1:iternum
                cuAngle = atan2(V2, V1); cuNorm = norm([V1 V2]);
                VeloComputed = cuNorm * ...
                    cos(cuAngle + radarAngle);
                % 误差计算
                error = VeloMeasured - VeloComputed;
                if norm(err) > norm(error)
                    err = error;
                    minV1 = V1;
                    minV2 = V2;
                end
                if norm(error) < sigma
                    break
                end

                Jf = [2 * V1 / cuNorm * cos(cuAngle + radarAngle) + ...
                     V2 / cuNorm * sin(cuAngle + radarAngle) 2 * V2 / cuNorm * ...
                    cos(cuAngle + radarAngle) - V1 / cuNorm * sin(cuAngle + radarAngle)];
             
                delta_info = inv(Jf' * Jf) * Jf' * error;

                if norm(delta_info) < sigma
                    break
                end
                V1 = V1 + delta_info(1);
                V2 = V2 + delta_info(2);
            end
            
            V1 = minV1; V2 = minV2;
            ERR_ORIN = sum(abs(norm([Velo_(ff, :)]) * ...
                    cos(atan2(Velo_(ff, 2), Velo_(ff, 1)) + radarAngle) - VeloVec{ff}{tt}));
            ERR_OPTI = sum(abs(norm([V1 V2]) * ...
                cos(atan2(V2, V1) + radarAngle) - VeloVec{ff}{tt}));
            % 评估当前速度的可靠性
            if length(PosiX{ff}) >= tt && length(PosiX{ff + 1}) >= tt
                X = mean(PosiX{ff}{tt}); Y = mean(PosiY{ff}{tt});
                EX = mean(PosiX{ff + 1}{tt}); EY = mean(PosiY{ff + 1}{tt});
                MSE_ORIN = norm([EX EY] - ([X Y] + Velo_(ff, :) * dt));
                MSE_OPTI = norm([EX EY] - ([X Y] + [V1 V2] * dt));
                if 2 * MSE_OPTI + ERR_OPTI < MSE_ORIN + ERR_ORIN
                    Velo_(ff, 1) = V1; Velo_(ff, 2) = V2;
                else
                    1; 
                end
            else
                if ERR_OPTI < ERR_ORIN
                    Velo_(ff, 1) = V1; Velo_(ff, 2) = V2;
                end
            end
        end
        validIndex = find(ValidEstimatedVelo(:, tt));
        VeloValid  = zeros(size(ValidEstimatedVelo, 1), 2); 
        if isempty(Velo_)
            ValidEstimatedVelo(1, tt) = 0;
        else
            VeloValid(validIndex, :) = Velo_;
            ValidTar = [ValidTar; tt];
        end
        VeloEstimated(tt, :, :) = VeloValid;
    end
end
