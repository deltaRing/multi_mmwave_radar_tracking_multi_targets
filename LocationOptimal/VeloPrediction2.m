% 前向速度预测
% 输入1：测量多普勒 TarNum x Frame x Measures
% 输入2
% 输入3
% 输入4
% 输入5：优化后速度向量 TarNum x Frame - Delta x Nz
% 输入6：收敛因子 mu
% 输入7：M 滤波器权重大小
% 输入8：deltaStep 
% 输入9：
% 输出1：滤波器权重 W
% 输出2：滤波器输出 Y
% 输出3：误差序列 En
function [W, Vpre, En, VpreValid] = VeloPrediction2(VeloVec, RadarVec, PosiX, PosiY, ...
    dVOptimal, dVvalid, dVveloItem, mu, M, deltaStep, iternum, dt)
    Nz = 2;
    sigma = 1e-8;
    itr  = size(dVOptimal, 2);
    W = []; Vpre = []; En = []; VpreValid = [];

    if itr < M + deltaStep, return; end

    tarNum = length(dVvalid);
    En   = zeros(tarNum, Nz, itr);   % 初始化误差信号
    W    = zeros(tarNum, Nz, M, itr);    % 初始化权值矩阵，每一列代表一次迭代
    Vpre = [];
    
    for tt = 1:tarNum
        rtt = dVvalid(tt);
        if sum(dVveloItem(:, rtt)) < M, continue; end % Too less
        VpreValid = [VpreValid; rtt];
        for dd = 1:Nz 
            for kk = M:itr - 1
                if ~dVveloItem(kk, rtt), continue, end
                x = dVOptimal(tt, kk:-1:kk-M + 1, dd);
                y = x * squeeze(W(tt, dd, :, kk-1));              % 滤波器的输出
                dn = dVOptimal(tt, kk + 1, dd);          % 下一时刻的速度值
                en(tt, dd, kk) = dn - y;                 % 第k次迭代的误差
                W(tt, dd, :, kk) = squeeze(W(tt, dd, :, kk-1))+2*mu*en(tt, dd, kk)*x';     % 滤波器权值计算的迭代式
            end

            for kk = 1:deltaStep 
                expectInput = squeeze(dVOptimal(tt, itr:-1:itr + kk - M, dd));
                index = 1;
                while length(expectInput) < M
                    expectInput(end + 1) = Vpre(tt, index, dd);
                    index = index + 1;
                end
                Vpre(tt, kk, dd) = expectInput * squeeze(W(tt, dd, :, end-1)); %
            end
        end

        % 高斯牛顿法 
        for kk = 1:deltaStep
            V1 = Vpre(tt, kk, 1); V2 = Vpre(tt, kk, 2);
            if length(VeloVec{itr + kk}) < rtt, continue; end
            VeloMeasured = VeloVec{itr + kk}{rtt};
            if isempty(VeloMeasured), continue, end
            radarAngle = -RadarVec{itr + kk}{tt}(:, 3);
            radarPosi  = RadarVec{itr + kk}{tt}(:, 1:2);
            targetPosi = [PosiX{itr + kk}{tt} PosiY{itr + kk}{tt}];
            deltaR = targetPosi - radarPosi;
            
            err = inf; minV1 = inf; minV2 = inf;
            for it = 1:iternum
                cuAngle = acos([V1 V2] * deltaR'/ norm([V1 V2]) / norm(deltaR))';
                cuNorm = norm([V1 V2]);
                VeloComputed = cuNorm * ...
                    cos(cuAngle);
                % 误差计算
                error = VeloMeasured - VeloComputed;
                if norm(err) > norm(error)
                    err = error;
                    minV1 = V1;
                    minV2 = V2;
                end
                if norm(error) < sigma, break; end
%                 Jf = [2 * V1 / cuNorm * cos(cuAngle) + ...
%                       V2 / cuNorm * sin(cuAngle) 2 * V2 / cuNorm * ...
%                       cos(cuAngle) - V1 / cuNorm * sin(cuAngle)];
                    Jf = [V1 / cuNorm * cos(cuAngle) - ...
                     sin(cuAngle) * (deltaR(1) / norm(deltaR) - (V1^2 * deltaR(1) + V1 * V2 * deltaR(2)) / (cuNorm^2 * norm(deltaR))) ...
                     V2 / cuNorm * cos(cuAngle) - ...
                     sin(cuAngle) * (deltaR(2) / norm(deltaR) - (V2^2 * deltaR(2) + V1 * V2 * deltaR(1)) / (cuNorm^2 * norm(deltaR)))
                     ];

                delta_info = inv(Jf' * Jf + 1e-8 * eye(2)) * Jf' * error;
    
                % if norm(delta_info) < sigma, break; end
                V1 = V1 + delta_info(1);
                V2 = V2 + delta_info(2);
            end
            V1 = minV1; V2 = minV2;
            
            orinV1 = Vpre(tt, kk, 1); orinV2 = Vpre(tt, kk, 2);
            
            cuAngle = acos([V1 V2] * deltaR'/ norm([V1 V2]) / norm(deltaR))';
            cuNorm = norm([V1 V2]);
            VeloComputed = cuNorm * ...
                    cos(cuAngle);
                
            orAngle = acos([orinV1 orinV2] * deltaR'/ norm([orinV1 orinV2]) / norm(deltaR))';
            orNorm = norm([orinV1 orinV2]);
            VeloOrinComputed = orNorm * ...
                    cos(orAngle);
            
            if itr + kk + 1 >= length(PosiX)
                % 如果就是当前值 不进行可靠性判断 只进行速度判断
                ERR_ORIN = sum(abs(VeloOrinComputed - VeloMeasured));
                ERR_OPTI = sum(abs(VeloComputed - VeloMeasured));
                if ERR_OPTI < ERR_ORIN, Vpre(tt, kk, 1) = V1; Vpre(tt, kk, 2) = V2; end
            else
                ERR_ORIN = sum(abs(VeloOrinComputed - VeloMeasured));
                ERR_OPTI = sum(abs(VeloComputed - VeloMeasured));
                % 评估当前速度的可靠性
                if length(PosiX{itr + kk}) >= rtt && length(PosiX{itr + kk + 1}) >= rtt
                    X = mean(PosiX{itr + kk}{rtt}); Y = mean(PosiY{itr + kk}{rtt});
                    EX = mean(PosiX{itr + kk + 1}{rtt}); EY = mean(PosiY{itr + kk + 1}{rtt});
                    MSE_ORIN = norm([EX EY] - ([X Y] + squeeze(Vpre(tt, kk, :)) * dt));
                    MSE_OPTI = norm([EX EY] - ([X Y] + [V1 V2] * dt));
                    if MSE_OPTI + ERR_OPTI < MSE_ORIN + 0.5 * ERR_ORIN
                        Vpre(tt, kk, 1) = V1; Vpre(tt, kk, 2) = V2; 
                    else
                        1;
                    end
                else
                    if ERR_OPTI < ERR_ORIN, Vpre(tt, kk, 1) = V1; Vpre(tt, kk, 2) = V2; end
                end
            end
        end
    end
end
