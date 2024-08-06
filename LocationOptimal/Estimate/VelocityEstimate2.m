% �ٶȹ����Ӻ���
% ����1����ȡ���ٶ������� Frame x Observes x Tar
% ����2����ȡ��Xλ�������� Frame x Observes x Tar
% ����3����ȡ��Yλ�������� Frame x Observes x Tar
% ����4�����״����� Frame x Observes x 3 (PosiX PosiY radarYaw)
% ����5����λ����ʱ�� dt
% ��ѡ����1��
% ��ѡ����2��
% ��ѡ����3��
% ��ѡ����4��
% ���1����ȷ���Ƶ��ٶ�����
function [VeloEstimated, ValidTar, ValidEstimatedVelo] = VelocityEstimate2(VeloVec, PosiX, PosiY, ...
                            RadarVec, dt, deltaStep, minimalStep, ...
                            iternum, sigma)
    if nargin == 5
        deltaStep = 5;
        minimalStep = 10; % Origin:10
        iternum = 1000;
        sigma = 1e-8;
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
        VeloMatrix = []; PosiMatrix = []; RadarMatrix = [];
        for ff = 1:frameLength
            if length(VeloVec{ff}) < tt
                VeloMatrix{ff} = [];
                RadarMatrix{ff} = zeros(1, 2);
                PositionMatrix{ff} = zeros(1, 2);
                PosiMatrix(ff, :) = zeros(1, 2);
                continue;
            end % �޸�Ŀ��
            if isempty(PosiX{ff}{tt}) || isempty(VeloVec{ff}{tt})
                if ff == 1 || isempty(VeloMatrix{ff - 1})
                    VeloMatrix{ff} = [];
                    RadarMatrix{ff} = zeros(1, 2);
                    PositionMatrix{ff} = zeros(1, 2);
                    PosiMatrix(ff, :) = zeros(1, 2);
                else
                    VeloMatrix{ff} = VeloMatrix{ff - 1};
                    RadarMatrix{ff} = RadarMatrix{ff - 1};
                    PositionMatrix{ff} = PositionMatrix{ff - 1};
                    PosiMatrix(ff, :) = PosiMatrix(ff - 1, :);
                end
            else
                VeloMatrix{ff} = VeloVec{ff}{tt};
                RadarMatrix{ff} = RadarVec{ff}{tt};
                PositionMatrix{ff} = [PosiX{ff}{tt} PosiY{ff}{tt}];
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
            % ��С���˷�
            estimateVeloX = inv(obMatrixTime' * obMatrixTime) * obMatrixTime' * deltaMatrix(:, 1);
            estimateVeloY = inv(obMatrixTime' * obMatrixTime) * obMatrixTime' * deltaMatrix(:, 2);
            Velox = [Velox estimateVeloX(1)];
            Veloy = [Veloy estimateVeloY(1)];
        end
        Velo_ = [Velox; Veloy]';
        
        % ��˹ţ�ٷ� 
        for ff = 1:length(Velox)
            if ~ValidEstimatedVelo(ff, tt), continue; end
            if isempty(RadarVec{ff}{tt}), continue; end

            err = inf;
            minV1 = inf; minV2 = inf;
            V1 = Velo_(ff, 1); V2 = Velo_(ff, 2);
            VeloMeasured = VeloVec{ff}{tt};
            targetPosi = [PosiX{ff}{tt} PosiY{ff}{tt}];
            radarPosi  = RadarVec{ff}{tt}(:, 1:2);
            radarAngle = -RadarVec{ff}{tt}(:, 3);
            deltaR = targetPosi - radarPosi;
            for it = 1:iternum
                cuAngle = acos([V1 V2] * deltaR'/ norm([V1 V2]) / norm(deltaR))';
                cuNorm = norm([V1 V2]);
                VeloComputed = cuNorm * ...
                    cos(cuAngle);
                % ������
                
                error = VeloMeasured - VeloComputed;
                if norm(err) > norm(error)
                    err = error;
                    minV1 = V1;
                    minV2 = V2;
                end
                if norm(error) < sigma
                    break
                end
%                 Jf = [2 * V1 / cuNorm * cos(cuAngle) + ...
%                      V2 / cuNorm * sin(cuAngle) ...
%                      2 * V2 / cuNorm * ...
%                     cos(cuAngle) - V1 / cuNorm * sin(cuAngle)];
                Jf = [V1 / cuNorm * cos(cuAngle) - ...
                     sin(cuAngle) * (deltaR(1) / norm(deltaR) - (V1^2 * deltaR(1) + V1 * V2 * deltaR(2)) / (cuNorm^2 * norm(deltaR))) ...
                     V2 / cuNorm * cos(cuAngle) - ...
                     sin(cuAngle) * (deltaR(2) / norm(deltaR) - (V2^2 * deltaR(2) + V1 * V2 * deltaR(1)) / (cuNorm^2 * norm(deltaR)))
                     ];
             
                delta_info = inv(Jf' * Jf + 1e-8 * eye(2)) * Jf' * error;

                if norm(delta_info) < sigma
                    break
                end
                V1 = V1 + delta_info(1);
                V2 = V2 + delta_info(2);
            end
            
            V1 = minV1; V2 = minV2;
            orinV1 = Velo_(ff, 1); orinV2 = Velo_(ff, 2);
            
            cuAngle = acos([V1 V2] * deltaR'/ norm([V1 V2]) / norm(deltaR))';
            cuNorm = norm([V1 V2]);
            VeloComputed = cuNorm * ...
                    cos(cuAngle);
                
            orAngle = acos([orinV1 orinV2] * deltaR'/ norm([orinV1 orinV2]) / norm(deltaR))';
            orNorm = norm([orinV1 orinV2]);
            VeloOrinComputed = orNorm * ...
                    cos(orAngle);
            
            ERR_ORIN = sum(abs(VeloOrinComputed - VeloVec{ff}{tt}));
            ERR_OPTI = sum(abs(VeloComputed - VeloVec{ff}{tt}));
            % ������ǰ�ٶȵĿɿ���
            if length(PosiX{ff}) >= tt && length(PosiX{ff + 1}) >= tt
                X = mean(PosiX{ff}{tt}); Y = mean(PosiY{ff}{tt});
                EX = mean(PosiX{ff + 1}{tt}); EY = mean(PosiY{ff + 1}{tt});
                MSE_ORIN = norm([EX EY] - ([X Y] + Velo_(ff, :) * dt));
                MSE_OPTI = norm([EX EY] - ([X Y] + [V1 V2] * dt));
                if 2 * MSE_OPTI + ERR_OPTI < MSE_ORIN + 0.5 * ERR_ORIN
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

function y = norm_(data)
    y = sqrt(data(:, 1).^2 + data(:, 2).^ 2);
end