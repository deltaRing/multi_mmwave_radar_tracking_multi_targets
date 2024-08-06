% ǰ���ٶ�Ԥ��
% ����1������������ TarNum x Frame x Measures
% ����2
% ����3
% ����4
% ����5���Ż����ٶ����� TarNum x Frame - Delta x Nz
% ����6���������� mu
% ����7��M �˲���Ȩ�ش�С
% ����8��deltaStep 
% ����9��
% ���1���˲���Ȩ�� W
% ���2���˲������ Y
% ���3��������� En
function [W, Vpre, En, VpreValid] = VeloPrediction(VeloVec, RadarVec, PosiX, PosiY, ...
    dVOptimal, dVvalid, dVveloItem, mu, M, deltaStep, iternum, dt)
    Nz = 2;
    sigma = 1e-4;
    itr  = size(dVOptimal, 2);
    W = []; Vpre = []; En = []; VpreValid = [];

    if itr < M + deltaStep, return; end

    tarNum = length(dVvalid);
    En   = zeros(tarNum, Nz, itr);   % ��ʼ������ź�
    W    = zeros(tarNum, Nz, M, itr);    % ��ʼ��Ȩֵ����ÿһ�д���һ�ε���
    Vpre = [];
    
    for tt = 1:tarNum
        rtt = dVvalid(tt);
        if sum(dVveloItem(:, rtt)) < M, continue; end % Too less
        VpreValid = [VpreValid; rtt];
        for dd = 1:Nz 
            for kk = M:itr - 1
                if ~dVveloItem(kk, rtt), continue, end
                x = dVOptimal(tt, kk:-1:kk-M + 1, dd);
                y = x * squeeze(W(tt, dd, :, kk-1));              % �˲��������
                dn = dVOptimal(tt, kk + 1, dd);          % ��һʱ�̵��ٶ�ֵ
                en(tt, dd, kk) = dn - y;                 % ��k�ε��������
                W(tt, dd, :, kk) = squeeze(W(tt, dd, :, kk-1))+2*mu*en(tt, dd, kk)*x';     % �˲���Ȩֵ����ĵ���ʽ
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

        % ��˹ţ�ٷ� 
        for kk = 1:deltaStep
            V1 = Vpre(tt, kk, 1); V2 = Vpre(tt, kk, 2);
            if length(VeloVec{itr + kk}) < rtt, continue; end
            VeloMeasured = VeloVec{itr + kk}{rtt};
            if isempty(VeloMeasured), continue, end
            radarAngle = -RadarVec{itr + kk}{tt}(:, 3);
            
            err = inf; minV1 = inf; minV2 = inf;
            for it = 1:iternum
                cuAngle = atan2(V2, V1); cuNorm = norm([V1 V2]);
                VeloComputed = cuNorm * ...
                    cos(cuAngle + radarAngle);
                % ������
                error = VeloMeasured - VeloComputed;
                if norm(err) > norm(error)
                    err = error;
                    minV1 = V1;
                    minV2 = V2;
                end
                if norm(error) < sigma, break; end
                Jf = [2 * V1 / cuNorm * cos(cuAngle + radarAngle) + ...
                      V2 / cuNorm * sin(cuAngle + radarAngle) 2 * V2 / cuNorm * ...
                      cos(cuAngle + radarAngle) - V1 / cuNorm * sin(cuAngle + radarAngle)];
                delta_info = inv(Jf' * Jf) * Jf' * error;
    
                % if norm(delta_info) < sigma, break; end
                V1 = V1 + delta_info(1);
                V2 = V2 + delta_info(2);
            end
            V1 = minV1; V2 = minV2;
            
            if itr + kk + 1 >= length(PosiX)
                % ������ǵ�ǰֵ �����пɿ����ж� ֻ�����ٶ��ж�
                ERR_ORIN = sum(abs(norm(squeeze(Vpre(tt, kk, :))) * ...
                    cos(atan2(Vpre(tt, kk, 2), Vpre(tt, kk, 1)) + radarAngle) - VeloMeasured));
                ERR_OPTI = sum(abs(norm([V1 V2]) * cos(atan2(V2, V1) + radarAngle) - VeloMeasured));
                if ERR_OPTI < ERR_ORIN, Vpre(tt, kk, 1) = V1; Vpre(tt, kk, 2) = V2; end
            else
                ERR_ORIN = sum(abs(norm(squeeze(Vpre(tt, kk, :))) * ...
                    cos(atan2(Vpre(tt, kk, 2), Vpre(tt, kk, 1)) + radarAngle) - VeloMeasured));
                ERR_OPTI = sum(abs(norm([V1 V2]) * cos(atan2(V2, V1) + radarAngle) - VeloMeasured));
                % ������ǰ�ٶȵĿɿ���
                if length(PosiX{itr + kk}) >= rtt && length(PosiX{itr + kk + 1}) >= rtt
                    X = mean(PosiX{itr + kk}{rtt}); Y = mean(PosiY{itr + kk}{rtt});
                    EX = mean(PosiX{itr + kk + 1}{rtt}); EY = mean(PosiY{itr + kk + 1}{rtt});
                    MSE_ORIN = norm([EX EY] - ([X Y] + squeeze(Vpre(tt, kk, :)) * dt));
                    MSE_OPTI = norm([EX EY] - ([X Y] + [V1 V2] * dt));
                    if 2 * MSE_OPTI + ERR_OPTI < MSE_ORIN + ERR_ORIN
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
