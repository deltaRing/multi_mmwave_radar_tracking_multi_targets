% 记录测量值
% 输入1：measurements 测量值 M x N x [tt measX measY (measZ) measV radarX radarY (radarZ) radarYaw (radarPitch)]
% 输入2：time 时间值
% 输入3：dTargets 每个雷达估计的值
% 输入4：dVelocity 每个雷达测量的多普勒值
% 输入5：dRadar 每个雷达的参数值
% 输入6：dAssociate 经验关联范围
% 输入7：maxNum 最大值
% 输入8：expireTime 过期时间 (如果最后一部分观测值时间大于该值 整个删除)
% 输入9：Nz 维度
% 输出1：meas：航迹结果
function meas = RecordMeasureInfo(measurements, time, dTargets, dVelocity, ...
                            dRadar, dAssociate, maxNum, expireTime, Nz)
    if nargin == 5
        dAssociate = 2.5;
        maxNum = 50;
        expireTime = 0.5;
        Nz = 2;
    end
    meas = [];
    
    % 矫正测量值
    for rr = size(dTargets, 2):-1:1
        if isempty(dTargets{rr}), continue; end
        if Nz == 2
            radarPos = [dRadar{rr}.x dRadar{rr}.y];
            radarYaw = -dRadar{rr}.yaw;
            rotMatrix  = rotz(radarYaw / pi * 180); 
            rotMatrix  = rotMatrix(1:2, 1:2);
            for tt = length(dTargets{rr}):-1:1
                dTargets{rr}{tt} = rotMatrix' * dTargets{rr}{tt} + radarPos';
            end
        else
            
        end
    end
    
    if ~isempty(measurements)
        for ii = length(measurements):-1:1
            while length(measurements{ii}) > maxNum
                % 删除第一个元素
                measurements{ii}(1) = [];
            end

            if abs(measurements{ii}{end}{1}(1) - time) > expireTime % 时间
                measurements(ii) = []; % 删除整个观测
            end
        end
        for ii = 1:length(measurements)
            measures     = measurements{ii}{end};
            size_measure = length(measurements{ii});
            for mm = 1:length(measures)
                % 2D
                if Nz == 2
                    info = measures{mm}(2:3);
                else


                end

                dataIndex = 1;
                for rr = size(dTargets, 2):-1:1
                    if Nz == 2
                        radarInfo = [dRadar{rr}.x dRadar{rr}.y dRadar{rr}.yaw];
                    else

                    end
                    if isempty(dTargets{rr}), continue; end
                    for tt = length(dTargets{rr}):-1:1
                        testInfo = [dTargets{rr}{tt}]';
                        deltaInfo = norm(info - testInfo);
                        if deltaInfo < dAssociate
                            measurements{ii}{size_measure + 1}{dataIndex} = [time testInfo dVelocity{rr}{tt} radarInfo];
                            dTargets{rr}(tt)  = [];
                            dVelocity{rr}(tt) = [];
                            dataIndex = dataIndex + 1;
                        end
                    end
                end

            end
        end
    end
    
    % 剩下无法关联的观测建立为新观测
    for rr = size(dTargets, 2):-1:1
        if Nz == 2
            radarInfo = [dRadar{rr}.x dRadar{rr}.y dRadar{rr}.yaw];
        else
            
        end
        if isempty(dTargets{rr}), continue; end
        
        for tt = length(dTargets{rr}):-1:1
            testInfo = [dTargets{rr}{tt}]';
            measurements{end + 1}{1}{1} = [time testInfo dVelocity{rr}{tt} radarInfo];
        end
    end

    meas = measurements;
end