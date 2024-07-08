% 记录真实的测量值
% 输入0：过去的观测
% 输入1：观测值
% 输入2：当前的观测时间
% 输入3：雷达数据
% 输入4：关联距离
% 输入5：最大容纳量
% 输入6：过期时间
% 输入7：Nz 维度
function meas = RecordRealMeasureInfo(measurements, radarMeasure, time, radarInfo, ...
                                dAssociate, maxNum, expireTime, Nz)
    if nargin == 4
        dAssociate = 2.0; % Origin:2.0
        maxNum = 50;
        expireTime = 1.5; % Origin:1.5
        Nz = 2;
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
                for rr = size(radarMeasure, 1):-1:1
                    if Nz == 2
                        radarInfo_ = radarInfo(rr, :);
                    else

                    end
                    testInfo = radarMeasure(rr, 2:3);
                    velo_    = radarMeasure(rr, 4);
                    deltaInfo = norm(info - testInfo);
                    if deltaInfo < dAssociate
                        radarMeasure(rr, :) = [];
                        measurements{ii}{size_measure + 1}{dataIndex} = [time testInfo velo_ radarInfo_];
                        dataIndex = dataIndex + 1;
                    end
                end

            end
        end
    end

    % 剩下无法关联的观测建立为新观测
    for rr = size(radarMeasure, 1):-1:1
        if Nz == 2
            radarInfo_ = radarInfo(rr, :);
        else
            
        end
        testInfo = radarMeasure(rr, 2:3);
        velo_    = radarMeasure(rr, 4);
        measurements{end + 1}{1}{1} = [time testInfo velo_ radarInfo_];
    end

    meas = measurements;
end