% ��¼��ʵ�Ĳ���ֵ
% ����0����ȥ�Ĺ۲�
% ����1���۲�ֵ
% ����2����ǰ�Ĺ۲�ʱ��
% ����3���״�����
% ����4����������
% ����5�����������
% ����6������ʱ��
% ����7��Nz ά��
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
                % ɾ����һ��Ԫ��
                measurements{ii}(1) = [];
            end

            if abs(measurements{ii}{end}{1}(1) - time) > expireTime % ʱ��
                measurements(ii) = []; % ɾ�������۲�
            end
        end
        for ii = 1:length(measurements)
            measures     = measurements{ii}{end};
            size_measure = length(measurements{ii});
            for mm = 1:length(measures)
                % 2D
                if Nz == 2
                    info = measures{mm}(2:4);
                else


                end

                dataIndex = 1;
                for rr = size(radarMeasure, 1):-1:1
                    if Nz == 2
                        radarInfo_ = radarInfo(rr, :);
                    else

                    end
                    testInfo = radarMeasure(rr, 2:4);
%                     velo_    = radarMeasure(rr, 4);
                    deltaInfo = norm(info(1:2) - testInfo(1:2));
                    if deltaInfo < dAssociate
                        radarMeasure(rr, :) = [];
%                         measurements{ii}{size_measure + 1}{dataIndex} = [time testInfo velo_ radarInfo_];
                        measurements{ii}{size_measure + 1}{dataIndex} = [time testInfo radarInfo_];
                        dataIndex = dataIndex + 1;
                    end
                end

            end
        end
    end

    % ʣ���޷������Ĺ۲⽨��Ϊ�¹۲�
    for rr = size(radarMeasure, 1):-1:1
        if Nz == 2
            radarInfo_ = radarInfo(rr, :);
        else
            
        end
        testInfo = radarMeasure(rr, 2:4);
%         velo_    = radarMeasure(rr, 4);
%         measurements{end + 1}{1}{1} = [time testInfo velo_ radarInfo_];
        measurements{end + 1}{1}{1} = [time testInfo radarInfo_];
    end

    meas = measurements;
end
