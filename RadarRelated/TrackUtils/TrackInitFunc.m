% 将所有无法关联的点迹进行初始化
% 输入1：目标位置信息 tarPosiInfo  {N x 2 / N x 3}
% 输入2：目标速度信息 tarVeloInfo （速度未知 将随机初始化速度）{N x 1}
% 输入3：当前帧数信息 FrameIndex 
% 输入4：当前步进时间信息 dt
% 输入5：当前ID号
% 输入6：已有航迹（可是 空航迹也可是 有目标的航迹）
% 输出1：已有航迹 + 新建航迹
% 输出2：已分配ID
function [Tracks, ID] = TrackInitFunc(tarPosInfo, tarVeloInfo, ...
                                            FrameIndex, dt, ID, Tracks)
    for tt = 1:length(tarPosInfo)
        VeloInfo = [];
        Velo = tarVeloInfo{tt}; % 速度信息
        if length(Velo) == 1
            if length(tarPosInfo{tt}) == 2
                weight = rand([1, 2]);
                VeloInfo = Velo * weight;
            elseif length(tarPosInfo{tt}) == 3
                weight = rand([1, 3]);
                VeloInfo = Velo * weight;
            end
        elseif length(Velo) == 2 || length(Velo) == 3
            VeloInfo = Velo;
        else
            error('TrackInitFunc: Not Support This Dimension of Velocity')
        end
        Tracks{length(Tracks) + 1} = trackInit(tarPosInfo{tt}, ...
                                       VeloInfo, dt, FrameIndex, ID);
        ID = ID + 1;
    end
end