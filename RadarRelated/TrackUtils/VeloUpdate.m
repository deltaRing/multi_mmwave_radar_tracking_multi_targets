% 更新速度
% 输入1：newTracks JPDA后的航迹
% 输入2：oldTracks JPDA前的航迹
% 输入3：observed  是否存在新数据更新航迹
% 输出1：更新速度后的航迹
function TrackInfo = VeloUpdate(newTracks, oldTracks, observed)
    if length(newTracks) ~= length(oldTracks)
        error('VeloUpdate: Track  Length not Equal')
    end
    
    TrackInfo = {};
    for tt = 1:length(newTracks)
        if observed(tt)
            StatusNew = newTracks{tt}.X;
            StatusOld = oldTracks{tt}.X;
            deltaStatus = StatusNew - StatusOld;
            dt          = newTracks{tt}.t;
            if newTracks{tt}.Nz == 3
                vx = deltaStatus(1) / dt;
                vy = deltaStatus(3) / dt;
                vz = deltaStatus(5) / dt;
                x  = StatusNew(1);
                y  = StatusNew(3);
                z  = StatusNew(5);
                newTracks{tt}.X = [x vx y vy z vz]';
            elseif newTracks{tt}.Nz == 2
                vx = deltaStatus(1) / dt;
                vy = deltaStatus(3) / dt;
                x  = StatusNew(1);
                y  = StatusNew(3);
                newTracks{tt}.X = [x vx y vy]';
            else
                error('VeloUpdate: Track Not Support High Dimension');
            end
        else
            % 不进行更新
            
        end
    end
    TrackInfo = newTracks;
end