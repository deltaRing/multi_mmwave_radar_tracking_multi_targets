% 记录航迹信息
% 输入1：track 已知航迹
% 输入2：newTracks 新到来的航迹
% 输入3：maxNum 最大容纳航迹
% 输出1：Tracks 已包含的航迹
function Tracks = RecordTraceInfo(track, newTracks, maxNum)
    newTracksFlag = zeros([1, length(newTracks)]);
    for tt = 1:length(track)
        % 删除多余的航迹
        while length(track{tt}) > maxNum
            track{tt}(1) = [];
        end
        tid = track{tt}{1}.ID;
        
        for ttt = 1:length(newTracks)
            if newTracksFlag(ttt), continue; end
            
            ttid = newTracks{ttt}.ID;
            if tid == ttid
                track{tt}{length(track{tt}) + 1} = newTracksFlag(ttt);
                newTracksFlag(ttt) = 1;
            end
        end
    end
    Tracks = track;
end