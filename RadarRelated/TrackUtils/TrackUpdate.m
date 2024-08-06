% 更新并删除过期航迹
% 输入1：Tracks 原有的航迹
% 输入2：Observed 观测到的航迹有效矩阵 （若有目标1 无0）
% 输出1：更新的航迹
function [Track] = TrackUpdate(Tracks, observed)
    delete_index = [];
    for tt = 1:length(observed)
        if observed(tt)
            if Tracks{tt}.Type == 0 % 未确认
                Tracks{tt}.LossFrame = 0; % 丢失航迹数清零
                Tracks{tt}.ObservedFrame = Tracks{tt}.ObservedFrame + 1; % 观测数自增
                if Tracks{tt}.ObservedFrame >= Tracks{tt}.ConfirmMax
                    Tracks{tt}.Type = 1; % 航迹确认
                end
            elseif Tracks{tt}.Type == 1 % 确认航迹
                Tracks{tt}.LossFrame = 0; % 丢失航迹数清零
                Tracks{tt}.ObservedFrame = Tracks{tt}.ObservedFrame + 1; % 观测数自增
            elseif Tracks{tt}.Type == 2 % 丢失航迹
                
            end
        else
            Tracks{tt}.LossFrame = Tracks{tt}.LossFrame + 1; % 丢失帧数自增
%             Tracks{tt}.X         = Tracks{tt}.F * Tracks{tt}.X;
            if Tracks{tt}.LossFrame >= Tracks{tt}.LossFrameMax
                delete_index = [delete_index tt]; % 加入删除队列中
                Tracks{tt}.Type = 2; % 转为丢失航迹
            end
        end
    end
    if ~isempty(delete_index), Tracks(delete_index) = []; end
    Track = Tracks;
end