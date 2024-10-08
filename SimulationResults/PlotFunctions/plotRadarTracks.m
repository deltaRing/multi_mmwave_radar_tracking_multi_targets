% 绘制雷达航迹
% 输入1：Tracks 雷达航迹
% 输入2：ID 航迹数目
% 输入3：figureIndex 图像索引
% 无输出
function plotRadarTracks(Tracks, ID, figureIndex)
    if nargin == 2, figureIndex = 10000; end
    
    figure(figureIndex)
    frames = size(Tracks, 2);
    
    plot_target_total = {};
    hold on
    for tt = 1:ID - 1
        plot_target = [];
        for ff = 1:frames
            if isempty(Tracks{ff})
                
            else
                for target_index = 1:length(Tracks{ff})
                    if isempty(Tracks{ff}{target_index}), continue; end
                    if Tracks{ff}{target_index}(1) == tt && Tracks{ff}{target_index}(4) == 1 % 对每个ID进行检测
                        plot_target = [plot_target; Tracks{ff}{target_index}(2:end)];
                    end
                end
            end
        end
        plot_target_total{tt} = plot_target;
        if isempty(plot_target), continue; end
        %plot(plot_target(:, 1), plot_target(:, 2), 'r-.', 'linewidth', 1)
        scatter(plot_target(:, 1), plot_target(:, 2), 5, 'filled', 'r')
    end
    hold off
end