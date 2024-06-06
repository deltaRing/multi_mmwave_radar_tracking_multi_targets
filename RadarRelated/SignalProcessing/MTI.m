% 动目标指示
% 输入1：RadarEcho 雷达回波 结构：[通道数目] optical x 采样点数目 x 脉冲数
% 输入2：MTI_delay 几次MTI滤波
% 输出1：MTI_result 动目标指示结果
function MTI_result = MTI(RadarEcho, MTI_delay)
    if length(size(RadarEcho)) == 2
        MTI_result = RadarEcho(:, 1:end-MTI_delay) - RadarEcho(:, MTI_delay + 1:end);
    elseif length(size(RadarEcho)) == 3
        MTI_result = RadarEcho(:, :, 1:end-MTI_delay) - RadarEcho(:, :, MTI_delay + 1:end);
    else
        MTI_result = [];
        error('MTI: Higher Dimension Detected, Not Support')
    end
end