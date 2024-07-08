% 绘制误差曲线
% 输入1：航迹
% 输入2：观测到的数据
% 输入3：绘图的index
% 输入4：观测到的误差信息
% 输入5：当前frame
function Error = plotErrorCurve(Tracks, observed_data, index, Error, ff)
    if nargin == 2, index = 10001; end
    figure(index)
    hold on

    for ii = 1:length(Tracks)
        line = []; 
        if ii > length(observed_data), break; end
        for jj = 1:length(observed_data{ii})
            if isempty(observed_data{ii}{jj}), continue; end
            error = ...
                norm(observed_data{ii}{jj}([1, 3]) - Tracks{ii}.X([1, 3]));
            line = [line error];
        end
        Error{Tracks{ii}.ID}{ff} = mean(line);
    end

    for ii = 1:length(Error)
        line = [];
        for jj = 1:size(Error{ii}, 2)
            if isempty(Error{ii}{jj}), continue; end
            line = [line; jj Error{ii}{jj}];
        end
        if ~isempty(line), plot(line(:, 1), line(:, 2)); end
    end
    hold off
end