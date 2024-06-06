% 获取事件
% 获取JPDA关联事件
% 输入1：量测矩阵 measure_confirm_matrix
% 输入2：最大事件数 max_events
% 输出1：interconnect_matrix
% 输出2：event_num 有效事件数目
%
function [interconnect_matrix, event_num] = GetEvents(measure_confirm_matrix, max_events)
    size_measures = size(measure_confirm_matrix, 1);
    size_targets  = size(measure_confirm_matrix, 2);
    interconnect_matrix = zeros(size_measures, size_targets, max_events);
    event_num = 0;
    
    if size_measures ~= 0
        vectors = {};
        % 拆解向量
        for mm = 1:size_measures
            index = find(measure_confirm_matrix(mm, :) == 1);
            for tt = 1:length(index)
                vectors{mm}{tt} = zeros(1, size_targets); % 每组测量值组合为不同的向量
                vectors{mm}{tt}(index(tt)) = 1;
            end
        end
        
        composed_result = {};
        temp_result = [];
        tt = -1;
        mm = 1;
        [~, composed_result] = build_events_trees(vectors, ...
                                            composed_result, temp_result, tt, mm, ...
                                            size_measures, size_targets);
                                        
        event_num = length(composed_result);
        for iii = 1:event_num
            if iii > max_events, break, end
            interconnect_matrix(:, :, iii) = composed_result{iii};
        end
    
    else
        return
    end
end

% 检测有效组合
% temp_result 当前检测结果
% composed_result 组合的交叉矩阵
% vectors: 组合的向量
% tt: 第几个目标
% mm: 第几个测量值
function [valid, composed_result] = build_events_trees(vectors, ...
                                            composed_result, temp_result, tt, mm, ...
                                            measurements, targets)
    valid = 0;
    % 检查
    if mm <= 0
        error('JPDA SubFunction Building Events Trees: measurements Error');
    end
    
    if tt <= 0 && ~isempty(temp_result)
        error('JPDA SubFunction Building Events Trees: target num Error');
    end
    
    if targets <= 1
        error('JPDA SubFunction Building Events Trees: no validate targets')    
    end
    
    if isempty(temp_result)
        % 只有一个量测
        if size(vectors, 2) == 1 && isempty(temp_result)
            for iii = 1:length(vectors{mm})
                composed_result{iii} = vectors{mm}{iii};
            end
            return
        end
        
        for ttt = 1:length(vectors{mm})
            % 针对不同的测量值
            temp_result = vectors{mm}{ttt};
            for tttt = 1:length(vectors{mm + 1})
                [valid, composed_result] = build_events_trees(vectors, ...
                                            composed_result, temp_result, tttt, mm + 1, ...
                                            measurements, targets);
            end
        end
        % 初步工作
        return;
    end
    
    % 杂波始终是第一个测量值
    % 检测当前构造的量测组合是否有效
    temp_result = [temp_result; vectors{mm}{tt}];
    for ii = 2:targets
        if sum(temp_result(:, ii)) > 1
            valid = 0;
            return;
        end
    end
    
    % 已经完成搜索
    if mm >= measurements
        composed_result{length(composed_result) + 1} = temp_result;
        valid = 1;
        return;
    end
    
    % 继续搜索
    for ttt = 1:length(vectors{mm + 1})
        [valid, composed_result] = build_events_trees(vectors, ...
                                            composed_result, temp_result, ttt, mm + 1, ...
                                            measurements, targets);
        
    end

end
