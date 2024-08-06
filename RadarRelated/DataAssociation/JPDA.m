% 联合概率数据关联 （JPDA）
% 输入1：跟踪航迹
% 输入2：量测值 4 x N / 6 x N
% 格式 [x vx y vy z vz]
% 输入3：Pd 检测概率
% 输入4：正确量测落入跟踪门内的概率
% 输入5：门限
% 输入6：最大量测
% 输入7：最大事件数目
% 输出1：最新的跟踪航迹部分 Tracks
% 输出2：航迹是否观测到新数据
% 输出3：未能进行观测到的新量测数据
function [Tracks, observed, empty_measurements, observed_track] = JPDA(tracks, measurements, ...
                                    Pd, Pg, g_sigma, ...
                                    lambda, max_measures, max_events)
    if nargin == 2
        Pd = 1;                    %检测概率
        Pg = 0.99;                 %正确量测落入跟踪门内得概率
        g_sigma = 5.0;            %门限,经验值，与R和Q有关，必须合理设置
        lambda = 2;                                          
        max_measures = 10000;       % 最大的量测值
        max_events   = 100000;      % 最大事件数目
    end

    empty_measurements = [];
    observed = [];
    observed_track = [];     % 观测到的航迹
    gamma = lambda*10^(-6);  % 每一个单位面积(km^2)内产生lambda个杂波 
    Tracks = tracks;
    target_Num = length(tracks);
    if target_Num <= 0; return; end

    nz = tracks{1}.Nz * 2; % 得到维度信息
    S = zeros(nz, nz, target_Num); % 新息协方差矩阵
    Z_pre = zeros(nz, target_Num); % 观测值预测
    X_pre = zeros(nz, target_Num); % 状态值预测
    ellipse_Volume = zeros(1, target_Num); % 椭圆门预测
    observed = zeros(1, target_Num);

for tt = 1:target_Num
    X_pre(:, tt) = tracks{tt}.F * tracks{tt}.X; % X(K|K-1)
    P_pre = tracks{tt}.F * tracks{tt}.P * tracks{tt}.F' + ...
            tracks{tt}.Q;
    Z_pre(:, tt) = tracks{tt}.H * X_pre(:, tt); % Z(K|K-1)
    S(:, :, tt) = tracks{tt}.H * P_pre * tracks{tt}.H' + ...
                  tracks{tt}.R; % https://blog.csdn.net/weixin_40106401/article/details/115921604
    ellipse_Volume(tt) = pi * g_sigma * sqrt(det(S(:,:,tt)));     %每个目标计算椭圆跟踪门的面积  
end

valid_measurements = [];
num_valid_measurements = 0;                                                                          %记录有效观测个数
[~, num] = size(measurements); %dim表示观测维度，num表示观测值的个数
measure_confirm_matrix = zeros(max_measures, target_Num + 1);

%% 构建观测确认矩阵 Omega
for tt = 1 : num %目标和杂波
    flag = 0;   %观测值是否有效
    for ttt = 1 : target_Num %目标，也可以认为是存在的航迹。
        delta_measurement = measurements(:, tt) - Z_pre(:, ttt);  %测量值和预测值的误差值。
        delta_measurement_cov = delta_measurement' / S(:, :, ttt) * delta_measurement;
        delta_measurement_cov = delta_measurement' * S(:, :, ttt) * delta_measurement; %通过每个目标测量噪声的协方差矩阵，得到y1(杂波或者目标)与Z_predic(目标)相似程度。                      
        if delta_measurement_cov <= g_sigma                                                    
            flag = 1;
            measure_confirm_matrix(num_valid_measurements + 1, 1) = 1; %确认矩阵每一行第一个元素必定为1。
            measure_confirm_matrix(num_valid_measurements + 1, ttt+1) = 1; %如果在门限范围内设置为1
            observed(ttt) = 1;
            observed_track{ttt}{tt} = measurements(:, tt);
        else

        end
    end  %% 也就是这两层循环会产生确认矩阵的爆炸，因为在下一帧循环的时候，该目标c即为当前帧得到的所有航迹
    if flag == 1   %该检测目标不论是和哪个目标观测上了，都记录下来。
        valid_measurements = [valid_measurements measurements(:, tt)];                                           %把落入跟踪门中的所有回波放入y中
        num_valid_measurements = num_valid_measurements + 1;                                                            %记录有效观测个数
    else
        empty_measurements = [empty_measurements measurements(:, tt)]; % 记录空数据 用于其他航迹初始化
    end
end

% 如果无有效观测直接返回
if num_valid_measurements == 0
    return;
end

% 取有效观测
measure_confirm_matrix = measure_confirm_matrix(1 : num_valid_measurements, 1 : target_Num + 1);

% 产生互联矩阵
[interconnect_matrix, event_num] = GetEvents(measure_confirm_matrix, max_events);

% 计算后验概率
% 计算每个事件个数的概率
Pr = zeros(1, event_num); %num为事件个数
for ii = 1:event_num
    num_false_measurements = num_valid_measurements;
    N = 1;  %求解公式参数之一
    for jj = 1:num_valid_measurements
        mea_indicator = sum(interconnect_matrix(jj, 2:end, ii)); 
        if mea_indicator % 如果存在目标
        	num_false_measurements = num_false_measurements - 1;
            target_index = find(interconnect_matrix(jj, 2:end, ii) == 1);
            b = (valid_measurements(:, jj) - Z_pre(:, target_index))' * ...
                inv(S(:, :, target_index)) * ...
                (valid_measurements(:, jj) - Z_pre(:, target_index));
            N = N / sqrt(det(2 * pi * S(:,:,target_index))) * exp(-1/2 * b);  %计算正态分布函数 
            % 参考文献的3-18公式
        end
    end
    if Pd == 1
        a = 1;
    else
        a = 1;
        for jj = 1: target_Num
            target_indicator = sum(interconnect_matrix(:, jj+1, ii));
            a = a * Pd^target_indicator * (1 - Pd)^(1 - target_indicator);
        end
    end
    V = sum(ellipse_Volume);
    a1 = 1;
    for jj = 1:num_false_measurements
        a1 = a1 * jj;
    end
    Pr(ii) = N * a * a1 / (V^num_false_measurements);
end
Pr = Pr / sum(Pr);

% 计算关联概率
U = zeros(num_valid_measurements + 1, target_Num);
for ii = 1:target_Num
    for jj = 1:num_valid_measurements
        for kk = 1:event_num
            U(jj, ii) = U(jj, ii) + Pr(kk) * interconnect_matrix(jj, ii + 1, kk);
        end
    end
end
% 杂波概率
% beta_0_t
U(num_valid_measurements + 1, :) = 1 - sum(U(1:num_valid_measurements, :));

% Kalman Filter
K = [];
for ii = 1:target_Num
    P_pre = tracks{ii}.F * tracks{ii}.P * tracks{ii}.F' + tracks{ii}.Q; 
    K(:, :, ii) = P_pre * tracks{ii}.H' * inv(S(:, :, ii));
    tracks{ii}.P = P_pre - (1 - U(num_valid_measurements + 1, ii)) * K(:, :, ii) * ...
        S(:, :, ii) * K(:, :, ii)'; 
end
% 协方差矩阵更新

for ii = 1:target_Num
    a = 0; b = 0; x_est = 0;
    for jj = 1:num_valid_measurements
        x_est = x_est + U(jj, ii) * (X_pre(:, ii) + K(:, :, ii)* ...
                (valid_measurements(:, jj) - Z_pre(:, ii)));
    end
    x_est = U(jj+1, ii) * X_pre(:, ii) + x_est; % 杂波预测
    x_est_out(:, ii) = x_est;
    for jj = 1:num_valid_measurements + 1
        if jj == num_valid_measurements + 1
            a = X_pre(:, ii);
        else
            a = X_pre(:, ii) + K(:,:,ii) * ...
                (valid_measurements(:, jj) - Z_pre(:, ii)); % V 新息计算
        end
        b = b + U(jj, ii) * (a * a' - x_est * x_est');
    end % x_est = x_pre + K(sum(p_ * v_))
    % b = sum((x_pre + KV)^2 - (x_pre + K(sum(p_ * v_))^2)
    tracks{ii}.P = tracks{ii}.P + b;
    tracks{ii}.X = x_est;
end

Tracks = tracks;
end