% ���ϸ������ݹ��� ��JPDA��
% ����1�����ٺ���
% ����2������ֵ 4 x N / 6 x N
% ��ʽ [x vx y vy z vz]
% ����3��Pd ������
% ����4����ȷ��������������ڵĸ���
% ����5������
% ����6���������
% ����7������¼���Ŀ
% ���1�����µĸ��ٺ������� Tracks
% ���2�������Ƿ�۲⵽������
% ���3��δ�ܽ��й۲⵽������������
function [Tracks, observed, empty_measurements, observed_track] = JPDA(tracks, measurements, ...
                                    Pd, Pg, g_sigma, ...
                                    lambda, max_measures, max_events)
    if nargin == 2
        Pd = 1;                    %������
        Pg = 0.99;                 %��ȷ��������������ڵø���
        g_sigma = 5.0;            %����,����ֵ����R��Q�йأ������������
        lambda = 2;                                          
        max_measures = 10000;       % ��������ֵ
        max_events   = 100000;      % ����¼���Ŀ
    end

    empty_measurements = [];
    observed = [];
    observed_track = [];     % �۲⵽�ĺ���
    gamma = lambda*10^(-6);  % ÿһ����λ���(km^2)�ڲ���lambda���Ӳ� 
    Tracks = tracks;
    target_Num = length(tracks);
    if target_Num <= 0; return; end

    nz = tracks{1}.Nz * 2; % �õ�ά����Ϣ
    S = zeros(nz, nz, target_Num); % ��ϢЭ�������
    Z_pre = zeros(nz, target_Num); % �۲�ֵԤ��
    X_pre = zeros(nz, target_Num); % ״ֵ̬Ԥ��
    ellipse_Volume = zeros(1, target_Num); % ��Բ��Ԥ��
    observed = zeros(1, target_Num);

for tt = 1:target_Num
    X_pre(:, tt) = tracks{tt}.F * tracks{tt}.X; % X(K|K-1)
    P_pre = tracks{tt}.F * tracks{tt}.P * tracks{tt}.F' + ...
            tracks{tt}.Q;
    Z_pre(:, tt) = tracks{tt}.H * X_pre(:, tt); % Z(K|K-1)
    S(:, :, tt) = tracks{tt}.H * P_pre * tracks{tt}.H' + ...
                  tracks{tt}.R; % https://blog.csdn.net/weixin_40106401/article/details/115921604
    ellipse_Volume(tt) = pi * g_sigma * sqrt(det(S(:,:,tt)));     %ÿ��Ŀ�������Բ�����ŵ����  
end

valid_measurements = [];
num_valid_measurements = 0;                                                                          %��¼��Ч�۲����
[~, num] = size(measurements); %dim��ʾ�۲�ά�ȣ�num��ʾ�۲�ֵ�ĸ���
measure_confirm_matrix = zeros(max_measures, target_Num + 1);

%% �����۲�ȷ�Ͼ��� Omega
for tt = 1 : num %Ŀ����Ӳ�
    flag = 0;   %�۲�ֵ�Ƿ���Ч
    for ttt = 1 : target_Num %Ŀ�꣬Ҳ������Ϊ�Ǵ��ڵĺ�����
        delta_measurement = measurements(:, tt) - Z_pre(:, ttt);  %����ֵ��Ԥ��ֵ�����ֵ��
        delta_measurement_cov = delta_measurement' / S(:, :, ttt) * delta_measurement;
        delta_measurement_cov = delta_measurement' * S(:, :, ttt) * delta_measurement; %ͨ��ÿ��Ŀ�����������Э������󣬵õ�y1(�Ӳ�����Ŀ��)��Z_predic(Ŀ��)���Ƴ̶ȡ�                      
        if delta_measurement_cov <= g_sigma                                                    
            flag = 1;
            measure_confirm_matrix(num_valid_measurements + 1, 1) = 1; %ȷ�Ͼ���ÿһ�е�һ��Ԫ�رض�Ϊ1��
            measure_confirm_matrix(num_valid_measurements + 1, ttt+1) = 1; %��������޷�Χ������Ϊ1
            observed(ttt) = 1;
            observed_track{ttt}{tt} = measurements(:, tt);
        else

        end
    end  %% Ҳ����������ѭ�������ȷ�Ͼ���ı�ը����Ϊ����һ֡ѭ����ʱ�򣬸�Ŀ��c��Ϊ��ǰ֡�õ������к���
    if flag == 1   %�ü��Ŀ�겻���Ǻ��ĸ�Ŀ��۲����ˣ�����¼������
        valid_measurements = [valid_measurements measurements(:, tt)];                                           %������������е����лز�����y��
        num_valid_measurements = num_valid_measurements + 1;                                                            %��¼��Ч�۲����
    else
        empty_measurements = [empty_measurements measurements(:, tt)]; % ��¼������ ��������������ʼ��
    end
end

% �������Ч�۲�ֱ�ӷ���
if num_valid_measurements == 0
    return;
end

% ȡ��Ч�۲�
measure_confirm_matrix = measure_confirm_matrix(1 : num_valid_measurements, 1 : target_Num + 1);

% ������������
[interconnect_matrix, event_num] = GetEvents(measure_confirm_matrix, max_events);

% ����������
% ����ÿ���¼������ĸ���
Pr = zeros(1, event_num); %numΪ�¼�����
for ii = 1:event_num
    num_false_measurements = num_valid_measurements;
    N = 1;  %��⹫ʽ����֮һ
    for jj = 1:num_valid_measurements
        mea_indicator = sum(interconnect_matrix(jj, 2:end, ii)); 
        if mea_indicator % �������Ŀ��
        	num_false_measurements = num_false_measurements - 1;
            target_index = find(interconnect_matrix(jj, 2:end, ii) == 1);
            b = (valid_measurements(:, jj) - Z_pre(:, target_index))' * ...
                inv(S(:, :, target_index)) * ...
                (valid_measurements(:, jj) - Z_pre(:, target_index));
            N = N / sqrt(det(2 * pi * S(:,:,target_index))) * exp(-1/2 * b);  %������̬�ֲ����� 
            % �ο����׵�3-18��ʽ
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

% �����������
U = zeros(num_valid_measurements + 1, target_Num);
for ii = 1:target_Num
    for jj = 1:num_valid_measurements
        for kk = 1:event_num
            U(jj, ii) = U(jj, ii) + Pr(kk) * interconnect_matrix(jj, ii + 1, kk);
        end
    end
end
% �Ӳ�����
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
% Э����������

for ii = 1:target_Num
    a = 0; b = 0; x_est = 0;
    for jj = 1:num_valid_measurements
        x_est = x_est + U(jj, ii) * (X_pre(:, ii) + K(:, :, ii)* ...
                (valid_measurements(:, jj) - Z_pre(:, ii)));
    end
    x_est = U(jj+1, ii) * X_pre(:, ii) + x_est; % �Ӳ�Ԥ��
    x_est_out(:, ii) = x_est;
    for jj = 1:num_valid_measurements + 1
        if jj == num_valid_measurements + 1
            a = X_pre(:, ii);
        else
            a = X_pre(:, ii) + K(:,:,ii) * ...
                (valid_measurements(:, jj) - Z_pre(:, ii)); % V ��Ϣ����
        end
        b = b + U(jj, ii) * (a * a' - x_est * x_est');
    end % x_est = x_pre + K(sum(p_ * v_))
    % b = sum((x_pre + KV)^2 - (x_pre + K(sum(p_ * v_))^2)
    tracks{ii}.P = tracks{ii}.P + b;
    tracks{ii}.X = x_est;
end

Tracks = tracks;
end