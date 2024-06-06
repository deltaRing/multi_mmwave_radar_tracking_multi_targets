clc;clear all;close all;

%%参数定义
T = 20;                                                            %采样次数
dt = 1;                                                            %T为采样间隔
MC_number = 1;                                                     %Monte Carlo仿真次数
target_num = 2;                                                    %目标个数
target_initposition = [1500 500; 
                       300 400; 
                       500 1500; 
                       400 300];                              %目标的起始位置和速度(x,vx,y,vy)         
Pd = 1;                                                            %检测概率
Pg = 0.99;                                                         %正确量测落入跟踪门内得概率
g_sigma = 9.21;                                                    %门限,经验值，与R和Q有关，必须合理设置
lambda = 2;
gamma = lambda*10^(-6);                                            %每一个单位面积(km^2)内产生lambda个杂波 

P = zeros(4, 4, target_num, T);                                       %协方差矩阵
P_init = diag([10000 0 10000 0]);        %单个初始协方差矩阵 ，x,vx,y,vy四个随机量协方差矩阵。真实值和估计值的过程噪声协方差矩阵。
for i = 1 : target_num
    P(:, :, i, 1) = P_init;
end
F = [1 dt 0 0;                                                           
     0 1 0 0;
     0 0 1 dt;
     0 0 0 1];                                             %状态转移矩阵
Q = diag([4 0 4 0]);                                       %系统过程噪声协方差
H = [1 0 0 0;
     0 0 1 0];                                             %观测矩阵
R = diag([100 100]);                                       %观测协方差矩阵

X_est_out = zeros(4, target_num, T);                                   %存储目标的各时刻的滤波值
X_est_out_MC = zeros(4, target_num, T, MC_number);                    %MC_number次Montle Carlo仿真所得全部结果存储矩阵
Z_out = zeros(2, target_num, T);                           %观测值x,y
X_out = zeros(4, target_num, T);                                   %实际状态值x,vx,y,vy   

%%生成真实状态的模拟数据
X_out(:, :, 1) = target_initposition;                                          %实际位置矩阵初始化 
for i = 1 : target_num
    for t = 2 : T                                                                     %实际位置 
        X_out(:, i, t) = (F * X_out(:, i, t-1)) + (sqrt(Q) * (randn(4, 1)));        
    end
end
%%画图
X_target_1 = zeros(1, T);         
Y_target_1 = zeros(1, T);         
for t = 1 : T
    X_target_1(t) = X_out(1, 1, t);
    Y_target_1(t) = X_out(3, 1, t);
end
figure;
plot(X_target_1, Y_target_1, 'b-');
hold on;
X_target_2 = zeros(1, T);         
Y_target_2 = zeros(1, T);         
for t = 1:T
    X_target_2(t) = X_out(1, 2, t);
    Y_target_2(t) = X_out(3, 2, t);
end
plot(X_target_2, Y_target_2, 'r-');
xlabel('x(m)'),ylabel('y(m)');
legend('目标a的实际位置','目标b的实际位置');
grid;

%%开始仿真
for m = 1 : MC_number    
    %%1.产生观测值
    for t = 1 : T %采样次数
        for i = 1 : target_num %目标个数，  %各传感器观测的位置
            Z_out(:, i, t) = H * X_out(:, i, t) + sqrt(R) * randn(2, 1);
        end
    end
    %%2.产生杂波,并确定有效观测 
    S = zeros(2, 2, target_num);
    Z_prediction = zeros(2, target_num);                                                               %存储两个目标的观测预测值,即只包括x,y坐标
    X_prediction = zeros(4, target_num);                                                               %存储两个目标的状态预测值,即包括x,y坐标和x,y方向速度
    ellipse_Volume = zeros(1, target_num);
    NOISE_target_1 = [];                                                                    %存储目标1的杂波
    NOISE_target_2 = [];                                                                    %存储目标2的杂波

    for t = 1 : T
        %产生杂波数据
        noise = [];               %单次杂波矩阵
        NOISE = [];               %所有target的杂波矩阵
        for i = 1 : target_num      %航迹
            if t ~= 1
                X_prediction(:, i) = F * X_est_out(:, i, t-1);                                       %用前一时刻的滤波值来预测当前的值(kalman滤波的第一个表达式)
                P_prediction = F * P(:, :, i, t-1) * F' + Q;
            else
                X_prediction(:, i) = target_initposition(:, i);                                       %第一次采样我们用真实位置当预测值 
                P_prediction = P(:, :, i, 1);
            end
            Z_prediction(:, i) = H * X_prediction(:, i);                                                 %提取预测值的x,y坐标，舍弃x,y速度
            S(:, :, i) = H * P_prediction * H' + R;       
            ellipse_Volume(i) = pi * g_sigma * sqrt(det(S(:,:,i)));                              %每个目标计算椭圆跟踪门的面积   
            number_returns = floor(ellipse_Volume(i) * gamma + 1);                      %椭圆跟踪门内的错误回波数
            side = sqrt((ellipse_Volume(i) * gamma + 1) / gamma) / 2;                       %将椭圆跟踪门等效为正方形，并求出正方形边长的二分之一
            noise_x = X_prediction(1, i) + side - 2 * rand(1, number_returns) * side;         %在预测值周围产生多余回波。
            noise_y = X_prediction(3, i) + side - 2 * rand(1, number_returns) * side;         %注意：当某一次number_returns小于等于0时会出错，再运行一次即可。
            noise = [noise_x; noise_y];
            NOISE = [NOISE noise];
            if i == 1
                NOISE_target_1 = [NOISE_target_1 noise];
            else
                NOISE_target_2 = [NOISE_target_2 noise];
            end
        end
        current_measurements = [NOISE Z_out(:, 1, t) Z_out(:, 2, t)];  %记录此刻的观测值，即杂波，目标1，目标2的观测值。

        %%现在才真正开始jpda的计算，前面只是产生实验所需要的数据。 
        %%产生观测确认矩阵measure_confirm_matrix 
        valid_measurements = [];
        num_valid_measurements = 0;                                                                          %记录有效观测个数
        [dim, num] = size(current_measurements); %dim表示观测维度，num表示观测值的个数
        measure_confirm_matrix = zeros(1000, target_num + 1);
        for j = 1 : num %目标和杂波
            flag = 0;   %观测值是否有效
            for i = 1 : target_num %目标，也可以认为是存在的航迹。
                delta_measurement = current_measurements(:, j) - Z_prediction(:, i);  %测量值和预测值的误差值。
                delta_measurement_cov = delta_measurement' / S(:, :, i) * delta_measurement; %通过每个目标测量噪声的协方差矩阵，得到y1(杂波或者目标)与Z_predic(目标)相似程度。                      
                if delta_measurement_cov <= g_sigma                                                    
                    flag = 1;
                    measure_confirm_matrix(num_valid_measurements + 1, 1) = 1; %确认矩阵每一行第一个元素必定为1。
                    measure_confirm_matrix(num_valid_measurements + 1, i+1) = 1; %如果在门限范围内设置为1
                end
                %是否应该有else，对于不在波门范围内的目标，建立新的航迹。？？？疑问。建立新航迹的方法包括历史轨迹和打分两种，比较复杂，先忽略
            end  %% 也就是这两层循环会产生确认矩阵的爆炸，因为在下一帧循环的时候，该目标c即为当前帧得到的所有航迹
            if flag == 1   %该检测目标不论是和哪个目标观测上了，都记录下来。
                valid_measurements = [valid_measurements current_measurements(:, j)];                                           %把落入跟踪门中的所有回波放入y中
                num_valid_measurements = num_valid_measurements + 1;                                                            %记录有效观测个数
            end
        end
        measure_confirm_matrix = measure_confirm_matrix(1 : num_valid_measurements, 1 : target_num + 1);

        %%4.产生互联矩阵interconnect_matrix,其中num表示可行联合事件个数
        interconnect_matrix = zeros(num_valid_measurements, 3, 10000);
        interconnect_matrix(:, 1, 1:10000) = 1;  %第一列都设置为1，表明每个目标都有可能来自杂波。
        if num_valid_measurements ~= 0                                 %num_valid_measurements=0表示两个目标都没有观测到，就是连两个目标和杂波都没有观测到。
            event_num = 1; %矩阵个数，也就是事件个数
            for i = 1 : num_valid_measurements
                if measure_confirm_matrix(i, 2) == 1 %第一条航迹关联
                    interconnect_matrix(i, 2, event_num) = 1;
                    interconnect_matrix(i, 1, event_num) = 0;
                    event_num = event_num + 1;
                    for j = 1 : num_valid_measurements
                        if (i ~= j) & (measure_confirm_matrix(j, 3) == 1)
                            interconnect_matrix(i, 2, event_num) = 1;
                            interconnect_matrix(i, 1, event_num) = 0;
                            interconnect_matrix(j, 3, event_num) = 1;
                            interconnect_matrix(j, 1, event_num) = 0;
                            event_num = event_num + 1;
                        end
                    end
                end
            end                                   

            for i = 1 : num_valid_measurements
                if measure_confirm_matrix(i, 3) == 1
                    interconnect_matrix(i, 3, event_num) = 1;
                    interconnect_matrix(i, 1, event_num) = 0;
                    event_num = event_num + 1;
                end
            end
        else
            flag = 1;
        end
        interconnect_matrix = interconnect_matrix(:, :, 1:event_num);           %穷举法拆分的结果存在interconnect_matrix中
        
        %%  5.计算后验概率Pr,其中
        %%  num_false_measurements表示假量测,
        %%  mea_indicator表示观测指示器,是否每个量测值都被目标关联
        %%  target_indicator表示目标指示器 ，是否每个目标都被检测到。
        %%  计算每个事件个数的概率
        Pr = zeros(1, event_num); %num为事件个数
        for i = 1 : event_num 
            num_false_measurements = num_valid_measurements;
            N = 1;  %求解公式参数之一
            for j = 1 : num_valid_measurements    %num_valid_measurements为在有效门限内的杂波和目标总数
                mea_indicator = sum(interconnect_matrix(j, 2:3, i));                                      %参考文献中式4-48
                if mea_indicator == 1 %表示两个航迹中，有关联上的目标
                    num_false_measurements = num_false_measurements - 1;
                    if interconnect_matrix(j, 2, i) == 1                                                  %如果观测与目标1关联
                        b = (valid_measurements(:, j) - Z_prediction(:, 1))' * inv(S(:, :, 1)) * (valid_measurements(:, j) - Z_prediction(:, 1));
                        N = N / sqrt(det(2 * pi * S(:,:,1))) * exp(-1/2 * b);                          %计算正态分布函数                         
                    else                                                                   %如果观测与目标2关联
                        b = (valid_measurements(:, j) - Z_prediction(:, 2))' * inv(S(:, :, 2)) * (valid_measurements(:, j) - Z_prediction(:, 2));
                        N = N / sqrt(det(2 * pi *S(:,:,2))) * exp(-1/2 * b);                          %计算正态分布函数                         
                    end                                                                        
                end
            end
            if Pd == 1
                a = 1;
            else
                a = 1;
                for j = 1 : target_num
                    target_indicator = sum(interconnect_matrix(:, j+1, i));                               %参考文献中式4-49
                    a = a * Pd^target_indicator * (1 - Pd)^(1 - target_indicator);                   %计算检测概率
                end
            end                                                                            
            V = ellipse_Volume(1) + ellipse_Volume(2);                                         %表示整个空域的体积
            a1 = 1;
            for j = 1 : num_false_measurements
                a1 = a1*j;
            end
            Pr(i) = N * a * a1 / (V^num_false_measurements);   %mk因为是常量，在归一化过程中会被约掉，因此略去，虚假量测采用均匀分布，也略去
        end
        Pr = Pr / sum(Pr);
        %%6.计算关联概率U
        %%联合关联概率即为最终的每个航迹和目标之间关联的可能性系数。
        U = zeros(num_valid_measurements + 1, target_num);
        for i = 1 : target_num
            for j = 1 : num_valid_measurements
                for k = 1 : event_num
                    U(j, i) = U(j, i) + Pr(k) * interconnect_matrix(j, i+1, k);     %利用后验概率pr计算每个关联矩阵的概率。
                end
            end
        end
        U(num_valid_measurements + 1, :) = 1 - sum(U(1:num_valid_measurements, 1:target_num));                %无量测与目标T互联的关联概率存入U（num_valid_measurements+1,:),归一化
        %%7.Kalman滤波开始
        for i = 1 : target_num    
            if t ~= 1                        
                P_prediction = F * P(:, :, i, t-1) * F' + Q;
            else
                P_prediction = P(:, :, i, 1);
            end                                                                      %更新协方差矩阵
            K(:, :, i) = P_prediction * H' * inv(S(:, :, i));
            P(:, :, i, t) = P_prediction - (1 - U(num_valid_measurements + 1, i)) * K(:, :, i) * S(:, :, i) * K(:, :, i)';
        end
        for i = 1 : target_num
            a = 0;         
            b = 0;
            x_est = 0;                                                                   %随便设置的中间参数
            for j = 1 : num_valid_measurements
                x_est = x_est + U(j, i)*(X_prediction(:, i) + K(:, :, i)*(valid_measurements(:, j) - Z_prediction(:, i)));
            end
            x_est = U(j+1, i) * X_prediction(:, i) + x_est; 
            X_est_out(:, i, t) = x_est;
            for j = 1 : num_valid_measurements + 1
                if j == num_valid_measurements + 1
                    a = X_prediction(:, i);
                else
                    a = X_prediction(:, i) + K(:, :, i)*(valid_measurements(:, j) - Z_prediction(:, i));
                end
                b = b + U(j,i) * (a*a' - x_est*x_est');
            end
            P(:, :, i, t) = P(: ,: ,i ,t) + b; 
            X_est_out_MC(:, i, t, m) = X_est_out(:, i, t);   
        end
    end
end


%%画图                                            %滤波值作平均
%%1.滤波结果
figure;
%目标a,b的观测位置
for i = 1 : target_num
    measure_x = zeros(1, T);
    measure_y = zeros(1, T);
    for j = 1 : T
        measure_x(j) = Z_out(1, i, j);
        measure_y(j) = Z_out(2, i, j);
    end
    if i == 1
       plot(measure_x(:), measure_y(:), 'r:')
    else 
       plot(measure_x(:), measure_y(:), 'b:')
    end
    hold on;
end
%目标a,b的杂波位置
for i = 1 : target_num
    if i == 1
       plot(NOISE_target_1(1, :), NOISE_target_1(2, :), 'r.');
    else
       plot(NOISE_target_2(1, :), NOISE_target_2(2, :), 'b.');
   end
   hold on;
end
%目标a,b的估计位置
for i = 1 : target_num
    estimate_x = zeros(1, T);
    estimate_y = zeros(1, T);
    for j = 1 : T
        estimate_x(j) = X_est_out(1, i, j);
        estimate_y(j) = X_est_out(3, i, j);
    end
    if i == 1
        plot(estimate_x(:), estimate_y(:), 'r-');
    else 
        plot(estimate_x(:), estimate_y(:), 'g-');
    end
    hold on;
end
xlabel('x/m'), ylabel('y/m');
legend('目标a的观测位置', '目标b的观测位置', '目标a的杂波', '目标b的杂波', '目标a的估计位置', '目标b的估计位置');grid;

%%2.误差
figure;
mse = zeros(target_num, T);
for i = 1 : target_num
    for t = 1 : T
        for M = 1 : MC_number                                                              %最小均方误差
            mse_t = (X_est_out_MC(1, i, t, M) - X_out(1, i, t))^2 + (X_est_out_MC(3, i, t, M) - X_out(3, i, t))^2;
            mse(i, t) = mse(i, t) + mse_t;
        end
        mse(i, j) = sqrt(mse(i, j) / MC_number);
    end
    %求最大值
    [max_num, max_index] = max(mse(i, :));
    str = strcat('\itT=', num2str(max_index), ' \itError=', num2str(max_num), '(m^2)');
    text(max_index, 0.8*max_num, str);
    hold on;
    if i == 1
        plot([max_index max_index],[0 max_num],'r');
        hold on;
        plot(1:T, mse(i, :), 'r:') 
        hold on;
    else 
        plot([max_index max_index],[0 max_num],'b');
        hold on;
        plot(1:T, mse(i, :), 'b:') 
        hold on;
    end
end
xlabel('times'),ylabel('测量值与估计值均方差/m^2');
legend('目标a的误差最大值','目标a的误差','目标b的误差最大值','目标b的误差');grid;

%rmse
for i = 1 : target_num    
    figure;
    rmse_est = zeros(1,T);
    rmse_measure = zeros(1,T);
    for t = 1 : T
        rmse_est(t) = sqrt((X_est_out(1, i, t) - X_out(1, i, t))^2 + (X_est_out(3, i, t) - X_out(3, i, t))^2);
        rmse_measure(t) = sqrt((Z_out(1, i, t) - X_out(1, i, t))^2 + (Z_out(2, i, t) - X_out(3, i, t))^2);
    end
    plot(1:T, rmse_est,'k:'); 
    if i == 1
        xlabel('times'), ylabel('RMSE of a');    
    else 
        xlabel('times'), ylabel('RMSE of b');
    end
    grid;
end