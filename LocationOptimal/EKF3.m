% 扩展卡尔曼滤波3 
% 输入1：雷达观测到的俯仰角 E
% 输入2：雷达观测到的方位角 A
% 输入3：雷达观测到的距离   R
% 输入4：过去航迹的观测值 Z (1 x 6) X Vx Y Vy Z Vz
% 输入5：过去航迹的协方差 P 
%  (过去的航迹观测大小和雷达新观测到的大小必须一致)
%  (如果没有观测到新的数据那就不要加进去（可以为NULL）)
% 可选输入1：EKF2 二维还是三维
% 可选输入2：T    单帧的时间
% 输出1：新的航迹观测值 Zn
% 输出2：新的航迹协方差 Pn
function [Zn, Pn] = EKF3(Posi, Velo, Zn, Pn, EKF2, t)
% 设置默认值
if nargin == 4, EKF2 = 0; t = 0.1; end
% 初始化噪声
Q_ = 10;     % 运动方程中的噪声
R_ = 1;      % 观测方程中的噪声
% 运动方程
% Xt = Xt-1 + Vxt-1 * t 
% Yt = Yt-1 + Vyt-1 * t
% Zt = Zt-1 + Vzt-1 * t
% Vx = Vxt-1 + n(t)
% Vy = Vyt-1 + n(t)
% Vz = Vzt-1 + n(t)
% 如何将运动方程转变为观测方程
% azi   = arctan(y/x)
% ele   = arctan(z/√(x^2 + y^2))
% range = √(x^2 + y^2 + z^2) 
% 运动方程求导
% [1 t 0 0 0 0;  X
%  0 1 0 0 0 0;  Vx
%  0 0 1 t 0 0;  Y
%  0 0 0 1 0 0;  Vy
%  0 0 0 0 1 t;  Z
%  0 0 0 0 0 1]; Vz
% 观测方程求导
% [ -y / (x^2 + y^2) 0 x / (x^2 + y^2) 0 0 0;
%    x * z / (range^2 * range_xy) 0 y * z / (range^2 * range_xy) 0 -range_xy / range^2 0;
%    x / range 0 y / range 0 z / range 0]
% 其中
% range_xy = √(x^2 + y^2)
% range = √(x^2 + y^2 + z^2)
% 2024 更新
% 观测值 [xx vx yy vy zz vz] % 位置与速度

for ii = 1:size(Zn, 1)
    if EKF2
        if ~isempty(Posi{ii})
            % 用不到Elevation
            r_x_ = Posi(1);
            r_y_ = Posi(2);
            v_x_ = Velo(1); % 这个速度估计容易出事
            v_y_ = Velo(2); % 这个速度容易出事
            r_x  = Zn(ii, 1) + Zn(ii, 2) * t;
            r_y  = Zn(ii, 3) + Zn(ii, 4) * t;
            v_x  = Zn(ii, 2);
            v_y  = Zn(ii, 4);
            Zf   = [r_x, v_x, ...
                     r_y, v_y];      % 预测的状态
            Z_   = [r_x_ v_x_ ...
                     r_y_ v_y_];     % 真实的观测值
        else
            continue;
        end
        
        F = [1 t 0 0; 
            0 1 0 0;
            0 0 1 t;
            0 0 0 1]; % 6 x 6
        H = [1 t 0 0;
            0 t 0 0;
            0 0 1 t;
            0 0 0 t;
            ]; % 6 x 6
    else
        % 如果有数据
        % 构造真实状态
        if ~isempty(Posi{ii})
            r_x_  = Posi(1);
            r_y_  = Posi(2);
            r_z_  = Posi(3);
            v_x_  = Velo(1);
            v_y_  = Velo(2);
            v_z_  = Velo(3);
            r_x   = Zn(ii, 1) + Zn(ii, 2) * t;
            r_y   = Zn(ii, 3) + Zn(ii, 4) * t;
            r_z   = Zn(ii, 5) + Zn(ii, 6) * t; % 预测值
            v_x   = Zn(ii, 2);
            v_y   = Zn(ii, 4);
            v_z   = Zn(ii, 6);
            Zf    = [r_x, v_x, ...
                     r_y, v_y, ...
                     r_z, v_z];      % 预测的状态
            Z_    = [r_x_ v_x_ ...
                     r_y_ v_y_ ...
                     r_z_ v_z_];     % 真实的观测值
        else
            % 不进行更新
            continue;
        end

        F = [1 t 0 0 0 0; 
            0 1 0 0 0 0;
            0 0 1 t 0 0;
            0 0 0 1 0 0;
            0 0 0 0 1 t;
            0 0 0 0 0 1]; % 6 x 6
        H = [1 t 0 0 0 0;
            0 t 0 0 0 0;
            0 0 1 t 0 0;
            0 0 0 t 0 0;
            0 0 0 0 1 t;
            0 0 0 0 0 1
            ]; % 6 x 6
    end
    
    P_ = F * Pn{ii} * F' + Q_ * eye(6); % 6 x 6
    K = P_ * H' / (H * P_ * H' + R_ * eye(3));
    X = X_ + (K * (Z_ - Zf)')';
    Pn{ii} = P_ - K * H * P_;          % 更新协方差
    Zn(ii, :) = X;                     % 更新状态
end
end