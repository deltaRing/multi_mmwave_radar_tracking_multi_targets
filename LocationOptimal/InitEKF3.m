% 初始化EKF状态
% 输入1：X X坐标
% 输入2：Y Y坐标
% 输入3：Z Z坐标
% 输入4：Xn 原来的状态 
% 输入5：Pn 原来的协方差
% 可选输入1：EKF2 使用2维 还是 3维的EKF
% 输出1：Tracks
% 输出2：ID
function [Tracks, ID] = InitEKF3(Tracks, Posi, Velo, EKF2, ID)
    TarNum = size(Velo, 1);
    for tt = 1:TarNum
        if ~EKF2
            Tracks{end + 1}.Xn = [Posi{tt}(1) Velo(tt, 1) ...
                            Posi{tt}(2) Velo(tt, 2) ...
                            Posi{tt}(3) Velo(tt, 3)];
            Tracks{end + 1}.Pn = eye(6);
        else
            Tracks{end + 1}.Xn = [Posi{tt}(1) Velo(tt, 1) ...
                            Posi{tt}(2) Velo(tt, 2)];
            Tracks{end + 1}.Pn = eye(4);
        end
        Tracks{end + 1}.ID = ID;
        ID = ID + 1;
    end    
end