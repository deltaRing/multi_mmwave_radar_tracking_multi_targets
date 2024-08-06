% TrackInit 跟踪航迹初始化
% 输入1：位置信息 Posi
% 输入2：速度信息 Velo
% 输入3：时间
% 输入4：起始帧标号 FrameIndex
% 输入5：航迹ID
% 输出1：航迹结构体 Tracks
function Tracks = trackInit(Posi, Velo, t, FrameIndex, ID, lambda)
    if nargin == 5
        lambda = 1;
    end
    Tracks.t        = t;
    Tracks.InitPosi = Posi;
    Tracks.InitVelo = Velo;
    Tracks.FrameIndex = FrameIndex;
    Tracks.ObservedFrame = 1; % 观测次数
    Tracks.LossFrame = 0;     % 丢失次数
    Tracks.LossFrameMax = 10; % 丢失上限 Single: 10
    Tracks.ConfirmMax   = 5;  % 至少需要观测多少次数 Single:5
    Tracks.Type = 0;          % 0：临时航迹 1：确认航迹 2：删除航迹
    Tracks.ID   = ID;
    Tracks.Lambda = lambda;
    if length(Posi) == 2
        Tracks.Nz = 2; % 维度
        Tracks.X = [Posi(1) Velo(1) Posi(2) Velo(2)]';
        Tracks.Q = eye(4);                  % 系统过程噪声协方差
        Tracks.H = [1 0 0 0;
                    0 1 0 0;
                    0 0 1 0;
                    0 0 0 1];               % 观测矩阵
        Tracks.F = [1 t 0 0;                % 状态转移矩阵
                    0 1 0 0;
                    0 0 1 t;
                    0 0 0 1];
        Tracks.R = diag([1 1 1 1]);         % 观测协方差矩阵
        Tracks.P = diag([1 1 1 1]); % 协方差矩阵
    elseif length(Posi) == 3
        Tracks.Nz = 3; % 维度
        Tracks.X = [Posi(1) Velo(1) Posi(2) Velo(2) Posi(3) Velo(3)]';
        Tracks.Q = eye(6);   % 系统过程噪声协方差
        Tracks.H = [1 0 0 0 0 0;
                    0 1 0 0 0 0;
                    0 0 1 0 0 0;
                    0 0 0 1 0 0;
                    0 0 0 0 1 0;
                    0 0 0 0 0 1];                   % 观测矩阵
        Tracks.F = [1 t 0 0 0 0;                    % 状态转移矩阵
                    0 1 0 0 0 0;
                    0 0 1 t 0 0;
                    0 0 0 1 0 0;
                    0 0 0 0 1 t;
                    0 0 0 0 0 1];
        Tracks.R = diag([1 100 1 100 1 100]);             % 观测协方差矩阵
        Tracks.P = diag([1 100 1 100 1 100]); % 协方差矩阵
    else
        Tracks = [];
        error('Track Initial: Not Support High Dimension');
    end
end

