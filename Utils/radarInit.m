%% 雷达数目
Net_RadarNum = 3;
% 雷达结构体
GHz  = 1e9;   % GHz
MHz  = 1e6;   % MHz
us   = 1e-6;  % 微秒
c    = 3.0e8; % 光速
Ksps = 1e6;
N    = 256;         %距离向FFT点数
M    = 128;         %多普勒向FFT点数
Q    = 512;       %角度FFT

for radar = 1:Net_RadarNum
%% 雷达参数（使用mmWave Studio默认参数）
    Radar(radar).Num = Net_RadarNum;
    Radar(radar).fnumber = 256; % MUST BE UNITED
    Radar(radar).fname = ''; % 处理的文件名字
    Radar(radar).RadarParam.B = 4000*1e6;       %调频带宽
    Radar(radar).RadarParam.K = 58*1e12;  %调频斜率
    Radar(radar).RadarParam.T = Radar(radar).RadarParam.B/Radar(radar).RadarParam.K;         %采样时间
    Radar(radar).RadarParam.Tc = 65e-6;     %chirp总周期
    Radar(radar).RadarParam.fs = 10*1e6;       %采样率
    Radar(radar).RadarParam.f0 = 77e9;       %初始频率
    Radar(radar).RadarParam.lambda = c/Radar(radar).RadarParam.f0;   %雷达信号波长
    Radar(radar).RadarParam.d = Radar(radar).RadarParam.lambda/2;    %天线阵列间距
    Radar(radar).RadarParam.n_samples = 256; %采样点数/脉冲
    Radar(radar).RadarParam.n_chirps=64;   %每帧脉冲数
    Radar(radar).RadarParam.PRI = 4e-3;
    Radar(radar).FFTParam.n_RX=4;        %RX天线通道数
    Radar(radar).FFTParam.n_TX=3;        %TX天线通道数
    Radar(radar).ProcessParam.range_axis = linspace(0, ...
                                Radar(radar).RadarParam.fs * ...
                                c / 2 / Radar(radar).RadarParam.K, N);
    Radar(radar).ProcessParam.velo_axis = linspace(-Radar(radar).RadarParam.lambda ...
                                / 4 / Radar(radar).RadarParam.Tc, ...
                                Radar(radar).RadarParam.lambda / 4 / Radar(radar).RadarParam.Tc, M);
    Radar(radar).ProcessParam.an_axis_az = linspace(-asin(Radar(radar).RadarParam.lambda/2/Radar(radar).RadarParam.d), ...
                                asin(Radar(radar).RadarParam.lambda/2/Radar(radar).RadarParam.d), Q);
    Radar(radar).ProcessParam.an_axis_el = linspace(-asin(Radar(radar).RadarParam.lambda/2/Radar(radar).RadarParam.d), ...
                                asin(Radar(radar).RadarParam.lambda/2/Radar(radar).RadarParam.d), Q);
    Radar(radar).Geometry.Location = [0 0 0]; % X Y Z
    Radar(radar).Geometry.Angel    = [0];     % Azimuth only
end