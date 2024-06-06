% 脉冲压缩 -> 得到
% 毫米波雷达脉冲压缩需要在快时间维度上进行FFT
% 输入1：毫米波雷达回波 (AntennaNum x 采样数 x 脉冲数目)
% 输入2：FFT数 FFTNum
% 输出1：距离像
function RangeProfile = PulseCompression(RadarEcho, FFTNum)
    if length(size(RadarEcho)) == 3
        num_Ant = size(RadarEcho, 1);
        for aa = 1:num_Ant
            RangeProfile(aa, :, :) = fft(squeeze(RadarEcho(aa, :, :)), FFTNum);
        end
    elseif length(size(RadarEcho)) == 2
        RangeProfile = fft(RadarEcho, FFTNum);
    else
        RangeProfile = [];
        error('Pulse Compression: Not Support other Dimension')
    end
end