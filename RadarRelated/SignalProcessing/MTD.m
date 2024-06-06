% 动目标检测
% 毫米波雷达 通过 发射多脉冲实现 获取 距离-多普勒 谱图
% 输入1：距离像结果 RangeProfile (Ant_Num x RangeFFT x Chirps)
% 输入2：速度向FFT点数 FFTNum
% 输出1：MTD结果（距离-多普勒谱图）
function MTD_result = MTD(RangeProfile, FFTNum)
    if length(size(RangeProfile)) == 3
        Ant_Num = size(RangeProfile, 1);
        for aa = 1:Ant_Num
            MTD_result(aa, :, :) = fftshift(fft(RangeProfile(aa,:,:), FFTNum, 2), 2);
        end
    elseif length(size(RangeProfile)) == 2
        MTD_result = fftshift(fft(RangeProfile(:,:), FFTNum, 2), 2);
    else
        MTD_result = [];
        error('MTD: Higher Dimension Detected, Not support')
    end
end