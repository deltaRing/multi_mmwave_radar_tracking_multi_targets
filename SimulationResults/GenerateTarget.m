% 利用仿真雷达生成目标
% 输入1：仿真雷达 
% 输入2：仿真目标
% 输入3：当前时间
% 输入4：开启假目标
% 输入5：假目标平均数目
% 输入6：假目标平均偏差值
% 输入7：假目标平均速度值
% 输出1：生成的假目标
% 输出2：假目标速度
function [GeneratedTargets, GeneratedTargetsVelo] = GenerateTarget(Radars, Targets, time, ...
    fakeTargetEnable, fakeTargetAveageNum, fakeTargetSigma, fakeTargetVeloSigma)
if nargin == 3
    fakeTargetEnable = 0;
    fakeTargetAveageNum = 3;
    fakeTargetSigma = 2.5;
    fakeTargetVeloSigma = 1.0;
end

GeneratedTargets     = {};
GeneratedTargetsVelo = {};
    
    for rr = 1:length(Radars)
        radar = Radars{rr};
        radarInitLoc     = [radar.x radar.y];
        radarVelo        = [radar.vx radar.vy];
        radarLoc         = radarInitLoc + radarVelo * time;
        radarAngle       = -radar.yaw;
        rotMatrix        = rotz(radarAngle / pi * 180); 
        detectTarget     = {};     % 定位目标
        detectTargetVelo = {}; % 定位速度
        for tt = 1:length(Targets)
            target        = Targets{tt};
            targetInitLoc = [target.x target.y];
            targetVelo    = [target.vx(time) target.vy(time)];
            targetLoc     = targetInitLoc + targetVelo * time + randn(size(targetVelo)) * target.Q;
            deltaLoc      = targetLoc - radarLoc;
            deltaLoc      = rotMatrix(1:2, 1:2) * deltaLoc';
            deltaAngle    = atan2(deltaLoc(2), deltaLoc(1));
            deltaVelo     = norm(targetVelo) * cos(atan2(targetVelo(2), targetVelo(1)) + radarAngle);
            if abs(deltaAngle) > radar.sYaw / 2
                continue; % 无法探测
            end
            if norm(deltaLoc) > radar.sRange
                continue; % 无法探测
            end
            if norm(deltaVelo) < 1
                continue; % 无法探测
            end

            detectTarget{end + 1} = ...
                deltaLoc + randn(size(deltaLoc)) * radar.MeasureError;
            detectTargetVelo{end + 1} = ...
                deltaVelo + randn([1, 1]) * radar.MeasureErrorVelo;
            % detectTargetVelo{end + 1} = ...
            %     targetVelo;
            if fakeTargetEnable
                for ii = 1:randi(fakeTargetAveageNum)
                    detectTarget{end + 1} = ...
                        deltaLoc + randn(size(deltaLoc)) * fakeTargetSigma;
                    detectTargetVelo{end + 1} = ...
                        deltaVelo + randn([1, 1]) * radar.MeasureErrorVelo + ...
                        randn([1, 1]) * fakeTargetVeloSigma;
                end
            end
        end
        GeneratedTargets{rr} = detectTarget;
        GeneratedTargetsVelo{rr} = detectTargetVelo;
    end


end

% 正角度是逆时针旋转
