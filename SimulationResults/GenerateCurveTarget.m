% 生成曲线运动目标
% 输入1：仿真雷达 
% 输入2：仿真目标
% 输入3：当前时间
% 输出1：生成的假目标
% 输出2：假目标速度
function [GeneratedTargets, GeneratedTargetsVelo] = GenerateCurveTarget(Targets, Radars, time)

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
            targetVelo    = [target.vx target.vy];
            targetLoc     = targetInitLoc + targetVelo * time + randn(size(targetVelo)) * target.Q;
            deltaLoc      = targetLoc - radarLoc;
            deltaLoc      = rotMatrix(1:2, 1:2) * deltaLoc';
            deltaAngle    = atan2(deltaLoc(2), deltaLoc(1));
            deltaVelo     = norm(targetVelo) * cos(atan2(target.vy, target.vx) + radarAngle);
            if abs(deltaAngle) > radar.sYaw / 2
                continue; % 无法探测
            end
            if norm(deltaLoc) > radar.sRange
                continue; % 无法探测
            end
            detectTarget{end + 1} = ...
                deltaLoc + randn(size(deltaLoc)) * radar.MeasureError;
            detectTargetVelo{end + 1} = ...
                deltaVelo + randn([1, 1]) * radar.MeasureErrorVelo;
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
