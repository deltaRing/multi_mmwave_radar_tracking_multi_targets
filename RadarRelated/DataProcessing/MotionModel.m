% 运动模型函数
% 假设无人机在单位时间内为匀速运动
% 输入1：无人机于该时刻的状态 status_k [N x 6] 其中6：[x vx y vy z vz]
% 输入2：单位时间 frame_time 
% 输出1：无人机于下一时刻的状态 status_k_1 [N x 6]
function [status_k_1] = MotionModel(status_k, frame_time)
    status_k_1 = [];
    droneNum = size(status_k, 1); % 无人机数目
    for nn = 1:droneNum
        xx = status_k(nn, 1);
        vx = status_k(nn, 2);
        yy = status_k(nn, 3);
        vy = status_k(nn, 4);
        zz = status_k(nn, 5);
        vz = status_k(nn, 6);
        
        xx = xx + vx * frame_time;
        yy = yy + vy * frame_time;
        zz = zz + vz * frame_time;
        status_k_1(nn, :) = [xx vx yy vy zz vz];
    end
end

