main
save("main_9.mat", "TracksRecord")
main_KF
save("main_9_KF.mat", "TracksRecord")
main_PDA
save("main_9_PDA.mat", "TracksRecord")
main_JPDA
save("main_9_JPDA.mat", "TracksRecord")

% R = 8;       % �ο���Ԫ����Ϊ 5  ������5 6 2.75 0.5 lambda
% P = 6;        % ������Ԫ����Ϊ 3 // 5 6
% Pfa = 2.5e-2; % �龯�� 

% 1 4
%  R = 10;       % �ο���Ԫ����Ϊ 5 2.75 0.25 lambda
%  P = 6;        % ������Ԫ����Ϊ 3
%  Pfa = 2.5e-2; % �龯�� 

%  2 3 7 8
%  R = 9;       % �ο���Ԫ����Ϊ 5 2.75 0.25 lambda
%  P = 6;        % ������Ԫ����Ϊ 3
%  Pfa = 2.5e-2; % �龯�� 