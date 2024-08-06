% ��Ԫƽ�����龯�ʼ���� 
% ���룺
% map������-��������ͼ
% R: �ο���Ԫ
% P��������Ԫ
% Pfa���龯��
% �����
% detect_map���������ͼ
% detect_result�������
% detect_threshold�� �����ֵ
function [detect_map, detect_result, detect_threshold] = CA_CFAR(map, R, P, Pfa)
    if nargin == 1
        R = 8;       % �ο���Ԫ����Ϊ 5
        P = 6;        % ������Ԫ����Ϊ 3
        Pfa = 2.5e-2; % �龯�� 
        % R = 12;       % �ο���Ԫ����Ϊ 5
        % P = 8;       % ������Ԫ����Ϊ 3
        % Pfa = 1e-2; % �龯�� 
    end

    size_x = size(map, 1);
    size_y = size(map, 2);
    L_slipper       = R + P; % �ܹ���Ҫ�ĵ�Ԫ
    L_slipper_P     = P;     % ������Ԫ
    detect_map      = zeros(size_x, size_y);    % �����ͼ
    detect_result   = [];    % �����
    detect_threshold = [];
    compute_result = [];
    
    for tt = 1:size_y
        for rr = L_slipper:size_x
            test_unit_rr = rr;    % ��¼���Ե�Ԫ
            end_unit_rr = rr + L_slipper; % ��¼�����Ĳ��Ե�Ԫ
            if end_unit_rr > size_x, continue, end
            test_unit_ps = test_unit_rr - L_slipper_P;    % ��¼��ʼ��Ԫ�Լ�������Ԫ
            test_unit_pe = test_unit_rr + L_slipper_P;    % ��¼������Ԫ
            test_units = [map(test_unit_ps:rr, tt); ...
                map(test_unit_pe:end_unit_rr, tt)];     % ��¼��Ԫ
            if abs(map(test_unit_rr, tt)) > sum(abs(test_units)) * (Pfa^(-1/R/2)-1)
                detect_map(test_unit_rr, tt) = map(test_unit_rr, tt);
                detect_result = [detect_result; test_unit_rr, tt, map(test_unit_rr, tt)];
            end
            detect_threshold(test_unit_rr, tt) = sum(abs(test_units)) * (Pfa^(-1/R/2)-1);
            compute_result(test_unit_rr, tt) = abs(map(test_unit_rr, tt));
        end
    end

end