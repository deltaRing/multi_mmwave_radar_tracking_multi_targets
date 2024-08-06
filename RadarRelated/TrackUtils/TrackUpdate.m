% ���²�ɾ�����ں���
% ����1��Tracks ԭ�еĺ���
% ����2��Observed �۲⵽�ĺ�����Ч���� ������Ŀ��1 ��0��
% ���1�����µĺ���
function [Track] = TrackUpdate(Tracks, observed)
    delete_index = [];
    for tt = 1:length(observed)
        if observed(tt)
            if Tracks{tt}.Type == 0 % δȷ��
                Tracks{tt}.LossFrame = 0; % ��ʧ����������
                Tracks{tt}.ObservedFrame = Tracks{tt}.ObservedFrame + 1; % �۲�������
                if Tracks{tt}.ObservedFrame >= Tracks{tt}.ConfirmMax
                    Tracks{tt}.Type = 1; % ����ȷ��
                end
            elseif Tracks{tt}.Type == 1 % ȷ�Ϻ���
                Tracks{tt}.LossFrame = 0; % ��ʧ����������
                Tracks{tt}.ObservedFrame = Tracks{tt}.ObservedFrame + 1; % �۲�������
            elseif Tracks{tt}.Type == 2 % ��ʧ����
                
            end
        else
            Tracks{tt}.LossFrame = Tracks{tt}.LossFrame + 1; % ��ʧ֡������
%             Tracks{tt}.X         = Tracks{tt}.F * Tracks{tt}.X;
            if Tracks{tt}.LossFrame >= Tracks{tt}.LossFrameMax
                delete_index = [delete_index tt]; % ����ɾ��������
                Tracks{tt}.Type = 2; % תΪ��ʧ����
            end
        end
    end
    if ~isempty(delete_index), Tracks(delete_index) = []; end
    Track = Tracks;
end