% TrackInit ���ٺ�����ʼ��
% ����1��λ����Ϣ Posi
% ����2���ٶ���Ϣ Velo
% ����3��ʱ��
% ����4����ʼ֡��� FrameIndex
% ����5������ID
% ���1�������ṹ�� Tracks
function Tracks = trackInit(Posi, Velo, t, FrameIndex, ID, lambda)
    if nargin == 5
        lambda = 1;
    end
    Tracks.t        = t;
    Tracks.InitPosi = Posi;
    Tracks.InitVelo = Velo;
    Tracks.FrameIndex = FrameIndex;
    Tracks.ObservedFrame = 1; % �۲����
    Tracks.LossFrame = 0;     % ��ʧ����
    Tracks.LossFrameMax = 10; % ��ʧ���� Single: 10
    Tracks.ConfirmMax   = 5;  % ������Ҫ�۲���ٴ��� Single:5
    Tracks.Type = 0;          % 0����ʱ���� 1��ȷ�Ϻ��� 2��ɾ������
    Tracks.ID   = ID;
    Tracks.Lambda = lambda;
    if length(Posi) == 2
        Tracks.Nz = 2; % ά��
        Tracks.X = [Posi(1) Velo(1) Posi(2) Velo(2)]';
        Tracks.Q = eye(4);                  % ϵͳ��������Э����
        Tracks.H = [1 0 0 0;
                    0 1 0 0;
                    0 0 1 0;
                    0 0 0 1];               % �۲����
        Tracks.F = [1 t 0 0;                % ״̬ת�ƾ���
                    0 1 0 0;
                    0 0 1 t;
                    0 0 0 1];
        Tracks.R = diag([1 1 1 1]);         % �۲�Э�������
        Tracks.P = diag([1 1 1 1]); % Э�������
    elseif length(Posi) == 3
        Tracks.Nz = 3; % ά��
        Tracks.X = [Posi(1) Velo(1) Posi(2) Velo(2) Posi(3) Velo(3)]';
        Tracks.Q = eye(6);   % ϵͳ��������Э����
        Tracks.H = [1 0 0 0 0 0;
                    0 1 0 0 0 0;
                    0 0 1 0 0 0;
                    0 0 0 1 0 0;
                    0 0 0 0 1 0;
                    0 0 0 0 0 1];                   % �۲����
        Tracks.F = [1 t 0 0 0 0;                    % ״̬ת�ƾ���
                    0 1 0 0 0 0;
                    0 0 1 t 0 0;
                    0 0 0 1 0 0;
                    0 0 0 0 1 t;
                    0 0 0 0 0 1];
        Tracks.R = diag([1 100 1 100 1 100]);             % �۲�Э�������
        Tracks.P = diag([1 100 1 100 1 100]); % Э�������
    else
        Tracks = [];
        error('Track Initial: Not Support High Dimension');
    end
end

