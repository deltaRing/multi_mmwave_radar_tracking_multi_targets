% ??è¿¹æ?´æ??
% measures:æµ????¼ï?x y (z) vx vy (vz)ï¼?
% Tracks:å·²å»ºç«?????è¿?
% KFenable:?¡å??¼æ»¤æ³¢æ?æ´?
function [tracks, redundantMeasures, observed, observed_track] = TraceUpdate(measures, Tracks, SimilarRange, KFenable)
    if nargin == 2
        SimilarRange = 2.75;
        KFenable = 1;
    end
    tracks = []; redundantMeasures = []; observed_track = {}; observed = [];
    if isempty(Tracks)
        error('TraceUpdate: Tracks are empty, need built Tracks')
    end
    
    observedMeasure = zeros(1, size(measures, 1));
    observed = zeros(1, length(Tracks));
    for tt = 1:length(Tracks)
        xx = Tracks{tt}.X(1); yy = Tracks{tt}.X(3);
        vx = Tracks{tt}.X(2); vy = Tracks{tt}.X(4);
        Pcov = Tracks{tt}.P;
        for mm = 1:size(measures, 1)
            if observedMeasure(mm), continue; end
            xxx = measures(mm, 1); yyy = measures(mm, 2);
            vxx  = measures(mm, 3); vyy  = measures(mm, 4);
            deltaInfo = [xxx - xx Tracks{tt}.Lambda * (vxx - vx) ...
                yyy - yy Tracks{tt}.Lambda * (vyy - vy)]'; % for main.m
%             deltaInfo = [xxx - xx (vxx - vx) yyy - yy (vyy - vy)]'; % for others
            distance  = sqrt(deltaInfo' * inv(Pcov) * deltaInfo);
            observed_track{tt}{mm} = [xxx vxx yyy vyy]';
            if distance < SimilarRange
                observedMeasure(mm) = 1;
                observed(tt) = 1;
                if KFenable
                    Z  = [xxx vxx yyy vyy];
                    X_ = Tracks{tt}.F * Tracks{tt}.X;
                    P_ = Tracks{tt}.F * Tracks{tt}.P * Tracks{tt}.F' + Tracks{tt}.Q; 
                    K  = P_ * Tracks{tt}.H' / (Tracks{tt}.H * P_ * Tracks{tt}.H' + Tracks{tt}.R);
                    Tracks{tt}.X = X_ + K * (Z' - Tracks{tt}.H * X_);
                    Tracks{tt}.P = (eye(4) - K * Tracks{tt}.H) * P_;
                    break
                else
                    Tracks{tt}.X = [xxx vxx ...
                        yyy vyy]';
                    break
                end
            end
        end
    end

    indexTrack = find(observed == 0);
    for tt = 1:length(indexTrack)
        xxx = Tracks{tt}.X(1); yyy = Tracks{tt}.X(3);
        vxx = Tracks{tt}.X(2); vyy = Tracks{tt}.X(4);
        Z  = [xxx vxx yyy vyy];
        X_ = Tracks{tt}.F * Tracks{tt}.X;
        P_ = Tracks{tt}.F * Tracks{tt}.P * Tracks{tt}.F' + Tracks{tt}.Q; 
        K  = P_ * Tracks{tt}.H' / (Tracks{tt}.H * P_ * Tracks{tt}.H' + Tracks{tt}.R);
        Tracks{tt}.X = X_ + K * (Z' - Tracks{tt}.H * X_);
        Tracks{tt}.P = (eye(4) - K * Tracks{tt}.H) * P_;
    end

    index = find(observedMeasure == 0);
    redundantMeasures = measures(index, :);
    tracks = Tracks;
end