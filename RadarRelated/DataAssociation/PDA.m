function [Tracks, observed, empty_measurements, observed_track] = ...
    PDA(tracks, measurements, Range)

    if nargin == 2, Range = 2.5; end

    empty_measurements = [];
    observed = zeros([1, length(tracks)]);
    observed_track = [];     % 观测到的航迹
    Tracks = [];

    for ii = 1:length(tracks)

        % extract track coordinates and covariance
        X = tracks{ii}.X;
        P = tracks{ii}.P;
        F = tracks{ii}.F;
        H = tracks{ii}.H;
        Q = tracks{ii}.Q;
        R = tracks{ii}.R;

        % make prediction
        xpred   = F*X;
        Ppred   = F*P*F' + Q;
        zpred   = H*xpred;
        S_tmp       = H*Ppred*H' + R;
        S = (S_tmp + S_tmp.') / 2;  %% For Positive definite / Symmetric / Square
        
        % 计算马氏距离
        MeasureMents = [];
        xx = tracks{ii}.X(1); yy = tracks{ii}.X(3);
        vx = tracks{ii}.X(2); vy = tracks{ii}.X(4);
        for iii = size(measurements, 2):-1:1
            % x vx y vy
            xxx = measurements(1, iii); yyy = measurements(3, iii);
            vxx  = measurements(2, iii); vyy  = measurements(4, iii);
            deltaInfo = [xxx - xx vxx - vx yyy - yy vyy - vy]';
            distance  = sqrt(deltaInfo' * inv(P) * deltaInfo);
            if distance < Range
                MeasureMents = [MeasureMents measurements(:, iii)];
                observed(ii) = 1;
                observed_track{iii}{ii} = measurements(:, iii);
                measurements(:, iii) = [];
            end
            
        end

        %--------------Process the measurements and do Probabilistic Data Association
        if isempty(MeasureMents)
            z=zpred; %No measurement case
            v=zeros(size(z,1),1);

            %For Probability Data Association
            Beta_0=1; %switching value

            %Calc Kalman gain
            K=(Ppred*H')/(H*Ppred*H'+R);
            P_c=Ppred-K*S*K';
            P_tilda=0;

        else
            z=MeasureMents;
            %For Probability Data Association
            Beta=[]; LR=[]; Sum_LR=0; v=zeros(size(z,1),1); Beta_0=0; %switching value

            %Calc LR
            for j=1:size(MeasureMents,2)
                LR(j)=mvnpdf(z(:,j),zpred,S);
                %The error may occur on 'mvnpdf' : SIGMA must be a square, symmetric, positive definite matrix.
                Sum_LR=Sum_LR+LR(j);
            end

            %Calc Beta
            for j=1:size(MeasureMents,2)
                Beta(j)=LR(j)/Sum_LR;
            end

            %Calc Combined Innovation
            for j=1:size(MeasureMents,2)
                v=v+Beta(j)*(z(:,j)-zpred);
            end

            %Calc Kalman gain
            K=(Ppred*H')/(H*Ppred*H'+R);
            P_c=Ppred-K*S*K';

            %----------------for Ptilda------------
            Spread_of_Innovations_temp=0;
            for j=1:size(MeasureMents,2)
                v_l=(z(:,j)-zpred);
                Spread_of_Innovations_temp=Spread_of_Innovations_temp+(Beta(j)*(v_l*v_l'));
            end
            P_tilda=K*(Spread_of_Innovations_temp-v*v')*K';

        end

        X=xpred+K*v;
        %----------------------------------------------
        P=Beta_0*Ppred+(1-Beta_0)*P_c+P_tilda;
        %----------------------------------------------
        tracks{ii}.X=X;   %Parameter Update
        tracks{ii}.P=P;
    end
    Tracks = tracks;
    empty_measurements = measurements;
end