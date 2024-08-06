function [a, r] = getMouseClickCoordinates(RAmap, rangeAxis, angleAxis, numPoint, index)
    if nargin == 3, index = 10002; numPoint = 1; end
    figure(index)
    imagesc(angleAxis, ...
            rangeAxis, ...
            db(RAmap))
    hold on
    expand_range = 3;
    expand_angle = 5;
    a = [];
    r = [];
    for np = 1:numPoint
        w = waitforbuttonpress;
        
        if w == 0
            point = get(gca, 'CurrentPoint');
            a_ = point(1, 1);
            r_ = point(1, 2);
            a = [a a_];
            r = [r r_];

            % [rangeIndex, angleIndex] = ...
            %     getIndex([r_ a_], rangeAxis, angleAxis);
            % PosRStart = rangeIndex - expand_range;
            % PosREnd   = rangeIndex + expand_range;
            % PosAStart = angleIndex - expand_angle;
            % PosAEnd   = angleIndex + expand_angle;
            % if PosRStart < 1, PosRStart = 1; end
            % if PosAStart < 1, PosAStart = 1; end
            % if PosREnd > length(rangeAxis), PosREnd = length(rangeAxis); end
            % if PosAEnd > length(angleAxis), PosAEnd = length(angleAxis); end
            % currentValue = abs(RAmap(rangeIndex, angleIndex));
            % for rr = PosRStart:PosREnd
            %     for aa = PosAStart:PosAEnd
            %         if currentValue < abs(RAmap(rr, aa))
            %             currentValue = abs(RAmap(rr, aa));
            %             a_ = angleAxis(aa);
            %             r_ = rangeAxis(rr);
            %         end
            %     end
            % end
            scatter(a_, r_, 300, 'r', 'x')
            drawnow
        else 
            disp('Skipped')
        end
    end
end

function [rangeIndex, angleIndex] = getIndex(newPos, range_axis, angle_axis)
    rangeIndex = -1;
    angleIndex = -1;
    diff_range = abs(newPos(1) - range_axis);
    diff_angle = abs(newPos(2) - angle_axis);
    [~, rangeIndex] = min(diff_range);
    [~, angleIndex] = min(diff_angle);
end