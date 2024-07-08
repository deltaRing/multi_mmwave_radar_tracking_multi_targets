function [a, r] = getMouseClickCoordinates(RAmap, rangeAxis, angleAxis, numPoint, index)
    if nargin == 3, index = 10002; numPoint = 1; end
    figure(index)
    imagesc(angleAxis, ...
            rangeAxis, ...
            db(RAmap))
    hold on
    
    for np = 1:numPoint
        waitforbuttonpress;
    
        point = get(gca, 'CurrentPoint');
        a = point(1, 1);
        r = point(1, 2);

        scatter(r, a, 50, 'filled', 'r')
    end
end