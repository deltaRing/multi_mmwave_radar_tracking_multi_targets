% 绘制雷达探测区域
% 输入1：真实的雷达位置
% 输入2：绘图的标号
%
function plotRadarDetectArea(realradars, figureIndex)
    if nargin == 1, figureIndex = 10000; end
    figure(figureIndex)
    
    for rr = 1:length(realradars)
        if realradars{rr}.Dimension ~= 2
            error('Radar Detect Area: ')
        end
        
        radar_Loc          = [realradars{rr}.x realradars{rr}.y];
        radar_center_angle = realradars{rr}.yaw;
        radar_search_angle = realradars{rr}.sYaw;
        theta_start        = radar_center_angle - radar_search_angle / 2;
        theta_end          = radar_center_angle + radar_search_angle / 2;
        range              = realradars{rr}.sRange;
        [panX, panY]       = PanEdges(radar_Loc, theta_start, theta_end, range);
        
        PanVertices = [panX panY];
        patch('Faces', linspace(1, length(panX), length(panX)), ...
            'Vertices',PanVertices,'FaceColor','green','FaceAlpha',.3);
    end
    
end

function [pEdgesx, pEdgesy] = PanEdges(Center, ThetaStart, ThetaEnd, ...
                                    Range, steps, baseline_steps)
    if nargin == 4
        steps = 50;
        baseline_steps = 10;
    end
    pEdgesx = [];
    pEdgesy = [];
    if length(Center) > 2
       error('Pan Edges Generate: Support 2 dimension only')
    end
    
    if ThetaStart > ThetaEnd
       error('Pan Edges Generate: ThetaStart need less than ThetaEnd')
    end
    
    % 起始点绘制以及结束点绘制
    start_point = [];
    end_point   = [];
    pEdgesx = [pEdgesx; Center(1)];
    pEdgesy = [pEdgesy; Center(2)];
    for an = ThetaStart:abs(ThetaStart - ThetaEnd) / steps:ThetaEnd
        delta_X = cos(an) * Range;
        delta_Y = sin(an) * Range;
        if an == ThetaStart
            start_point = [delta_X delta_Y];
            for rr = 1:baseline_steps
                pEdgesx = [pEdgesx; Center(1) + ...
                    start_point(1) / baseline_steps * rr];
                pEdgesy = [pEdgesy; Center(2) + ...
                    start_point(2) / baseline_steps * rr];
            end
        end
        pEdgesx = [pEdgesx; delta_X + Center(1)];
        pEdgesy = [pEdgesy; delta_Y + Center(2)];
        if an == ThetaEnd
            end_point   = [delta_X delta_Y]; 
            for rr = baseline_steps:1
                pEdgesx = [pEdgesx; Center(1) + ...
                    end_point(1) / baseline_steps * rr];
                pEdgesy = [pEdgesy; Center(2) + ...
                    end_point(2) / baseline_steps * rr];
            end
        end
    end
    
    pEdgesx = [pEdgesx; Center(1)];
    pEdgesy = [pEdgesy; Center(2)];
    
end