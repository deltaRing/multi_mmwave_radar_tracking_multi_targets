% Write data to Deep Learning
function writeDatatoDeepLearning(name, data_r, data_a,...
        radar_axis, angle_axis)
    fid = fopen(name, 'w');
    for ii = 1:length(data_r)
        for iii = 1:length(radar_axis)
            if abs(data_r(ii) - radar_axis(iii)) < 1e-1
                rangeIndex = iii;
                break;
            end
        end
        for iii = 1:length(angle_axis)
            if abs(data_a(ii) - angle_axis(iii)) < 1e-2
                angleIndex = iii;
                break;
            end
        end
        fprintf(fid, '%d\t%d\t%d,%d\n', ii-1, 1, rangeIndex, angleIndex);
    end
    fclose(fid);
end