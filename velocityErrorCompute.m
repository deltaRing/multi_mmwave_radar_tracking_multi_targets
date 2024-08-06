% velocity Error compute
load velo6.mat
radarInit

errVelo = [];
for vv = 1:length(velo_opt)
    if isempty(verr{vv})
        errVelo = [errVelo inf];
        continue;
    end
   
    ver = verr{vv}(end, :);
    
    errVelo = [errVelo ver(1)];
end

errVelo(find(errVelo == inf)) = [];
mean(errVelo)