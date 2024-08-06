% Read Deep Learning Result
xx_ = []; yy_ = [];
for tt = 1:length(TracksTrace)
    if ~isempty(TracksTrace{tt})
        xx_ = [xx_ TracksTrace{tt}(:, 1)'];
        yy_ = [yy_ TracksTrace{tt}(:, 2)'];
    end
end

figure(10)
scatter(xx_, yy_, 5, 'r', 'filled')

% Locate Result
xx_ = []; yy_ = [];
for tt = 1:length(Target)
    if ~isempty(Target{tt})
        xx_ = [xx_ Target{tt}(:, 1)'];
        yy_ = [yy_ Target{tt}(:, 2)'];
    end
end

figure(11)
scatter(xx_, yy_, 5, 'r', 'filled')