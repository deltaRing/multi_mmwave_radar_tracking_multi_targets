function [dTargets, vTargets] = GenerateSimulationResults(sRadars, sTargets, ...
                                    Frames, dt)
    time = dt * Frames;
    dTargets = {}; vTargets = {};
    [dTargets, vTargets] = GenerateTarget(sRadars, sTargets, time);
end