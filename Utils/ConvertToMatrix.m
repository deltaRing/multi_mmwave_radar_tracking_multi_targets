function PosiM = ConvertToMatrix(Posi)
    PosiM = [];
    TarNum = -1;
    for ii = 1:length(Posi)
        if TarNum <= length(Posi{ii})
            TarNum = length(Posi{ii});
        end
    end

    for ff = 1:length(Posi)
        for tt = 1:TarNum
            PosiM{tt} = Posi{ff}
        end
    end
end