function DeeplearningConvert(fname)
    fid = fopen(fname, 'r');
    if fid < 0, fprintf('无法打开文件\n'); end

    while ~feof(fid)
        line = fgetl(fid);
        
    end

    fclose(fid);
end