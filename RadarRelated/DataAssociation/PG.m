% 得到波门概率
% gamma: chi2inv(1-alpha,nz) % alpha = 0.99 e.g.
function P_G = PG(gamma,N_z)
% 只实现了4维
    switch (N_z)
        case 1
            P_G = 2*(normcdf(sqrt(gamma)) - normcdf(0));
        case 2
            P_G = 1-exp(-gamma/2);
        case 3
            P_G = 2*(normcdf(sqrt(gamma)) - normcdf(0)) - sqrt(2*gamma/pi)*exp(-gamma/2);
        case 4
            P_G = 1-(1+gamma/2)*exp(-gamma/2);
        otherwise
    end
end