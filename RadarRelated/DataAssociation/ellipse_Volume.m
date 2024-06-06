% S = HPH + R
function [Volume] = ellipse_Volume(gamma,N_z,S)
    switch (N_z)
        case 1
            Volume = 2*sqrt(gamma)*sqrt(det(S));
        case 2
            Volume = pi*gamma*sqrt(det(S));
        case 3
            Volume = 4*pi/3*gamma^(3/2)*sqrt(det(S));
        otherwise
    end
end