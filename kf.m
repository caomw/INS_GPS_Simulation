function [Xk_1,Pk] = kf(Phik,Qk,Pk,Xk,Hk,Rk,Zk)
%KF 此处显示有关此函数的摘要
%   此处显示详细说明
if nargin < 7
    Xk_1 = Phik*Xk;
    Pk = Phik*Pk*Phik + Qk;
else
    Xkk_1 = Phik*Xk;
    Pkk_1 = Phik*Pk*Phik' + Qk;
    Pxz = Pkk_1*Hk';
    Pzz = Hk*Pxz + Rk;
    Kk = Pxz/Pzz;
    Xk_1 = Xkk_1 + Kk*(Zk-Hk*Xkk_1);
%     Pk = Pkk_1 - Kk*Pzz*Kk';
    CC = (eye(18,18) - Kk*Hk);
    Pk = CC*Pkk_1*CC' + Kk*Rk*Kk';
end
end

