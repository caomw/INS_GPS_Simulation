function [Xk,Pk] = cholkf(Phik,Qk,Pk,Xk,Hk,Rk,Zk)
%CHOLKF �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��

%      ����ƽ�����˲�
if nargin < 7
        Xk = Phik*Xk;
%     P = A*P*A'+ QQ;
    Pk = Phik*Pk*Phik'+ Qk;
else
%     ʱ�����
    Xk = Phik*Xk;
%     P = A*P*A'+ QQ;
    Pk = Phik*Pk*Phik'+ Qk;
%     �������
    LM = chol(P,'lower');
    for j = 1:3
        aj = (Hk(j,:)*LM)';
        bj = 1/(aj'*aj + Rk(j,j));
        gamaj = 1/(1 + (bj*Rk(j,j))^0.5);
        Kj = bj*LM*aj;
        Xk = Xk + Kj*(Zk(j,1) - Hk(j,:)*Xk);
        LM = LM - gamaj*Kj*aj';
    end
    Pk = LM*LM';
end
end

