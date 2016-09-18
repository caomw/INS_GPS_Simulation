function quat = cnb2quat( cnb )
%CNB2QUAT �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
quat = zeros(4,1);
quat(2) = sqrt(abs(1 + cnb(1,1) - cnb(2,2) - cnb(3,3)))/2;
quat(3) = (cnb(1,2) + cnb(2,1))/(4*quat(2));
quat(4) = (cnb(1,3) + cnb(3,1))/(4*quat(2));
quat(1) = (cnb(2,3) - cnb(3,2))/(4*quat(2));
end

