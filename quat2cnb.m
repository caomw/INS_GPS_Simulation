function cnb = quat2cnb( quat )
%QUAT2CNB �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
    cnb(1,1) = quat(1)*quat(1) + quat(2)*quat(2) - quat(3)*quat(3) - quat(4)*quat(4);
    cnb(1,2) = 2*(quat(2)*quat(3)+quat(1)*quat(4));
    cnb(1,3) = 2*(quat(2)*quat(4)-quat(1)*quat(3));
    cnb(2,1) = 2*(quat(2)*quat(3)-quat(1)*quat(4));
    cnb(2,2) = quat(1)*quat(1)-quat(2)*quat(2)+quat(3)*quat(3)-quat(4)*quat(4);
    cnb(2,3) = 2*(quat(3)*quat(4)+quat(1)*quat(2));
    cnb(3,1) = 2*(quat(2)*quat(4)+quat(1)*quat(3));
    cnb(3,2) = 2*(quat(3)*quat(4)-quat(1)*quat(2));
    cnb(3,3) = quat(1)*quat(1) - quat(2)*quat(2) - quat(3)*quat(3) + quat(4)*quat(4);

end

