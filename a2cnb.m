function cnb = a2cnb(atti)
%A2CNB �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
sp = sin(atti(1));cp = cos(atti(1));    %����
sr = sin(atti(2));cr = cos(atti(2));    %��ת
sy = sin(atti(3));cy = cos(atti(3));
cnb(1,1) = cy*cr+sy*sp*sr;
cnb(1,2) = -sy*cr+cy*sr*sp;
cnb(1,3) = -sr*cp;
cnb(2,1) = sy*cp;
cnb(2,2) = cy*cp;
cnb(2,3) = sp;
cnb(3,1) = cy*sr-sy*sp*cr;
cnb(3,2) = -sy*sr-cy*sp*cr;
cnb(3,3) = cp*cr;
end

