function cen = pos2cen(pos)
%POS2CEN 此处显示有关此函数的摘要
%   此处显示详细说明
sla = sin(pos(1));cla = cos(pos(1));slo = sin(pos(2)); clo = cos(pos(2));
cen(1,1) = -slo;
cen(1,2) = clo;
cen(1,3) = 0;
cen(2,1) = -sla*clo;
cen(2,2) = -sla*slo;
cen(2,3) = cla;
cen(3,1) = cla*clo;
cen(3,2) = cla*slo;
cen(3,3) = sla;

end

