function pos = cen2pos(cen)
%CEN2POS 此处显示有关此函数的摘要
%   此处显示详细说明
    
    pos(1) = asin(cen(3,3));
    pos(2) = atan(cen(3,2)/cen(3,1));
    if cen(3,1) < 0
        if pos(2) > 0
            pos(2) = pos(2) - pi;
        else
            pos(2) = pos(2) + pi;
        end
    end
    pos(3) = 0;
end

