function cen = update_cen( cen,wnen )
%UPDATE_CEN 此处显示有关此函数的摘要
%   此处显示详细说明
    global step;
    cen(1,1) = cen(1,1) + step*(wnen(3)*cen(2,1)-wnen(2)*cen(3,1));
    cen(1,2) = cen(1,2) + step*(wnen(3)*cen(2,2)-wnen(2)*cen(3,2));
    cen(1,3) = cen(1,3) + step*(wnen(3)*cen(2,3)-wnen(2)*cen(3,3));
    cen(2,1) = cen(2,1) + step*(-wnen(3)*cen(1,1)+wnen(1)*cen(3,1));
    cen(2,2) = cen(2,2) + step*(-wnen(3)*cen(1,2)+wnen(1)*cen(3,2));
    cen(2,3) = cen(2,3) + step*(-wnen(3)*cen(1,3)+wnen(1)*cen(3,3));
    cen(3,1) = cen(3,1) + step*(wnen(2)*cen(1,1)-wnen(1)*cen(2,1));
    cen(3,2) = cen(3,2) + step*(wnen(2)*cen(1,2)-wnen(1)*cen(2,2));
    cen(3,3) = cen(3,3) + step*(wnen(2)*cen(1,3)-wnen(1)*cen(2,3));

end

