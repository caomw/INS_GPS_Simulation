function quat = update_quat(quat,wbnb)
%UPDATE_QUAT 此处显示有关此函数的摘要
%   此处显示详细说明
    global step;
    delthetax = wbnb(1)*step;
    delthetay = wbnb(2)*step;
    delthetaz = wbnb(3)*step;
    delthetasq = delthetax*delthetax + delthetay*delthetay + delthetaz*delthetaz;
    deltheta(1,1) = 0;
    deltheta(1,2) = -delthetax;
    deltheta(1,3) = -delthetay;
    deltheta(1,4) = -delthetaz;
    deltheta(2,1) = delthetax;
    deltheta(2,2) = 0;
    deltheta(2,3) = delthetaz;
    deltheta(2,4) = -delthetay;
    deltheta(3,1) = delthetay;
    deltheta(3,2) = -delthetaz;
    deltheta(3,3) = 0;
    deltheta(3,4) = delthetax;
    deltheta(4,1) = delthetaz;
    deltheta(4,2) = delthetay;
    deltheta(4,3) = -delthetax;
    deltheta(4,4) = 0;
    
    quat = ((1 - delthetasq)*eye(4,4) + (0.5 - delthetasq/48)*deltheta)*quat;
    quat = quat/norm(quat);
    
%     q(1) = q(1) + step*(-wbnb(1)*q(2)-wbnb(2)*q(3)-wbnb(3)*q(4))/2;
%     q(2) = q(2) + step*(wbnb(1)*q(1)+wbnb(3)*q(3)-wbnb(2)*q(4))/2;
%     q(3) = q(3) + step*(wbnb(2)*q(1)-wbnb(3)*q(2)+wbnb(1)*q(4))/2;
%     q(4) = q(4) + step*(wbnb(3)*q(1)+wbnb(2)*q(2)-wbnb(1)*q(3))/2;
end

