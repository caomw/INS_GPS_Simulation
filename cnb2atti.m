function atti = cnb2atti( cnb )
%CNB2ATTI 此处显示有关此函数的摘要
%   此处显示详细说明
    atti(1) = asin(cnb(2,3));
    atti(2) = atan(-cnb(1,3)/cnb(3,3));
    atti(3) = atan(cnb(2,1)/cnb(2,2));
    if cnb(3,3)<0
        if atti(2)<0
            atti(2) = atti(2) + pi;
        else
            atti(2) = atti(2) - pi;
        end
    end
    if cnb(2,2)<0
        atti(3) = atti(3) + pi;
    end
%     ins_atti(i,1) = asin(cnb(2,3));
%     ins_atti(i,2) = asin(-cnb(1,3))/cos(ins_atti(i,1));
%     ins_atti(i,3) = asin(cnb(2,1))/cos(ins_atti(i,1));
%     if cnb(3,3)<0
%         if ins_atti(i,2)<0
%             ins_atti(i,2) =  pi - ins_atti(i,2) ;
%         else
%             ins_atti(i,2) = -pi - ins_atti(i,2) ;
%         end
%     end
%     if cnb(2,2)>0 && cnb(2,1)<0
%         ins_atti(i,3) = ins_atti(i,3) + 2*pi;
%     else if cnb(2,2)<0
%             ins_atti(i,3) = pi - ins_atti(i,3);
%         end
%     end
end

