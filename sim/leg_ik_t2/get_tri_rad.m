function [rad] = get_tri_rad(ll1,ll2,ll3)
%get rad towards l3

rad = acos((ll1^2+ll2^2-ll3^2)/(2*ll1*ll2));
end

