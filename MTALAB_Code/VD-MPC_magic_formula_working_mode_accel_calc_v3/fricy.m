function fy=fricy(alpha,F_z)
    persistent B C D E;
    if isempty(B)
        B=11.275; C=1.56; D=1; E=-0.97;
    end
    %aB=asin(sin(alpha));
    F_z;
    f =  D * sin(C * atan(B * alpha - E * (B * alpha - atan(B * alpha))));

    %f=D* sin(C*atan(B*alpha - E * ( B * alpha - atan( B* alpha))));
    fy= -1.4 *f.*F_z;

    

    end

% for i=1:4
    %     if fy(i)*alpha(i)>0
    %         fy(i)=-fy(i);
    %     end




% function fy=fricy(alpha,Fz)
% persistent B C D E;
% if isempty(B)
% B=11.275; C=1.56; D=-.95; E=-1.999;
% end
% aB=asin(sin(alpha));
% f=D*sin(C*atan(B*(1-E)*aB+E*atan(B*aB)));
% fy=f.*Fz;
%  for i=1:4
%      if fy(i)*alpha(i)>0
%          fy(i)=-fy(i);
%      end
%  end